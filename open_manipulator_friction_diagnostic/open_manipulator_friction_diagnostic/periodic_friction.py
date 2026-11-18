"""Exploratory positive load/speed/angle law, without posture-ID inputs.

Positive trigonometric basis functions keep predicted loss nonnegative without
clipping predictions. Frequency and regularization are declared experiment
choices, not a claim of identified gearbox physics or torque accuracy.
"""
from copy import deepcopy
import numpy as np
from scipy.optimize import nnls,minimize,LinearConstraint
from .common_friction import check,soft_load_features,weights
from .automatic_calibration import content_hash


def angular_basis(position,frequency,orders,bin_width=0.):
    q=np.atleast_1d(position).astype(float)
    if (not np.isfinite(q).all() or not np.isfinite(frequency) or frequency<=0. or
            not 1<=len(orders)<=4 or any(k<=0 or int(k)!=k for k in orders) or
            len(set(orders))!=len(orders) or not np.isfinite(bin_width) or bin_width<0.):
        raise ValueError('Invalid periodic coordinates')
    return np.column_stack([np.ones(len(q))]+[
        1+sign*np.sinc(frequency*k*bin_width/(2*np.pi))*fn(frequency*k*q)
        for k in orders for fn in (np.sin,np.cos) for sign in (1,-1)])


def configuration_group(c,joint):
    """Shifted windows of the same other-joint pose stay in the same fold."""
    q=np.asarray(c['q']);return tuple(np.round(np.mean(np.delete(q,joint,axis=1),axis=0),3))


def design(model,position,speed,gravity,bin_width=0.):
    q,v,g=np.broadcast_arrays(np.atleast_1d(position),np.atleast_1d(speed),np.atleast_1d(gravity))
    if q.ndim!=1 or not np.isfinite([q,v,g]).all():raise ValueError('Invalid runtime coordinates')
    load=soft_load_features(g,model['load_width_nm'],model['load_feature_scale_nm'])
    knots=model.get('load_hat_knots_nm',[])
    if knots:
        a=np.asarray(knots,float)
        if a.ndim!=1 or not 3<=len(a)<=16 or not np.isfinite(a).all() or np.any(np.diff(a)<=0):
            raise ValueError('Invalid additional load coordinates')
        # Nonnegative physical-load features, never posture IDs. Existing
        # analytic coordinates remain first so zero extension is exact.
        load=np.column_stack([load,weights(a,g)])
    sw=weights(model['speed_knots_rad_s'],abs(v))
    h=angular_basis(q,model['frequency_rad_per_rad'],model['orders'],bin_width)
    return np.einsum('ij,ik,il->ijkl',load,sw,h).reshape(len(q),-1)


def cycle_design(m,c):
    q=np.asarray(c['q'])[:,m['joint']-1]
    width=np.ptp(q)/max(len(q)-1,1)
    return design(m,q,c['speed'],c['signed_gravity_ros_nm'],width)


def predict(model,position,speed,gravity,direction,bin_width=0.):
    result=design(model,position,speed,gravity,bin_width)@np.asarray(model['coefficients'][direction])
    if model.get('load_hat_mean_extension',False):
        # Preserve the frozen signed-load law outside its measured load knots.
        # The angular boundary value remains bounded; only its period mean is
        # extrapolated, with the SAME outward nonnegative slope as the macro.
        _,v,g=np.broadcast_arrays(np.atleast_1d(position),np.atleast_1d(speed),np.atleast_1d(gravity))
        knots=np.asarray(model['load_hat_knots_nm']);ns=len(model['speed_knots_rad_s'])
        means=np.asarray(model['coefficients'][direction]).reshape(3+len(knots),ns,-1).sum(axis=2)[3:]
        values=weights(model['speed_knots_rad_s'],abs(v))@means.T
        lower=np.maximum(0.,(values[:,0]-values[:,1])/(knots[1]-knots[0]))
        upper=np.maximum(0.,(values[:,-1]-values[:,-2])/(knots[-1]-knots[-2]))
        result+=np.maximum(0.,knots[0]-g)*lower+np.maximum(0.,g-knots[-1])*upper
    return result


def evaluate(model,position,speed,gravity,direction):
    return predict(model,position,speed,gravity,direction)


def cycle_prediction(model,cycle,direction):
    q=np.asarray(cycle['q'])[:,model['joint']-1]
    return predict(model,q,cycle['speed'],cycle['signed_gravity_ros_nm'],direction,np.ptp(q)/max(len(q)-1,1))


def fit(cycles,joint,frequency=100.,orders=(1,2),ridge=10.,fit_load_dependence=True,angular_max_speed=None,load_width_nm=2.,macro_smoothing=None,macro_model=None,*,separable_speed_fit=False):
    check(cycles,joint)
    if not np.isfinite(ridge) or ridge<0.:raise ValueError('Invalid regularization')
    if not np.isfinite(load_width_nm) or load_width_nm<=0.:raise ValueError('Invalid load width')
    speeds=sorted({c['speed'] for c in cycles})
    if angular_max_speed is not None and (not np.isfinite(angular_max_speed) or angular_max_speed<min(speeds)):
        raise ValueError('Invalid angular speed support')
    if len(speeds)>32:raise ValueError('At most 32 speed coordinates supported')
    gs=np.concatenate([c['signed_gravity_ros_nm'] for c in cycles])
    model=dict(schema='common_periodic_friction_v1',joint=joint+1,joints=[joint+1],family='positive_periodic_load',
        speed_knots_rad_s=speeds,speed_range_rad_s=[min(speeds),max(speeds)],
        gravity_range_nm=[float(min(gs)),float(max(gs))],load_width_nm=float(load_width_nm),load_feature_scale_nm=max(float(max(abs(gs))),1.),
        frequency_rad_per_rad=float(frequency),orders=list(orders),ridge=float(ridge),
        coefficients={},coefficient_order='load,speed,angular_basis',
        development_sha256=content_hash(cycles),
        automatic_deployment_allowed=False,hardware_qualified=False,
        physical_periodic_cause_identified=False,static_breakaway_identified=False,
        gravity_or_current_conversion_changed=False)
    from .kinetic_provenance import development_provenance
    model.update(development_provenance(cycles))
    if macro_smoothing is not None or macro_model is not None:
        return _mean_preserving_fit(model,cycles,joint,macro_smoothing,fit_load_dependence,angular_max_speed,macro_model,separable_speed_fit)
    x=np.concatenate([cycle_design(model,c)/np.sqrt(len(c['q'])*len(cycles)) for c in cycles]);n=x.shape[1]
    active=np.arange(n) if fit_load_dependence else np.arange(n//3)
    if angular_max_speed is not None:
        h=1+4*len(orders)
        active=np.array([k for k in active if k%h==0 or speeds[(k//h)%len(speeds)]<=angular_max_speed])
        model['angular_max_speed_rad_s']=float(angular_max_speed)
    norms=np.maximum(np.sqrt(np.mean(x[:,active]**2,axis=0)),1e-10)
    lhs=np.vstack([x[:,active]/norms,np.sqrt(ridge/len(active))*np.eye(len(active))])
    for d in ('positive','negative'):
        y=np.concatenate([np.asarray(c[d+'_resistance_proxy_nm'])/np.sqrt(len(c['q'])*len(cycles)) for c in cycles])
        coef=np.zeros(n)
        coef[active]=nnls(lhs,np.r_[y,np.zeros(len(active))],maxiter=20000)[0]/norms
        model['coefficients'][d]=coef.tolist()
    model['load_dependence_fitted']=fit_load_dependence
    model['training_review']=review(model,cycles)
    return model


def _mean_preserving_fit(model,cycles,joint,smoothing,loaded,cutoff,macro=None,separable_speed_fit=False):
    """Fit zero-DC angular residuals without changing the macro loss curve.

    Nonnegative paired harmonic coefficients share each macro coefficient's
    budget. This gives exactly the existing positive runtime basis, preserving
    its period mean and nonnegative loss for every load/speed coordinate.
    Per-cycle centering prevents unknown cycle bias from fitting angular ripple.
    """
    from . import common_friction as base
    from .single_joint_model import table_values
    if macro is None:
        macro=base.fit(cycles,joint,'smooth_load' if loaded else 'velocity',smoothing,
                       model['load_width_nm'] if loaded else None,spatial=True)
    else:
        from .kinetic_runtime import model_parameters
        if (macro.get('schema')!=base.SCHEMA or macro.get('joint')!=joint+1 or
                macro.get('development_sha256')!=content_hash(cycles) or
                macro.get('speed_knots_rad_s')!=model['speed_knots_rad_s'] or
                macro.get('gravity_range_nm')!=model['gravity_range_nm']):
            raise ValueError('Frozen macro must describe this joint and exactly these training cycles')
        model_parameters(macro,joint+1)
        macro=deepcopy(macro)
    loaded=macro['family']!='velocity'
    if macro['family'] in base.SMOOTH_FAMILIES:
        model['load_width_nm']=macro['load_width_nm']
        model['load_feature_scale_nm']=macro['load_feature_scale_nm']
    signed=macro['family']=='signed_load'
    if signed:
        model['load_hat_knots_nm']=list(macro['gravity_knots_nm'])
        model['load_hat_mean_extension']=True
    speeds=model['speed_knots_rad_s'];ns=len(speeds);h=1+4*len(model['orders'])
    nload=3+len(model.get('load_hat_knots_nm',[]))
    active=[k for k in range(nload*ns*(h-1)) if cutoff is None or speeds[(k//(h-1))%ns]<=cutoff]
    blocks=[]
    for c in cycles:
        x=cycle_design(model,c).reshape(len(c['q']),nload*ns,h)
        z=(x[:,:,1:]-x[:,:,:1]).reshape(len(c['q']),-1)[:,active]
        blocks.append((z-z.mean(axis=0))/np.sqrt(len(z)*len(cycles)))
    x=np.concatenate(blocks)
    for d in ('positive','negative'):
        baseline=(np.r_[np.zeros(3*ns),np.asarray(macro['tables'][d]).ravel()] if signed else
                  np.asarray(macro['soft_load_coefficients'][d]).ravel() if loaded else
                  np.r_[macro['tables'][d][1],np.zeros(2*ns)])
        budget=np.repeat(baseline,h-1)[active];z=x*budget
        residuals=[np.asarray(c[d+'_resistance_proxy_nm'])-
                   table_values(macro,c['speed'],c['signed_gravity_ros_nm'],d) for c in cycles]
        y=np.concatenate([(r-r.mean())/np.sqrt(len(r)*len(cycles)) for r in residuals])
        Q=z.T@z+model['ridge']/len(active)*np.eye(len(active));linear=z.T@y
        groups=np.zeros((len(baseline),len(active)))
        for column,k in enumerate(active):groups[k//(h-1),column]=1.
        solution=np.zeros(len(active))
        blocks=([np.flatnonzero([(k//(h-1))%ns==s for k in active]) for s in range(ns)]
                if separable_speed_fit else [np.arange(len(active))])
        for indices in blocks:
            if not len(indices):continue
            outside=np.setdiff1d(np.arange(len(active)),indices)
            if separable_speed_fit and np.any(abs(Q[np.ix_(indices,outside)])>1e-12):
                raise ValueError('Angular objective is not separable by measured speed')
            local=Q[np.ix_(indices,indices)];rhs=linear[indices];constraints=groups[:,indices]
            if separable_speed_fit:
                constraints=constraints[np.any(constraints,axis=1)]
            result=minimize(lambda v:(.5*v@local@v-rhs@v,local@v-rhs),np.zeros(len(indices)),
                jac=True,method='SLSQP',bounds=[(0.,1.)]*len(indices),
                constraints=LinearConstraint(constraints,0.,1.),options={'maxiter':2000,'ftol':1e-11})
            if not result.success:raise ValueError('Angular residual fit failed: '+result.message)
            solution[indices]=result.x
        angular=np.zeros(nload*ns*(h-1));angular[active]=budget*solution
        angular=angular.reshape(nload*ns,h-1);dc=baseline-angular.sum(axis=1)
        if min(dc)<-1e-8:raise ValueError('Angular fit exceeded the nonnegative macro budget')
        # Only solver round-off at an active zero boundary is removed.
        coefficients=np.column_stack([np.maximum(dc,0.),angular])
        if not np.allclose(coefficients.sum(axis=1),baseline,atol=1e-8,rtol=0):
            raise ValueError('Angular fit changed the macro period mean')
        model['coefficients'][d]=coefficients.ravel().tolist()
    model.update(fit_method='mean_preserving_angular_residual',macro_model=macro,
                 macro_model_sha256=content_hash(macro),macro_family=macro['family'],
                 macro_smoothing=macro['smoothing'],load_dependence_fitted=loaded)
    if cutoff is not None:model['angular_max_speed_rad_s']=float(cutoff)
    model['training_review']=review(model,cycles)
    return model


def review(model,cycles):
    rows=[]
    for c in cycles:
        if c['joint']+1!=model['joint']:raise ValueError('Wrong joint')
        for d in ('positive','negative'):
            y=np.asarray(c[d+'_resistance_proxy_nm']);pred=cycle_prediction(model,c,d);e=pred-y
            rows.append(dict(cycle=c['cycle_id'],group=c['group'],direction=d,
                mean_passed=bool(abs(np.mean(e))<=.08+.2*max(0.,np.mean(y))),
                spatial_pass_fraction=float(np.mean(abs(e)<=.08+.2*np.maximum(y,0.))),
                minimum_prediction_nm=float(min(pred)),rmse_nm=float(np.sqrt(np.mean(e*e)))))
    return dict(rows=rows,means_passed=all(r['mean_passed'] for r in rows),
        mean_failures=sum(not r['mean_passed'] for r in rows),
        spatial_failures=sum(r['spatial_pass_fraction']<.95 or not r['mean_passed'] for r in rows),
        rmse_nm=float(np.sqrt(np.mean([r['rmse_nm']**2 for r in rows]))),
        physical_output_torque_validated=False,automatic_deployment_allowed=False)


def cross_validate(cycles,joint,**kwargs):
    groups=sorted({configuration_group(c,joint) for c in cycles})
    if len(groups)<3:raise ValueError('Three independent other-joint configurations required')
    folds=[]
    for g in groups:
        train=[c for c in cycles if configuration_group(c,joint)!=g]
        held=[c for c in cycles if configuration_group(c,joint)==g]
        model=fit(train,joint,**kwargs)
        folds.append(dict(held_configuration=list(g),training_cycle_ids=[c['cycle_id'] for c in train],**review(model,held)))
    return dict(folds=folds,mean_failures=sum(f['mean_failures'] for f in folds),
        spatial_failures=sum(f['spatial_failures'] for f in folds),fresh_validation=False)
