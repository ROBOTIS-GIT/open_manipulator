"""Common low-speed angular template, separate from rest/dithering identification.

Each direction may select a different load curvature/regularization. A bounded
interpolation exports both into the existing positive load-hat runtime; no
controller, gravity, conversion, transition or dither law changes are needed.
This is development model selection, never automatic hardware qualification.
"""
from copy import deepcopy
import numpy as np
from scipy.optimize import minimize, LinearConstraint
from . import common_friction as base, periodic_friction as periodic
from .automatic_calibration import content_hash
from .single_joint_model import table_values
from .kinetic_provenance import development_provenance
from .kinetic_review import review
from .kinetic_runtime import model_parameters


def fit(cycles, joint, recipe):
    required={'macro_family','macro_smoothing','load_width_nm','objective','frequency',
              'orders','ridge','angular_max_speed'}
    if set(recipe)!=required or recipe['macro_family'] not in ('velocity', *base.SMOOTH_FAMILIES):
        raise ValueError('Declare a complete shared velocity or smooth-load template recipe')
    f,orders,cutoff,ridge=(recipe[k] for k in ('frequency','orders','angular_max_speed','ridge'))
    periodic.angular_basis([0.],f,orders)
    if not np.isfinite([cutoff,ridge]).all() or ridge<0 or cutoff<min(c['speed'] for c in cycles):
        raise ValueError('Invalid template speed/ridge')
    macro=base.fit(cycles,joint,recipe['macro_family'],recipe['macro_smoothing'],
                   recipe['load_width_nm'],spatial=True,objective=recipe['objective'])
    speeds=macro['speed_knots_rad_s'];ns=len(speeds);h=1+4*len(orders)
    required_gap=2*np.pi/(6*f*max(orders));matrices=[];resolved=[]
    for c in cycles:
        q=np.asarray(c['q'])[:,joint];width=np.ptp(q)/(len(q)-1)
        H=periodic.angular_basis(q,f,orders,width)[:,1:]-1
        support=c.get('raw_angular_support',{})
        raw_ok=all(d in support and np.isfinite(support[d]['max_gap_rad']) and
            0<support[d]['max_gap_rad']<=required_gap and
            np.ptp(support[d]['angle_interval_rad'])>=2*np.pi/f for d in ('positive','negative'))
        if c['speed']>cutoff:H*=0
        elif raw_ok and np.ptp(q)>=2*np.pi/f and np.max(np.diff(np.sort(q)))<=required_gap:
            resolved.append(c['cycle_id'])
        matrices.append(H)
    if not resolved:raise ValueError('No raw-resolved full period for shared template')
    m=dict(schema='common_periodic_friction_v1',family='positive_periodic_load',joint=joint+1,
        joints=[joint+1],speed_knots_rad_s=speeds,speed_range_rad_s=macro['speed_range_rad_s'],
        gravity_range_nm=macro['gravity_range_nm'],load_width_nm=macro['load_width_nm'] or 2.,
        load_feature_scale_nm=macro['load_feature_scale_nm'],frequency_rad_per_rad=f,
        orders=list(orders),coefficients={},coefficient_order='load,speed,angular_basis',
        angular_max_speed_rad_s=cutoff,recipe=deepcopy(recipe),macro_model=macro,
        macro_model_sha256=content_hash(macro),fit_method='shared_relative_zero_mean_gear_template',
        template_invariance_assumption='relative low-speed ripple shared across load and speed',
        automatic_deployment_allowed=False,hardware_qualified=False,static_breakaway_identified=False,
        gravity_or_current_conversion_changed=False,rest_identification_included=False,
        load_dependence_fitted=macro['family'] != 'velocity', **development_provenance(cycles))
    observability={}
    for d in ('positive','negative'):
        blocks=[];ys=[]
        for c,H in zip(cycles,matrices):
            mean=table_values(macro,c['speed'],c['signed_gravity_ros_nm'],d)
            z=H*mean[:,None];residual=np.asarray(c[d+'_resistance_proxy_nm'])-mean
            blocks.append((z-z.mean(0))/np.sqrt(len(z)*len(cycles)))
            ys.append((residual-residual.mean())/np.sqrt(len(z)*len(cycles)))
        X=np.concatenate(blocks);y=np.concatenate(ys);signed=X[:,::2]
        rank=int(np.linalg.matrix_rank(signed));condition=float(np.linalg.cond(signed))
        if rank<signed.shape[1] or condition>1000:raise ValueError('Unobservable shared angular template')
        Q=X.T@X+ridge*np.eye(h-1);v=X.T@y
        result=minimize(lambda z:(.5*z@Q@z-v@z,Q@z-v),np.zeros(h-1),jac=True,
            method='SLSQP',bounds=[(0.,1.)]*(h-1),
            constraints=LinearConstraint(np.ones((1,h-1)),0.,1.),options={'ftol':1e-12})
        if not result.success:raise ValueError('Shared template fit failed: '+result.message)
        if np.min(result.x)<0 or sum(result.x)>1+1e-10:raise ValueError('Invalid angular budget')
        baseline=(np.asarray(macro['soft_load_coefficients'][d]) if macro['family'] != 'velocity'
                  else np.vstack([macro['tables'][d][1], np.zeros((2, ns))]))
        coef=np.zeros((3,ns,h))
        coef[:,:,0]=baseline
        for k,speed in enumerate(speeds):
            if speed<=cutoff:
                coef[:,k,0]=baseline[:,k]*(1-sum(result.x))
                coef[:,k,1:]=baseline[:,k,None]*result.x
        if np.min(coef)<-1e-10:raise ValueError('Angular budget violated positivity')
        coef=np.maximum(coef,0.)
        if not np.allclose(coef.sum(2),baseline,atol=1e-9,rtol=0):raise ValueError('Changed period mean')
        m['coefficients'][d]=coef.ravel().tolist()
        observability[d]=dict(rank=rank,columns=signed.shape[1],condition_number=condition)
    m['observability']=dict(directions=observability,resolved_cycle_ids=resolved,
        shared_across_speeds=True,per_speed_harmonic_identification=False)
    model_parameters(m,joint+1)
    return m


def project(models, load_knots, maximum_error_nm):
    """Positive linear interpolation with an all-angle/all-speed error bound.

    The bound is for the declared knot interval only. Outside it the existing
    outward nonnegative mean-slope extension is used, with no accuracy claim.
    """
    if set(models)!={'positive','negative'}:raise ValueError('Two directional sources required')
    p,n=models['positive'],models['negative']
    for k in ('joint','speed_knots_rad_s','speed_range_rad_s','gravity_range_nm',
              'development_sha256','development_payload_hashes','frequency_rad_per_rad'):
        if k not in p or p[k]!=n.get(k):raise ValueError('Directional domain differs: '+k)
    knots=np.asarray(load_knots,float)
    if (knots.ndim!=1 or not 3<=len(knots)<=16 or not np.isfinite(knots).all() or
            np.any(np.diff(knots)<=0) or not np.isfinite(maximum_error_nm) or maximum_error_nm<=0 or
            knots[0]>p['gravity_range_nm'][0] or knots[-1]<p['gravity_range_nm'][1]):
        raise ValueError('Projection domain must include measured loads')
    orders=sorted(set(p['orders']+n['orders']));h=1+4*len(orders);ns=len(p['speed_knots_rad_s'])
    if len(orders)>4:raise ValueError('Too many shared harmonic coordinates')
    out=deepcopy(p)
    for k in ('macro_model','macro_model_sha256','recipe','observability','angular_max_speed_rad_s'):
        out.pop(k,None)
    exact=p['load_width_nm']==n['load_width_nm'] and p['load_feature_scale_nm']==n['load_feature_scale_nm']
    out.update(orders=orders,
        coefficients={},fit_method='directional_shared_template_load_interpolation',
        shared_template_sources={d:dict(content_sha256=content_hash(m),recipe=m['recipe']) for d,m in models.items()})
    if exact:
        out.update(fit_method='directional_shared_template_exact_coordinates')
    else:
        out.update(load_hat_knots_nm=knots.tolist(),load_hat_mean_extension=True)
    errors={}
    for d,m in models.items():
        model_parameters(m,m['joint'])
        c=np.asarray(m['coefficients'][d]).reshape(3,ns,-1)
        if exact:
            converted=np.zeros((3,ns,h));converted[:,:,0]=c[:,:,0]
            for i,order in enumerate(m['orders']):
                k=orders.index(order);converted[:,:,1+4*k:5+4*k]=c[:,:,1+4*i:5+4*i]
            out['coefficients'][d]=converted.ravel().tolist();errors[d]=0.
            continue
        values=np.einsum('kl,lsh->ksh',base.soft_load_features(knots,m['load_width_nm'],m['load_feature_scale_nm']),c)
        converted=np.zeros((3+len(knots),ns,h));converted[3:,:,0]=values[:,:,0]
        for i,order in enumerate(m['orders']):
            k=orders.index(order);converted[3:,:,1+4*k:5+4*k]=values[:,:,1+4*i:5+4*i]
        out['coefficients'][d]=converted.ravel().tolist()
        # All non-DC bases are in [0,2]. f'' of hypot(g,w) peaks
        # at the point in each interval closest to zero. Linear interpolation
        # error <= max|f''|*interval_width**2/8 (convex loss, nonnegative weights).
        coefficient=float(np.max(c[1:,:,0].sum(0)+2*c[1:,:,1:].sum((0,2))))
        nearest=np.where((knots[:-1]<=0)&(knots[1:]>=0),0.,np.minimum(abs(knots[:-1]),abs(knots[1:])))
        w=m['load_width_nm'];curvature=coefficient/(2*m['load_feature_scale_nm'])*w*w/(nearest*nearest+w*w)**1.5
        errors[d]=float(np.max(curvature*np.diff(knots)**2/8))
        if errors[d]>maximum_error_nm:raise ValueError('Projection error exceeds declared budget')
    out['projection']=dict(error_bound_nm=errors,maximum_allowed_error_nm=maximum_error_nm,
        load_interval_nm=[float(knots[0]),float(knots[-1])],bound_valid_for_all_angles_and_speeds=True,
        outside_interval_accuracy_proven=False,runtime_law_changed=False,exact_coordinates=exact)
    model_parameters(out,out['joint'])
    return out


def select(cycles,joint,recipes,load_knots,maximum_error_nm=.02,fraction=.95,previous_model=None,*,max_folds=None):
    from .kinetic_selection import selection_folds
    from .kinetic_support import measured_support,preserve_support
    base.check(cycles,joint);groups,coverage=selection_folds(cycles,joint,max_folds)
    if not recipes:raise ValueError('Declared recipes required')
    results=[]
    for recipe in recipes:
        row=dict(recipe=deepcopy(recipe))
        try:
            source=fit(cycles,joint,recipe)
            training=review(project(dict(positive=source,negative=source),load_knots,maximum_error_nm),cycles,fraction)
            folds=[]
            for ids in groups:
                train=[c for c in cycles if c['cycle_id']not in ids];held=[c for c in cycles if c['cycle_id']in ids]
                source=fit(train,joint,recipe)
                folds.append(dict(training_cycle_ids=[c['cycle_id'] for c in train],held_cycle_ids=ids,
                    **review(project(dict(positive=source,negative=source),load_knots,maximum_error_nm),held,fraction)))
            row.update(training=training,folds=folds)
        except ValueError as exc:row['rejected']=str(exc)
        results.append(row)
    choices={}
    for direction in ('positive','negative'):
        viable=[]
        for r in results:
            if 'rejected'in r:continue
            train=[x for x in r['training']['rows'] if x['direction']==direction]
            held=[x for f in r['folds'] for x in f['rows'] if x['direction']==direction]
            if all(x['mean_passed'] for x in train):
                score=float(np.mean([((x['predicted_mean_nm']-x['measured_mean_nm'])/
                    (base.ABSOLUTE_NM+base.RELATIVE*x['measured_mean_nm']))**2 for x in held]))
                viable.append((score,r['recipe'],all(x['mean_passed'] for x in held)))
        if not viable:raise ValueError('No mean-eligible shared template for '+direction)
        # Match macro selection: fit the observed means first, prefer candidates
        # that also predict withheld configurations, and retain failed CV as an
        # explicit diagnostic when no training-eligible recipe passes it.
        held_eligible=[v for v in viable if v[2]]
        choices[direction]=min(held_eligible or viable,key=lambda v:(v[0],len(v[1]['orders'])))[1]
    sources={d:fit(cycles,joint,r) for d,r in choices.items()}
    model=project(sources,load_knots,maximum_error_nm);folds=[]
    for ids in groups:
        train=[c for c in cycles if c['cycle_id']not in ids];held=[c for c in cycles if c['cycle_id']in ids]
        fitted=project({d:fit(train,joint,r) for d,r in choices.items()},load_knots,maximum_error_nm)
        folds.append(dict(training_cycle_ids=[c['cycle_id'] for c in train],held_cycle_ids=ids,
            **review(fitted,held,fraction)))
    model.update(training_review=review(model,cycles,fraction),
        configuration_cv_coverage=coverage,
        posture_review=dict(means_passed=all(f['means_passed'] for f in folds),
            passed=all(f['spatial_passed'] for f in folds),checks=sum(len(f['rows']) for f in folds)),
        selection=dict(method='directional_shared_template_configuration_development_cv',
            priority='training_means_then_all_configuration_means_then_mean_error',
            selected_recipes=choices,candidates=results,folds=folds,fresh_validation=False),
        kinetic_protocol='common_kinetic_v2',measurement_support=measured_support(cycles,joint),
        missing=['fresh_validation','kinetic_release_coverage'])
    if not coverage['all_configurations_assessed']:
        model['missing'].append('incomplete_configuration_cv')
    if not model['training_review']['means_passed']:
        raise ValueError('Combined export did not preserve mean eligibility')
    if not model['posture_review']['means_passed']:
        model['missing'].append('unseen_configuration_mean_prediction')
    if not model['training_review']['spatial_passed'] or not model['posture_review']['passed']:
        model['missing'].append('spatial_prediction')
    if previous_model is not None:model['replacement_support_review']=preserve_support(previous_model,model)
    return model
