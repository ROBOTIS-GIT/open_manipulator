"""Offline identification of load-sensitive steady friction, without activation.

Use nominal URDF gravity as the load feature in both fitting and deployment.
The half-sum of measured currents is a diagnostic, not a runtime load sensor.
This model describes symmetric cycle means; it does not identify breakaway or
separate gravity-model error from directional friction.
"""
import numpy as np
from scipy.optimize import least_squares

from .fit import spatial_weights
from .poses import Robot
from .protocol import JOINTS, validation_poses


def attach_nominal_load(rows, meta, urdf, torque_scaling=None):
    robot = Robot(urdf, JOINTS)
    c = meta['config']; poses = {p['name']: p['q_rad'] for p in meta['posture_validation_poses']}
    scaling = np.ones(6) if torque_scaling is None else np.asarray(torque_scaling)
    result = []
    for row in rows:
        j = row['joint']; name = row['validation_pose']
        if name:
            anchor = poses[name]
        else:
            anchor = c['calibration_centers_rad' if row['repeat'] == 0 else 'validation_centers_rad'][j]
        q = np.tile(anchor, (len(row['position_rad']), 1)).astype(float)
        q[:, j] = row['position_rad']
        load = np.abs(robot.gravity_batch(q)[:, j]*scaling[j])
        result.append(dict(row, joint_position_samples_rad=q.tolist(), nominal_load_samples_nm=load.tolist(),
                           nominal_mean_load_nm=float(spatial_weights(row['position_rad'])@load)))
    return result


def fit_configuration_candidates(rows, absolute, relative, context_joint=2):
    """Empirical load/other-joint coupling, fitted only on development data.

    A positive sin(q_context)^p amplitude with p in (2,4,6). These alternatives
    have equal complexity; RSS and AICc therefore give the same ordering.
    This is not an identified gearbox mechanism or a passivity condition.
    """
    v=np.array([r['speed'] for r in rows]);y=np.array([r['friction_nm'] for r in rows])
    g=np.array([r['nominal_mean_load_nm'] for r in rows])
    if len(rows)<30 or any(r['joint']==context_joint for r in rows):
        raise ValueError('Configuration model requires separate load and posture excitation')
    q=np.array([r['joint_position_samples_rad'][0][context_joint] for r in rows])
    if len(np.unique(q))<3:raise ValueError('Need at least three context angles')
    candidates=[]
    for power in (2,4,6):
        harmonic=np.sin(q)**power
        def predict(z):
            fc,b,lc,s0,peak,vs=z
            return fc+b*v+lc*g+(s0+peak*g*harmonic)*np.exp(-(v/vs)**2)
        solutions=[least_squares(lambda z:predict(z)-y,[3.,1.,.5,.1,2.,vs],
            bounds=([0,0,0,0,0,min(v)],[np.inf]*5+[max(v)]),max_nfev=3000)
            for vs in (.006,.012,.025,.06)]
        best=min(solutions,key=lambda s:s.fun@s.fun);residual=predict(best.x)-y
        rss=float(residual@residual);n=len(y);k=8 # six continuous, power selection, variance
        adequate=bool(np.all(abs(residual)<=absolute+relative*y))
        interior=bool(min(v)*(1+1e-4)<best.x[5]<max(v)*(1-1e-4))
        candidates.append(dict(parameters=best.x.tolist(),power=power,context_joint=context_joint,
            rss=rss,aicc=float(n*np.log(max(rss/n,np.finfo(float).tiny))+2*k+2*k*(k+1)/(n-k-1)),
            training_adequate=adequate,transition_identifiable=interior,
            training_residuals_nm=residual.tolist()))
    eligible=[x for x in candidates if x['training_adequate'] and x['transition_identifiable']]
    return dict(selected=min(eligible,key=lambda s:s['aicc']) if eligible else None,candidates=candidates,
        empirical_configuration_extension=True,validation_used_for_selection=False,
        mechanism_identified=False,passivity_proven=False)


def magnitude(speed, load, p):
    """Fc, delta-Fs, vs, b, load-Coulomb, load-Stribeck, load-vs."""
    fc, delta, vs, b, lc, ls, lvs = p
    v = np.abs(speed)
    return fc+delta*np.exp(-(v/vs)**2)+b*v+np.abs(load)*(lc+ls*np.exp(-(v/lvs)**2))


def fit_candidates(rows, absolute, relative):
    """Training data only. Report failed candidates too; never soften acceptance."""
    v = np.array([r['speed'] for r in rows]); y = np.array([r['friction_nm'] for r in rows])
    load = np.array([r['nominal_mean_load_nm'] for r in rows])
    if len(rows) < 16 or np.ptp(load) < .1:
        raise ValueError('Need independent speeds and nominal load excitation')
    candidates = []
    for name, base_peak, load_peak in (
            ('viscous_load', False, False),
            ('viscous_load_stribeck', False, True),
            ('stribeck_load', True, False),
            ('stribeck_load_stribeck', True, True)):
        # Unused transitions are fixed, not counted as fitted parameters.
        active = [0, 3, 4]+([1, 2] if base_peak else [])+([5, 6] if load_peak else [])
        def unpack(z):
            p = np.array([0., 0., min(v), 0., 0., 0., min(v)])
            p[active] = z
            return p
        lo = np.array([min(v) if i in (2, 6) else 0. for i in active])
        hi = np.array([max(v) if i in (2, 6) else np.inf for i in active])
        solutions = []
        for vs in np.geomspace(min(v)*1.01, max(v)/1.01, 4):
            for lvs in (np.geomspace(min(v)*1.01, max(v)/1.01, 4) if load_peak else [min(v)]):
                initial = np.array([min(y)*.8, max(np.ptp(y)*.2,.01),vs,.1,.1,.1,lvs])[active]
                solutions.append(least_squares(lambda z: magnitude(v,load,unpack(z))-y,
                    initial,bounds=(lo,hi),max_nfev=3000))
        best = min(solutions, key=lambda x: x.fun@x.fun); p = unpack(best.x)
        predicted = magnitude(v,load,p); rss=float(np.sum((predicted-y)**2))
        n,k=len(y),len(active)+1
        interior=all(min(v)*(1+1e-4)<p[i]<max(v)*(1-1e-4)
                     for i in active if i in (2,6))
        adequate=bool(np.all(abs(predicted-y)<=absolute+relative*y))
        aicc=n*np.log(max(rss/n,np.finfo(float).tiny))+2*k+2*k*(k+1)/(n-k-1)
        candidates.append(dict(name=name,parameters=p.tolist(),aicc=float(aicc),
            training_adequate=adequate,transition_identifiable=interior,
            training_residuals_nm=(predicted-y).tolist(), fitted_coefficients=len(active),
            training_rows=[dict(joint=r['joint'],speed=r['speed'],pose=r['validation_pose'],
                repeat=r['repeat'],measured_nm=r['friction_nm'],load_nm=r['nominal_mean_load_nm']) for r in rows]))
    eligible=[x for x in candidates if x['training_adequate'] and x['transition_identifiable']]
    selected=min(eligible,key=lambda x:x['aicc']) if eligible else None
    return dict(selected=selected,candidates=candidates,validation_used_for_selection=False,
                load_feature='absolute unchanged nominal URDF gravity',
                static_breakaway_identified=False,passivity_proven=False)


def position_magnitude(speed, load, q, p):
    """Positive empirical position-dependent Stribeck amplitude, no output clipping.

    Fc, b, load coefficient, constant peak, periodic peak, vs, phase, order.
    This is an OMY extension, not a reproduction of Neuromeka firmware.
    """
    fc,b,lc,s0,sq,vs,phase,order=p
    return fc+b*np.abs(speed)+lc*np.abs(load)+(s0+sq*np.sin(order*q+phase)**2)*np.exp(-(np.abs(speed)/vs)**2)


def fit_position_candidates(rows, absolute, relative):
    v=np.array([r['speed'] for r in rows]);y=np.array([r['friction_nm'] for r in rows])
    load=np.array([r['nominal_mean_load_nm'] for r in rows])
    if len(rows)<24 or len({r['validation_pose'] for r in rows})<3:
        raise ValueError('Position extension requires three independently sampled pose regions')
    # Integrate the periodic term over the actually matched positions. Fitting
    # sin(mean(q)) instead would differ from the real-time function on a sweep.
    moments={n:(np.array([spatial_weights(r['position_rad'])@np.cos(2*n*np.array(r['position_rad'])) for r in rows]),
                np.array([spatial_weights(r['position_rad'])@np.sin(2*n*np.array(r['position_rad'])) for r in rows]))
             for n in (1,2,3)}
    candidates=[]
    for order,(cosine,sine) in moments.items():
        for free_phase in (False,True):
            def predict(z):
                fc,b,lc,s0,sq,vs=z[:6];phase=z[6] if free_phase else 0.
                harmonic=.5*(1-cosine*np.cos(2*phase)+sine*np.sin(2*phase))
                return fc+b*v+lc*load+(s0+sq*harmonic)*np.exp(-(v/vs)**2)
            lower=[0.,0.,0.,0.,0.,min(v)]+([-np.pi/2] if free_phase else [])
            upper=[np.inf]*5+[max(v)]+([np.pi/2] if free_phase else [])
            solutions=[]
            for vs in np.geomspace(min(v)*1.01,max(v)/1.01,4):
                for phase in ([-1.,0.,1.] if free_phase else [0.]):
                    z=[min(y)*.8,.1,.2,.01,np.ptp(y),vs]+([phase] if free_phase else [])
                    solutions.append(least_squares(lambda p:predict(p)-y,z,bounds=(lower,upper),max_nfev=3000))
            best=min(solutions,key=lambda f:f.fun@f.fun);z=best.x
            p=list(z[:6])+[float(z[6]) if free_phase else 0.,float(order)]
            residual=predict(z)-y;rss=float(residual@residual);n=len(y)
            k=len(z)+2 # variance and selection of harmonic order
            score=n*np.log(max(rss/n,np.finfo(float).tiny))+2*k+2*k*(k+1)/(n-k-1)
            interior=min(v)*(1+1e-4)<p[5]<max(v)*(1-1e-4)
            candidates.append(dict(name='position_stribeck',order=order,free_phase=free_phase,
                parameters=list(map(float,p)),aicc=float(score),training_adequate=bool(np.all(abs(residual)<=absolute+relative*y)),
                transition_identifiable=bool(interior),training_residuals_nm=residual.tolist(),
                training_rows=[dict(joint=r['joint'],speed=r['speed'],pose=r['validation_pose'],repeat=r['repeat'],
                    measured_nm=r['friction_nm'],load_nm=r['nominal_mean_load_nm']) for r in rows]))
    eligible=[x for x in candidates if x['training_adequate'] and x['transition_identifiable']]
    return dict(selected=min(eligible,key=lambda x:x['aicc']) if eligible else None,candidates=candidates,
        validation_used_for_selection=False,empirical_position_extension=True,
        unseen_pose_generalization_validated=False,static_breakaway_identified=False,passivity_proven=False)
