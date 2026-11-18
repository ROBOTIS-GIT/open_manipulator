"""Joint-independent steady friction identification against a trusted gravity model.

The signed-load law is affine in load on each side of zero (four power
quadrants), with piecewise-linear speed dependence. Its three load coordinates
are physical basis coordinates, never posture IDs. Position, date and measured
current are not runtime inputs. Rest/reversal dynamics remain a separate stage.
"""
from copy import deepcopy

import numpy as np
from scipy.optimize import nnls

from .automatic_calibration import checked_cycles, content_hash, payload_hash
from .single_joint_model import evaluate, parameters, table_values


SCHEMA = 'common_friction_v1'
FAMILIES = ('velocity', 'shared_smooth_load', 'signed_load', 'smooth_load')
SMOOTH_FAMILIES = ('shared_smooth_load', 'smooth_load')
SMOOTHING = (0., .01, .1)
LOAD_WIDTHS_NM = (.25, .5, 1., 1.5, 2., 3., 4., 6., 8.)
ABSOLUTE_NM = .08
RELATIVE = .20
MIN_LOAD_SPAN_NM = .25
SELECTION_TIE = .01  # one percent of the predeclared prediction allowance


def check(cycles, joint):
    checked_cycles(cycles)
    if joint not in range(6) or any(c['joint'] != joint for c in cycles):
        raise ValueError('Exactly one joint per model; use fit_bank for multiple joints')
    if len({payload_hash(c) for c in cycles}) != len(cycles):
        raise ValueError('Duplicate cycle payload')
    if len({c['speed'] for c in cycles}) < 3:
        raise ValueError('At least three measured speeds required')
    if min(np.mean(c[d+'_resistance_proxy_nm']) for c in cycles
           for d in ('positive', 'negative')) <= 0:
        raise ValueError('Nonpositive mean friction residual under the trusted gravity assumption')


def weights(knots, values):
    values = np.asarray(values)
    return np.column_stack([np.interp(values, knots, row) for row in np.eye(len(knots))])


def soft_load_features(gravity, width, scale):
    g = np.asarray(gravity)
    r = np.hypot(g, width)
    return np.column_stack([np.ones(g.size), (r+g)/(2*scale), (r-g)/(2*scale)])


def fit(cycles, joint, family='signed_load', smoothing=.01, load_width_nm=None, *, spatial=False,
        objective='absolute'):
    check(cycles, joint)
    if objective not in ('absolute', 'normalized_cycle_mean'):
        raise ValueError('Unknown fitting objective')
    if family not in FAMILIES or smoothing not in SMOOTHING:
        raise ValueError('Unknown predeclared model candidate')
    if ((family in SMOOTH_FAMILIES and load_width_nm not in LOAD_WIDTHS_NM)
            or (family not in SMOOTH_FAMILIES and load_width_nm is not None)):
        raise ValueError('Invalid smooth-load width')
    speeds = sorted({c['speed'] for c in cycles})
    if len(speeds) > 32:
        raise ValueError('At most 32 measured speed coordinates supported; no knots were silently removed')
    gravity = np.concatenate([c['signed_gravity_ros_nm'] for c in cycles])
    group_loads = [np.mean([np.mean(c['signed_gravity_ros_nm']) for c in cycles if c['group'] == g])
                   for g in sorted({c['group'] for c in cycles})]
    observed_loads = gravity if spatial else group_loads
    if family != 'velocity' and np.ptp(observed_loads) < MIN_LOAD_SPAN_NM:
        raise ValueError('Unobservable load variation; compare the velocity family')
    # Each sign uses its observed extent. Requiring positivity at an artificial
    # +G copied from a much larger negative load would bias the positive branch.
    load_knots = np.array([min(float(gravity.min()), -1.), 0.,
                           max(float(gravity.max()), 1.)])
    feature_scale = max(float(np.max(abs(gravity))), 1.)
    design = []
    for c in cycles:
        sw = weights(speeds, [c['speed']])[0]
        lw = (weights(load_knots, c['signed_gravity_ros_nm']) if family == 'signed_load'
              else soft_load_features(c['signed_gravity_ros_nm'],load_width_nm,feature_scale)
              if family in SMOOTH_FAMILIES else np.ones((len(c['q']),1)))
        if spatial:
            # Retain the measured load changes inside a sweep. Total squared
            # residual weight is one per cycle, regardless of its bin count.
            # Entire configurations are still held out together by the selector.
            rows = (np.column_stack([np.tile(sw,(len(lw),1)),lw[:,1:]])
                    if family == 'shared_smooth_load'
                    else np.einsum('ij,k->ijk',lw,sw).reshape(len(lw),-1))
            design.append(rows/np.sqrt(len(rows)))
        else:
            # Legacy cycle-mean fitting remains reproducible for old archives.
            lw=lw.mean(axis=0)
            design.append(np.r_[sw,lw[1:]] if family == 'shared_smooth_load'
                          else np.outer(lw,sw).ravel())
    x = np.concatenate(design) if spatial else np.asarray(design)
    active = np.max(abs(x), axis=0) > 1e-10
    if family == 'signed_load' and not active.reshape(3, len(speeds))[1].all():
        raise ValueError('Unobservable zero-load curve; endpoint-only data cannot define the intercept')
    a = x[:, active]
    norm = np.sqrt(np.sum(a*a,axis=0)/len(cycles)) if spatial else np.sqrt(np.mean(a*a, axis=0))
    normalized = a/norm
    rank = int(np.linalg.matrix_rank(normalized, tol=1e-7))
    condition = float(np.linalg.cond(normalized))
    if rank < a.shape[1] or condition > 1000.:
        raise ValueError('Unobservable load/speed design; regularization cannot supply missing data')
    nload = 1 if family == 'velocity' else 3
    smooth = np.kron(np.eye(nload), np.diff(np.eye(len(speeds)), n=2, axis=0))
    if family == 'shared_smooth_load':
        smooth = np.pad(np.diff(np.eye(len(speeds)), n=2, axis=0), ((0, 0), (0, 2)))
    lhs = np.vstack([normalized/np.sqrt(len(cycles)), np.sqrt(smoothing)*smooth[:, active]/norm])
    tables, soft_coefficients = {}, {}
    for d in ('positive', 'negative'):
        y = (np.concatenate([np.asarray(c[d+'_resistance_proxy_nm'])/np.sqrt(len(c['q'])) for c in cycles])
             if spatial else np.array([np.mean(c[d+'_resistance_proxy_nm']) for c in cycles]))
        rhs = np.r_[y/np.sqrt(len(cycles)), np.zeros(smooth.shape[0])]
        coef = np.zeros(x.shape[1])
        if objective == 'normalized_cycle_mean':
            # Equal relative-cycle error, not a sensor-noise estimate. The
            # original absolute-Nm objective remains bitwise reproducible.
            cw=np.array([1/(ABSOLUTE_NM+RELATIVE*np.mean(c[d+'_resistance_proxy_nm']))
                         for c in cycles])
            ow=np.concatenate([np.full(len(c['q']),w) for c,w in zip(cycles,cw)]) if spatial else cw
            wl,wr=lhs.copy(),rhs.copy()
            wl[:len(x)]*=ow[:,None];wr[:len(x)]*=ow
            coef[active]=nnls(wl,wr,maxiter=10000)[0]/norm
        else:
            coef[active] = nnls(lhs, rhs, maxiter=10000)[0]/norm
        rows = (np.vstack([coef[:len(speeds)], np.full(len(speeds), coef[-2]),
                           np.full(len(speeds), coef[-1])]) if family == 'shared_smooth_load'
                else coef.reshape(nload, len(speeds)))
        if family == 'velocity':
            rows = np.tile(rows, (3, 1))
        elif family == 'signed_load':
            # An entirely unobserved load sign is explicitly unsupported. It
            # inherits the zero-load curve for numerical export, not evidence.
            mask = active.reshape(3, len(speeds))
            for k in (0, 2):
                if not mask[k].any():
                    rows[k] = rows[1]
        if family in SMOOTH_FAMILIES:
            soft_coefficients[d] = rows
        elif (np.min(rows) < 0 or np.min(rows[:, 0]) <= 0
              or (family != 'signed_load' and np.min(rows) <= 0)):
            raise ValueError('Candidate needs nonnegative kinetic loss and positive rest yield; do not clip coefficients')
        # NNLS can reach zero at a load/speed corner absent from sparse data.
        # This is a dissipative boundary, not a negative loss. The entire first
        # speed column stays strictly positive for the unchanged rest-state law.
        tables[d] = rows.tolist()
    export_error = 0.
    if family in SMOOTH_FAMILIES:
        # Three values are retained for reporting; runtime evaluates the
        # analytic law itself, including when load is outside observed bounds.
        tables = {d:(soft_load_features(load_knots, load_width_nm, feature_scale)@c).tolist()
                  for d,c in soft_coefficients.items()}
        if min(np.min(t) for t in tables.values()) <= 0:
            raise ValueError('Smooth candidate reaches nonpositive friction')
    from .kinetic_provenance import development_provenance
    result = dict(schema=SCHEMA, joint=joint+1, joints=[joint+1], family=family, smoothing=smoothing,
                load_width_nm=load_width_nm, load_feature_scale_nm=feature_scale,
                soft_load_coefficients={d:c.tolist() for d,c in soft_coefficients.items()},
                export_maximum_interpolation_error_nm=export_error,
                gravity_knots_nm=load_knots.tolist(), speed_knots_rad_s=speeds,
                tables=tables, gravity_range_nm=[float(gravity.min()), float(gravity.max())],
                speed_range_rad_s=[speeds[0], speeds[-1]],
                trusted_existing_gravity=True, gravity_or_current_conversion_changed=False,
                unregularized_rank=rank, active_columns=int(sum(active)), condition_number=condition,
                load_sign_observed=dict(positive=bool(max(observed_loads) > MIN_LOAD_SPAN_NM),
                                        negative=bool(min(observed_loads) < -MIN_LOAD_SPAN_NM)),
                fit_observations='spatial_equal_cycle' if spatial else 'cycle_means',
                independent_cycle_count=len(cycles),
                **development_provenance(cycles),
                hardware_qualified=False, static_breakaway_identified=False)
    if objective != 'absolute':result['fit_objective']=objective
    return result


def runtime_model(model):
    """Exact adapter to the existing C++ law, not a fitted posture lookup."""
    if model['schema'] != SCHEMA:
        raise ValueError('Unknown common model')
    result = deepcopy(model)
    result.update(schema='single_joint_signed_load_v3', common_schema=SCHEMA,
                  load_clusters=[], per_load_speed_model='shared_physical_load_basis',
                  speed_extension='constant_endpoint', load_extension='outward_nonnegative_slope')
    if model['family'] in SMOOTH_FAMILIES:
        result.update(load_extension='analytic_smooth_load_unqualified_outside_observed_range',
                      runtime_evaluation='analytic_smooth_load')
    return result


def review(model, cycles, independent=False):
    result = evaluate(runtime_model(model), cycles, independent)
    paired_rows = []
    for c in cycles:
        plus = table_values(model, c['speed'], c['signed_gravity_ros_nm'], 'positive')
        minus = table_values(model, c['speed'], c['signed_gravity_ros_nm'], 'negative')
        s, b = (plus+minus)/2, (plus-minus)/2
        target = np.asarray(c['target'])
        bias = (np.asarray(c['positive_resistance_proxy_nm'])-c['negative_resistance_proxy_nm'])/2
        paired_rows.append(dict(cycle_id=c['cycle_id'], pair_mean_error_nm=float(np.mean(s-target)),
                                direction_bias_mean_error_nm=float(np.mean(b-bias))))
    result['paired_diagnostics'] = paired_rows
    result['mean_failures'] = sum(not r['mean_passed'] for r in result['rows'])
    result['mean_mae_nm'] = float(np.mean([abs(r['predicted_mean_nm']-r['measured_mean_nm']) for r in result['rows']]))
    result['spatial_failures'] = sum(not r['passed'] for r in result['rows'])
    # A bounding range is only a necessary support check, not a certificate
    # that every posture inside that range has been independently validated.
    outside = []
    lo, hi = model['gravity_range_nm']
    vlo, vhi = model['speed_range_rad_s']
    for c in cycles:
        g = np.asarray(c['signed_gravity_ros_nm'])
        reasons = []
        if c['speed'] < vlo-1e-9 or c['speed'] > vhi+1e-9:
            reasons.append('unmeasured_speed')
        if g.min() < lo-1e-6 or g.max() > hi+1e-6:
            reasons.append('unmeasured_load_range')
        for direction, sign in (('positive', 1), ('negative', -1)):
            if (not model['load_sign_observed'][direction] and
                    np.max(sign*g) > MIN_LOAD_SPAN_NM):
                reasons.append('unobserved_load_sign_'+direction)
        if reasons:
            outside.append(dict(cycle_id=c['cycle_id'], reasons=reasons))
    result['support'] = dict(within_development_bounds=not outside, outside=outside,
                            bounds_do_not_prove_posture_generalization=True)
    return result


def posture_folds(cycles):
    """Keep renamed/revisited/overlapping physical postures in the same fold.

    Recorded sweeps of the target joint may differ in width. Overlap with the
    same other-joint configuration is not independent posture generalization.
    Original cycle and group labels are retained for provenance/reporting.
    """
    groups = sorted({c['group'] for c in cycles})
    parent = {g:g for g in groups}
    def root(g):
        while parent[g] != g:
            g = parent[g]
        return g
    supports = []
    for c in cycles:
        q = np.asarray(c['q']); j = c['joint']
        supports.append((c['group'],j,q.mean(axis=0),q[:,j].min(),q[:,j].max()))
    for i,(a,j,qa,lo,hi) in enumerate(supports):
        for b,k,qb,left,right in supports[:i]:
            if (a!=b and k==j and np.max(abs(np.delete(qa-qb,j)))<.01
                    and max(lo,left) <= min(hi,right)+.01):
                parent[root(a)] = root(b)
    folded = {}
    for g in groups:
        folded.setdefault(root(g),[]).append(g)
    return sorted(folded.values())


def select(cycles, joint):
    """Whole-posture development CV; never call it fresh final validation."""
    check(cycles, joint)
    groups = posture_folds(cycles)
    if len(groups) < 3:
        raise ValueError('Three independent posture groups required for model comparison')
    candidates = []
    for family in FAMILIES:
        for smooth, width in ((s,w) for s in SMOOTHING for w in
                              (LOAD_WIDTHS_NM if family in SMOOTH_FAMILIES else (None,))):
            folds, rejected = [], None
            for members in groups:
                group = '|'.join(members)
                train = [c for c in cycles if c['group'] not in members]
                held = [c for c in cycles if c['group'] in members]
                try:
                    model = fit(train, joint, family, smooth, width)
                    folds.append(dict(group=group, posture_groups=members, training_cycle_ids=[c['cycle_id'] for c in train],
                                      **review(model, held, independent=True)))
                except ValueError as exc:
                    rejected = str(exc)
                    break
            if rejected:
                candidates.append(dict(family=family, smoothing=smooth, load_width_nm=width, rejected=rejected))
            else:
                normalized = [np.mean([((r['predicted_mean_nm']-r['measured_mean_nm'])/
                              (ABSOLUTE_NM+RELATIVE*max(0., r['measured_mean_nm'])))**2 for r in f['rows']])
                              for f in folds]
                candidates.append(dict(family=family, smoothing=smooth, load_width_nm=width, folds=folds,
                                       score=float(np.sqrt(np.mean(normalized)))))
    viable = [c for c in candidates if 'score' in c]
    if not viable:
        raise ValueError('No observable common candidate: '+str(candidates))
    # Do not invent load sensitivity from numerical noise or a negligible CV
    # gain. Prefer the simpler physical law within the declared score margin.
    best = min(c['score'] for c in viable)
    equivalent = [c for c in viable if c['score'] <= best+SELECTION_TIE]
    chosen = min(equivalent, key=lambda c: (FAMILIES.index(c['family']), c['score'], c['smoothing']))
    model = fit(cycles, joint, chosen['family'], chosen['smoothing'], chosen['load_width_nm'])
    model['selection'] = dict(score=chosen['score'], candidates=candidates,
                              fresh_validation=False, method='whole_posture_development_cv',
                              physical_posture_folds=groups,
                              simpler_family_score_margin=SELECTION_TIE)
    model['training_review'] = review(model, cycles)
    model['posture_review'] = dict(means_passed=all(f['means_passed'] for f in chosen['folds']),
                                   passed=all(f['passed'] for f in chosen['folds']),
                                   mean_failures=sum(f['mean_failures'] for f in chosen['folds']),
                                   checks=sum(len(f['rows']) for f in chosen['folds']))
    model['missing'] = []
    if not model['posture_review']['means_passed']:
        model['missing'].append('unseen_posture_mean_prediction')
    if not model['training_review']['passed'] or not model['posture_review']['passed']:
        model['missing'].append('spatial_prediction')
    model['missing'] += ['fresh_validation', 'joint_specific_transition_validation']
    model['automatic_deployment_allowed'] = False
    return model


def fit_bank(cycles, identity, joints=(1,)):
    """Zero-based selected joints. Default hardware work remains J2 only."""
    if not joints or len(set(joints)) != len(joints) or any(j not in range(6) for j in joints):
        raise ValueError('Invalid joint selection')
    if set(identity) != {'robot_unit', 'robot_fingerprint', 'baseline_sha256'} or not all(identity.values()):
        raise ValueError('Robot and unchanged baseline identity required')
    models = {}
    for j in joints:
        model = select([c for c in cycles if c['joint'] == j], j)
        model['identity'] = deepcopy(identity)
        models[f'joint{j+1}'] = model
    return dict(schema='common_friction_bank_v1', identity=deepcopy(identity), models=models,
                gravity_assumption='existing_model_trusted', temperature_model=False,
                hardware_qualified=False, automatic_deployment_allowed=False)


def validate(model, cycles, evidence, mock=False):
    if (evidence.get('candidate_sha256') != content_hash(model)
            or evidence.get('identity') != model.get('identity')
            or evidence.get('role') != 'fresh_final_validation'
            or evidence.get('source') not in (('hardware', 'synthetic') if mock else ('hardware',))
            or evidence.get('completed') is not True or evidence.get('contact_free') is not True
            or evidence.get('cycle_ids') != [c['cycle_id'] for c in cycles]):
        raise ValueError('Fresh evidence must bind this model, robot and baseline')
    result = review(model, cycles, independent=True)
    result.update(stage='STEADY_VALIDATED' if result['passed'] and model['posture_review']['passed']
                  and result['support']['within_development_bounds']
                  else 'NEEDS_REVIEW', hardware_deployable=False, static_breakaway_identified=False)
    return result


def trial_parameters(model, baseline, transition, mock=False, rest=None):
    """One-joint runtime adapter; transition evidence is not inferred from steady data."""
    if transition.get('model_sha256') != content_hash(model) or transition.get('joint') != model['joint']:
        raise ValueError('Transition design must bind the exact model and joint')
    if not mock and transition.get('numerical_release_validated') is not True:
        raise ValueError('Joint-specific transition validation required before a supervised hand test')
    values = [transition[k] for k in ('stiffness_nm_rad', 'decay_per_s', 'output_fraction')]
    if not np.isfinite(values).all() or values[0] <= 0 or values[1] < 0 or not 0 < values[2] <= 1:
        raise ValueError('Invalid explicit controller design')
    p = parameters(runtime_model(model), baseline, values[0], values[2])
    p['stateful_decay'] = [values[1] if k == model['joint']-1 else 0. for k in range(6)]
    distance=transition.get('load_transition_distance_rad',0.)
    if not np.isfinite(distance) or distance<0.:
        raise ValueError('Invalid load-dependent transition distance')
    p['stateful_load_transition_distance']=distance
    if rest is not None:
        # Rest assistance is an independently measured controller component;
        # never derive its amplitude from the steady friction curve.
        chosen=rest.get('selected')
        if (rest.get('joint')!=model['joint'] or not model.get('identity') or
                rest.get('identity')!=model['identity'] or
                rest.get('source') not in (('hardware','synthetic') if mock else ('hardware',)) or
                transition.get('rest_response_sha256')!=content_hash(rest) or
                not chosen or not chosen.get('passed') or not chosen.get('coverage')):
            raise ValueError('Matching independently selected rest response required')
        amplitude,frequency=chosen['amplitude_nm'],chosen['frequency_hz']
        if not np.isfinite([amplitude,frequency]).all() or amplitude<=0. or not 0.<frequency<=100.:
            raise ValueError('Invalid measured rest response waveform')
        if not mock and transition.get('combined_release_validated') is not True:
            raise ValueError('Exact kinetic-plus-dither combined validation required')
        from .dither_bundle import DESIGN
        p.update(stateful_dither_with_elastic=True,stateful_dither_j6_only=False,
            stateful_dither_amplitude_nm=amplitude,stateful_dither_frequency_hz=frequency,
            stateful_dither_full_velocity=DESIGN['full_velocity'],
            stateful_dither_zero_velocity=DESIGN['zero_velocity'],
            stateful_dither_ramp_seconds=DESIGN['ramp_seconds'])
    return p
