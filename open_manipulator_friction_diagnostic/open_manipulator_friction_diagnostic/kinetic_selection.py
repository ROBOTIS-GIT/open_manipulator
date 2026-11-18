"""Shared kinetic candidate selection for every joint; no rest-waveform fitting.

Legacy fits remain reproducible in common_friction. New identification compares
all declared candidates on the same held configurations and local residuals.
"""
from copy import deepcopy
import numpy as np
from . import common_friction as base
from . import periodic_friction as periodic
from .automatic_calibration import checked_cycles, content_hash
from .kinetic_review import review
from .kinetic_provenance import development_provenance, validation_independent


def configuration_folds(cycles, joint, max_folds=None):
    # Unlike window-based grouping, disjoint target-angle windows of the same
    # held configuration must not be used as independent posture evidence.
    groups = []
    for c in cycles:
        q = np.mean(np.delete(np.asarray(c['q']), joint, axis=1), axis=0)
        groups.append((c['cycle_id'], q))
    parent = list(range(len(groups)))
    def root(i):
        while parent[i] != i:
            i = parent[i]
        return i
    for i, (_, q) in enumerate(groups):
        for k, (_, p) in enumerate(groups[:i]):
            if max(abs(q-p)) < .01:
                parent[root(i)] = root(k)
    result = {}
    for i, (key, _) in enumerate(groups):
        result.setdefault(root(i), []).append(key)
    groups = list(result.values())
    if max_folds is not None:
        if type(max_folds) is not int or max_folds < 3:
            raise ValueError('Use at least three whole-configuration folds')
        if len(groups) > max_folds:
            balanced = [[] for _ in range(max_folds)]
            for group in sorted(groups, key=lambda g: (-len(g), tuple(g))):
                min(balanced, key=len).extend(group)
            groups = balanced
    return groups


def selection_folds(cycles, joint, max_folds=None):
    """Score only trainable, speed-supported whole-configuration holdouts.

    Sparse collection has one full speed anchor and representative speeds in
    other postures. Withholding that anchor can leave an untrainable curve or
    require unmeasured-speed extrapolation. Keep all cycles in the final fit,
    but report these folds as unassessed instead of discarding every candidate.
    """
    groups = configuration_folds(cycles, joint, max_folds)
    if len(groups) < 3:
        raise ValueError('Three independent other-joint configurations required for common kinetic selection')
    eligible, skipped = [], []
    for ids in groups:
        speeds = sorted({c['speed'] for c in cycles if c['cycle_id'] not in ids})
        held_speeds = sorted({c['speed'] for c in cycles if c['cycle_id'] in ids})
        reason = ('fewer_than_three_training_speeds' if len(speeds) < 3 else
                  'held_speed_outside_training_range' if
                  held_speeds[0] < speeds[0]-1e-12 or held_speeds[-1] > speeds[-1]+1e-12 else None)
        if reason:
            skipped.append(dict(held_cycle_ids=ids, reason=reason,
                                training_speeds_rad_s=speeds, held_speeds_rad_s=held_speeds))
        else:
            eligible.append(ids)
    if len(eligible) < 2:
        raise ValueError('Need two trainable whole-configuration holdouts within measured speed support')
    return eligible, dict(assessed_folds=len(eligible), total_folds=len(groups),
        all_configurations_assessed=not skipped, skipped_folds=skipped,
        all_measurements_used_in_final_fit=True, independent_validation=False)


def fit_candidate(cycles, joint, spec):
    if spec['family'] != 'positive_periodic_load':
        model=base.fit(cycles, joint, spec['family'], spec['smoothing'], spec['load_width_nm'],spatial=True)
        model.update(development_provenance(cycles))
        return model
    frequency = spec['frequency']; orders = spec['orders']
    load_width=spec.get('load_width_nm',2.)
    if not np.isfinite(load_width) or load_width<=0.:
        raise ValueError('Invalid load width')
    periodic.angular_basis([0.], frequency, orders)
    # Narrow historical observations may be retained, but each speed must also
    # have a resolved full-period sweep within this training fold.
    speeds=sorted({c['speed'] for c in cycles});resolved=set();design=[]
    cutoff=spec.get('angular_max_speed')
    if cutoff is not None and (not np.isfinite(cutoff) or cutoff<min(speeds)):
        raise ValueError('Invalid angular speed support')
    angular_speeds={v for v in speeds if cutoff is None or v<=cutoff}
    gs=np.concatenate([c['signed_gravity_ros_nm'] for c in cycles])
    load_dependence=np.ptp(gs)>=base.MIN_LOAD_SPAN_NM
    scale=max(float(max(abs(gs))),1.)
    macro=None
    if 'macro_family' in spec:
        if spec.get('macro_smoothing') is None:
            raise ValueError('An explicit macro family needs its smoothing recipe')
        family=spec['macro_family']
        # Refit ONLY the training fold; never pass a full-data macro into CV.
        macro=base.fit(cycles,joint,family,spec['macro_smoothing'],
            load_width if family in base.SMOOTH_FAMILIES else None,spatial=True)
        load_dependence=family!='velocity'
    for c in cycles:
        q = np.asarray(c['q'])[:, joint]
        width=np.ptp(q)/max(len(q)-1,1)
        support=c.get('raw_angular_support')
        # Unknown legacy provenance cannot establish angular resolution.
        # Such cycles may still contribute targets alongside verified sweeps.
        required_gap=2*np.pi/(6*frequency*max(orders))
        raw_resolved=bool(support) and all(
            d in support and np.isfinite(support[d]['max_gap_rad'])
            and 0<support[d]['max_gap_rad']<=required_gap
            and np.ptp(support[d]['angle_interval_rad'])>=2*np.pi/frequency
            for d in ('positive','negative'))
        if raw_resolved and np.ptp(q)>=2*np.pi/frequency and np.max(np.diff(np.sort(q)))<=required_gap:
            resolved.add(c['speed'])
        # Independent harmonic columns, rather than the intentionally redundant
        # positive 1 +/- sin/cos basis used for nonnegative prediction.
        h=np.column_stack([np.ones(len(q))]+[np.sinc(frequency*k*width/(2*np.pi))*fn(frequency*k*q)
            for k in orders for fn in (np.sin,np.cos)])
        load=(base.weights(macro['gravity_knots_nm'],c['signed_gravity_ros_nm'])
              if macro is not None and macro['family']=='signed_load' else
              base.soft_load_features(c['signed_gravity_ros_nm'],load_width,scale)
              if load_dependence else np.ones((len(q),1)))
        sw=np.tile(base.weights(speeds,[c['speed']]),(len(q),1))
        design.append(np.einsum('ij,ik,il->ijkl',load,sw,h).reshape(len(q),-1)/np.sqrt(len(q)))
    if not angular_speeds.issubset(resolved):
        raise ValueError('Periodic candidate needs a resolved full angular period at each speed')
    x=np.concatenate(design)
    if cutoff is not None:
        h=1+2*len(orders)
        active=[k for k in range(x.shape[1]) if k%h==0 or speeds[(k//h)%len(speeds)]<=cutoff]
        x=x[:,active]
    norm=np.linalg.norm(x,axis=0)
    if macro is not None and macro['family']=='signed_load':
        # An unmeasured load sign is unsupported, not an invented coefficient.
        x=x[:,norm>1e-12];norm=norm[norm>1e-12]
    normalized=x/np.maximum(norm,1e-12)
    rank=int(np.linalg.matrix_rank(normalized,tol=1e-7));condition=float(np.linalg.cond(normalized))
    if rank<x.shape[1] or condition>1000.:
        raise ValueError('Unobservable load-speed-angle design; regularization cannot supply missing data')
    model=periodic.fit(cycles, joint, frequency=frequency, orders=orders,
                        ridge=spec['ridge'], fit_load_dependence=bool(load_dependence),angular_max_speed=cutoff,
                        load_width_nm=load_width,macro_smoothing=spec.get('macro_smoothing'),macro_model=macro)
    model['observability']=dict(rank=rank,columns=x.shape[1],condition_number=condition,
                                resolved_speeds_rad_s=sorted(angular_speeds),
                                velocity_curve_speeds_rad_s=speeds)
    model.update(development_provenance(cycles))
    return model


def choose_viable(results):
    viable=[r for r in results if 'score' in r]
    if not viable:
        raise ValueError('No observable kinetic candidate')
    # Even a diagnostic fallback must explain the data used to fit it when
    # such a candidate exists. Extrapolating held-configuration folds can favor
    # an over-smoothed law that fails its full-data training means (J3).
    fitted=[r for r in viable if r.get('training_means_passed') is True]
    eligible=[r for r in fitted if all(f['means_passed'] for f in r['folds'])]
    pool=eligible or fitted or viable
    best=min(r['score'] for r in pool)
    families=(*base.FAMILIES,'positive_periodic_load')
    chosen=min((r for r in pool if r['score']<=best+base.SELECTION_TIE),
               key=lambda r:(families.index(r['family']),r['score']))
    return chosen,bool(eligible)


def select(cycles, joint, periodic_candidates=(), fraction=.95, previous_model=None, *, max_folds=None):
    base.check(cycles, joint)
    groups, coverage = selection_folds(cycles, joint, max_folds)
    specs = [dict(family=f, smoothing=s, load_width_nm=w) for f in base.FAMILIES
             for s in base.SMOOTHING for w in (base.LOAD_WIDTHS_NM if f in base.SMOOTH_FAMILIES else (None,))]
    for candidate in periodic_candidates:
        if not {'frequency','orders','ridge'}<=set(candidate) or set(candidate)-{'frequency','orders','ridge','angular_max_speed','load_width_nm','macro_smoothing','macro_family'}:
            raise ValueError('Declare frequency, orders, ridge and optional angular_max_speed/load_width_nm/macro_smoothing/macro_family')
        specs.append(dict(candidate, family='positive_periodic_load'))
    results = []
    for spec in specs:
        folds = []
        try:
            for ids in groups:
                train = [c for c in cycles if c['cycle_id'] not in ids]
                held = [c for c in cycles if c['cycle_id'] in ids]
                model = fit_candidate(train, joint, spec)
                folds.append(dict(training_cycle_ids=[c['cycle_id'] for c in train],
                    held_cycle_ids=ids, group='|'.join(sorted({c['group'] for c in held})),
                    **review(model, held, fraction)))
            # Check the actual full-data fit, not only the fold-specific fits.
            training=review(fit_candidate(cycles, joint, spec),cycles,fraction)
        except ValueError as exc:
            results.append(dict(spec, rejected=str(exc)))
            continue
        results.append(dict(spec, folds=folds,
            training_means_passed=training['means_passed'],
            training_spatial_passed=training['spatial_passed'],
            score=float(np.sqrt(np.mean([f['score']**2 for f in folds])))))
    chosen,mean_eligible=choose_viable(results)
    spec = {k:v for k,v in chosen.items()
            if k not in ('folds','score','training_means_passed','training_spatial_passed')}
    model = fit_candidate(cycles, joint, spec)
    model.update(development_provenance(cycles),kinetic_protocol='common_kinetic_v2')
    model['configuration_cv_coverage'] = coverage
    model['selection'] = dict(method='whole_configuration_spatial_development_cv',
        priority='training_means_then_all_configuration_means_then_local_error',
        all_configuration_mean_eligible=mean_eligible,
        score=chosen['score'], score_kind='equal_cycle_direction_local_normalized_rmse',
        folds=chosen['folds'], candidates=results, fresh_validation=False,
        periodic_rejections=[r for r in results if r['family']=='positive_periodic_load' and 'rejected' in r],
        simpler_family_score_margin=base.SELECTION_TIE)
    model['training_review'] = review(model, cycles, fraction)
    model['posture_review'] = dict(means_passed=all(f['means_passed'] for f in chosen['folds']),
        passed=all(f['spatial_passed'] for f in chosen['folds']),
        checks=sum(len(f['rows']) for f in chosen['folds']))
    model['missing'] = []
    if not model['posture_review']['means_passed']:
        model['missing'].append('unseen_configuration_mean_prediction')
    if not model['posture_review']['passed'] or not model['training_review']['spatial_passed']:
        model['missing'].append('spatial_prediction')
    model['missing'] += ['fresh_validation', 'kinetic_release_coverage']
    if not coverage['all_configurations_assessed']:
        model['missing'].append('incomplete_configuration_cv')
    model['rest_identification_included'] = False
    model['automatic_deployment_allowed'] = False
    from .kinetic_support import measured_support, preserve_support
    model['measurement_support'] = measured_support(cycles, joint)
    if previous_model is not None:
        model['replacement_support_review'] = preserve_support(previous_model, model)
    return model


def fit_bank(cycles, identity, joints, periodic_candidates=(), fraction=.95, previous_bank=None):
    if not joints or len(set(joints)) != len(joints) or any(j not in range(6) for j in joints):
        raise ValueError('Invalid joint selection')
    if set(identity) != {'robot_unit', 'robot_fingerprint', 'baseline_sha256'} or not all(identity.values()):
        raise ValueError('Robot and unchanged baseline identity required')
    if isinstance(periodic_candidates,dict):
        if (not set(periodic_candidates)<= {f'joint{k}' for k in range(1,7)} or
                any(not isinstance(v,(list,tuple)) for v in periodic_candidates.values())):
            raise ValueError('Periodic recipes must map joint1..joint6 to candidate lists')
    if previous_bank is not None and (
            previous_bank.get('schema')!='common_friction_bank_v1' or previous_bank.get('identity')!=identity
            or not {f'joint{j+1}' for j in joints}<=set(previous_bank.get('models',{}))):
        raise ValueError('Previous bank identity or selected joints differ')
    models = {}
    for j in joints:
        prior=previous_bank['models'][f'joint{j+1}'] if previous_bank is not None else None
        gain = fraction[f'joint{j+1}'] if isinstance(fraction, dict) else fraction
        if type(gain) not in (int,float) or not np.isfinite(gain) or not 0 < gain <= 1:
            raise ValueError('Invalid kinetic fraction')
        recipes=(periodic_candidates.get(f'joint{j+1}',()) if isinstance(periodic_candidates,dict)
                 else periodic_candidates)
        m = select([c for c in cycles if c['joint']==j], j, recipes, gain,prior)
        m['identity'] = deepcopy(identity)
        models[f'joint{j+1}'] = m
    return dict(schema='common_friction_bank_v1', kinetic_protocol='common_kinetic_v2',
        identity=deepcopy(identity), models=models, gravity_assumption='existing_model_trusted',
        temperature_model=False, hardware_qualified=False, automatic_deployment_allowed=False,
        rest_identification_included=False)


def validate(model, cycles, evidence, mock=False, fraction=.95):
    if (evidence.get('candidate_sha256') != content_hash(model)
            or evidence.get('identity') != model.get('identity')
            or evidence.get('role') != 'fresh_final_validation'
            or evidence.get('source') not in (('hardware','synthetic') if mock else ('hardware',))
            or evidence.get('completed') is not True or evidence.get('contact_free') is not True
            or evidence.get('cycle_ids') != [c['cycle_id'] for c in cycles]):
        raise ValueError('Fresh evidence must bind this model, robot and baseline')
    checked_cycles(cycles)
    independent=validation_independent(model,cycles)
    result = review(model, cycles, fraction)
    outside = [c['cycle_id'] for c in cycles if not (
        model['speed_range_rad_s'][0] <= c['speed'] <= model['speed_range_rad_s'][1]
        and min(c['signed_gravity_ros_nm']) >= model['gravity_range_nm'][0]-1e-6
        and max(c['signed_gravity_ros_nm']) <= model['gravity_range_nm'][1]+1e-6)]
    result.update(support=dict(within_development_bounds=not outside, outside=outside),
        passed=result['spatial_passed'], independent=independent,
        independence_evidence='frozen_development_payload_hashes' if independent else 'missing_development_provenance',
        stage='STEADY_VALIDATED' if independent and result['spatial_passed'] and not outside
            and model.get('posture_review',{}).get('passed') else 'NEEDS_REVIEW',
        static_breakaway_identified=False)
    if model.get('measurement_support'):
        from .kinetic_support import speed_load_coverage
        result['support']['speed_load_combinations']=[dict(cycle_id=c['cycle_id'],
            **speed_load_coverage(model['measurement_support'],
                [c['speed']]*len(c['signed_gravity_ros_nm']),c['signed_gravity_ros_nm'])) for c in cycles]
        result['support']['within_development_bounds_is_marginal_only']=True
    return result
