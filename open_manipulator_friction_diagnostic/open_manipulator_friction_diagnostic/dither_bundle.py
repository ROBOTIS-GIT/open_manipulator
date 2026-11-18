"""J6 dither experiments, optionally with a sourced elastic/Coulomb component."""
import copy
import json
from pathlib import Path
from .coasting_bundle import sha


DESIGN = dict(amplitude_nm=.56, frequency_hz=10., full_velocity=.003,
              zero_velocity=.02, ramp_seconds=.5)


def frequency_design(frequency_hz):
    if frequency_hz not in (5., 10.):
        raise ValueError('Unreviewed dither frequency')
    return dict(DESIGN, frequency_hz=float(frequency_hz))


def runtime_hashes():
    root = Path(__file__).resolve().parents[2]
    names = ['open_manipulator_friction_diagnostic/include/friction_compensation_controller/'+n
             for n in ('rest_dither.hpp', 'stateful_friction.hpp', 'elastoplastic_replay.hpp',
                       'position_velocity_observer.hpp', 'calibrated_friction.hpp', 'kinetic_curve.hpp')]
    names.append('open_manipulator_friction_diagnostic/open_manipulator_friction_diagnostic/dither_bundle.py')
    names.extend('open_manipulator_friction_diagnostic/open_manipulator_friction_diagnostic/'+name
                 for name in ('automatic_calibration.py','rest_response.py','transition_design.py'))
    names.append('open_manipulator_friction_diagnostic/tools/stress_j6_curve.cpp')
    return {n: sha(root/n) for n in names}


def rest_evidence_digest(selection, records, conditions):
    """Bind exact inputs and selected design, not platform-dependent LAPACK bits.

    All acceptance tests are recomputed on the target. Runtime hashes bind their
    implementation; raw-file hashes bind acquisition records byte for byte.
    """
    from .automatic_calibration import content_hash
    chosen = selection.get('selected')
    if not chosen or not chosen['passed'] or not chosen['coverage']:
        raise ValueError('No repeatably supported rest design')
    return content_hash(dict(records=records, conditions=conditions,
        identity=selection['identity'], source=selection['source'],
        selected={key: chosen[key] for key in ('amplitude_nm', 'frequency_hz')}))


def load_dither_bundle(folder, urdf, baseline_path):
    from .integration import fingerprint, original_gravity
    meta = json.loads((Path(folder)/'candidate.json').read_text())
    design = frequency_design(meta.get('design', {}).get('frequency_hz'))
    automatic = meta.get('automatic_rest_response')
    if automatic is not None:
        from .rest_response import extract
        from .automatic_calibration import select_rest
        if (automatic.get('conditions') != ['matched_j6_load0','matched_j6_load1'] or
                automatic.get('runs') != ['rest_ready_confirm','rest_vertical_confirm']):
            raise ValueError('Unexpected automatic rest evidence scope')
        traces=[]; records={}
        for name in automatic['runs']:
            values, review=extract(Path(folder)/'rest_evidence'/name)
            traces.extend(values)
            records[name]=review['source_files_sha256']
        selection=select_rest(traces,.000011984224905356572,automatic['conditions'])
        expected_identity=dict(robot_unit=meta['robot_unit'],robot_fingerprint=fingerprint(urdf),
                               baseline_sha256=sha(baseline_path))
        if (selection['source']!='hardware' or selection['identity']!=expected_identity or
                selection['selected'] is None or rest_evidence_digest(
                    selection, records, automatic['conditions'])!=automatic.get('evidence_sha256')):
            raise ValueError('Rest response is not repeatably supported by these hardware records')
        chosen=selection['selected']
        design=dict(DESIGN,amplitude_nm=chosen['amplitude_nm'],frequency_hz=chosen['frequency_hz'])
    combined = meta.get('elastic_coulomb_enabled', False)
    moving_endpoint = meta.get('moving_curve_endpoint_rad_s')
    if moving_endpoint is not None and (moving_endpoint != .04 or not combined):
        raise ValueError('Unreviewed moving-friction endpoint')
    if (meta.get('schema') != 'supervised_j6_rest_dither_v1' or
            meta.get('hardware_qualified') is not False or meta.get('joint') != 'joint6' or
            meta.get('robot_fingerprint') != fingerprint(urdf) or
            meta.get('baseline_sha256') != sha(baseline_path) or
            meta.get('runtime_hashes') != runtime_hashes() or
            meta.get('design') != design or not isinstance(combined, bool) or
            meta.get('kinetic_compensation_enabled') is not combined):
        raise ValueError('J6 dither experiment identity/source/design mismatch')
    params = copy.deepcopy(original_gravity(baseline_path))
    if params['joints'] != [f'joint{i}' for i in range(1, 7)]:
        raise ValueError('Expected original six-axis gravity configuration')
    # Positive state parameters keep the normal runtime validation intact but
    # are NOT used for a spring, rate, or damping command in this isolated mode.
    params.update(calibrated_friction_enabled=False, stateful_friction_enabled=True,
        stateful_joint_mask=[0, 0, 0, 0, 0, 1], stateful_observer_window=.02,
        stateful_positive_yield=[0.]*5+[1.], stateful_negative_yield=[0.]*5+[1.],
        stateful_positive_viscous=[0.]*6, stateful_negative_viscous=[0.]*6,
        stateful_stiffness=[0.]*5+[1500.], stateful_decay=[0.]*5+[5.],
        stateful_damping=[0.]*6, stateful_maximum_velocity=[0.]*6,
        stateful_curve_j6_enabled=False, stateful_dither_j6_only=not combined,
        stateful_dither_with_elastic=combined,
        stateful_dither_amplitude_nm=design['amplitude_nm'],
        stateful_dither_frequency_hz=design['frequency_hz'],
        stateful_dither_full_velocity=design['full_velocity'],
        stateful_dither_zero_velocity=design['zero_velocity'],
        stateful_dither_ramp_seconds=design['ramp_seconds'], friction_diagnostics_rate=50.)
    if combined:
        stress_path = Path(folder)/'stress.json'
        if meta.get('stress_sha256') != sha(stress_path):
            raise ValueError('Missing matching combined stress evidence')
        stress = json.loads(stress_path.read_text())
        if automatic is not None and stress.get('dither_design') != {
                'amplitude_nm':design['amplitude_nm'],'frequency_hz':design['frequency_hz']}:
            raise ValueError('Stress tested different automatically selected dither settings')
        if stress.get('moving_curve_endpoint_rad_s') != moving_endpoint:
            raise ValueError('Stress tested a different moving-friction mode')
        results = stress.get('results', [])
        low_speed = meta.get('transition_design') is not None
        if (stress.get('synthetic_only') is not True or stress.get('dither_elastic') is not True or
                len(results) != 1 or results[0].get('scale') != .95 or results[0].get('cases') != (2016 if low_speed else 864) or
                any(results[0].get(k) != 0 for k in ('stationary_self_start', 'restart_after_hand_stop',
                    'speed_growth_after_release', 'numerical_escape'))):
            raise ValueError('Combined software stress did not pass')
        expected = dict(file='curve_review.json', scale=.95, stiffness=1500., decay=5.)
        evidence = meta.get('elastic_source', {})
        if evidence.get('sha256') != sha(Path(folder)/'curve_review.json'):
            raise ValueError('Missing elastic/Coulomb source evidence')
        review = json.loads((Path(folder)/'curve_review.json').read_text())
        if review.get('schema') != 'j6_coasting_curve_review_v1':
            raise ValueError('Unexpected directional friction source')
        if low_speed:
            from .transition_design import lowest_speed_design
            curve = review['curve']
            design = lowest_speed_design(curve['positive']['magnitude_nm'][0],
                curve['negative']['magnitude_nm'][0], curve['positive']['speed_rad_s'][0], 5.)
            stress_source = Path(__file__).resolve().parents[2]/'open_manipulator_friction_diagnostic/tools/stress_j6_low_speed.cpp'
            if (moving_endpoint != .04 or meta['transition_design'] != design or
                    meta.get('transition_stress_source_sha256') != sha(stress_source) or
                    stress.get('transition_design') != dict(stiffness_nm_rad=design['stiffness_nm_rad'], decay_per_s=5.) or
                    stress.get('low_speed_cases_included') is not True or
                    stress.get('encoder_dither_noise_counts') != 2):
                raise ValueError('Low-speed transition design or stress does not match the measured curve')
            expected['stiffness'] = design['stiffness_nm_rad']
            params['stateful_stiffness'][-1] = design['stiffness_nm_rad']
        if any(evidence.get(k) != v for k,v in expected.items()):
            raise ValueError('Missing elastic/Coulomb source evidence')
        for direction in ('positive', 'negative'):
            value = review['curve'][direction]['magnitude_nm'][0]
            # The source curve is preserved. Only its baseline level is used;
            # no interpolated or extrapolated velocity-dependent rate is added.
            params['stateful_'+direction+'_yield'][-1] = value
        params['stateful_curve_output_scale'] = evidence['scale']
        if moving_endpoint is not None:
            speeds = review['curve']['positive']['speed_rad_s']
            if speeds != review['curve']['negative']['speed_rad_s'] or moving_endpoint not in speeds:
                raise ValueError('Moving-friction endpoint must be a measured shared speed knot')
            n = speeds.index(moving_endpoint)+1
            selected = dict(speed_rad_s=speeds[:n],
                positive_nm=review['curve']['positive']['magnitude_nm'][:n],
                negative_nm=review['curve']['negative']['magnitude_nm'][:n])
            if stress.get('tested_moving_curve') != selected:
                raise ValueError('Stress tested different directional friction knots')
            params.update(stateful_curve_j6_enabled=True, stateful_curve_hold_endpoint=True,
                stateful_curve_speed=selected['speed_rad_s'],
                stateful_curve_positive=selected['positive_nm'],
                stateful_curve_negative=selected['negative_nm'])
    return params
