"""Compose independently verified single-axis laws without changing their output.

This is wiring, not identification or a new deployment qualification. Rest
waveforms remain independent inputs. Original gravity parameters must agree.
"""
from copy import deepcopy
import re
import numpy as np

ARRAYS=('positive_yield','negative_yield','positive_viscous','negative_viscous',
        'stiffness','decay','damping','maximum_velocity')

_CONTROLLER='open_manipulator_friction_diagnostic/'
CORE_SOURCES=tuple(_CONTROLLER+name for name in (
    'src/friction_compensation_controller.cpp',
    'src/friction_compensation_controller_parameters.yaml',
    *('include/friction_compensation_controller/'+name+'.hpp' for name in (
        'friction_compensation_controller','kinetic_bank','stateful_friction',
        'elastoplastic_replay','directional_load_law','kinetic_curve','rest_dither',
        'position_velocity_observer','friction_reset_interlock','calibrated_friction','modified_lugre',
        'effort_reference'))))


def verify_prior_controller(report,root):
    """Require complete controller provenance before reusing physical evidence."""
    from .automatic_calibration import file_hash
    hashes=report.get('source_hashes') or {}
    if not set(CORE_SOURCES).issubset(hashes):
        raise ValueError('Physical controller source evidence is incomplete')
    # Old snapshots also included finite-probe-only headers. Their recipe is
    # not part of the hand controller. Keep their exclusion narrow and verify
    # that neither enters the hand controller's local include graph.
    probe_only={_CONTROLLER+'include/friction_compensation_controller/'+name+'.hpp'
                for name in ('common_release_sequence','probe_abort_gravity')}
    dependencies=set(CORE_SOURCES);pending=list(dependencies)
    while pending:
        path=root/pending.pop()
        for header in re.findall(r'^\s*#\s*include\s*["<]([^">]+\.hpp)[">]',
                                 path.read_text(),re.MULTILINE):
            if header.startswith('friction_compensation_controller/'):
                name=_CONTROLLER+'include/'+header
            elif (path.parent/header).is_file():
                name=str((path.parent/header).relative_to(root))
            else:continue  # External ROS/library headers have separate builds.
            if name not in dependencies:
                if not (root/name).is_file():
                    raise ValueError('Physical controller dependency is missing')
                dependencies.add(name);pending.append(name)
    if not dependencies.issubset(hashes):
        raise ValueError('Physical controller dependency evidence is incomplete')
    for name,digest in hashes.items():
        if name in probe_only-dependencies:continue
        if name.startswith(_CONTROLLER) and file_hash(root/name)!=digest:
            raise ValueError('Physical controller implementation changed')


def compose(sources):
    if not sources:raise ValueError('Empty kinetic bank')
    baseline=lambda p:{k:v for k,v in p.items() if not k.startswith('stateful_')}
    p=deepcopy(sources[0]);mask=[0]*6
    arrays={k:[0.]*6 for k in ARRAYS}
    for source in sources:
        active=[i for i,v in enumerate(source['stateful_joint_mask']) if v]
        if (baseline(source)!=baseline(p) or not source.get('stateful_friction_enabled')
                or len(active)!=1 or len(source['stateful_joint_mask'])!=6):
            raise ValueError('Expected independent single-axis laws with identical gravity baseline')
        if source.get('stateful_observer_window')!=sources[0].get('stateful_observer_window'):
            raise ValueError('Shared velocity observer differs between physical settings')
        i=active[0];j=i+1
        if (mask[i] or source.get('stateful_load_joint')!=j
                or source.get('stateful_curve_j6_enabled') or source.get('stateful_dither_j6_only')
                or any(source['stateful_damping']) or source.get('stateful_bank_enabled')):
            raise ValueError('Duplicate axis or mixed/non-common kinetic configuration')
        mask[i]=1
        for name in ARRAYS:
            row=source['stateful_'+name]
            if len(row)!=6 or not np.isfinite(row).all():raise ValueError('Invalid joint array')
            arrays[name][i]=row[i]
        prefix=f'stateful_bank.joint{j}.'
        smooth=source.get('stateful_load_smooth_width_nm',0.)>0
        signed=len(source.get('stateful_load_gravity_knots',[0.]))>1
        frequency=source.get('stateful_load_periodic_frequency',0.)
        if smooth==signed and not (frequency>0 and smooth):
            raise ValueError('Expected exactly one supported common load law')
        values=dict(kind=1 if frequency>0 else 0 if smooth else 2,
            width=source.get('stateful_load_smooth_width_nm',0.),
            load_scale=source.get('stateful_load_feature_scale_nm',1.),
            frequency=frequency,orders=source.get('stateful_load_periodic_orders',[1,2]),
            speeds=source['stateful_load_speed_knots'],
            loads=source.get('stateful_load_gravity_knots',[0.]),
            positive=source['stateful_load_smooth_positive' if smooth else 'stateful_load_positive_table'],
            negative=source['stateful_load_smooth_negative' if smooth else 'stateful_load_negative_table'],
            transition_distance=source['stateful_load_transition_distance'],
            fraction=source['stateful_curve_output_scale'],
            dither_amplitude=source.get('stateful_dither_amplitude_nm',0.) if source.get('stateful_dither_with_elastic') else 0.,
            dither_frequency=source.get('stateful_dither_frequency_hz',1.),
            dither_full_velocity=source.get('stateful_dither_full_velocity',.004),
            dither_zero_velocity=source.get('stateful_dither_zero_velocity',.012),
            dither_ramp=source.get('stateful_dither_ramp_seconds',.5))
        p.update({prefix+k:deepcopy(v) for k,v in values.items()})
        if source.get('stateful_load_hat_mean_extension',False):
            p[prefix+'load_hat_mean_extension']=True
    p.update(stateful_bank_enabled=True,stateful_joint_mask=mask,stateful_load_joint=0,
        stateful_load_smooth_width_nm=0.,stateful_load_periodic_frequency=0.,
        stateful_dither_with_elastic=False,stateful_dither_j6_only=False,stateful_curve_j6_enabled=False)
    p.update({'stateful_'+k:v for k,v in arrays.items()})
    p.pop('stateful_load_hat_mean_extension',None)
    return p


def _valid_fraction(value):
    return type(value) in (int,float) and np.isfinite(value) and 0<value<=1


def bank_fractions(parameters):
    """Actual per-axis gains; the legacy scalar is not used by a running bank."""
    mask=parameters.get('stateful_joint_mask',[])
    if (not parameters.get('stateful_bank_enabled') or len(mask)!=6
            or any(x not in (0,1) for x in mask) or not any(mask)):
        raise ValueError('Expected an active six-axis kinetic bank')
    fractions={f'joint{j+1}':parameters.get(f'stateful_bank.joint{j+1}.fraction')
               for j,enabled in enumerate(mask) if enabled}
    if not all(_valid_fraction(x) for x in fractions.values()):
        raise ValueError('Invalid recorded per-joint fractions')
    return fractions


def retune_completed_bank(parameters, report, fraction):
    """Supervised scalar retest or explicit joint overrides, preserving all model data.

    A mapping changes only its named active joints. New reports record every
    applied fraction; old uniform reports remain readable. Neither format
    transfers physical qualification to the newly requested settings.
    """
    old=bank_fractions(parameters)
    recorded=report.get('kinetic_fractions')
    scalar=report.get('kinetic_fraction')
    if recorded is None:
        fractions_match=(parameters.get('stateful_curve_output_scale')==scalar
                         and all(v==scalar for v in old.values()))
    else:
        fractions_match=(recorded==old and
                         (scalar is None or all(v==scalar for v in old.values())))
    changes=fraction if isinstance(fraction,dict) else {j:fraction for j in old}
    if not changes or not set(changes)<=set(old) or not all(_valid_fraction(v) for v in changes.values()):
        raise ValueError('Expected valid fractions for explicitly selected active joints')
    controller=report.get('last_running_controller') or {}
    recording=report.get('recording') or {}
    observed=controller.get('enabled') is True
    if (controller.get('enabled') is False and controller.get('active') is True
            and controller.get('configured') is True):
        # A normal operator OFF before shutdown is not a failed experiment.
        # Require actual logged nonzero ON output on every participating axis;
        # an OFF-only bringup still cannot qualify as a completed hand trial.
        exposure=report.get('joint_command_exposure') or {}
        observed=True
        for joint in old:
            row=exposure.get(joint) or {}
            phase=(row.get('phases') or {}).get('enabled_nonzero_output') or {}
            positive=lambda x:type(x) in (int,float) and np.isfinite(x) and x>0
            observed=observed and row.get('nonzero_output_observed') is True and all(
                positive(phase.get(k)) for k in ('samples','covered_duration_s',
                                               'logged_nonzero_output_cycles','command_peak_nm'))
            observed=observed and phase.get('samples',0)>=2
    if (not fractions_match or report.get('mock') is not False
            or report.get('candidate_kind')!='supervised_bank_all_axis_gravity'
            or not all(report.get(k) is True for k in ('executed','finished','motors_were_enabled',
                'controller_processes_stopped','torque_off_verified_after_shutdown'))
            or report.get('restores')!=[True]*6 or report.get('failure') or report.get('supervisor_fault')
            or controller.get('fault') is not False or not observed
            or recording.get('dropped_records')!=0 or recording.get('error')
            or report.get('compensation_parameters')!={k:v for k,v in parameters.items() if k.startswith('stateful_')}):
        raise ValueError('Require exact completed, restored, fault-free bank hand evidence')
    result=deepcopy(parameters)
    for joint,value in changes.items():
        result[f'stateful_bank.{joint}.fraction']=float(value)
    if not isinstance(fraction,dict):
        result['stateful_curve_output_scale']=float(fraction)
    return result


def load_bank(folder,urdf,baseline,fraction,*,_ancestors=()):
    """Reuse independently reviewed physical settings; never invent a qualified axis."""
    import json
    from pathlib import Path
    from .automatic_calibration import file_hash
    from .integration import fingerprint
    from .common_probe_hand import load_completed_hand_parameters,load_comparison_parameters
    folder=Path(folder)
    if folder.resolve() in _ancestors:
        raise ValueError('Cyclic bank provenance')
    lineage=(*_ancestors,folder.resolve())
    bank=json.loads((folder/'candidate.json').read_text())
    if bank.get('schema') not in ('supervised_kinetic_bank_v1','supervised_kinetic_bank_fraction_retest_v1',
                                'supervised_kinetic_bank_replacement_v1', 'configured_kinetic_bank_v1') or bank.get('automatic_deployment_allowed') is not False:
        raise ValueError('Expected an explicit supervised bank manifest')
    expected=bank['identity']
    if expected['baseline_sha256']!=file_hash(baseline) or expected['robot_fingerprint']!=fingerprint(urdf):
        raise ValueError('Bank baseline or robot differs')
    if bank['schema']=='supervised_kinetic_bank_replacement_v1':
        from .kinetic_bank_update import load_update
        return load_update(bank,urdf,baseline,fraction,_ancestors=lineage)
    if bank['schema']=='supervised_kinetic_bank_fraction_retest_v1':
        trial=bank['fraction_retest'];prior=Path(trial['prior_bank']);hand=Path(trial['prior_hand'])
        if (trial['requested_fraction']!=fraction or file_hash(prior/'candidate.json')!=trial['prior_bank_sha256']
                or set(trial['hand_files_sha256'])!={'execution.json',*[f'restore_j{j}.json' for j in range(1,7)]}):
            raise ValueError('Changed retest request or incomplete hand evidence')
        old_bank=json.loads((prior/'candidate.json').read_text())
        if (old_bank.get('schema') not in ('supervised_kinetic_bank_v1','supervised_kinetic_bank_replacement_v1')
                or old_bank.get('identity')!=expected
                or old_bank.get('components')!=bank.get('components')):
            raise ValueError('A fraction retest cannot change the original models or chain retests')
        for name,digest in trial['hand_files_sha256'].items():
            if file_hash(hand/name)!=digest:raise ValueError('Changed completed hand evidence')
        report=json.loads((hand/'execution.json').read_text())
        if report.get('candidate_sha256')!=trial['prior_bank_sha256']:
            raise ValueError('Prior hand used a different bank')
        root=Path(__file__).resolve().parents[2]
        verify_prior_controller(report,root)
        for j in range(1,7):
            r=json.loads((hand/f'restore_j{j}.json').read_text())
            if not (r.get('target_joint')==j and r.get('passed') and r.get('torque_enable_verified_off') and r.get('torque_enabled') is False):
                raise ValueError('Prior hand restoration is incomplete')
        previous_request=report.get('kinetic_fraction_request',report.get('kinetic_fraction'))
        original=load_bank(prior,urdf,baseline,previous_request,_ancestors=lineage)
        return retune_completed_bank(original,report,fraction)
    configured = bank['schema'] == 'configured_kinetic_bank_v1'
    if configured and (fraction is not None or not bank.get('components')):
        raise ValueError('Configured banks load their recorded gains before applying operator YAML')
    if not configured and not _valid_fraction(fraction):
        raise ValueError('Joint overrides require an explicit completed-bank retest manifest')
    result=[];seen=set()
    for row in bank['components']:
        gain = row.get('recorded_fraction') if configured else fraction
        if not _valid_fraction(gain): raise ValueError('Missing recorded component fraction')
        path=Path(row['path'])
        if not path.is_absolute():path=folder/path
        model=json.loads((path/'candidate.json').read_text())
        if model['identity']!=expected or file_hash(path/'candidate.json')!=row['candidate_sha256']:
            raise ValueError('Bank component model or identity changed')
        if row['joint']!=model['joint'] or row['joint'] in seen:
            raise ValueError('Bank joint mismatch or duplicate')
        seen.add(row['joint'])
        if row['kind']=='completed_hand_session':
            if file_hash(path/'execution.json')!=row['execution_sha256']:
                raise ValueError('Bank hand evidence changed')
            if configured and json.loads((path/'execution.json').read_text()).get('common_parameters',{}).get('common_output_fraction')!=gain:
                raise ValueError('Configured bank gain differs from its recorded hand trial')
            p=load_completed_hand_parameters(path,urdf,baseline,gain)
        elif row['kind']=='release_comparison':
            p=load_comparison_parameters(path,urdf,baseline,gain)
        elif row['kind']=='supervised_coast_series':
            from .coast_handoff import load_parameters
            if file_hash(path/'coast_series.json')!=row['series_sha256']:
                raise ValueError('Bank physical coast evidence changed')
            p=load_parameters(path,urdf,baseline,gain)
        else:raise ValueError('Unsupported bank component evidence')
        result.append(p)
    return compose(result)
