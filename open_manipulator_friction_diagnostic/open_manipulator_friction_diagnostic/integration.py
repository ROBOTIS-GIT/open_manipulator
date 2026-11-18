"""Build ROS bringup using original OMY description and gravity parameters."""
import hashlib
import json
from pathlib import Path
import tempfile
from datetime import datetime, timezone
import uuid
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .protocol import JOINTS, config, sha


class MergeLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=True)
        value = loader.construct_object(value_node, deep=True)
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key].update(value)
        elif key in result:
            raise ValueError(f'Duplicate non-mapping YAML key: {key}')
        else:
            result[key] = value
    return result


MergeLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def original_gravity(path):
    # Upstream configuration repeats the /** key. Ordinary safe_load would lose controllers.
    document = yaml.load(Path(path).read_text(), Loader=MergeLoader)
    return document['/**']['gravity_compensation_controller']['ros__parameters']


def fingerprint(urdf):
    root = ET.fromstring(urdf)
    elements = []
    for tag in ('link', 'joint'):
        for node in sorted(root.findall(tag), key=lambda x: x.attrib['name']):
            # Mesh paths may differ between installed and source trees. Geometry is unchanged.
            for el in node.iter():
                el.text = None; el.tail = None
                if el.tag == 'mesh':
                    el.attrib['filename'] = el.attrib['filename'].split('open_manipulator_description/')[-1]
            elements.append(ET.tostring(node, encoding='unicode'))
    return hashlib.sha256(''.join(elements).encode()).hexdigest()


def description(mode, mock=False):
    from ament_index_python.packages import get_package_share_directory
    import xacro
    path = Path(get_package_share_directory('open_manipulator_description'))/'urdf/omy_f3m/omy_f3m.urdf.xacro'
    return xacro.process_file(str(path), mappings={
        'ros2_control_type': 'omy_f3m_'+mode,
        'use_mock_hardware': str(mock).lower(), 'use_sim': 'false'}).toxml()


def baseline_path():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('open_manipulator_bringup'))/'config/omy_f3m_leader_ai/hardware_controller_manager.yaml'


def verify_current_units(c, model_folder=None):
    """Check spec-derived candidate thresholds against installed software units only."""
    expected = c.get('expected_current_units_nm_per_count', {})
    if not expected:
        return
    if model_folder is None:
        from ament_index_python.packages import get_package_share_directory
        model_folder = Path(get_package_share_directory('dynamixel_hardware_interface'))/'param/dxl_model'
    for model, scale in expected.items():
        if model not in ('ym080_230_a099', 'ym070_210_a099'):
            raise ValueError('Unknown actuator model for current-unit verification')
        rows = {}
        for line in (Path(model_folder)/(model+'.model')).read_text().split('[control table]')[0].splitlines():
            fields = line.split('\t')
            if fields[0] in ('Present Current', 'Goal Current'):
                rows[fields[0]] = fields
        for item in ('Present Current', 'Goal Current'):
            row = rows.get(item, [])
            if (len(row) != 5 or row[2:4] != ['N m', 'signed'] or float(row[4]) != 0 or
                    not np.isfinite(scale) or scale <= 0 or
                    not np.isclose(float(row[1]), scale, rtol=1e-8, atol=0)):
                raise ValueError(f'{model}: installed {item} conversion differs; review torque thresholds before bringup')


def load_profile(path, c, urdf, baseline, mock=False, experimental_zero_delay=False,
                 assistance_preset='recorded',allow_unqualified_candidate=False):
    path = Path(path).expanduser().resolve()
    p = json.loads(path.read_text())
    from .loaded_campaign import reject_unsafe_trial
    reject_unsafe_trial(path.parent)
    from .deployment import require_simulation
    require_simulation(mock)
    if p.get('schema') == 7:
        from .lugre import controller_parameters as lugre_parameters
        return lugre_parameters(p, c, urdf, baseline)
    if assistance_preset == 'identified':
        from .campaign import controller_parameters
        return controller_parameters(path, c, urdf, baseline, experimental_zero_delay, mock,allow_unqualified_candidate)
    if allow_unqualified_candidate:raise ValueError('Unqualified experiment requires identified preset')
    from .trial_limits import QUALIFICATION, ACCEPTANCE, limits_for_report
    trial = p.get('qualification') == QUALIFICATION
    from .runtime_tuning import PRESETS, strong_trial_settings
    if assistance_preset not in PRESETS:
        raise ValueError('Unknown assistance preset')
    if assistance_preset != 'recorded' and (not trial or not experimental_zero_delay):
        raise ValueError('Strong assistance requires an explicit experimental trial')
    if p.get('qualification') and not trial:
        raise ValueError('Unknown/obsolete trial qualification; reprocess the saved data')
    if trial != experimental_zero_delay:
        raise ValueError('Trial profile requires explicit allow_experimental_zero_delay:=true; never load it as qualified')
    if trial:
        from .trial import validate_trial_evidence
        validate_trial_evidence(path.parent, c, p)
    use_load = c['friction_model'] == 'velocity_load'
    method = 'paired_load_stribeck' if use_load else 'paired_stribeck'
    if (p['schema'] != 3 or p['joints'] != JOINTS or p['effort_unit'] != 'joint_equivalent_Nm' or
            p['method'] != method or p.get('load_compensation_enabled') != use_load or not p['passed'] or
            p['robot_unit'] != c['robot_unit'] or p['robot_fingerprint'] != fingerprint(urdf) or
            p['baseline_hash'] != sha(baseline)):
        raise ValueError('Profile robot, model, baseline, units or validation do not match')
    if p['source'] not in (('hardware', 'synthetic') if mock else ('hardware',)):
        raise ValueError('Synthetic/Gazebo profiles cannot be used on hardware')
    if not mock and not trial and c.get('sensor_timing_verified') is not True:
        raise ValueError('Unknown sensor timing cannot qualify a hardware friction profile')
    report_file = 'trial_identification.json' if trial else 'identification.json'
    for file, field in [(report_file, 'identification_sha256'),
                        ('measurements.npz', 'measurements_sha256'), ('trajectory.npz', 'plan_sha256')]:
        if sha(path.parent/file) != p[field]:
            raise ValueError(f'Profile evidence hash mismatch: {file}')
    report = json.loads((path.parent/report_file).read_text())
    if not report['passed'] or report['parameters'] != p['parameters']:
        raise ValueError('Parameters do not match the validated identification report')
    from .protocol import validation_poses
    expected_poses = set() if trial else {x['name'] for x in validation_poses(c)}
    if trial and (report.get('qualification') != p['qualification'] or
                  report.get('sensor_timing_verified') is not False or
                  report.get('sensor_timing_assumption') != 'zero_delay' or
                  report.get('posture_validation_performed') is not False):
        raise ValueError('Missing explicit trial assumptions')
    if trial:
        limits = limits_for_report(report, c)
        if (p.get('acceptance_policy') != ACCEPTANCE or report.get('acceptance_policy') != ACCEPTANCE
                or p.get('estimator') != report.get('estimator')
                or type(report.get('pointwise_model_passed')) is not bool
                or p.get('pointwise_model_passed') != report['pointwise_model_passed']
                or p.get('trial_runtime_limits') != limits or report.get('trial_runtime_limits') != limits
                or not all(limits['observed_reserve_passed'])
                or not all(x.get('mean_model_passed') and x.get('observed_reserve_passed')
                           for x in report['validation'])):
            raise ValueError('Missing or changed bounded-trial acceptance/limits')
    if (report.get('schema') != 3 or report.get('method') != method or
            report.get('load_compensation_enabled') != use_load or
            report.get('posture_validation_used_for_fit') is not False or
            report['load_bounds_nm'] != p['load_bounds_nm'] or len(report['validation']) != 6 or not all(
            check.get('passed') and check.get('independent_repeat_validation')
            and (check.get('validation_load_covered') or not use_load)
            and {x['name'] for x in check.get('posture_checks', [])} == expected_poses
            and all(x['passed'] and x['samples_used_for_fit'] is False for x in check['posture_checks'])
            for check in report['validation'])):
        raise ValueError('Missing held-out repeat/posture validation evidence')
    if use_load and not report.get('completion_load_check', {}).get('passed'):
        raise ValueError('Missing completion-pose load coverage')
    bounds = np.asarray(p['load_bounds_nm'])
    if (bounds.shape != (6, 2) or not np.isfinite(bounds).all() or np.any(bounds < 0) or
            np.any(bounds[:, 1] <= bounds[:, 0])):
        raise ValueError('Invalid validated load bounds')
    params = np.asarray(p['parameters'])
    if (params.shape != (6, 5) or not np.isfinite(params).all() or np.any(params < 0) or
            np.any(params[:, 1] < params[:, 0]) or np.any(params[:, 2] <= 0) or np.any(params[:, 4] > 1.) or
            (not use_load and np.any(params[:, 4] != 0))):
        raise ValueError('Invalid friction coefficients')
    # A configuration change requires re-calibration; runtime A/B uses a service instead.
    if c != p['config']:
        raise ValueError('Configuration changed since calibration; use the saved config.yaml or recalibrate')
    base = original_gravity(baseline)
    base.update(calibrated_friction_enabled=True,
                calibrated_fc=params[:, 0].tolist(), calibrated_fs=params[:, 1].tolist(),
                calibrated_vs=params[:, 2].tolist(), calibrated_viscous=params[:, 3].tolist(),
                calibrated_gain=limits['gain'] if trial else c['compensation_gain'],
                calibrated_max_torque=limits['max_compensation_nm'] if trial else list(map(float, c['max_compensation_nm'])),
                calibrated_slew_rate=list(map(float, c['slew_rate_nm_s'])),
                calibrated_load_gain=params[:, 4].tolist(),
                calibrated_enforce_load_bounds=use_load,
                calibrated_load_min=bounds[:, 0].tolist(), calibrated_load_max=bounds[:, 1].tolist(),
                calibrated_deadband=c['velocity_deadband_rad_s'],
                calibrated_min_velocity=min(c['speeds_rad_s']),
                calibrated_max_velocity=max(c['speeds_rad_s']))
    if assistance_preset == 'strong':
        # A requested runtime experiment, never alter the saved model, config,
        # profile qualification, or the original gravity/torque conversion.
        base.update(strong_trial_settings(report, c))
    return base


def launch_setup(context, freedrive=False):
    from launch.actions import IncludeLaunchDescription, LogInfo
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    mock = arg('use_mock_hardware').lower() == 'true'
    if freedrive:
        # Reject before description generation, file loading or hardware bringup.
        from .deployment import require_simulation
        require_simulation(mock)
    record_only = False
    posture_only = False
    experimental = False
    assistance_preset = 'recorded'
    unqualified=False
    if freedrive:
        mode = arg('allow_experimental_zero_delay').lower()
        if mode not in ('true', 'false'):
            raise ValueError('allow_experimental_zero_delay must be true or false')
        experimental = mode == 'true'
        assistance_preset = arg('assistance_preset')
        mode=LaunchConfiguration('allow_unqualified_candidate',default='false').perform(context).lower()
        if mode not in ('true','false'):raise ValueError('allow_unqualified_candidate must be true or false')
        unqualified=mode=='true'
    if not freedrive:
        mode = arg('record_only').lower()
        if mode not in ('true', 'false'):
            raise ValueError('record_only must be true or false')
        record_only = mode == 'true'
        mode = arg('posture_validation_only').lower()
        if mode not in ('true', 'false'):
            raise ValueError('posture_validation_only must be true or false')
        posture_only = mode == 'true'
        if record_only and posture_only:
            raise ValueError('Choose original sweeps or posture validation')
    cpath = Path(arg('config')).expanduser().resolve()
    c = config(cpath, hardware=not mock, record_only=record_only or posture_only or experimental)
    if not mock:
        verify_current_units(c)
    out = Path(arg('output')).expanduser().resolve()
    if not freedrive:
        # Validate before any bringup can open the serial port.
        if (out/'runs/PAUSE_NEW_COLLECTIONS').exists() or (out/'PAUSE_NEW_COLLECTIONS').exists():
            raise ValueError('New collection is paused; hardware bringup has not started')
        from .protocol import generate, generate_posture_validation, validate_urdf_plan
        from .completion import completion_pose
        meta, data = generate_posture_validation(c) if posture_only else generate(c, base_only=record_only)
        urdf = description('position', mock)
        validate_urdf_plan(meta, data, urdf)
        review_path = LaunchConfiguration('motion_review', default='').perform(context)
        if not mock:
            from .motion_review import require_review
            require_review(review_path, c, urdf)
        completion_pose(dict(c, completion_pose='zero') if record_only or posture_only else c, urdf)
        session = out/'runs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
        if (out/'latest').exists() and not (out/'latest').is_symlink():
            raise ValueError('latest must be a managed symlink, not an existing directory')
        bringup = Path(get_package_share_directory('open_manipulator_bringup'))/'launch/omy_f3m.launch.py'
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup)),
                    launch_arguments={'use_mock_hardware': str(mock).lower(),
                                      'init_position': 'false', 'ros2_control_type': 'omy_f3m_position'}.items()),
                Node(package='open_manipulator_friction_diagnostic', executable='calibrate', output='screen',
                     arguments=['--config', str(cpath), '--output', str(session), '--latest', str(out/'latest'),
                                '--motion-review', review_path]
                     + (['--mock'] if mock else []) + (['--record-only'] if record_only else [])
                     + (['--posture-only'] if posture_only else []))]
    urdf = description('current', mock)
    profile = Path(arg('profile')).expanduser().resolve()
    params = load_profile(profile, c, urdf, baseline_path(), mock, experimental,
                          assistance_preset,unqualified)
    manager = yaml.load(baseline_path().read_text(), Loader=MergeLoader)['/**']['controller_manager']['ros__parameters']
    generated = {'controller_manager': {'ros__parameters': manager},
                 'gravity_compensation_controller': {'ros__parameters': params}}
    with tempfile.NamedTemporaryFile(mode='w', prefix='omy_freedrive_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(generated, f); controller_file = f.name
    if params.get('calibrated_lugre'):
        notice = ('SIMULATION ONLY: dynamic LuGre with the patent parameter condition and '
                  'latched speed/acceleration assistance trips. No robot passivity certificate. ')
    elif assistance_preset == 'identified':
        notice = ('UNSCALED EXPERIMENT: gain=1; no torque cap, slew limit or velocity fades. '
                  'Original gravity controller and motor conversion retained. '
                  'Steady mean validation does not establish static breakaway, passivity or freedrive performance. ')
        if unqualified:notice='UNQUALIFIED CANDIDATE: independent validation FAILED. Explicit manual experiment only. '+notice
    else:
        notice = ('EXPERIMENTAL: assumed zero sensor delay; bounded mean assistance, NOT full friction cancellation. '
              f'Runtime preset {assistance_preset}; stronger settings do not upgrade model qualification. '
              f'Gain {params["calibrated_gain"]}; joint caps {params["calibrated_max_torque"]} ROS Nm. '
              f'Slew {params["calibrated_slew_rate"]} ROS Nm/s. '
                  if experimental else '')
    return [LogInfo(msg=notice+'Current mode: original gravity controller + calibrated friction. No homing.'),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': urdf}]),
            Node(package='controller_manager', executable='ros2_control_node', output='screen',
                 parameters=[{'robot_description': urdf}, controller_file]),
            Node(package='controller_manager', executable='spawner',
                 arguments=['gravity_compensation_controller', 'joint_state_broadcaster'], output='screen')]
