"""Separate contact-responsive QP controller using the existing motor lifecycle."""
from pathlib import Path
import math
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def normalize_gains(gains):
    """ROS double parameters must stay doubles even when YAML uses integer literals."""
    result = dict(gains)
    vectors = {
        'nominal.stiffness', 'nominal.damping', 'inner.stiffness', 'inner.damping',
        'contact.task_limit_nm', 'contact.correction_limit_nm', 'contact.motion_limit_nm',
        'contact.nominal_error_rad', 'model.rotor_inertia'}
    scalars = {'qp.error_slack_weight', 'qp.tolerance', 'qp.max_period_s', 'fallback.ramp_s'}
    for name in vectors | scalars:
        if name not in result:
            continue
        value = result[name]
        values = value if name in vectors else [value]
        if (not isinstance(values, list) or
                (name in vectors and len(values) != 6) or
                any(type(x) not in (int, float) or not math.isfinite(x) for x in values)):
            raise ValueError(f'{name}: expected finite numeric ' +
                             ('array of six values' if name in vectors else 'value'))
        result[name] = [float(x) for x in values] if name in vectors else float(value)
    return result


def setup(context):
    from open_manipulator_friction_diagnostic.artifact import calibration_path, load_runtime_calibration
    from open_manipulator_friction_diagnostic.integration import description
    from open_manipulator_friction_diagnostic.launch_support import (
        boolean, controller_document, runtime_action, runtime_directory)
    value = lambda name: LaunchConfiguration(name).perform(context)
    artifact = Path(value('calibration_file')).expanduser() if value('calibration_file') else calibration_path()
    mock = boolean(value('use_mock_hardware'))
    calibration = load_runtime_calibration(artifact, hardware=not mock)
    compensation = yaml.safe_load(Path(value('controllers')).expanduser().read_text())
    gains = yaml.safe_load(Path(value('gains')).expanduser().read_text())['contact_responsive_controller']['ros__parameters']
    permitted = {
        'gravity_compensation_enabled', 'friction_compensation_enabled',
        'nominal.stiffness', 'nominal.damping',
        'inner.stiffness', 'inner.damping', 'contact.task_limit_nm',
        'contact.correction_limit_nm', 'contact.motion_limit_nm', 'contact.nominal_error_rad',
        'model.root_link', 'model.tip_link', 'model.rotor_inertia', 'qp.error_slack_weight',
        'qp.tolerance', 'qp.max_iterations', 'qp.max_period_s', 'fallback.ramp_s'}
    if not set(gains) <= permitted:
        raise ValueError(f'Unknown contact controller settings: {sorted(set(gains)-permitted)}')
    for switch in ('gravity_compensation_enabled', 'friction_compensation_enabled'):
        if switch in gains and type(gains[switch]) is not bool:
            raise ValueError(f'{switch} must be YAML true or false')
    gains = normalize_gains(gains)
    parameters = controller_document('current', calibration, compensation, hardware=not mock)
    name = 'friction_compensation_controller'  # Address used by the existing motor supervisor.
    parameters['controller_manager']['ros__parameters'][name]['type'] = (
        'open_manipulator_contact_responsive_controller/ContactResponsiveController')
    parameters[name]['ros__parameters'].update(gains)
    runtime = runtime_action('current', description('current', mock=mock), parameters,
                             runtime_directory(), mock=mock)
    return [RegisterEventHandler(OnProcessExit(target_action=runtime,
        on_exit=[EmitEvent(event=Shutdown(reason='Contact controller runtime stopped'))])), runtime]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('controllers', default_value=PathJoinSubstitution([
            FindPackageShare('open_manipulator_friction_diagnostic'), 'config', 'controllers.yaml'])),
        DeclareLaunchArgument('gains', default_value=PathJoinSubstitution([
            FindPackageShare('open_manipulator_contact_responsive_controller'), 'config', 'contact_responsive.yaml'])),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        OpaqueFunction(function=setup)])
