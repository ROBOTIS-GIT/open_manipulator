"""One bringup: original gravity/friction lifecycle plus in-update impedance."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def setup(context):
    from open_manipulator_friction_diagnostic.artifact import calibration_path, load_runtime_calibration
    from open_manipulator_friction_diagnostic.integration import description
    from open_manipulator_friction_diagnostic.launch_support import boolean, controller_document, runtime_action, runtime_directory
    value = lambda name: LaunchConfiguration(name).perform(context)
    artifact = Path(value('calibration_file')).expanduser() if value('calibration_file') else calibration_path()
    mock = boolean(value('use_mock_hardware'))
    calibration = load_runtime_calibration(artifact, hardware=not mock)
    compensation = yaml.safe_load(Path(value('controllers')).expanduser().read_text())
    gains = yaml.safe_load(Path(value('gains')).expanduser().read_text())['joint_impedance_controller']['ros__parameters']
    permitted = {'gravity_compensation_enabled', 'friction_compensation_enabled',
                 'impedance.stiffness', 'impedance.damping', 'impedance.integral_gain',
                 'impedance.integral_limit', 'impedance.integral_leak',
                 'wrench.enabled', 'wrench.root_link', 'wrench.tip_link',
                 'wrench.publish_rate_hz', 'wrench.damping_m', 'wrench.characteristic_length_m',
                 'wrench.residual_filter_time_s', 'wrench.acceleration_filter_time_s'}
    if not set(gains) <= permitted:
        raise ValueError(f'Unknown impedance parameters: {sorted(set(gains)-permitted)}')
    for switch in ('gravity_compensation_enabled', 'friction_compensation_enabled', 'wrench.enabled'):
        if switch in gains and type(gains[switch]) is not bool:
            raise ValueError(f'{switch} must be YAML true or false')
    parameters = controller_document('current', calibration, compensation, hardware=not mock)
    # The existing motor supervisor addresses this internal controller name.
    # Only the type changes in this new launch's private parameter document.
    name = 'friction_compensation_controller'
    parameters['controller_manager']['ros__parameters'][name]['type'] = (
        'open_manipulator_joint_impedance_controller/JointImpedanceController')
    parameters[name]['ros__parameters'].update(gains)
    runtime = runtime_action('current', description('current', mock=mock), parameters, runtime_directory(), mock=mock)
    return [RegisterEventHandler(OnProcessExit(target_action=runtime,
        on_exit=[EmitEvent(event=Shutdown(reason='Impedance motor runtime stopped'))])), runtime]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('controllers', default_value=PathJoinSubstitution([
            FindPackageShare('open_manipulator_friction_diagnostic'), 'config', 'controllers.yaml'])),
        DeclareLaunchArgument('gains', default_value=PathJoinSubstitution([
            FindPackageShare('open_manipulator_joint_impedance_controller'), 'config', 'joint_impedance.yaml'])),
        OpaqueFunction(function=setup)])
