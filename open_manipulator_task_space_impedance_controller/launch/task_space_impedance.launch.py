"""Independent Cartesian controller; reuse the existing motor lifecycle unchanged."""
import math
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def normalize_gains(gains):
    if not isinstance(gains, dict):
        raise ValueError('Task controller parameters must be a mapping')
    result = dict(gains)
    vectors = {'task.stiffness', 'task.damping'}
    switches = {'gravity_compensation_enabled', 'friction_compensation_enabled'}
    links = {'task.root_link', 'task.tip_link'}
    if set(result) - vectors - switches - links - {'task.publish_rate_hz'}:
        raise ValueError('Unknown task-space impedance parameter')
    for key, value in result.items():
        if key in vectors:
            if (not isinstance(value, list) or len(value) != 6 or
                    any(type(x) not in (int, float) or not math.isfinite(x) or x < 0 for x in value)):
                raise ValueError(f'{key} requires six finite nonnegative numbers')
            result[key] = [float(x) for x in value]
        elif key in switches:
            if type(value) is not bool:
                raise ValueError(f'{key} must be true or false')
        elif key in links:
            if not isinstance(value, str) or not value:
                raise ValueError(f'{key} must be a nonempty link name')
        else:
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 400:
                raise ValueError('task.publish_rate_hz must be in (0,400]')
            result[key] = float(value)
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
    gains = normalize_gains(yaml.safe_load(Path(value('gains')).expanduser().read_text())[
        'task_space_impedance_controller']['ros__parameters'])
    parameters = controller_document('current', calibration, compensation, hardware=not mock)
    # Private parameter document only. This name is required by the existing
    # motor supervisor; the plugin type and Cartesian topics are distinct.
    name = 'friction_compensation_controller'
    parameters['controller_manager']['ros__parameters'][name]['type'] = (
        'open_manipulator_task_space_impedance_controller/TaskSpaceImpedanceController')
    parameters[name]['ros__parameters'].update(gains)
    runtime = runtime_action('current', description('current', mock=mock), parameters,
                             runtime_directory(), mock=mock)
    return [RegisterEventHandler(OnProcessExit(target_action=runtime,
        on_exit=[EmitEvent(event=Shutdown(reason='Task-space impedance runtime stopped'))])), runtime]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('controllers', default_value=PathJoinSubstitution([
            FindPackageShare('open_manipulator_friction_diagnostic'), 'config', 'controllers.yaml'])),
        DeclareLaunchArgument('gains', default_value=PathJoinSubstitution([
            FindPackageShare('open_manipulator_task_space_impedance_controller'), 'config', 'task_space_impedance.yaml'])),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        OpaqueFunction(function=setup)])
