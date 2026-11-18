"""Direct effort-input gravity/friction bringup. No mock, replay or refitting."""
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
    data = load_runtime_calibration(artifact, hardware=not mock)
    settings = yaml.safe_load(Path(value('controllers')).expanduser().read_text())
    parameters = controller_document('current', data, settings, hardware=not mock)
    runtime = runtime_action('current', description('current', mock=mock), parameters, runtime_directory(), mock=mock)
    return [RegisterEventHandler(OnProcessExit(target_action=runtime,
                on_exit=[EmitEvent(event=Shutdown(reason='Motor runtime stopped'))])), runtime]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('controllers', default_value=PathJoinSubstitution([
            FindPackageShare('open_manipulator_friction_diagnostic'), 'config', 'controllers.yaml'])),
        OpaqueFunction(function=setup)])
