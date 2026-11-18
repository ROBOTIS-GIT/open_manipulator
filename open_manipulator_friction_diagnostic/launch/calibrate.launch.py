"""Position bringup, slow zero approach, acquire, fit, save: one command."""
from pathlib import Path
import signal
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def setup(context, *, resume_override=None, retry_count=0):
    from open_manipulator_friction_diagnostic.artifact import calibration_path, load_calibration
    from open_manipulator_friction_diagnostic.device_identity import data_directory, require_unit
    from open_manipulator_friction_diagnostic.calibration_plan import prepare_session
    from open_manipulator_friction_diagnostic.integration import description, baseline_path
    from open_manipulator_friction_diagnostic.launch_support import boolean, controller_document, runtime_action, staged_collection_directory
    value = lambda name: LaunchConfiguration(name).perform(context)
    sim, validate, dither, resume = [boolean(value(name)) for name in ('use_sim', 'validate', 'collect_dither', 'resume')]
    if resume_override is not None:
        resume = resume_override
    retries = int(value('max_acquisition_retries'))
    if not 0 <= retries <= 2:
        raise ValueError('max_acquisition_retries must be 0, 1 or 2')
    if resume and sim:
        raise ValueError('Resume is for interrupted hardware acquisitions only')
    if sim and dither:
        raise ValueError('Dither selection requires measured motor current; use collect_dither:=false in Gazebo')
    output = Path(value('output')).expanduser().resolve() if value('output') else data_directory() / 'collection'
    artifact = Path(value('calibration_file')).expanduser().resolve() if value('calibration_file') else calibration_path()
    previous = load_calibration(artifact, allow_partial=True) if artifact.is_file() else None
    if previous and not sim:
        require_unit(previous)
    urdf = description('position')
    with staged_collection_directory(output, artifact) as stage:
        metadata, trajectory = prepare_session(value('config'), stage, urdf, baseline_path(),
            validate=validate, previous=previous, joints=value('joints') or None,
            device_config=value('device_config') or None)
        if not sim:
            require_unit(metadata['config'])
        if resume:
            from open_manipulator_friction_diagnostic.collection_resume import prepare_resume
            info, _ = prepare_resume(output, metadata, trajectory)
    command = [sys.executable, '-m', 'open_manipulator_friction_diagnostic.calibration_node',
        '--output', str(output), '--calibration-file', str(artifact), '--baseline', str(baseline_path())]
    if resume:
        source = output.with_name(output.name+'.previous')
        print(f"RESUMING at {info['start_time_s']:.2f}s; retained {len(info['completed_cycles'])} complete paired motions", flush=True)
        command += ['--resume-from', str(source)]
    if sim:
        command += ['--simulation']
    if validate:
        command += ['--validate']
    if dither:
        command += ['--collect-dither']
    collector = ExecuteProcess(cmd=command, output='screen', sigterm_timeout='45', sigkill_timeout='10')
    if sim:
        from ament_index_python.packages import get_package_share_directory
        import xacro
        from open_manipulator_friction_diagnostic.simulation import simulation_actions
        xacro_path = Path(get_package_share_directory('open_manipulator_description'))/'urdf/omy_f3m/omy_f3m.urdf.xacro'
        sim_urdf = xacro.process_file(str(xacro_path), mappings={
            'ros2_control_type': 'omy_f3m_position', 'use_sim': 'true', 'use_mock_hardware': 'false'}).toxml()
        actions = simulation_actions(output, sim_urdf, real_time_factor=float(value('real_time_factor')),
                                     video_fps=float(value('video_fps')))
        recorder = actions[-1]
        return [RegisterEventHandler(OnProcessExit(target_action=recorder,
                    on_exit=[EmitEvent(event=Shutdown(reason='Gazebo recording finalized'))])),
                *actions, collector]
    runtime = runtime_action('position', urdf, controller_document('position'), output/'runtime',
                             defer_position_enable=True)
    exits = {}
    def finished(which, event, ctx):
        exits[which] = event.returncode
        if ctx.is_shutdown:
            return []
        if which == 'collector' and 'runtime' not in exits:
            import json
            try:
                stopped = json.loads((output/'runtime/runtime.json').read_text())
                if stopped.get('all_off_confirmed') and stopped.get('phase') == 'stopped':
                    return []  # Let the already-stopped supervisor finish cleanup.
            except (OSError, ValueError):
                pass
            # Even an unexpected collector crash must request an orderly stop.
            return [EmitEvent(event=SignalProcess(signal_number=signal.SIGINT,
                                                 process_matcher=matches_action(runtime)))]
        if len(exits) < 2:
            return []
        from open_manipulator_friction_diagnostic.collection_resume import prepare_retry
        try:
            recovery = prepare_retry(output, metadata, trajectory, attempt=retry_count, limit=retries,
                                     collector_code=exits['collector'], runtime_code=exits['runtime'])
        except Exception as exc:
            print(f'AUTO_RETRY not eligible: {exc}', flush=True)
            recovery = None
        if recovery is not None:
            print(f"REACQUIRING attempt {recovery['attempt']}/{retries}: "
                  f"retained {len(recovery['completed_cycles'])} complete paired motions; "
                  f"restart at {recovery['start_time_s']:.2f}s. Raw data: {recovery['archive']}", flush=True)
            return [OpaqueFunction(function=lambda next_context: setup(
                next_context, resume_override=True, retry_count=retry_count+1))]
        return [EmitEvent(event=Shutdown(reason='Calibration finished'))]
    return [RegisterEventHandler(OnProcessExit(target_action=collector,
                on_exit=lambda event, ctx: finished('collector', event, ctx))),
            RegisterEventHandler(OnProcessExit(target_action=runtime,
                on_exit=lambda event, ctx: finished('runtime', event, ctx))), runtime, collector]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('open_manipulator_friction_diagnostic'), 'config', 'calibration.yaml'])),
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('device_config', default_value='',
            description='Per-unit sensor timing YAML; empty loads the current unit\'s device.yaml if present'),
        DeclareLaunchArgument('joints', default_value='', description='all or comma-separated joints (2,3); empty uses YAML, whose default is all'),
        DeclareLaunchArgument('output', default_value='', description='Default: per-unit data directory / collection'),
        DeclareLaunchArgument('collect_dither', default_value='false'),
        DeclareLaunchArgument('validate', default_value='false'),
        DeclareLaunchArgument('resume', default_value='false'),
        DeclareLaunchArgument('max_acquisition_retries', default_value='2',
            description='After a confirmed-OFF acquisition timing fault, retry from the paired-motion checkpoint (0 disables)'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('real_time_factor', default_value='10.0'),
        DeclareLaunchArgument('video_fps', default_value='1.0'),
        OpaqueFunction(function=setup)])
