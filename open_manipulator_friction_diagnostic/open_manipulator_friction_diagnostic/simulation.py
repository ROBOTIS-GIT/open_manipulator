"""Actual Gazebo Harmonic assets and launch actions for the shared collector."""
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


def _element(parent, tag, text=None, **attributes):
    result = ET.SubElement(parent, tag, **attributes)
    if text is not None:
        result.text = str(text)
    return result


def world_xml(real_time_factor=10., video_fps=5.):
    """Keep the 400 Hz physics timestep; factor zero means unthrottled Gazebo."""
    if not math.isfinite(real_time_factor) or not 0 <= real_time_factor <= 30:
        raise ValueError('Gazebo real-time factor must be from zero (unlimited) to 30')
    if not math.isfinite(video_fps) or not 1 <= video_fps <= 30:
        raise ValueError('Camera frame rate must be between 1 and 30 fps')
    sdf = ET.Element('sdf', version='1.9')
    world = _element(sdf, 'world', name='friction_calibration')
    physics = _element(world, 'physics', name='physics', type='ignored')
    _element(physics, 'max_step_size', '.0025')
    _element(physics, 'real_time_factor', real_time_factor)
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')]:
        plugin = _element(world, 'plugin', filename=f'gz-sim-{filename}-system',
                          name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            _element(plugin, 'render_engine', 'ogre2')
    _element(world, 'gravity', '0 0 -9.81')
    scene = _element(world, 'scene')
    _element(scene, 'ambient', '.75 .75 .75 1')
    _element(scene, 'background', '.88 .91 .94 1')
    light = _element(world, 'light', name='sun', type='directional')
    _element(light, 'pose', '0 0 5 0 0 0')
    _element(light, 'diffuse', '.8 .8 .8 1')
    _element(light, 'direction', '-.4 .2 -1')
    floor = _element(world, 'model', name='floor')
    _element(floor, 'static', 'true')
    link = _element(floor, 'link', name='floor_link')
    collision = _element(link, 'collision', name='ground')
    _element(collision, 'pose', '0 0 -.02 0 0 0')
    plane = _element(_element(collision, 'geometry'), 'plane')
    _element(plane, 'normal', '0 0 1')
    _element(plane, 'size', '10 10')
    for i in range(-8, 9):
        for axis in (0, 1):
            visual = _element(link, 'visual', name=f'grid_{i}_{axis}')
            x, y = (i*.1, 0) if axis == 0 else (0, i*.1)
            _element(visual, 'pose', f'{x} {y} -.005 0 0 0')
            box = _element(_element(visual, 'geometry'), 'box')
            _element(box, 'size', '.002 1.6 .002' if axis == 0 else '1.6 .002 .002')
            material = _element(visual, 'material')
            _element(material, 'diffuse', '.55 .6 .65 1')
            _element(material, 'ambient', '.55 .6 .65 1')
    model = _element(world, 'model', name='review_camera')
    _element(model, 'static', 'true')
    _element(model, 'pose', '1.65 -2.2 1.35 0 .29 2.2143')
    camera = _element(_element(model, 'link', name='camera_link'), 'sensor',
                      name='review', type='camera')
    _element(camera, 'always_on', 'true')
    _element(camera, 'update_rate', video_fps)
    _element(camera, 'topic', '/review/image')
    optics = _element(camera, 'camera')
    _element(optics, 'horizontal_fov', '.72')
    image = _element(optics, 'image')
    for tag, value in [('width', 640), ('height', 480), ('format', 'R8G8B8')]:
        _element(image, tag, value)
    clip = _element(optics, 'clip')
    _element(clip, 'near', '.05')
    _element(clip, 'far', '20')
    return ET.tostring(sdf, encoding='unicode')


def _resolve_meshes(robot):
    for mesh in robot.findall('.//mesh'):
        uri = mesh.get('filename', '')
        if uri.startswith('package://'):
            from ament_index_python.packages import get_package_share_directory
            package, relative = uri[len('package://'):].split('/', 1)
            mesh.set('filename', str(Path(get_package_share_directory(package)) / relative))


def write_assets(output_dir, urdf_text, *, real_time_factor=10., video_fps=5.):
    """Write simulation-only controller, URDF, world and provenance files."""
    world = world_xml(real_time_factor, video_fps)
    robot = ET.fromstring(urdf_text)
    systems = robot.findall('ros2_control')
    if not systems or any(system.findtext('hardware/plugin') != 'gz_ros2_control/GazeboSimSystem'
                          for system in systems):
        raise ValueError('Gazebo assets require only GazeboSimSystem hardware plugins')
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    paths = {name: output / filename for name, filename in [
        ('world', 'gazebo_world.sdf'), ('urdf', 'gazebo_robot.urdf'),
        ('controllers', 'gazebo_controllers.yaml')]}
    _resolve_meshes(robot)
    for system in systems:
        system.set('is_async', 'false')
        for state in system.findall(".//state_interface[@name='position']"):
            initial = state.find("param[@name='initial_value']")
            if initial is None:
                initial = _element(state, 'param', name='initial_value')
            initial.text = '0.0'
    for gazebo in robot.findall('gazebo'):
        for sensor in gazebo.findall('sensor'):
            gazebo.remove(sensor)
    found_plugin = False
    for plugin in robot.findall('.//plugin'):
        if 'gz_ros2_control' in plugin.get('filename', ''):
            found_plugin = True
            parameters = plugin.find('parameters')
            if parameters is None:
                parameters = _element(plugin, 'parameters')
            parameters.text = str(paths['controllers'])
    if not found_plugin:
        raise ValueError('Missing Gazebo ros2_control simulator plugin')
    controllers = {
        'controller_manager': {'ros__parameters': {
            'update_rate': 400, 'use_sim_time': True,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'arm_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'}}},
        'arm_controller': {'ros__parameters': {
            'joints': [f'joint{i}' for i in range(1, 7)], 'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'], 'allow_partial_joints_goal': False,
            'use_sim_time': True}},
        'joint_state_broadcaster': {'ros__parameters': {'use_sim_time': True}}}
    paths['controllers'].write_text(yaml.safe_dump(controllers))
    paths['urdf'].write_text(ET.tostring(robot, encoding='unicode'))
    paths['world'].write_text(world)
    manifest = dict(simulator='Gazebo Harmonic', hardware_access=False,
                    real_time_factor=real_time_factor, video_fps=video_fps,
                    physics_step_s=.0025, initial_pose_rad=[0.]*6,
                    assets={key: dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                            for key, path in paths.items()})
    (output/'simulation_assets.json').write_text(json.dumps(manifest, indent=2))
    return paths


def simulation_actions(output_dir, urdf_text, *, real_time_factor=10., video_fps=5.):
    """Launch the real simulator and camera; collection uses the shared executable."""
    from launch.actions import ExecuteProcess, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch_ros.actions import Node
    paths = write_assets(output_dir, urdf_text, real_time_factor=real_time_factor, video_fps=video_fps)
    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', str(paths['world'])],
                            output='screen')
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': paths['urdf'].read_text(), 'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create', arguments=[
        '-file', str(paths['urdf']), '-name', 'omy', '-world', 'friction_calibration'])
    broadcaster = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'])
    arm = Node(package='controller_manager', executable='spawner',
               arguments=['arm_controller', '--controller-manager-timeout', '120'])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/review/image@sensor_msgs/msg/Image[gz.msgs.Image'])
    recorder = ExecuteProcess(cmd=[sys.executable, '-m',
        'open_manipulator_friction_diagnostic.video', '--output', str(Path(output_dir).resolve()),
        '--fps', str(video_fps)], output='screen', sigterm_timeout='120')
    return [RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[broadcaster])),
            RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=[arm])),
            gazebo, state, spawn, bridge, recorder]
