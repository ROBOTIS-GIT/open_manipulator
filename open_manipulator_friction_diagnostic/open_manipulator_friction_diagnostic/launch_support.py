"""Small launch assembly; compensation startup only loads ready parameters."""
from copy import deepcopy
from contextlib import contextmanager
from pathlib import Path
import tempfile
import json
import shutil
import yaml

from .artifact import JOINTS, controller_parameters


def boolean(value):
    if str(value).lower() not in ('true', 'false'):
        raise ValueError('Use true or false')
    return str(value).lower() == 'true'


def controller_document(mode, calibration=None, settings=None, *, hardware=True):
    name = 'friction_compensation_controller' if mode == 'current' else 'arm_controller'
    kind = ('open_manipulator_friction_diagnostic/FrictionCompensationController'
            if mode == 'current' else 'joint_trajectory_controller/JointTrajectoryController')
    result = {'controller_manager': {'ros__parameters': {
        'update_rate': 400,
        'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
        name: {'type': kind}}}}
    if mode == 'current':
        user = deepcopy(settings['friction_compensation_controller']['ros__parameters'])
        parameters = controller_parameters(calibration, user.pop('compensation_ratios'),
                                           user.pop('dithering_enabled'), hardware=hardware)
        if 'external_effort_timeout_s' in user:
            raise ValueError('Remove external_effort_timeout_s: external effort now lasts one control cycle only')
        if user:
            raise ValueError('Unknown controller setting in controllers.yaml')
        parameters.update(user, require_motor_ready=True)
    elif mode == 'position':
        parameters = dict(joints=JOINTS, command_interfaces=['position'],
                          state_interfaces=['position', 'velocity'],
                          allow_partial_joints_goal=False)
    else:
        raise ValueError('Unknown controller mode')
    result[name] = {'ros__parameters': parameters}
    return result


def runtime_action(mode, urdf, controllers, output, *, mock=False, defer_position_enable=False):
    from launch_ros.actions import Node
    output = Path(output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    description = output/'input.urdf'
    description.write_text(urdf)
    config = output/'controllers.yaml'
    config.write_text(yaml.safe_dump(controllers))
    return Node(package='open_manipulator_friction_diagnostic', executable='friction_runtime',
        arguments=['--mode', mode, '--urdf', str(description), '--controllers', str(config),
                   '--output', str(output)] + (['--mock'] if mock else []) +
                   (['--defer-position-enable'] if defer_position_enable else []),
        output='screen', sigterm_timeout='45', sigkill_timeout='10')


def runtime_directory():
    return Path(tempfile.mkdtemp(prefix='omy-friction-runtime-'))


def prepare_collection_directory(path, calibration_file):
    """Reuse one run name; rotate only a directory created by this package."""
    path = Path(path).expanduser().resolve()
    artifact = Path(calibration_file).expanduser().resolve()
    if artifact == path or path in artifact.parents:
        raise ValueError('Keep calibration_file outside the replaceable raw collection directory')
    marker = 'collection_owner.json'
    owner = {'package': 'open_manipulator_friction_diagnostic', 'schema': 1}
    if path.exists() and any(path.iterdir()):
        if not (path/marker).is_file() or json.loads((path/marker).read_text()) != owner:
            raise ValueError('Output directory is not an owned previous collection; choose another output')
        runtime = path/'runtime/runtime.json'
        if runtime.is_file():
            state = json.loads(runtime.read_text())
            if state.get('motor_enable_requested') and not state.get('all_off_confirmed'):
                raise ValueError('Previous collection has no confirmed motor OFF; resolve its runtime first')
        previous = path.with_name(path.name+'.previous')
        if previous.exists():
            if not (previous/marker).is_file() or json.loads((previous/marker).read_text()) != owner:
                raise ValueError('Refusing to replace an unowned previous collection directory')
            shutil.rmtree(previous)
        path.rename(previous)
    path.mkdir(parents=True, exist_ok=True)
    (path/marker).write_text(json.dumps(owner))


@contextmanager
def staged_collection_directory(path, calibration_file):
    """Keep both existing journals intact until all preparation has succeeded."""
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=path.name+'.preparing-', dir=path.parent))
    try:
        yield stage
        # Ownership and confirmed-OFF checks still apply at promotion time.
        prepare_collection_directory(path, calibration_file)
        (stage/'collection_owner.json').write_bytes((path/'collection_owner.json').read_bytes())
        (path/'collection_owner.json').unlink()
        path.rmdir()
        stage.rename(path)
    finally:
        if stage.exists():
            shutil.rmtree(stage)
