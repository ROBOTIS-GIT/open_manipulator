"""ROS-independent ordering for OMY power, command priming and brake transitions."""
import math
import xml.etree.ElementTree as ET


READBACK_ITEMS = (
    'Present Position', 'Present Velocity', 'Present Current', 'Goal Current',
    'Goal Position', 'Torque Enable', 'Controller State', 'Error Code',
    'Realtime Tick',
)
# The installed OMY rejected Operating Mode(33) as an indirect-read target.
# Read settings through DHI's ordinary item service before any torque request;
# do not treat a URDF parameter as observed hardware state.
CONFIGURATION_ITEMS = ('Operating Mode', 'Bus Watchdog', 'Goal Update Delay', 'Brake Delay')
SERVICES = {
    'get_dynamixel_data_srv_name': 'get_dxl_data',
    'set_dynamixel_data_srv_name': 'set_dxl_data',
    'reboot_dxl_srv_name': 'reboot_dxl',
    'set_dxl_torque_srv_name': 'set_dxl_torque',
}


def _parameter(parent, name, value):
    matches = parent.findall(f"param[@name='{name}']")
    item = matches[0] if matches else ET.SubElement(parent, 'param', name=name)
    item.text = str(value)
    for duplicate in matches[1:]:
        parent.remove(duplicate)


def _command_interface(parent, name):
    matches = parent.findall('command_interface')
    item = matches[0] if matches else ET.SubElement(parent, 'command_interface')
    item.clear()
    item.set('name', name)
    for duplicate in matches[1:]:
        parent.remove(duplicate)


def _current_joints(mode, probe_joint):
    if mode not in ('current', 'position', 'probe'):
        raise ValueError('mode must be current, position or probe')
    if mode == 'probe' and probe_joint not in range(1, 7):
        raise ValueError('probe mode requires probe_joint in 1..6')
    return tuple(range(1, 7)) if mode == 'current' else (probe_joint,) if mode == 'probe' else ()


def prepare_hardware_urdf(urdf: str, mode: str, *, probe_joint=None) -> str:
    """Keep DHI power/initialization; use its package-local read-service fix.

    `Torque Enable=0` selects no motors for DHI's automatic activation. Later
    SetDataToDxl torque writes update its cache, so normal driver cleanup works.
    Current writes still run while torque is OFF. Firmware limits/timing remain
    as supplied; position mode's Goal Current is a limit and must be preserved.
    """
    current_joints = _current_joints(mode, probe_joint)
    root = ET.fromstring(urdf)
    arms = []
    for system in root.findall('ros2_control'):
        hardware = system.find('hardware')
        if hardware is None:
            raise ValueError('ros2_control hardware is missing')
        plugin = hardware.findtext('plugin')
        if plugin not in ('dynamixel_hardware_interface/DynamixelHardware',
                          'open_manipulator_friction_diagnostic/GroupedDynamixelHardware',
                          'mock_components/GenericSystem'):
            raise ValueError(f'Unsupported hardware plugin: {plugin}')
        namespace = system.get('name', '').lower()
        if not namespace:
            raise ValueError('Hardware system name is required')
        for name, suffix in SERVICES.items():
            _parameter(hardware, name, namespace + '/' + suffix)
        _parameter(hardware, 'dynamixel_state_pub_msg_name', namespace + '/dxl_state')
        for obsolete in ('defer_torque_enable_at_start', 'use_grouped_sync_write'):
            for item in hardware.findall(f"param[@name='{obsolete}']"):
                hardware.remove(item)
        for gpio in system.findall('gpio'):
            if gpio.findtext("param[@name='type']") in ('dxl', 'virtual_dxl'):
                _parameter(gpio, 'Torque Enable', 0)
        if plugin != 'mock_components/GenericSystem':
            # The end-unit shares the controller process. Its per-cycle executor
            # construction also affects scheduling even when the gripper is OFF.
            hardware.find('plugin').text = 'open_manipulator_friction_diagnostic/GroupedDynamixelHardware'
        if system.find("joint[@name='joint6']") is None:
            continue
        arms.append(system)
        if plugin != 'mock_components/GenericSystem':
            if mode == 'probe':
                _parameter(hardware, 'use_grouped_sync_write', 'true')
        for j in range(1, 7):
            joint = system.find(f"joint[@name='joint{j}']")
            gpio = system.find(f"gpio[@name='dxl{j}']")
            if joint is None or gpio is None:
                raise ValueError(f'Missing arm joint/drive {j}')
            current = j in current_joints
            _command_interface(joint, 'effort' if current else 'position')
            _command_interface(gpio, 'Goal Current' if current else 'Goal Position')
            _parameter(gpio, 'Torque Enable', 0)
            _parameter(gpio, 'Operating Mode', 0 if current else 3)
            if current:
                _parameter(gpio, 'Goal Current', 0)
            for item in list(gpio.findall('state_interface')):
                if item.get('name') in CONFIGURATION_ITEMS:
                    gpio.remove(item)
            # Identical read fields keep all six motors on the same sync-read layout.
            existing = {item.get('name') for item in gpio.findall('state_interface')}
            for name in READBACK_ITEMS:
                if name not in existing:
                    ET.SubElement(gpio, 'state_interface', name=name)
    if len(arms) != 1:
        raise ValueError('Exactly one six-axis OMY arm is required')
    return ET.tostring(root, encoding='unicode')


class DriveLifecycle:
    """Produce torque requests; only observed drive states establish completion.

    observe() consumes periodic GPIO values. Realtime Tick must advance to mark
    data fresh: a broadcaster can keep publishing old hardware values after a
    read failure. requested() records an attempted write, including a lost reply.
    """

    def __init__(self, mode, *, started_at, probe_joint=None, freshness=.25, startup_timeout=15., shutdown_timeout=8., defer_enable=False):
        self.current_joints = _current_joints(mode, probe_joint)
        if defer_enable and mode != 'position':
            raise ValueError('Deferred enable is only supported for position collection')
        self.defer_enable = defer_enable
        self.enable_requested = not defer_enable
        self.mode = mode
        self.phase = 'priming'
        self.reason = ''
        self.started_at = started_at
        self.freshness = freshness
        self.startup_timeout = startup_timeout
        self.shutdown_timeout = shutdown_timeout
        self.stop_at = None
        self.axes = {}
        self._ticks = {}
        self._seen = {}
        self._sequence = {}
        self._requests = {}
        self._commands = {}
        self._command_at = -math.inf
        self.startup = {}
        self.configuration = {}

    @property
    def accept_external(self):
        return self.phase == 'running'

    @property
    def safe_to_exit(self):
        return self.phase == 'stopped'

    def observe(self, values, now):
        self.axes = {j: {**self.configuration.get(j, {}), **values[f'dxl{j}']}
                     for j in range(1, 7) if f'dxl{j}' in values}
        for j, row in self.axes.items():
            tick = row.get('Realtime Tick', math.nan)
            if not math.isfinite(tick):
                continue
            previous = self._ticks.get(j)
            if previous is not None and previous != tick:
                self._seen[j] = now
                self._sequence[j] = self._sequence.get(j, 0) + 1
            self._ticks[j] = tick

    def configure_readback(self, values):
        """Record startup service readback separately from fresh cyclic telemetry."""
        if self.phase != 'priming' or set(values) != set(range(1, 7)):
            raise ValueError('Configuration readback requires all six drives before enabling')
        for j, row in values.items():
            if (not all(math.isfinite(row.get(key, math.nan)) for key in CONFIGURATION_ITEMS)
                    or row['Operating Mode'] != (0 if j in self.current_joints else 3)
                    or not 0 <= row['Bus Watchdog'] <= 127
                    or row['Goal Update Delay'] < 0 or row['Brake Delay'] < 0):
                raise ValueError(f'Drive {j} returned invalid startup configuration')
        self.configuration = {j: {key: row[key] for key in CONFIGURATION_ITEMS}
                              for j, row in values.items()}

    def command(self, efforts, now):
        self._commands = dict(efforts)
        self._command_at = now

    def requested(self, joint, enabled, now):
        self._requests[joint] = (enabled, now, self._sequence.get(joint, 0))

    def request_stop(self, now, reason='Stop requested'):
        if self.stop_at is None:
            self.stop_at = now
            self.reason = reason
            self.phase = 'stopping'

    def request_position_start(self, now):
        if (not self.defer_enable or self.phase != 'awaiting_position_start' or
                self.stop_at is not None or not self._primed(now)):
            raise ValueError('Position start rejected: all six drives must remain freshly primed and OFF')
        self.enable_requested = True
        self.started_at = now
        self.phase = 'priming'

    def _fresh(self, j, now):
        return j in self.axes and 0 <= now - self._seen.get(j, -math.inf) <= self.freshness

    def _after_request(self, j):
        request = self._requests.get(j)
        # Two subsequent drive ticks avoid accepting a read from just before the
        # queued write was drained by the next hardware write() call.
        return request is None or self._sequence.get(j, 0) >= request[2] + 2

    def _valid(self, j):
        row = self.axes[j]
        required = ('Torque Enable', 'Controller State', 'Error Code', 'Operating Mode',
                    'Bus Watchdog', 'Goal Current', 'Goal Position', 'Present Position')
        return (all(math.isfinite(row.get(key, math.nan)) for key in required)
                and row['Error Code'] == 0 and 0 <= row['Bus Watchdog'] <= 127
                and row['Operating Mode'] == (0 if j in self.current_joints else 3))

    def _primed(self, now):
        if not all(self._fresh(j, now) and self._valid(j) for j in range(1, 7)):
            return False
        if not all(self.axes[j]['Torque Enable'] == 0 and
                   self.axes[j]['Controller State'] in (2, 3) for j in range(1, 7)):
            return False
        if not all(abs(self.axes[j]['Goal Position'] - self.axes[j]['Present Position'])
                   <= 2 * .000011984224905356572 for j in range(1, 7) if j not in self.current_joints):
            return False
        if not self.current_joints:
            return True
        if not 0 <= now - self._command_at <= self.freshness:
            return False
        for j in self.current_joints:
            command = self._commands.get(f'joint{j}', math.nan)
            unit = .07692307692 if j <= 2 else .04
            if not math.isfinite(command) or abs(self.axes[j]['Goal Current'] - command) > unit + 1e-9:
                return False
        return True

    def step(self, now):
        """Return (joint id, torque enabled) requests; never performs I/O."""
        if self.safe_to_exit:
            return ()
        if self.stop_at is not None:
            if now - self.stop_at >= self.shutdown_timeout:
                self.phase = 'shutdown_unconfirmed'
            complete = all(self._fresh(j, now) and self._after_request(j) and
                           self.axes[j].get('Torque Enable') == 0 and
                           self.axes[j].get('Controller State') in (2, 3) for j in range(1, 7))
            if complete:
                self.phase = 'stopped'
                return ()
            requests = []
            for j in range(1, 7):
                if not self._fresh(j, now) or not self._after_request(j):
                    continue
                row = self.axes[j]
                if row.get('Torque Enable') == 0 and row.get('Controller State') in (2, 3):
                    continue
                if row.get('Controller State') not in (2, 3, 5, 8):
                    continue
                previous = self._requests.get(j)
                if previous is None or previous[0] or now - previous[1] >= .5:
                    requests.append((j, False))
            return tuple(requests)
        if self.phase == 'awaiting_position_start':
            if (any(self._fresh(j, now) and (not self._valid(j) or
                    self.axes[j]['Torque Enable'] != 0) for j in range(1, 7)) or
                    not all(self._fresh(j, now) for j in range(1, 7))):
                self.request_stop(now, 'Drive readback lost or fault while preparing position start')
                return self.step(now)
            return ()
        if self.phase == 'priming':
            if self._primed(now):
                if not self.enable_requested:
                    self.phase = 'awaiting_position_start'
                    return ()
                self.startup = {
                    'command_primed_before_enable': True,
                    'physical_current_support_during_startup_verified': False,
                    'firmware_timing_modified': False,
                    'note': 'A preloaded goal does not prove current support during firmware brake/goal delay.',
                    'axes': {j: {
                        'goal_update_delay_ms': self.axes[j].get('Goal Update Delay'),
                        'brake_delay_ms': self.axes[j].get('Brake Delay'),
                        'goal_current_nm': self.axes[j].get('Goal Current'),
                        'goal_position_rad': self.axes[j].get('Goal Position'),
                        'operating_mode': self.axes[j].get('Operating Mode'),
                    } for j in range(1, 7)},
                }
                self.phase = 'enabling'
                return tuple((j, True) for j in range(1, 7))
        elif self.phase == 'enabling':
            if any(self._fresh(j, now) and (not self._valid(j) or
                   self.axes[j].get('Controller State') not in (2, 3, 4, 5)) for j in range(1, 7)):
                self.request_stop(now, 'Drive fault during torque enable')
                return self.step(now)
            if all(self._fresh(j, now) and self._valid(j) and self._after_request(j) and
                   j in self._requests and self.axes[j]['Torque Enable'] == 1 and
                   self.axes[j]['Controller State'] == 5 for j in range(1, 7)):
                self.phase = 'running'
        elif self.phase == 'running':
            command_fresh = self.mode == 'position' or 0 <= now - self._command_at <= self.freshness
            if not command_fresh or not all(self._fresh(j, now) and self._valid(j) and
                       self.axes[j]['Torque Enable'] == 1 and
                       self.axes[j]['Controller State'] == 5 for j in range(1, 7)):
                self.request_stop(now, 'Drive readback lost or drive left RUNNING')
                return self.step(now)
        if self.phase in ('priming', 'enabling') and now - self.started_at >= self.startup_timeout:
            self.request_stop(now, 'Drive startup timed out')
            return self.step(now)
        return ()

    def status(self):
        return {'phase': self.phase, 'reason': self.reason,
                'all_off_confirmed': self.safe_to_exit,
                'accept_external_effort': self.accept_external,
                'axes': self.axes, 'startup': self.startup,
                'configuration_readback': self.configuration}
