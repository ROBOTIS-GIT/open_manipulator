"""Prevent a mock or a second trial from sharing a live controller host context."""
import subprocess


def require_no_controller(process_names=None):
    # Linux comm is truncated to 15 characters. Check real processes, not the
    # ROS graph: a different RMW/domain still shares CPU and restoration checks.
    if process_names is None:
        process_names = subprocess.check_output(['ps', '-eo', 'comm='], text=True)
    if any(name.strip() in ('ros2_control_no', 'ros2_control_node')
           for name in process_names.splitlines()):
        raise RuntimeError('A controller manager is already running. Do not run '
                           'a mock or another trial on this host while it is active.')
