from types import SimpleNamespace

import pytest

from open_manipulator_bringup.hardware_error_watchdog import DockerRestartClient
from open_manipulator_bringup.hardware_error_watchdog import torque_disabled_motors
from open_manipulator_bringup.hardware_error_watchdog import torque_enable_states


def dynamic_joint_state(joint_names, interface_values):
    return SimpleNamespace(joint_names=joint_names, interface_values=interface_values)


def interface_value(interface_names, values):
    return SimpleNamespace(interface_names=interface_names, values=values)


def test_ignores_messages_without_torque_enable():
    msg = dynamic_joint_state(
        ['joint1'],
        [interface_value(['position', 'velocity'], [1.0, 0.0])],
    )

    assert torque_enable_states(msg) == []
    assert torque_disabled_motors(msg) == []


def test_ignores_torque_enabled_motor():
    msg = dynamic_joint_state(
        ['dxl11'],
        [
            interface_value(
                ['Present Position', 'Torque Enable'],
                [0.0, 1.0],
            )
        ],
    )

    assert torque_enable_states(msg) == [('dxl11', 1)]
    assert torque_disabled_motors(msg) == []


def test_detects_torque_disabled_motor():
    msg = dynamic_joint_state(
        ['dxl11'],
        [
            interface_value(
                ['Present Position', 'Torque Enable'],
                [0.0, 0.0],
            )
        ],
    )

    assert torque_disabled_motors(msg) == [('dxl11', 0)]


def test_detects_only_torque_disabled_motors():
    msg = dynamic_joint_state(
        ['dxl11', 'dxl12'],
        [
            interface_value(
                ['Present Position', 'Torque Enable'],
                [0.0, 1.0],
            ),
            interface_value(
                ['Present Position', 'Torque Enable'],
                [0.0, 0.0],
            ),
        ],
    )

    assert torque_disabled_motors(msg) == [('dxl12', 0)]


class FakeSocket:
    def __init__(self, response):
        self.response = response
        self.request = b''
        self.connected_path = None
        self.timeout = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        return False

    def settimeout(self, timeout):
        self.timeout = timeout

    def connect(self, path):
        self.connected_path = path

    def sendall(self, request):
        self.request += request

    def recv(self, _size):
        response = self.response
        self.response = b''
        return response


def test_docker_restart_client_posts_restart_request(monkeypatch):
    fake_socket = FakeSocket(b'HTTP/1.1 204 No Content\r\n\r\n')
    monkeypatch.setattr(
        'open_manipulator_bringup.hardware_error_watchdog.socket.socket',
        lambda *_args: fake_socket,
    )

    client = DockerRestartClient(
        socket_path='/tmp/docker.sock',
        container_name='open_manipulator',
        restart_timeout_sec=5,
    )

    assert client.restart_container()
    assert fake_socket.connected_path == '/tmp/docker.sock'
    assert b'POST /containers/open_manipulator/restart?t=5 HTTP/1.1' in fake_socket.request


def test_docker_restart_client_raises_on_non_success(monkeypatch):
    fake_socket = FakeSocket(b'HTTP/1.1 500 Internal Server Error\r\n\r\nboom')
    monkeypatch.setattr(
        'open_manipulator_bringup.hardware_error_watchdog.socket.socket',
        lambda *_args: fake_socket,
    )

    client = DockerRestartClient()

    with pytest.raises(RuntimeError, match='HTTP 500'):
        client.restart_container()
