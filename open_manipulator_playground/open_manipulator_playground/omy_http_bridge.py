#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# HTTP -> ROS 2 bridge for Reachy Mini teleoperation.
#
# Reachy has no ROS 2, so it POSTs hand-tracking commands over HTTP. This node
# runs an HTTP server (default 0.0.0.0:8000) and republishes each command onto
# /omy/task_goal (std_msgs/String JSON), which omy_teleop_executor consumes. No
# existing OMY nodes are modified.
#
# Reachy sender contract honored here:
#   - POST /omy/task, Content-Type: application/json
#   - body: {"task": str, "source": "reachy_mini", "parameters": {...}}
#   - ALWAYS respond 2xx within a few seconds. Business-level rejection is
#     reported in the JSON body ({"ok": false, ...}), NOT via 4xx/5xx, because
#     the sender treats any non-2xx as a ConnectionError.
#   - bind 0.0.0.0 so a remote host (Reachy) can reach it.

import json
import threading
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OmyHttpBridge(Node):

    def __init__(self):
        super().__init__('omy_http_bridge')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('path', '/omy/task')
        self.declare_parameter('task_goal_topic', '/omy/task_goal')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.path = self.get_parameter('path').get_parameter_value().string_value
        topic = self.get_parameter('task_goal_topic').get_parameter_value().string_value

        self.publisher = self.create_publisher(String, topic, 10)

        self._httpd = ThreadingHTTPServer((self.host, self.port), self._make_handler())
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'OmyHttpBridge listening http://{self.host}:{self.port}{self.path} '
            f'-> publishing {topic}')

    def publish_task_goal(self, task: str, source: str, parameters: dict):
        payload = json.dumps(
            {'task': task, 'source': source or 'reachy_mini', 'parameters': parameters},
            ensure_ascii=False)
        self.publisher.publish(String(data=payload))
        self.get_logger().info(f'-> task_goal: {payload}')

    def handle_payload(self, body: dict):
        """Process one decoded JSON body. Returns a response dict; HTTP status is
        always 200 (sender requires 2xx; rejection lives in the body)."""
        task = body.get('task')
        if not isinstance(task, str) or not task:
            return {'ok': False, 'reason': 'missing task'}
        parameters = body.get('parameters', {})
        if not isinstance(parameters, dict):
            return {'ok': False, 'reason': 'parameters must be an object'}
        source = body.get('source', 'reachy_mini')
        self.publish_task_goal(task, source, parameters)
        return {'ok': True, 'task': task}

    def _make_handler(self):
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                bridge.get_logger().debug(fmt % args)

            def _send_json(self, status, obj):
                data = json.dumps(obj).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path.rstrip('/') in ('', '/health', bridge.path.rstrip('/')):
                    self._send_json(200, {'ok': True, 'service': 'omy_http_bridge'})
                else:
                    self._send_json(404, {'ok': False, 'error': 'not found'})

            def do_POST(self):
                if self.path.rstrip('/') != bridge.path.rstrip('/'):
                    self._send_json(404, {'ok': False, 'error': f'unknown path {self.path}'})
                    return
                length = int(self.headers.get('Content-Length', 0))
                raw = self.rfile.read(length) if length > 0 else b''
                try:
                    body = json.loads(raw) if raw else {}
                except json.JSONDecodeError as exc:
                    # Bad JSON is a client error; 200 with ok:false keeps the
                    # sender from raising ConnectionError, and the body explains.
                    self._send_json(200, {'ok': False, 'reason': f'invalid JSON: {exc}'})
                    return
                if not isinstance(body, dict):
                    self._send_json(200, {'ok': False, 'reason': 'body must be an object'})
                    return
                try:
                    self._send_json(200, bridge.handle_payload(body))
                except Exception as exc:  # noqa: BLE001
                    bridge.get_logger().error(f'handler error: {exc}')
                    # Still 200 so the sender logs it cleanly; reason in body.
                    self._send_json(200, {'ok': False, 'reason': str(exc)})

        return Handler

    def destroy_node(self):
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmyHttpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
