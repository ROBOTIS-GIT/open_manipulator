#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# Periodically reads back OMY END's SyncTable Enable / SyncTable Enable HX5
# items and re-writes them to 1 if the firmware has dropped them back to 0.
#
# This is a workaround for the OMY END firmware self-disabling SyncTable
# Enable HX5 under heavy bus contention. Without this watchdog, the relay
# may become inactive at any time after launch and the HX5 hand stops
# responding to position commands.

import threading
from typing import List

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from dynamixel_interfaces.srv import GetDataFromDxl, SetDataToDxl


class SyncTableWatchdog(Node):
    def __init__(self) -> None:
        super().__init__('synctable_watchdog')

        self.declare_parameter('hub_id', 210)
        self.declare_parameter('items', ['SyncTable Enable', 'SyncTable Enable HX5'])
        self.declare_parameter('check_period_sec', 1.0)
        self.declare_parameter(
            'set_service',
            '/hand/dynamixel_hardware_interface/set_dxl_data',
        )
        self.declare_parameter(
            'get_service',
            '/hand/dynamixel_hardware_interface/get_dxl_data',
        )
        self.declare_parameter('get_timeout_sec', 1.0)
        self.declare_parameter('verbose_when_ok', False)

        self._hub_id: int = int(self.get_parameter('hub_id').value)
        self._items: List[str] = list(self.get_parameter('items').value)
        self._period: float = float(self.get_parameter('check_period_sec').value)
        set_srv = str(self.get_parameter('set_service').value)
        get_srv = str(self.get_parameter('get_service').value)
        self._get_timeout: float = float(self.get_parameter('get_timeout_sec').value)
        self._verbose_ok: bool = bool(self.get_parameter('verbose_when_ok').value)

        # Reentrant group + MultiThreadedExecutor lets the timer callback
        # block on a service future without deadlocking the executor.
        self._cb_group = ReentrantCallbackGroup()
        self._set_cli = self.create_client(
            SetDataToDxl, set_srv, callback_group=self._cb_group
        )
        self._get_cli = self.create_client(
            GetDataFromDxl, get_srv, callback_group=self._cb_group
        )

        self.get_logger().info(
            f'Watchdog: hub_id={self._hub_id} items={self._items} '
            f'period={self._period}s set={set_srv} get={get_srv}'
        )

        for cli, name in ((self._set_cli, set_srv), (self._get_cli, get_srv)):
            while rclpy.ok() and not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'Waiting for {name}...')

        self.create_timer(self._period, self._tick, callback_group=self._cb_group)

    def _wait_future(self, future, timeout_sec: float) -> bool:
        # rclpy.spin_until_future_complete cannot be called from inside a
        # callback because the executor is already spinning. Use an Event
        # that the future signals via its done callback instead. This
        # works because the service response is delivered on a different
        # thread of MultiThreadedExecutor.
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        return event.wait(timeout_sec)

    def _read_item(self, item: str):
        req = GetDataFromDxl.Request()
        req.id = self._hub_id
        req.item_name = item
        req.timeout_sec = self._get_timeout
        future = self._get_cli.call_async(req)
        if not self._wait_future(future, self._get_timeout + 1.0):
            return None
        resp = future.result()
        if resp is None or not getattr(resp, 'result', False):
            return None
        return int(getattr(resp, 'item_data', 0))

    def _write_item(self, item: str, value: int) -> bool:
        req = SetDataToDxl.Request()
        req.id = self._hub_id
        req.item_name = item
        req.item_data = value
        future = self._set_cli.call_async(req)
        if not self._wait_future(future, 2.0):
            return False
        resp = future.result()
        return bool(getattr(resp, 'result', False)) if resp is not None else False

    def _tick(self) -> None:
        for item in self._items:
            data = self._read_item(item)
            if data is None:
                self.get_logger().warn(f'{item}: readback timeout/failed')
                continue
            if data == 1:
                if self._verbose_ok:
                    self.get_logger().info(f'{item}: OK (1)')
                continue
            self.get_logger().warn(f'{item}: dropped to {data}, re-enabling')
            ok = self._write_item(item, 1)
            self.get_logger().info(
                f'{item}: re-enable {"queued" if ok else "FAILED"}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyncTableWatchdog()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
