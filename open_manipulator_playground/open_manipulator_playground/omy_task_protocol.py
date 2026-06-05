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
# Task protocol for OMY teleoperation driven by Reachy Mini.
#
# Matches the Reachy sender contract exactly:
#   payload = {"task": <str>, "source": "reachy_mini", "parameters": {...}}
#
# Teleop lifecycle (sender-enforced state machine):
#   ready -> teleop_start -> (armed) -> teleop_delta ... -> teleop_hold (disarm)
#         -> teleop_start (re-arm) -> teleop_delta ... -> teleop_stop (disarm)
#
# Contract rules the OMY receiver must honor:
#   - teleop_delta received while NOT armed must be rejected/ignored.
#   - after teleop_hold, OMY disarms; a new teleop_start is required to re-arm.
#   - if deltas stop for >1 s, OMY must auto-hold (stop), not keep moving.

import json
from dataclasses import dataclass
from dataclasses import field
from typing import Optional


# Tasks used by the Reachy teleop sender, plus a few predefined-pose tasks.
SUPPORTED_TASKS = {
    'ready',
    'teleop_start',
    'teleop_delta',
    'teleop_hold',
    'teleop_stop',
    'stop',
    'home',
    'open_gripper',
    'close_gripper',
}

GRIPPER_OPEN_VALUES = {'open'}
GRIPPER_CLOSE_VALUES = {'close', 'closed'}


@dataclass
class TeleopDeltaParams:
    # Cartesian increments in METERS (per the Reachy contract). NOT absolute.
    dx: float = 0.0   # + = forward
    dy: float = 0.0   # + = right
    dz: float = 0.0   # + = up
    gripper: Optional[str] = None   # "open" | "close" | "none"/None


@dataclass
class TaskGoal:
    task: str
    source: str = ''
    parameters: dict = field(default_factory=dict)
    raw: dict = field(default_factory=dict)


@dataclass
class ParseResult:
    ok: bool
    goal: Optional[TaskGoal] = None
    error: str = ''


def parse_task_goal(json_str: str) -> ParseResult:
    """Parse a JSON string into a TaskGoal. ok=False with error on any problem."""
    try:
        data = json.loads(json_str)
    except json.JSONDecodeError as exc:
        return ParseResult(ok=False, error=f'Invalid JSON: {exc}')

    if not isinstance(data, dict):
        return ParseResult(ok=False, error='Task goal must be a JSON object')

    task = data.get('task')
    if not isinstance(task, str) or not task:
        return ParseResult(ok=False, error='task field must be a non-empty string')

    if task not in SUPPORTED_TASKS:
        return ParseResult(ok=False, error=f'Unknown task: {task!r}')

    parameters = data.get('parameters', {})
    if not isinstance(parameters, dict):
        return ParseResult(ok=False, error='parameters must be a JSON object')

    return ParseResult(ok=True, goal=TaskGoal(
        task=task,
        source=str(data.get('source', '')),
        parameters=parameters,
        raw=data,
    ))


def parse_teleop_delta(parameters: dict):
    """
    Extract and validate teleop_delta parameters.
    Returns (ok, TeleopDeltaParams|None, error_message).
    """
    try:
        dx = float(parameters.get('dx', 0.0))
        dy = float(parameters.get('dy', 0.0))
        dz = float(parameters.get('dz', 0.0))
    except (TypeError, ValueError) as exc:
        return False, None, f'dx/dy/dz must be numeric: {exc}'

    gripper_raw = parameters.get('gripper', None)
    gripper = gripper_raw.lower().strip() if isinstance(gripper_raw, str) else None

    return True, TeleopDeltaParams(dx=dx, dy=dy, dz=dz, gripper=gripper), ''


def classify_gripper_command(gripper: Optional[str]) -> Optional[str]:
    """Return 'open', 'close', or None (no gripper action)."""
    if gripper in GRIPPER_OPEN_VALUES:
        return 'open'
    if gripper in GRIPPER_CLOSE_VALUES:
        return 'close'
    return None
