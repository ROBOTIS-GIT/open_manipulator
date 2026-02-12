#!/usr/bin/env python3
# Copyright 2024 ROBOTIS CO., LTD.
# SPDX-FileCopyrightText: 2024 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
from typing import List, Tuple


def plane_embed(
    plane: str,
    cx: float,
    cy: float,
    cz: float,
    x2d: float,
    y2d: float,
) -> List[float]:
    plane = (plane or 'xy').lower()
    if plane == 'xy':
        return [cx + x2d, cy + y2d, cz]
    if plane == 'xz':
        return [cx + x2d, cy, cz + y2d]
    if plane == 'yz':
        return [cx, cy + x2d, cz + y2d]
    return [cx + x2d, cy + y2d, cz]


def circle_point(
    t: float,
    center: List[float],
    radius: float,
    hz: float,
    plane: str,
    phase_deg: float = 0.0,
) -> List[float]:
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    phase = float(phase_deg) * math.pi / 180.0
    w = 2.0 * math.pi * float(hz)
    ct = math.cos(w * t + phase)
    st = math.sin(w * t + phase)
    return plane_embed(plane, cx, cy, cz, float(radius) * ct, float(radius) * st)


def heart_point(
    t: float,
    center: List[float],
    scale: float,
    hz: float,
    plane: str,
    phase_deg: float = 0.0,
) -> List[float]:
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    phase = float(phase_deg) * math.pi / 180.0
    w = 2.0 * math.pi * float(hz)
    th = w * t + phase

    x = 16.0 * (math.sin(th) ** 3)
    y = (
        13.0 * math.cos(th)
        - 5.0 * math.cos(2.0 * th)
        - 2.0 * math.cos(3.0 * th)
        - math.cos(4.0 * th)
    )

    x = (x / 18.0) * float(scale)
    y = (y / 18.0) * float(scale)

    return plane_embed(plane, cx, cy, cz, x, y)


def rounded_rect_point(
    u: float,
    width: float,
    height: float,
    corner_r: float,
) -> Tuple[float, float]:
    u = float(u) % 1.0
    w = max(float(width), 1e-9)
    h = max(float(height), 1e-9)
    a = 0.5 * w
    b = 0.5 * h

    r = max(0.0, float(corner_r))
    r = min(r, a - 1e-9, b - 1e-9)

    if r < 1e-6:
        perim = 2.0 * w + 2.0 * h
        s = u * perim
        if s < w:
            return (-a + s, -b)
        s -= w
        if s < h:
            return (a, -b + s)
        s -= h
        if s < w:
            return (a - s, b)
        s -= w
        return (-a, b - s)

    lx = max(w - 2.0 * r, 0.0)
    ly = max(h - 2.0 * r, 0.0)
    perim = 2.0 * (lx + ly) + 2.0 * math.pi * r
    s = u * perim
    arc_l = 0.5 * math.pi * r

    if s < lx:
        return ((-a + r) + s, -b)
    s -= lx

    if s < arc_l:
        ang = (-0.5 * math.pi) + (s / r)
        ccx, ccy = (a - r), (-b + r)
        return (ccx + r * math.cos(ang), ccy + r * math.sin(ang))
    s -= arc_l

    if s < ly:
        return (a, (-b + r) + s)
    s -= ly

    if s < arc_l:
        ang = 0.0 + (s / r)
        ccx, ccy = (a - r), (b - r)
        return (ccx + r * math.cos(ang), ccy + r * math.sin(ang))
    s -= arc_l

    if s < lx:
        return ((a - r) - s, b)
    s -= lx

    if s < arc_l:
        ang = 0.5 * math.pi + (s / r)
        ccx, ccy = (-a + r), (b - r)
        return (ccx + r * math.cos(ang), ccy + r * math.sin(ang))
    s -= arc_l

    if s < ly:
        return (-a, (b - r) - s)
    s -= ly

    ang = math.pi + (s / r)
    ccx, ccy = (-a + r), (-b + r)
    return (ccx + r * math.cos(ang), ccy + r * math.sin(ang))


def rectangle_point(
    t: float,
    center: List[float],
    width: float,
    height: float,
    corner_r: float,
    hz: float,
    plane: str,
    phase_deg: float = 0.0,
) -> List[float]:
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    f = float(hz)
    phase = float(phase_deg) * math.pi / 180.0

    if f <= 1e-9:
        u = 0.0
    else:
        theta = 2.0 * math.pi * f * t + phase
        u = (theta / (2.0 * math.pi)) % 1.0

    x2d, y2d = rounded_rect_point(u, width, height, corner_r)
    return plane_embed(plane, cx, cy, cz, x2d, y2d)
