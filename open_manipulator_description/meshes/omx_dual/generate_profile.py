#!/usr/bin/env python3
"""Generate the visual-only 30 mm / slot-7 extrusion using the standard library.

Measured: 30 mm outside section and 7 mm slot mouths.
Illustrative: 11 mm slot chambers, 8 mm depth, 2 mm lips, 6 mm square
centre bore and 1 mm outside chamfers. This is not manufacturing CAD.
The mesh is 1 metre long along X; URDF scales X to the member length.
"""

import math
from pathlib import Path
import struct


def section_polygons():
    """Tile the material with convex polygons, in millimetres, CCW in Y/Z."""
    grid = [-15, -14, -13, -7, -5.5, -3.5, -3,
            3, 3.5, 5.5, 7, 13, 14, 15]
    polygons = []
    for low_y, high_y in zip(grid, grid[1:]):
        for low_z, high_z in zip(grid, grid[1:]):
            y, z = (low_y + high_y) / 2, (low_z + high_z) / 2
            hole = abs(y) < 3 and abs(z) < 3
            for across, outward in [(y, z), (-z, y), (-y, -z), (z, -y)]:
                hole |= (outward > 13 and abs(across) < 3.5)
                hole |= (7 < outward < 13 and abs(across) < 5.5)
            if hole:
                continue
            polygon = [(low_y, low_z), (high_y, low_z),
                       (high_y, high_z), (low_y, high_z)]
            # Clip the four outer corners with a 1 mm, 45-degree chamfer.
            for sy, sz in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
                clipped = []
                for a, b in zip(polygon, polygon[1:] + polygon[:1]):
                    da = sy * a[0] + sz * a[1] - 29
                    db = sy * b[0] + sz * b[1] - 29
                    if da <= 0:
                        clipped.append(a)
                    if (da < 0 < db) or (db < 0 < da):
                        t = da / (da - db)
                        clipped.append(tuple(a[k] + t * (b[k] - a[k]) for k in (0, 1)))
                polygon = clipped
            if len(polygon) >= 3:
                polygons.append(polygon)
    return polygons


def mesh_triangles():
    triangles = []
    boundary = {}

    def vertex(x, yz):
        return (x, yz[0] / 1000, yz[1] / 1000)

    for polygon in section_polygons():
        for i in range(1, len(polygon) - 1):
            a, b, c = polygon[0], polygon[i], polygon[i + 1]
            triangles.append(tuple(vertex(0.5, p) for p in (a, b, c)))
            triangles.append(tuple(vertex(-0.5, p) for p in (c, b, a)))
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            if (b, a) in boundary:
                del boundary[b, a]
            else:
                boundary[a, b] = True
    for a, b in boundary:
        a0, a1, b0, b1 = vertex(-0.5, a), vertex(0.5, a), vertex(-0.5, b), vertex(0.5, b)
        triangles.extend([(a0, b0, b1), (a0, b1, a1)])
    return triangles


def write_mesh(path):
    triangles = mesh_triangles()
    with path.open('wb') as output:
        output.write(b'Approximate 3030 slot-7 visual extrusion; metre units'.ljust(80, b' '))
        output.write(struct.pack('<I', len(triangles)))
        for a, b, c in triangles:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            normal = [u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0]]
            magnitude = math.sqrt(sum(n*n for n in normal))
            normal = [n / magnitude for n in normal]
            output.write(struct.pack('<12fH', *normal, *a, *b, *c, 0))
    print(f'{path.name}: {len(triangles)} triangles')


if __name__ == '__main__':
    write_mesh(Path(__file__).with_name('profile_3030_slot7.stl'))
