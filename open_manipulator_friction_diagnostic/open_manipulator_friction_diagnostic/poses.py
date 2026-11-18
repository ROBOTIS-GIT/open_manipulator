"""URDF joint-axis and nominal gravity analysis, independent of robot naming conventions."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return transform


class Robot:
    def __init__(self, urdf, names):
        self.root = ET.fromstring(Path(urdf).read_text())
        self.names = names
        self.joints = list(self.root.findall('joint'))
        self.links = {link.get('name'): link for link in self.root.findall('link')}
        children = {j.find('child').get('link') for j in self.joints}
        self.base = next(name for name in self.links if name not in children)

    def gravity_batch(self, positions, chunk_size=2048):
        return self.static_load_batch(positions, chunk_size)['gravity_nm']

    def static_load_batch(self, positions, chunk_size=2048):
        """Unchanged URDF gravity plus downstream weight-wrench features.

        Radial force and bending moment are geometric proxies, not measured
        bearing/tooth loads. No payload/contact force is inferred.
        """
        """Vectorized fixed-URDF gravity for offline fixtures/coverage analysis."""
        result = []
        for begin in range(0, len(positions), chunk_size):
            q = np.asarray(positions[begin:begin+chunk_size]); n = len(q)
            tf = {self.base: np.broadcast_to(np.eye(4), (n, 4, 4))}
            ancestors = {self.base: []}; frames = {}; pending = self.joints.copy()
            while pending:
                progressed = False
                for joint in pending.copy():
                    parent, child = (joint.find(k).get('link') for k in ('parent', 'child'))
                    if parent not in tf:
                        continue
                    frame = tf[parent] @ origin(joint.find('origin'))
                    name = joint.get('name'); chain = ancestors[parent].copy()
                    if name in self.names:
                        j = self.names.index(name); chain.append(j)
                        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                        frames[j] = (frame[:, :3, 3], frame[:, :3, :3] @ axis)
                        motion = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
                        motion[:, :3, :3] = Rotation.from_rotvec(q[:, j, None]*axis).as_matrix()
                        frame = frame @ motion
                    tf[child] = frame; ancestors[child] = chain
                    pending.remove(joint); progressed = True
                if not progressed:
                    raise ValueError('Disconnected URDF')
            gravity = np.zeros((n, len(self.names)))
            forces = np.zeros((n, len(self.names), 3))
            moments = np.zeros_like(forces)
            for name, link in self.links.items():
                inertial = link.find('inertial')
                if inertial is None:
                    continue
                mass = float(inertial.find('mass').get('value'))
                com = (tf[name] @ origin(inertial.find('origin')))[:, :3, 3]
                for j in ancestors[name]:
                    point, axis = frames[j]
                    force = np.array([0., 0., -9.81*mass])
                    moment = np.cross(com-point, force)
                    gravity[:, j] -= np.sum(axis*moment, axis=1)
                    forces[:, j] += force
                    moments[:, j] += moment
            axes = np.stack([frames[j][1] for j in range(len(self.names))], axis=1)
            axial = np.sum(forces*axes, axis=2)
            radial = np.linalg.norm(forces-axial[:, :, None]*axes, axis=2)
            bending = np.linalg.norm(moments+gravity[:, :, None]*axes, axis=2)
            result.append(dict(gravity_nm=gravity, radial_force_n=radial,
                               axial_force_n=axial, bending_moment_nm=bending))
        return {key: np.concatenate([r[key] for r in result]) for key in result[0]}

    def forward(self, q):
        tf = {self.base: np.eye(4)}; frames = {}; pending = self.joints.copy()
        while pending:
            progressed = False
            for joint in pending.copy():
                parent, child = (joint.find(k).get('link') for k in ('parent', 'child'))
                if parent not in tf:
                    continue
                frame = tf[parent] @ origin(joint.find('origin'))
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ') if joint.find('axis') is not None else np.array([1., 0, 0])
                motion = np.eye(4); name = joint.get('name')
                if name in self.names:
                    i = self.names.index(name)
                    frames[name] = (frame[:3, 3], frame[:3, :3] @ axis, child)
                    motion[:3, :3] = Rotation.from_rotvec(axis*q[i]).as_matrix()
                tf[child] = frame @ motion
                pending.remove(joint); progressed = True
            if not progressed:
                raise ValueError('Disconnected/cyclic URDF')
        return tf, frames

    def analyze(self, q):
        tf, frames = self.forward(q)
        gravity = np.zeros(len(q)); axes = []
        for i, name in enumerate(self.names):
            point, axis, child = frames[name]; axes.append(axis)
            descendants = {child}
            for _ in self.joints:
                descendants.update(j.find('child').get('link') for j in self.joints if j.find('parent').get('link') in descendants)
            for link in descendants:
                inertial = self.links[link].find('inertial')
                if inertial is None:
                    continue
                mass = float(inertial.find('mass').get('value'))
                com = (tf[link] @ origin(inertial.find('origin')))[:3, 3]
                gravity[i] -= np.dot(axis, np.cross(com-point, [0, 0, -9.81*mass]))
        return dict(q_rad=list(q), axes_world=np.array(axes).tolist(),
                    alignment_abs_z=np.abs(np.array(axes)[:, 2]).tolist(), gravity_nm=gravity.tolist())


def report(urdf, centers, names=None, half_ranges=None):
    names = names or [f'joint{i}' for i in range(1, 7)]
    robot = Robot(urdf, names)
    zero = robot.analyze(np.zeros(6))
    result = {'zero': zero, 'calibration_poses': []}
    for j, center in enumerate(centers):
        a = robot.analyze(center)
        sweep = []
        half = half_ranges[j] if half_ranges is not None else .225
        for offset in np.linspace(-half, half, 61):
            q = np.array(center, dtype=float); q[j] += offset
            sweep.append(robot.analyze(q)['gravity_nm'][j])
        a.update(joint=names[j], swept_gravity_min_nm=float(min(sweep)),
                 swept_gravity_max_nm=float(max(sweep)))
        result['calibration_poses'].append(a)
    result['scope'] = 'URDF nominal gravity and axes; excludes cable forces, current filter, external contact and mass errors'
    return result
