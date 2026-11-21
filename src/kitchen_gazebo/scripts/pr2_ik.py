#!/usr/bin/env python3
"""Forward and inverse kinematics of the PR2 arms, read from the lab's PR2 URDF (numpy only).

Used now for its forward kinematics (Chain): kitchen_skills.py aims the head with it. The arm IK
is no longer used, since the robot no longer grasps (objects move into its hand); it stays
for a robot that grasps.

The IK places the gripper's tool frame (between the fingertips) at a position in the robot's
base frame with a given orientation, using the 7 arm joints; the torso height stays as it is.
It is a damped least-squares solver started from several seed poses, which is enough for the
table-top reaches of this scenario and reports *why* it failed (out of reach, joint limits).
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'upper_arm_roll', 'elbow_flex',
              'forearm_roll', 'wrist_flex', 'wrist_roll']


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, angle):
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


def transform(rotation=np.eye(3), translation=(0, 0, 0)):
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = translation
    return m


class Chain:
    """The joints from `root` to `tip`, in order."""

    def __init__(self, urdf_path, root, tip):
        joints = {}
        for j in ET.parse(urdf_path).getroot().findall('joint'):
            origin = j.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            axis = j.find('axis')
            limit = j.find('limit')
            joints[j.find('child').get('link')] = {
                'name': j.get('name'), 'type': j.get('type'), 'parent': j.find('parent').get('link'),
                'origin': transform(rpy_matrix(*rpy), xyz),
                'axis': np.array([float(v) for v in axis.get('xyz').split()]) if axis is not None else None,
                'lower': float(limit.get('lower', -math.inf)) if limit is not None else -math.inf,
                'upper': float(limit.get('upper', math.inf)) if limit is not None else math.inf,
            }
        self.joints, link = [], tip
        while link != root:
            joint = joints[link]
            self.joints.insert(0, joint)
            link = joint['parent']

    def fk(self, positions, axes_of=()):
        """Pose (4x4) of the tip; `positions` maps joint name -> value (missing = 0).
        With `axes_of` (joint names), also returns each named joint's (point, axis) in the root frame."""
        m, axes = np.eye(4), {}
        for j in self.joints:
            m = m @ j['origin']
            if j['name'] in axes_of:
                axes[j['name']] = (m[:3, 3].copy(), m[:3, :3] @ j['axis'])
            q = positions.get(j['name'], 0.0)
            if j['type'] in ('revolute', 'continuous'):
                m = m @ transform(axis_angle(j['axis'], q))
            elif j['type'] == 'prismatic':
                m = m @ transform(translation=j['axis'] * q)
        return (m, axes) if axes_of else m


class ArmIK:
    def __init__(self, urdf_path, side):
        self.side = side
        self.chain = Chain(urdf_path, 'base_footprint', f'{side}_gripper_tool_frame')
        self.names = [f'{side}_{n}_joint' for n in ARM_JOINTS]
        by_name = {j['name']: j for j in self.chain.joints}
        self.lower = np.array([by_name[n]['lower'] for n in self.names])
        self.upper = np.array([by_name[n]['upper'] for n in self.names])
        self.shoulder_name = f'{side}_shoulder_pan_joint'

    def tool_pose(self, positions):
        return self.chain.fk(positions)

    def shoulder_position(self, torso):
        chain = Chain.__new__(Chain)
        chain.joints = self.chain.joints[:[j['name'] for j in self.chain.joints].index(self.shoulder_name) + 1]
        return chain.fk({'torso_lift_joint': torso})[:3, 3]

    def _error(self, pose, position, rotation):
        dp = position - pose[:3, 3]
        r = rotation @ pose[:3, :3].T
        angle = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
        if angle < 1e-9:
            dr = np.zeros(3)
        else:
            dr = angle / (2 * math.sin(angle)) * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
        return np.concatenate([dp, 0.3 * dr])

    def solve(self, position, rotation, torso, seeds, iterations=150):
        """Joint values (dict) that put the tool at `position` with `rotation`, or (None, reason)."""
        position, rotation = np.asarray(position, float), np.asarray(rotation, float)
        reach = np.linalg.norm(position - self.shoulder_position(torso))
        best = (math.inf, None)
        for seed in seeds:
            q = np.clip(np.array([seed.get(n, 0.0) for n in self.names]), self.lower, self.upper)
            for _ in range(iterations):
                values = dict(zip(self.names, q), torso_lift_joint=torso)
                pose, axes = self.chain.fk(values, self.names)
                err = self._error(pose, position, rotation)
                if np.linalg.norm(err[:3]) < 0.005 and np.linalg.norm(err[3:]) < 0.3 * 0.05:
                    return dict(zip(self.names, q)), f'solved (tool {reach:.2f} m from the shoulder)'
                jac = np.zeros((6, 7))
                for i, n in enumerate(self.names):   # revolute joints: v = w x r, w = axis
                    point, axis = axes[n]
                    jac[:3, i] = np.cross(axis, pose[:3, 3] - point)
                    jac[3:, i] = 0.3 * axis
                step = jac.T @ np.linalg.solve(jac @ jac.T + 0.01 * np.eye(6), err)
                q = np.clip(q + np.clip(step, -0.3, 0.3), self.lower, self.upper)
            if np.linalg.norm(err) < best[0]:
                best = (np.linalg.norm(err), err)
        pos_err = np.linalg.norm(best[1][:3])
        if reach > 0.82:
            reason = f'target is {reach:.2f} m from the {self.side} shoulder, beyond the arm\'s reach (about 0.82 m)'
        else:
            reason = f'no joint solution within the joint limits (closest: {pos_err * 100:.1f} cm off)'
        return None, reason


def default_seeds(side):
    """Start poses for the solver. The wrist only bends one way, so pointing the gripper forward
    while the forearm slopes upward needs the forearm rolled by 180 deg (seeds with roll = pi)."""
    seeds = []
    for roll in (0.0, math.pi):
        for lift, elbow, wrist in ((0.3, -0.6, -0.3), (0.9, -1.6, -0.7), (0.5, -1.2, -1.0), (1.2, -2.0, -1.2)):
            seeds.append({f'{side}_shoulder_pan_joint': 0.0, f'{side}_shoulder_lift_joint': lift,
                          f'{side}_upper_arm_roll_joint': 0.0, f'{side}_elbow_flex_joint': elbow,
                          f'{side}_forearm_roll_joint': roll, f'{side}_wrist_flex_joint': wrist,
                          f'{side}_wrist_roll_joint': roll})
    return seeds


def grasp_rotation(yaw):
    """Tool frame for a front grasp: tool x along the robot heading, fingers opening sideways."""
    return rpy_matrix(0.0, 0.0, yaw)
