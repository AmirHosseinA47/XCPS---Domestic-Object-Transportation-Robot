#!/usr/bin/env python3
"""Robot skills for the kitchen scenario, offered as one ROS 2 service for the behaviour tree.

    ros2 run kitchen_gazebo kitchen_skills.py
    ros2 service call /kitchen_skills/execute kitchen_interfaces/srv/Skill \
        "{skill: navigate, params_json: '{\"to\": \"counter\"}'}"

Skills: move_torso, park_arms, look_at, navigate, detect, pick, place, object_pose, state.
Every call returns success, a message that says what happened and why (for the explanation
log), and a JSON result. Each call is also published on /kitchen_skills/events.

How the parts work:
  - navigate: Nav2 (NavigateToPose). A heading change of more than ~30 deg is done at the aisle
    centre, the only place where the PR2 with parked arms has room to turn (0.81 m radius).
  - detect: a simulated head camera (a "logical camera"): an object is seen when its centre is
    inside the camera's field of view (57 x 45 deg, 0.4-3.5 m) of the PR2's head_plate_frame.
    Object poses come from the simulator; occlusion is not modelled.
  - pick / place: the robot does not grasp and its arms stay parked. A few seconds after it has
    seen an object within reach, the object moves from the surface into the hand on its side
    (just below the parked gripper); at the goal it moves from the hand onto the surface. The
    checks are the ones an arm would need: the object must have been seen and be within reach,
    the goal must be within reach and free. Moving the object is a pose change in the simulator
    (gz service set_pose); in the hand a fixed joint (Gazebo DetachableJoint) holds it while the
    robot drives. Grasping with the arms is not needed for the project (decided with the
    professor) and was the slowest, least reliable part.
"""
import json
import math
import os
import subprocess
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from kitchen_interfaces.srv import Skill
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from make_gazebo_urdfs import CARRIED_OBJECTS, CONTROLLED_JOINTS
from pr2_ik import Chain

# Object geometry from the lab meshes: bounding-box size and mesh centre relative to the model origin.
OBJECTS = {
    'milk': {'size': (0.064, 0.065, 0.194), 'center': (0.0, 0.0, 0.009)},
    'breakfast_cereal': {'size': (0.147, 0.058, 0.224), 'center': (-0.001, -0.001, 0.010)},
    'bowl': {'size': (0.140, 0.139, 0.067), 'center': (0.0, -0.002, 0.0015)},
}
SIDE_NAME = {'l': 'left', 'r': 'right'}
ARM_JOINTS = {s: [n for n in CONTROLLED_JOINTS if n.startswith(f'{s}_') and 'gripper' not in n] for s in 'lr'}
GRIPPER_CLOSED = 0.0
# Where the robot can take an object from or put it down: in front of it, at most REACH from the
# robot centre (about the PR2 arm's reach from its shoulders), on a surface 0.5-1.3 m high.
REACH = 0.85
MIN_FRONT = 0.25
SURFACE_HEIGHTS = (0.5, 1.3)
HANDOVER_DELAY = 4.0   # s between seeing / arriving and the object moving (the time an arm would take)
HAND_GAP = 0.03        # m between the gripper's tool frame and the top of the object hanging below it
WORLD = 'kitchen'
CAMERA_FRAME = 'head_plate_frame'
CAMERA_FOV = (1.0, 0.8)           # horizontal, vertical (rad)
CAMERA_RANGE = (0.4, 3.5)
JOINT_SPEED = 0.8                 # rad/s for interpolated arm motions
TURN_MIN = 0.5                    # rad; smaller heading changes are done while driving


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Phases:
    """Times the phases of a skill; logged to find where time goes."""

    def __init__(self, logger, skill):
        self.logger, self.skill, self.t, self.times = logger, skill, time.time(), []

    def mark(self, name):
        now = time.time()
        self.times.append((name, round(now - self.t, 1)))
        self.t = now

    def log(self):
        self.logger.info(f'[{self.skill}] phases: ' + ', '.join(f'{n} {t} s' for n, t in self.times))


class SkillError(Exception):
    """A skill could not be done; the message says why."""


class KitchenSkills(Node):
    def __init__(self):
        super().__init__('kitchen_skills')
        self.cb = ReentrantCallbackGroup()
        share = get_package_share_directory('kitchen_gazebo')
        with open(os.path.join(share, 'config', 'scene.yaml')) as f:
            self.scene = yaml.safe_load(f)
        urdf = os.path.join(get_package_share_directory('pr2_description'), 'urdf', 'pr2_lab.urdf')
        self.head_pan_chain = Chain(urdf, 'base_footprint', 'head_pan_link')
        self.head_tilt_chain = Chain(urdf, 'base_footprint', 'head_tilt_link')
        self.park = {n: v for n, v in CONTROLLED_JOINTS.items()}

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.joints, self.objects, self.detected = {}, {}, {}
        self.carrying = None   # {'object', 'side', 'yaw'}: what is in which hand, and its yaw before
        # Joint states and object poses arrive at up to 500 Hz; only the newest one matters. A queue of
        # 10 let this Python node fall behind, and then even its TF was read 1-2 s late (tested).
        latest = QoSProfile(depth=1)
        self.create_subscription(JointState, '/joint_states', self.on_joints, latest, callback_group=self.cb)
        for obj in CARRIED_OBJECTS:
            self.create_subscription(TFMessage, f'/objects/{obj}/pose',
                                     lambda m, o=obj: self.on_object(o, m), latest, callback_group=self.cb)
        self.cmd = {j: self.create_publisher(Float64, f'/pr2/{j}/cmd_pos', 10) for j in CONTROLLED_JOINTS}
        self.hand = {(s, o, a): self.create_publisher(Empty, f'/pr2/{s}_gripper/{a}/{o}', 10)
                     for s in 'lr' for o in CARRIED_OBJECTS for a in ('attach', 'detach')}
        self.hand_state = {}   # (side, object) -> "attached" / "detached", as Gazebo reports it
        for side in 'lr':
            for obj in CARRIED_OBJECTS:
                self.create_subscription(String, f'/pr2/{side}_gripper/state/{obj}',
                                         lambda m, k=(side, obj): self.hand_state.__setitem__(k, m.data), 10,
                                         callback_group=self.cb)
        self.events = self.create_publisher(String, '/kitchen_skills/events', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb)
        self.create_service(Skill, '/kitchen_skills/execute', self.execute, callback_group=self.cb)
        self.ready = threading.Event()
        threading.Thread(target=self.release_objects, daemon=True).start()
        self.get_logger().info('kitchen skills: waiting for the robot and the objects')

    # ------------------------------------------------------------------ inputs
    def on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def on_object(self, name, msg):
        for t in msg.transforms:
            if t.child_frame_id in (name, '') or len(msg.transforms) == 1:
                r = t.transform.rotation
                p = t.transform.translation
                c = np.array([p.x, p.y, p.z]) + quat_matrix(r) @ np.array(OBJECTS[name]['center'])
                self.objects[name] = {'center': c, 'yaw': yaw_of(r), 'up': quat_matrix(r)[2, 2],
                                      'stamp': time.time()}
                return

    def robot_pose(self, fresh=False):
        """Robot pose in the map (x, y, yaw). fresh=True: see transform()."""
        t = self.transform('base_footprint', fresh)
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def transform(self, frame, fresh=True):
        """map -> frame. This Python node can receive TF up to ~1.5 s late while the robot drives
        (tested: right after Nav2 reported the goal, the buffered pose was still 0.5 m behind). With
        fresh=True it waits (up to 5 s) until the transform stops changing: the robot has stopped,
        so the newest transform is then the current one."""
        last, t0 = None, time.time()
        while True:
            t = self.tf.lookup_transform('map', frame, Time())
            if not fresh:
                return t
            p, q = t.transform.translation, t.transform.rotation
            now = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            if last is not None and np.max(np.abs(now - last)) < 0.002:
                return t
            if time.time() - t0 > 5.0:
                return t
            last = now
            time.sleep(0.3)

    def to_robot(self, p, pose=None):
        x, y, yaw = pose or self.robot_pose()
        dx, dy = p[0] - x, p[1] - y
        return np.array([math.cos(yaw) * dx + math.sin(yaw) * dy,
                         -math.sin(yaw) * dx + math.cos(yaw) * dy, p[2]])

    # ------------------------------------------------------------------ start-up
    def release_objects(self):
        """Harmonic welds every object to both hands when it appears; let them go once."""
        while rclpy.ok() and not (self.joints and all(o in self.objects for o in CARRIED_OBJECTS)):
            time.sleep(0.5)
        time.sleep(1.0)
        for _ in range(6):
            for (s, o, a), pub in self.hand.items():
                if a == 'detach':
                    pub.publish(Empty())
            time.sleep(0.3)
        time.sleep(2.0)
        lines = []
        for name, start in self.scene['objects'].items():
            c = self.objects[name]['center']
            lines.append(f'{name} {math.dist(c[:2], start[:2]) * 100:.1f} cm from its start pose')
        self.ready.set()
        self.event('startup', {}, True, 'objects released on the counter: ' + ', '.join(lines))
        self.get_logger().info('kitchen skills ready')

    # ------------------------------------------------------------------ service
    def execute(self, request, response):
        skills = {'move_torso': self.move_torso, 'park_arms': self.park_arms, 'look_at': self.look_at,
                  'navigate': self.navigate, 'detect': self.detect, 'pick': self.pick, 'place': self.place,
                  'object_pose': self.object_pose, 'state': self.state}
        try:
            params = json.loads(request.params_json or '{}')
            if request.skill not in skills:
                raise SkillError(f'unknown skill "{request.skill}" (known: {", ".join(skills)})')
            if not self.ready.wait(timeout=60):
                raise SkillError('the simulation is not ready (robot or objects missing)')
            message, result = skills[request.skill](params)
            response.success, response.message = True, message
            response.result_json = json.dumps(result)
        except SkillError as e:
            response.success, response.message, response.result_json = False, str(e), '{}'
        except Exception as e:  # noqa: BLE001 - report, don't crash the server
            response.success, response.message, response.result_json = False, f'internal error: {e!r}', '{}'
        self.event(request.skill, request.params_json, response.success, response.message)
        return response

    def event(self, skill, params, success, message):
        self.events.publish(String(data=json.dumps({'time': time.time(), 'skill': skill, 'params': params,
                                                    'success': success, 'message': message})))
        self.get_logger().info(f'[{skill}] {"ok" if success else "FAILED"}: {message}')

    # ------------------------------------------------------------------ joints
    def command(self, values):
        for name, value in values.items():
            self.cmd[name].publish(Float64(data=float(value)))

    def wait_joints(self, targets, tolerance, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            errors = {n: abs(self.joints.get(n, 1e9) - v) for n, v in targets.items()}
            if max(errors.values()) < tolerance:
                return None
            time.sleep(0.05)
        worst = max(errors, key=errors.get)
        return worst, errors[worst]

    def move_arm(self, side, goal, what):
        """Moves one arm to `goal` (joint values) along a straight line in joint space."""
        names = ARM_JOINTS[side]
        q0 = np.array([self.joints[n] for n in names])
        q1 = np.array([goal[n] for n in names])
        for i, n in enumerate(names):   # continuous joints: go the short way round
            if 'roll' in n and n.endswith(('forearm_roll_joint', 'wrist_roll_joint')):
                q1[i] = q0[i] + wrap(q1[i] - q0[i])
        duration = max(1.0, float(np.max(np.abs(q1 - q0))) / JOINT_SPEED)
        steps = int(duration * 25)
        for k in range(1, steps + 1):
            s = k / steps
            s = s * s * (3 - 2 * s)
            self.command(dict(zip(names, q0 + (q1 - q0) * s)))
            time.sleep(duration / steps)
        stuck = self.wait_joints(dict(zip(names, q1)), 0.04, 4.0)
        if stuck:
            raise SkillError(f'{SIDE_NAME[side]} arm did not reach {what}: {stuck[0]} stopped '
                             f'{stuck[1]:.2f} rad short (blocked by something?)')
        return duration

    # ------------------------------------------------------------------ skills
    def state(self, _p):
        x, y, yaw = self.robot_pose()
        return 'current state', {'robot': [x, y, yaw], 'torso': self.joints.get('torso_lift_joint'),
                                 'carrying': (self.carrying or {}).get('object'),
                                 'detected': sorted(self.detected)}

    def move_torso(self, p):
        height = float(p.get('height', self.scene['torso_height']))
        if not 0.0 <= height <= 0.33:
            raise SkillError(f'torso height {height} m is outside its range 0-0.33 m')
        start, t0 = self.joints['torso_lift_joint'], time.time()
        self.command({'torso_lift_joint': height})
        stuck = self.wait_joints({'torso_lift_joint': height}, 0.01, abs(height - start) / 0.013 * 1.5 + 5)
        if stuck:
            raise SkillError(f'torso stopped {stuck[1] * 100:.1f} cm short of {height:.2f} m')
        return (f'torso moved from {start:.2f} m to {height:.2f} m in {time.time() - t0:.0f} s '
                f'(the PR2 torso moves 1.3 cm/s)'), {'height': height}

    def park_arms(self, _p):
        for side in 'rl':
            self.move_arm(side, {n: self.park[n] for n in ARM_JOINTS[side]}, 'the park pose')
            self.command({f'{side}_gripper_l_finger_joint': GRIPPER_CLOSED})
        return 'right and left arm parked', {}

    def look_at(self, p):
        target = np.array([float(p['x']), float(p['y']), float(p['z'])])
        torso = self.joints.get('torso_lift_joint', 0.0)
        t = self.to_robot(target, self.robot_pose(fresh=True))
        pan_axis = self.head_pan_chain.fk({'torso_lift_joint': torso})[:3, 3]
        pan = max(-2.8, min(2.8, math.atan2(t[1] - pan_axis[1], t[0] - pan_axis[0])))
        tilt_axis = self.head_tilt_chain.fk({'torso_lift_joint': torso, 'head_pan_joint': pan})[:3, 3]
        horizontal = math.hypot(t[0] - tilt_axis[0], t[1] - tilt_axis[1])
        tilt = max(-0.37, min(1.29, math.atan2(tilt_axis[2] - t[2], horizontal)))
        self.command({'head_pan_joint': pan, 'head_tilt_joint': tilt})
        stuck = self.wait_joints({'head_pan_joint': pan, 'head_tilt_joint': tilt}, 0.02, 5.0)
        if stuck:
            raise SkillError(f'head did not reach its target ({stuck[0]} {stuck[1]:.2f} rad off)')
        time.sleep(0.3)  # let the TF of the head catch up before anyone looks through it
        return (f'head turned to ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}): '
                f'pan {math.degrees(pan):.0f} deg, tilt {math.degrees(tilt):.0f} deg down'), {'pan': pan, 'tilt': tilt}

    def detect(self, p):
        name = p['object']
        if name not in self.objects:
            raise SkillError(f'there is no {name} in the scene')
        cam = self.transform(CAMERA_FRAME)
        rot = quat_matrix(cam.transform.rotation)
        origin = np.array([cam.transform.translation.x, cam.transform.translation.y, cam.transform.translation.z])
        obj = self.objects[name]
        v = rot.T @ (obj['center'] - origin)   # object centre in the camera frame (x forward)
        distance = float(np.linalg.norm(v))
        h, vert = math.degrees(math.atan2(v[1], v[0])), math.degrees(math.atan2(v[2], v[0]))
        half_h, half_v = math.degrees(CAMERA_FOV[0] / 2), math.degrees(CAMERA_FOV[1] / 2)
        if v[0] <= 0:
            reason = f'{name} is behind the camera'
        elif abs(h) > half_h:
            reason = (f'{name} is {abs(h) - half_h:.0f} deg {"left" if h > 0 else "right"} of the camera\'s '
                      f'field of view (the head points elsewhere)')
        elif abs(vert) > half_v:
            reason = f'{name} is {abs(vert) - half_v:.0f} deg {"above" if vert > 0 else "below"} the field of view'
        elif not CAMERA_RANGE[0] <= distance <= CAMERA_RANGE[1]:
            reason = f'{name} is {distance:.2f} m away, outside the camera range {CAMERA_RANGE[0]}-{CAMERA_RANGE[1]} m'
        else:
            c = obj['center']
            self.detected[name] = {'x': float(c[0]), 'y': float(c[1]), 'z': float(c[2]), 'yaw': float(obj['yaw'])}
            return (f'saw the {name} at ({c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f}), {distance:.2f} m from the camera, '
                    f'{abs(h):.0f} deg {"left" if h > 0 else "right"} of the view centre'), self.detected[name]
        self.detected.pop(name, None)
        raise SkillError(f'{name} not seen: {reason}')

    def navigate(self, p):
        if 'to' in p:
            if p['to'] not in self.scene['standing_poses']:
                raise SkillError(f'unknown place "{p["to"]}" (known: {", ".join(self.scene["standing_poses"])})')
            x, y, yaw = self.scene['standing_poses'][p['to']]
            label = f'the {p["to"]}'
        else:
            x, y, yaw, label = float(p['x']), float(p['y']), float(p['yaw']), 'the goal'
        if not self.nav.wait_for_server(timeout_sec=10.0):
            raise SkillError('Nav2 is not running (no navigate_to_pose action server)')
        sx, sy, syaw = self.robot_pose()
        steps = [(f'drive to {label}', (x, y, yaw))]
        if abs(wrap(yaw - syaw)) > TURN_MIN:
            tx, ty = self.scene['turn_point']
            steps = [('back out to the aisle centre', (tx, ty, syaw)),
                     (f'turn {abs(math.degrees(wrap(yaw - syaw))):.0f} deg there', (tx, ty, yaw)),
                     (f'drive to {label}', (x, y, yaw))]
        t0, done = time.time(), []
        for what, (gx, gy, gyaw) in steps:
            self.nav_goal(gx, gy, gyaw, what)
            done.append(what)
        ex, ey, eyaw = self.robot_pose(fresh=True)
        error = math.dist((ex, ey), (x, y))
        why = (' (a heading change of more than 30 deg needs the aisle centre: turning with parked arms '
               'sweeps a 0.81 m circle)' if len(steps) > 1 else '')
        return (f'reached {label} in {time.time() - t0:.0f} s: {", ".join(done)}{why}; stopped {error * 100:.1f} cm '
                f'from the standing pose'), {'pose': [ex, ey, eyaw], 'error_m': error, 'steps': done}

    def nav_goal(self, x, y, yaw, what):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        send = self.nav.send_goal_async(goal)
        if not self.wait_future(send, 10.0) or not send.result().accepted:
            raise SkillError(f'Nav2 refused the goal "{what}"')
        result = send.result().get_result_async()
        if not self.wait_future(result, 180.0):
            send.result().cancel_goal_async()
            raise SkillError(f'navigation step "{what}" took longer than 3 minutes and was cancelled')
        status = result.result().status
        if status != 4:   # action_msgs/GoalStatus.STATUS_SUCCEEDED
            x0, y0, _ = self.robot_pose()
            raise SkillError(f'navigation step "{what}" failed (Nav2 status {status}); the robot stopped at '
                             f'({x0:.2f}, {y0:.2f}), {math.dist((x0, y0), (x, y)):.2f} m from the goal')

    @staticmethod
    def wait_future(future, timeout):
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.05)
        return True

    # ------------------------------------------------------------------ carrying
    def check_reach(self, point, what, pose=None):
        """Raises SkillError, saying why, if `point` (map) is not where the robot can take or put things."""
        rel = self.to_robot(point, pose)
        distance = math.hypot(rel[0], rel[1])
        if rel[0] < MIN_FRONT:
            side = 'behind' if rel[0] < 0 else ('left of' if rel[1] > 0 else 'right of')
            raise SkillError(f'{what} is {side} the robot ({rel[0]:.2f} m forward, {rel[1]:.2f} m to the side); '
                             f'it can only take or put things in front of it')
        if distance > REACH:
            raise SkillError(f'{what} is {distance:.2f} m from the robot centre, beyond its {REACH:.2f} m reach '
                             f'({distance - REACH:.2f} m too far; the robot must stand closer)')
        if not SURFACE_HEIGHTS[0] <= point[2] <= SURFACE_HEIGHTS[1] + 0.3:
            raise SkillError(f'{what} is at {point[2]:.2f} m height; it can only reach surfaces '
                             f'{SURFACE_HEIGHTS[0]}-{SURFACE_HEIGHTS[1]} m high')
        return distance

    def hand_joint(self, side, name, action, timeout=2.0):
        """Attaches the object to a hand or detaches it, and waits until Gazebo confirms. Moving an
        object that was still attached made the joint fling it over (tested)."""
        want = 'attached' if action == 'attach' else 'detached'
        self.hand_state.pop((side, name), None)
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.hand[(side, name, action)].publish(Empty())
            for _ in range(5):
                time.sleep(0.02)
                if self.hand_state.get((side, name)) == want:
                    return True
        return False

    @staticmethod
    def gz_service(service, reqtype, req, what):
        """Calls a Gazebo service (gz CLI); raises SkillError, saying what failed, if it does not answer true."""
        try:
            out = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/{service}', '--reqtype', reqtype,
                                  '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                                 capture_output=True, text=True, timeout=10)
        except (OSError, subprocess.TimeoutExpired) as e:
            raise SkillError(f'could not {what} in the simulator: {e}') from e
        if 'data: true' not in out.stdout:
            raise SkillError(f'the simulator refused to {what}: {(out.stdout + out.stderr).strip()[:200]}')

    def set_pose(self, name, position, yaw):
        """Moves a model in Gazebo (service /world/kitchen/set_pose)."""
        req = (f'name: "{name}", position: {{x: {position[0]:.4f}, y: {position[1]:.4f}, z: {position[2]:.4f}}}, '
               f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
        self.gz_service('set_pose', 'gz.msgs.Pose', req, f'move the {name}')

    def pause(self, paused):
        self.gz_service('control', 'gz.msgs.WorldControl', f'pause: {"true" if paused else "false"}',
                        'pause the simulation' if paused else 'resume the simulation')

    def origin_for(self, name, centre_xy, top):
        """Model origin that puts the object upright on a surface whose top is at `top`."""
        geom = OBJECTS[name]
        return np.array([centre_xy[0], centre_xy[1], top + geom['size'][2] / 2 - geom['center'][2] + 0.003])

    def wait_still(self, name, timeout=3.0):
        t0, last = time.time(), None
        while time.time() - t0 < timeout:
            time.sleep(0.25)
            c = self.objects[name]['center'].copy()
            if last is not None and np.linalg.norm(c - last) < 0.001:
                return
            last = c

    def pick(self, p):
        """Takes a seen object from a surface into the hand on its side (the arms stay parked)."""
        name = p['object']
        if self.carrying:
            raise SkillError(f'the {SIDE_NAME[self.carrying["side"]]} hand already holds the '
                             f'{self.carrying["object"]}; the robot carries one object at a time')
        if name not in self.detected:
            raise SkillError(f'the {name} has not been seen yet; the robot must detect it before taking it')
        obj = self.objects[name]
        seen = np.array([self.detected[name][k] for k in ('x', 'y', 'z')])
        if np.linalg.norm(obj['center'] - seen) > 0.05:
            moved = np.linalg.norm(obj['center'][:2] - seen[:2])
            self.detected.pop(name, None)
            raise SkillError(f'the {name} is no longer where the robot saw it (moved {moved * 100:.0f} cm); '
                             f'it must look again')
        robot = self.robot_pose(fresh=True)
        distance = self.check_reach(obj['center'], f'the {name}', robot)
        side = p.get('arm') or ('l' if self.to_robot(obj['center'], robot)[1] > 0 else 'r')
        time.sleep(HANDOVER_DELAY)
        # Just below the parked gripper, at the side of the robot: clear of the torso, the base and
        # the gripper's fingers, so nothing touches the object while the hand holds it.
        hand = self.transform(f'{side}_gripper_tool_frame').transform.translation
        geom = OBJECTS[name]
        centre = np.array([hand.x, hand.y, hand.z - HAND_GAP - geom['size'][2] / 2])
        # Hand-off with the simulation paused, so the object does not fall before the hand holds it.
        self.pause(True)
        try:
            self.set_pose(name, centre - np.array(geom['center']), obj['yaw'])
            held = self.hand_joint(side, name, 'attach')
        finally:
            self.pause(False)
        if not held:
            raise SkillError(f'the {SIDE_NAME[side]} hand did not take hold of the {name} '
                             f'(no confirmation from the simulator)')
        time.sleep(0.5)
        off = np.linalg.norm(self.objects[name]['center'] - centre)
        if off > 0.05:
            self.hand_joint(side, name, 'detach')
            raise SkillError(f'the {name} did not stay in the {SIDE_NAME[side]} hand ({off * 100:.0f} cm off)')
        self.carrying = {'object': name, 'side': side, 'yaw': obj['yaw']}
        self.detected.pop(name, None)   # it has moved; the old observation is stale
        return (f'took the {name} from {distance:.2f} m in front of it (within its {REACH:.2f} m reach) '
                f'into its {SIDE_NAME[side]} hand ({"left" if side == "l" else "right"} of the robot, '
                f'where the {name} stood)'), {'arm': side, 'distance_m': distance}

    def place(self, p):
        """Sets the carried object down upright at (x, y) on a surface."""
        name = p['object']
        if not self.carrying or self.carrying['object'] != name:
            what = (f'the {self.carrying["object"]} in its {SIDE_NAME[self.carrying["side"]]} hand'
                    if self.carrying else 'nothing')
            raise SkillError(f'the robot is not carrying the {name} (it holds {what})')
        surface = p.get('surface', 'island')
        top = float(p['z_surface']) if 'z_surface' in p else self.scene['surfaces'][surface]
        goal = np.array([float(p['x']), float(p['y']), top])
        distance = self.check_reach(goal, f'the goal ({goal[0]:.2f}, {goal[1]:.2f}) on the {surface}',
                                    self.robot_pose(fresh=True))
        # The goal spot must be free: no other object whose footprint would overlap.
        radius = max(OBJECTS[name]['size'][:2]) / 2
        for other, o in self.objects.items():
            if other == name:
                continue
            gap = math.dist(o['center'][:2], goal[:2]) - radius - max(OBJECTS[other]['size'][:2]) / 2
            if gap < 0.01 and abs(o['center'][2] - top) < 0.3:
                raise SkillError(f'the goal spot on the {surface} is occupied: the {other} stands '
                                 f'{math.dist(o["center"][:2], goal[:2]) * 100:.0f} cm from it, the {name} '
                                 f'would touch it')
        side, yaw = self.carrying['side'], self.carrying['yaw']
        time.sleep(HANDOVER_DELAY)
        # Hand-off with the simulation paused: once the hand lets go, the object falls freely while the
        # pose change (~0.5 s) is under way, and a tall cereal box then arrived falling and spinning
        # and tipped over (tested). Paused, it is let go and moved without moving at all.
        self.pause(True)
        try:
            if not self.hand_joint(side, name, 'detach'):
                raise SkillError(f'the {SIDE_NAME[side]} hand did not let go of the {name} '
                                 f'(no confirmation from the simulator)')
            self.carrying = None
            self.set_pose(name, self.origin_for(name, goal[:2], top), yaw)
        finally:
            self.pause(False)
        self.wait_still(name)
        obj = self.objects[name]
        error = math.dist(obj['center'][:2], goal[:2])
        if obj['up'] < 0.9:
            raise SkillError(f'the {name} fell over after it was set down on the {surface}')
        if abs(obj['center'][2] - (top + OBJECTS[name]['size'][2] / 2)) > 0.03:
            raise SkillError(f'the {name} is not standing on the {surface} after setting it down '
                             f'(height {obj["center"][2]:.2f} m)')
        return (f'set the {name} down from its {SIDE_NAME[side]} hand on the {surface}, {distance:.2f} m in front '
                f'of it, {error * 100:.1f} cm from the target, standing upright'), {'error_m': error,
                                                                                    'distance_m': distance}

    def object_pose(self, p):
        name = p['object']
        if name not in self.objects:
            raise SkillError(f'there is no {name} in the scene')
        c = self.objects[name]['center']
        msg = f'{name} is at ({c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f})'
        if 'x' in p:
            d = math.dist(c[:2], (float(p['x']), float(p['y'])))
            ok = d <= float(p.get('tolerance', 0.05))
            msg += f', {d * 100:.1f} cm from ({float(p["x"]):.2f}, {float(p["y"]):.2f})'
            if not ok:
                raise SkillError(msg + f' - farther than the {float(p.get("tolerance", 0.05)) * 100:.0f} cm allowed')
        return msg, {'x': float(c[0]), 'y': float(c[1]), 'z': float(c[2])}


def main():
    rclpy.init()
    node = KitchenSkills()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
