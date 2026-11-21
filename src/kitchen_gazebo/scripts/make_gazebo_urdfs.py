#!/usr/bin/env python3
"""Generates Gazebo-ready URDFs from the lab's PR2 and IAI kitchen URDFs.

The lab files (pr2_description/urdf/pr2_lab.urdf, iai_kitchen/urdf/kitchen_lab.urdf) stay
unchanged; the results go to kitchen_gazebo/urdf/. Run it again after changing this script:

    python3 src/kitchen_gazebo/scripts/make_gazebo_urdfs.py
"""
import math
import pathlib
import xml.etree.ElementTree as ET

SRC = pathlib.Path(__file__).resolve().parents[2]
PR2_LAB = SRC / 'pr2_description' / 'urdf' / 'pr2_lab.urdf'
KITCHEN_LAB = SRC / 'iai_kitchen' / 'urdf' / 'kitchen_lab.urdf'
OUT = SRC / 'kitchen_gazebo' / 'urdf'


def wrap(angle):
    """Angle in [-pi, pi] (continuous joints accept any value; the lab stores 16.996 rad)."""
    return math.atan2(math.sin(angle), math.cos(angle))


# Joints held by a Gazebo position controller, with their start position:
# arms in the lab's "park" pose (pycram robot_descriptions/pr2_description.py), torso down,
# head straight ahead, grippers closed. Every other movable joint is fixed for now.
CONTROLLED_JOINTS = {
    'torso_lift_joint': 0.0,
    'head_pan_joint': 0.0,
    'head_tilt_joint': 0.0,
    'laser_tilt_mount_joint': 0.0,
    'l_shoulder_pan_joint': 1.712,
    'l_shoulder_lift_joint': -0.264,
    'l_upper_arm_roll_joint': 1.38,
    'l_elbow_flex_joint': -2.12,
    'l_forearm_roll_joint': wrap(16.996),
    'l_wrist_flex_joint': -0.073,
    'l_wrist_roll_joint': 0.0,
    'r_shoulder_pan_joint': -1.712,
    'r_shoulder_lift_joint': -0.256,
    'r_upper_arm_roll_joint': -1.463,
    'r_elbow_flex_joint': -2.12,
    'r_forearm_roll_joint': 1.766,
    'r_wrist_flex_joint': -0.07,
    'r_wrist_roll_joint': 0.051,
    'l_gripper_l_finger_joint': 0.0,
    'r_gripper_l_finger_joint': 0.0,
}

# The real PR2 arms carry a spring counterbalance, so the shoulder-lift motors (30 Nm) never
# hold the arm's weight alone. Gazebo has no such spring; a higher effort limit stands in for it.
EFFORT_LIMIT_OVERRIDES = {'l_shoulder_lift_joint': 150.0, 'r_shoulder_lift_joint': 150.0}

# Joints driven by force (PID) instead of velocity commands. The prismatic torso does not move
# under velocity commands in Gazebo Harmonic (tested); with force it rises at its URDF limit.
FORCE_CONTROLLED_JOINTS = {'torso_lift_joint': {'p': 50000, 'i': 5000, 'max_force': 10000}}

# Objects the robot can carry. The robot does not grasp: it loads an object onto a carry tray in
# front of its torso and sets it down at the goal (kitchen_skills.py), which is enough for the
# project (decided with the professor). On the tray the object is held by a fixed joint created on
# request (gz DetachableJoint), so it cannot slide off while the robot drives.
CARRIED_OBJECTS = ('milk', 'breakfast_cereal', 'bowl')

# The carry tray: a plate fixed to the torso (it rises with it), in front of the torso column and
# inside the base outline, so the navigation footprint does not change. Offsets are relative to
# torso_lift_link; with the torso at 0.30 m the tray top is 0.955 m above the floor, just above
# the counter and island tops (0.85 m), and well below and between the parked arms (1.2 m, y +/-0.55).
TRAY = {'xyz': (0.25, 0.0, -0.145), 'size': (0.24, 0.30, 0.01), 'rim': 0.03}

JOINT_ONLY_TAGS = ('limit', 'mimic', 'dynamics', 'safety_controller', 'calibration', 'axis')


def make_fixed(joint):
    joint.set('type', 'fixed')
    for tag in JOINT_ONLY_TAGS:
        for child in joint.findall(tag):
            joint.remove(child)


def plugin(parent, filename, name, **params):
    element = ET.SubElement(parent, 'plugin', filename=filename, name=name)
    for key, value in params.items():
        ET.SubElement(element, key).text = str(value)
    return element


def write(root, path, comment):
    ET.indent(root, space='  ')
    text = ET.tostring(root, encoding='unicode')
    path.write_text(f'<?xml version="1.0"?>\n<!-- {comment} -->\n{text}\n')
    print(f'wrote {path}')


def add_base_laser(robot):
    """The PR2's base laser (a Hokuyo with a 270 deg view) as a Gazebo lidar publishing /scan."""
    reference = ET.SubElement(robot, 'gazebo', reference='base_laser_link')
    sensor = ET.SubElement(reference, 'sensor', name='base_laser', type='gpu_lidar')
    for key, value in (('topic', '/scan'), ('gz_frame_id', 'base_laser_link'),
                       ('update_rate', '10'), ('always_on', 'true'), ('visualize', 'false')):
        ET.SubElement(sensor, key).text = value
    lidar = ET.SubElement(sensor, 'lidar')
    horizontal = ET.SubElement(ET.SubElement(lidar, 'scan'), 'horizontal')
    # +/-2.0 rad instead of the real PR2's +/-2.356: the outermost beams hit the
    # robot's own rear base corners, and those self-points inside the footprint
    # made the Nav2 collision monitor stop every velocity command.
    for key, value in (('samples', '459'), ('resolution', '1'),
                       ('min_angle', '-2.0'), ('max_angle', '2.0')):
        ET.SubElement(horizontal, key).text = value
    range_ = ET.SubElement(lidar, 'range')
    for key, value in (('min', '0.1'), ('max', '10.0'), ('resolution', '0.01')):
        ET.SubElement(range_, key).text = value


def add_carry_tray(robot):
    """A plate with a low rim in front of the torso, on which the robot carries one object."""
    link = ET.SubElement(robot, 'link', name='carry_tray_link')
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', value='0.5')
    ET.SubElement(inertial, 'inertia', ixx='0.004', ixy='0', ixz='0', iyy='0.003', iyz='0', izz='0.006')
    sx, sy, sz = TRAY['size']
    rim = TRAY['rim']
    parts = [('plate', (0.0, 0.0, 0.0), (sx, sy, sz)),
             ('rim_front', (sx / 2 - 0.005, 0.0, rim / 2), (0.01, sy, rim)),
             ('rim_left', (0.0, sy / 2 - 0.005, rim / 2), (sx, 0.01, rim)),
             ('rim_right', (0.0, -sy / 2 + 0.005, rim / 2), (sx, 0.01, rim))]
    # Visual only: the carried object is held by a fixed joint, not by contact. A tray collision
    # fought that joint and pushed the robot (its base slides without friction) 43 cm (tested).
    for name, xyz, size in parts:
        visual = ET.SubElement(link, 'visual', name=f'carry_tray_{name}_visual')
        ET.SubElement(visual, 'origin', xyz=' '.join(f'{v:.4f}' for v in xyz), rpy='0 0 0')
        ET.SubElement(ET.SubElement(visual, 'geometry'), 'box', size=' '.join(f'{v:.4f}' for v in size))
        ET.SubElement(ET.SubElement(visual, 'material', name='carry_tray_grey'), 'color', rgba='0.55 0.57 0.60 1')
    joint = ET.SubElement(robot, 'joint', name='carry_tray_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='torso_lift_link')
    ET.SubElement(joint, 'child', link='carry_tray_link')
    ET.SubElement(joint, 'origin', xyz=' '.join(f'{v:.4f}' for v in TRAY['xyz']), rpy='0 0 0')


def make_pr2():
    robot = ET.parse(PR2_LAB).getroot()
    fixed = []
    for joint in robot.findall('joint'):
        name = joint.get('name')
        if joint.get('type') != 'fixed' and name not in CONTROLLED_JOINTS:
            make_fixed(joint)  # casters, wheels, gripper helper and mimic joints
            fixed.append(name)
        if name in EFFORT_LIMIT_OVERRIDES:
            joint.find('limit').set('effort', str(EFFORT_LIMIT_OVERRIDES[name]))

    joints = {j.get('name'): j for j in robot.findall('joint')}
    gazebo = ET.SubElement(robot, 'gazebo')
    for name, start in CONTROLLED_JOINTS.items():
        common = dict(joint_name=name, topic=f'/pr2/{name}/cmd_pos', initial_position=f'{start:.4f}')
        if name in FORCE_CONTROLLED_JOINTS:
            gains = FORCE_CONTROLLED_JOINTS[name]
            plugin(gazebo, 'gz-sim-joint-position-controller-system',
                   'gz::sim::systems::JointPositionController', **common,
                   use_velocity_commands='false', p_gain=gains['p'], i_gain=gains['i'], d_gain=0,
                   # Gazebo caps the integral term at 1 N by default; then the PID cannot hold the
                   # upper body and the torso stopped 4 cm short of 0.30 m (tested).
                   i_max=gains['max_force'], i_min=-gains['max_force'],
                   cmd_max=gains['max_force'], cmd_min=-gains['max_force'])
            continue
        # Command no faster than the joint's own URDF velocity limit.
        speed = min(1.5, float(joints[name].find('limit').get('velocity', 1.5)))
        plugin(gazebo, 'gz-sim-joint-position-controller-system',
               'gz::sim::systems::JointPositionController', **common,
               use_velocity_commands='true', p_gain=10, cmd_max=speed, cmd_min=-speed)
    # Base: driven like the real (holonomic) PR2 base by a planar velocity command. The casters
    # and wheels are fixed, so they are made frictionless and slide instead of steering.
    plugin(gazebo, 'gz-sim-velocity-control-system', 'gz::sim::systems::VelocityControl',
           topic='/pr2/cmd_vel')
    for link in robot.findall('link'):
        if link.get('name') in ('base_link', 'base_footprint') or '_caster_' in link.get('name'):
            surface = ET.SubElement(robot, 'gazebo', reference=link.get('name'))
            ET.SubElement(surface, 'mu1').text = '0'
            ET.SubElement(surface, 'mu2').text = '0'
    add_base_laser(robot)
    add_carry_tray(robot)
    for obj in CARRIED_OBJECTS:
        # The tray is merged into torso_lift_link when the URDF becomes SDF (fixed joint), so that
        # link is the joint's parent. Harmonic attaches as soon as the object exists
        # (initial_attach is only read by newer versions); kitchen_skills.py detaches every object
        # once at start-up.
        plugin(gazebo, 'gz-sim-detachable-joint-system', 'gz::sim::systems::DetachableJoint',
               parent_link='torso_lift_link', child_model=obj, child_link='link',
               attach_topic=f'/pr2/tray/attach/{obj}', detach_topic=f'/pr2/tray/detach/{obj}',
               output_topic=f'/pr2/tray/state/{obj}',
               initial_attach='false', suppress_child_warning='true')
    plugin(gazebo, 'gz-sim-joint-state-publisher-system', 'gz::sim::systems::JointStatePublisher')
    plugin(gazebo, 'gz-sim-odometry-publisher-system', 'gz::sim::systems::OdometryPublisher',
           odom_frame='odom', robot_base_frame='base_footprint', dimensions=3,
           odom_publish_frequency=30)
    write(robot, OUT / 'pr2_gazebo.urdf',
          'Generated by make_gazebo_urdfs.py from pr2_description/urdf/pr2_lab.urdf - do not edit')
    print(f'  PR2: {len(CONTROLLED_JOINTS)} controlled joints, {len(fixed)} movable joints fixed')


def make_kitchen():
    robot = ET.parse(KITCHEN_LAB).getroot()
    movable = [j for j in robot.findall('joint') if j.get('type') != 'fixed']
    for joint in movable:
        make_fixed(joint)  # drawers and doors stay closed for now
    gazebo = ET.SubElement(robot, 'gazebo')
    ET.SubElement(gazebo, 'static').text = 'true'
    write(robot, OUT / 'kitchen_gazebo.urdf',
          'Generated by make_gazebo_urdfs.py from iai_kitchen/urdf/kitchen_lab.urdf - do not edit')
    print(f'  kitchen: {len(movable)} drawer/door joints fixed, model static')


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    make_pr2()
    make_kitchen()
