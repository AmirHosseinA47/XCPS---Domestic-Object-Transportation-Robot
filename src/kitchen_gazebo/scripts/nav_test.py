#!/usr/bin/env python3
"""Drives the PR2 between the sink counter and the kitchen island with Nav2 and logs each leg.

    ros2 run kitchen_gazebo nav_test.py [--rounds 2] [--log-dir DIR]

Each leg records: goal, result, simulated duration, distance driven, path length planned,
recoveries, how far the robot stopped from the goal, and the steps it took (with the reason
for each step). The log (JSON) feeds later explanations.
"""
import argparse
import json
import math
import pathlib
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

# Standing poses in the map frame (x, y, yaw). Kept clear of the furniture for the PR2 with the
# lab's parked arms: base front 0.37 m ahead of its centre.
# counter: 11 cm from the dishwasher/drawer handles (x 1.18), facing the counter (+x);
# island:  8 cm from the island's front (x -0.67), facing the island (-x).
STANDING_POSES = {
    'counter': (0.70, 0.25, 0.0),
    'island': (-0.22, 0.75, math.pi),
}

# With the lab's parked arms the PR2 sweeps a circle of 0.81 m radius when it turns in place.
# The aisle between counter (x 1.18) and island (x -0.67) is 1.85 m wide, so the robot can only
# turn near the aisle's centre line (0.92 m to either side). It backs out there first, turns,
# then drives to the standing pose.
TURN_POINT = (0.255, 0.5)
TURN_RADIUS_M = 0.81
MIN_TURN_RAD = 0.5   # smaller heading changes are done while driving

RESULT_NAMES = {TaskResult.SUCCEEDED: 'succeeded', TaskResult.CANCELED: 'canceled',
                TaskResult.FAILED: 'failed'}


def pose(navigator, x, y, yaw):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = x, y
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return goal


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def path_length(path):
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses] if path else []
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def plan_steps(start_yaw, target, x, y, yaw):
    """The sub-goals for one leg, each with the reason the robot takes it."""
    if abs(angle_diff(yaw, start_yaw)) < MIN_TURN_RAD:
        return [(f'drive to the {target}', (x, y, yaw))]
    tx, ty = TURN_POINT
    return [
        ('back out to the aisle centre without turning '
         f'(turning needs {TURN_RADIUS_M} m of free space all round)', (tx, ty, start_yaw)),
        (f'turn to face the {target}', (tx, ty, yaw)),
        (f'drive to the {target}', (x, y, yaw)),
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--log-dir', default=str(pathlib.Path.home() / 'kitchen_nav_runs'))
    args = parser.parse_args()

    rclpy.init()
    nav = BasicNavigator()
    odom = {}
    nav.create_subscription(Odometry, '/odom', lambda m: odom.update(msg=m), 10)
    nav.waitUntilNav2Active(localizer='map_server')

    def current():
        rclpy.spin_once(nav, timeout_sec=0.2)
        p = odom['msg'].pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    legs = []
    for leg_index, target in enumerate(['counter', 'island'] * args.rounds, start=1):
        x, y, yaw = STANDING_POSES[target]
        sx, sy, syaw = current()
        plan = nav.getPath(pose(nav, sx, sy, syaw), pose(nav, x, y, yaw), use_start=True)
        t0 = nav.get_clock().now()
        driven, last, recoveries, steps, result = 0.0, (sx, sy), 0, [], 'succeeded'
        for reason, (gx, gy, gyaw) in plan_steps(syaw, target, x, y, yaw):
            ts = nav.get_clock().now()
            nav.goToPose(pose(nav, gx, gy, gyaw))
            step_recoveries = 0
            while not nav.isTaskComplete():
                feedback = nav.getFeedback()
                if feedback:
                    step_recoveries = feedback.number_of_recoveries
                rclpy.spin_once(nav, timeout_sec=0.1)
                p = odom['msg'].pose.pose.position
                driven += math.dist(last, (p.x, p.y))
                last = (p.x, p.y)
            step_result = RESULT_NAMES.get(nav.getResult(), str(nav.getResult()))
            recoveries += step_recoveries
            steps.append({'step': reason, 'goal_pose': [gx, gy, round(gyaw, 3)],
                          'result': step_result, 'recoveries': step_recoveries,
                          'duration_s': round((nav.get_clock().now() - ts).nanoseconds / 1e9, 1)})
            if step_result != 'succeeded':
                result = step_result
                break
        ex, ey, eyaw = current()
        leg = {
            'leg': leg_index, 'goal': target, 'goal_pose': [x, y, round(yaw, 3)],
            'result': result,
            'duration_s': round((nav.get_clock().now() - t0).nanoseconds / 1e9, 1),
            'distance_driven_m': round(driven, 2),
            'planned_path_m': round(path_length(plan), 2),
            'recoveries': recoveries,
            'final_position_error_m': round(math.dist((ex, ey), (x, y)), 3),
            'final_heading_error_deg': round(math.degrees(abs(angle_diff(eyaw, yaw))), 1),
            'steps': steps,
        }
        legs.append(leg)
        print(json.dumps({k: v for k, v in leg.items() if k != 'steps'}), flush=True)
        for s in steps:
            print(f'    {s["result"]:9s} {s["duration_s"]:6.1f} s  {s["step"]}', flush=True)

    log_dir = pathlib.Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    out = log_dir / f'nav_test_{time.strftime("%Y%m%d_%H%M%S")}.json'
    out.write_text(json.dumps({'standing_poses': STANDING_POSES, 'turn_point': TURN_POINT,
                               'legs': legs}, indent=2))
    print(f'log: {out}')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
