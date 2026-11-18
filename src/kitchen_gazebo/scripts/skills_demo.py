#!/usr/bin/env python3
"""Sets or clears the table by calling the robot skills directly (no behaviour tree).

    ros2 run kitchen_gazebo skills_demo.py --scenario set     # counter -> island
    ros2 run kitchen_gazebo skills_demo.py --scenario clean   # island -> counter

Used to test the skills; the behaviour tree (kitchen_transport_bt) runs the same steps with
retries, recovery and its explanation log. Every call and its message is printed and saved to
~/kitchen_skill_runs/<scenario>_<time>.jsonl.
"""
import argparse
import json
import os
import pathlib
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from kitchen_interfaces.srv import Skill

ORDER = ['milk', 'breakfast_cereal', 'bowl']   # the lab's order


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=['set', 'clean'], default='set')
    args = parser.parse_args()
    with open(os.path.join(get_package_share_directory('kitchen_gazebo'), 'config', 'scene.yaml')) as f:
        scene = yaml.safe_load(f)

    rclpy.init()
    node = rclpy.create_node('skills_demo')
    client = node.create_client(Skill, '/kitchen_skills/execute')
    if not client.wait_for_service(timeout_sec=60):
        raise SystemExit('kitchen_skills is not running (ros2 launch kitchen_gazebo kitchen_tasks.launch.py)')
    out = pathlib.Path.home() / 'kitchen_skill_runs'
    out.mkdir(exist_ok=True)
    log = (out / f'{args.scenario}_{time.strftime("%Y%m%d_%H%M%S")}.jsonl').open('w')

    def call(skill, **params):
        t0 = time.time()
        future = client.call_async(Skill.Request(skill=skill, params_json=json.dumps(params)))
        rclpy.spin_until_future_complete(node, future)
        r = future.result()
        entry = {'skill': skill, 'params': params, 'success': r.success, 'message': r.message,
                 'duration_s': round(time.time() - t0, 1)}
        log.write(json.dumps(entry) + '\n')
        log.flush()
        print(f'{"ok  " if r.success else "FAIL"} {skill:12s} {r.message}', flush=True)
        return r.success, json.loads(r.result_json or '{}')

    if args.scenario == 'set':
        source, target = 'counter', 'island'
        sources = {o: scene['objects'][o][:3] for o in ORDER}
        goals = {o: scene['table_poses'][o] for o in ORDER}
        order = ORDER
    else:
        source, target = 'island', 'counter'
        sources = {o: scene['table_poses'][o] + [scene['surfaces']['island'] + 0.08] for o in ORDER}
        goals = {o: scene['objects'][o][:2] for o in ORDER}
        order = list(reversed(ORDER))

    call('move_torso', height=scene['torso_height'])
    call('park_arms')
    results = {}
    for obj in order:
        ok, _ = call('navigate', to=source)
        sx, sy, sz = sources[obj]
        call('look_at', x=sx, y=sy, z=sz)
        ok, _ = call('detect', object=obj)
        if not ok:   # the lab's recovery: look again from where the robot really stands
            call('look_at', x=sx, y=sy, z=sz)
            ok, _ = call('detect', object=obj)
        if not ok or not call('pick', object=obj)[0]:
            results[obj] = 'not picked'
            continue
        call('navigate', to=target)
        gx, gy = goals[obj]
        ok, _ = call('place', object=obj, x=gx, y=gy, surface=target)
        results[obj] = 'placed' if ok else 'not placed'
    call('park_arms')
    for obj in order:
        gx, gy = goals[obj]
        call('object_pose', object=obj, x=gx, y=gy, tolerance=0.05)
    print('result:', results)
    log.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
