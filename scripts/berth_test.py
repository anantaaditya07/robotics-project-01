#!/usr/bin/env python3
"""D-28 wider-berth test: drive a goal pair past the person on open floor, N runs.

Run with system python (ROS sourced, underlay included) while semnav.launch.py runs with
world:=.../semnav_world_open.world:

    /usr/bin/python3 scripts/berth_test.py --label semantic_on --runs 3 --png data/eval/berth_on.png

Each run: go to START facing the person (so it is detected and confirmed), wait --settle s,
then go to GOAL. Logs min clearance to the person (centre distance minus robot 0.22 and person
0.35 radii) and ground-truth path length for START -> GOAL. On the first run the global costmap
(raw) and the first global plan after the goal is sent are saved as a PNG.
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
PERSON = ('person_1', (-0.005, -0.071))  # footprint-centre offset in the model frame


def repo_path(p):
    path = Path(p).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--label', required=True)
    ap.add_argument('--runs', type=int, default=3)
    ap.add_argument('--start', default='-2.0,-0.95,1.5708', help='x,y,yaw (faces the person)')
    ap.add_argument('--goal', default='-2.0,0.95,1.5708', help='x,y,yaw')
    ap.add_argument('--settle', type=float, default=4.0, help='s at START before the pass')
    ap.add_argument('--csv', default='data/eval/berth.csv')
    ap.add_argument('--png', default='')
    ap.add_argument('--robot-radius', type=float, default=0.22)
    ap.add_argument('--person-radius', type=float, default=0.35)
    args = ap.parse_args()

    import cv2
    import numpy as np
    import rclpy
    from gazebo_msgs.msg import ModelStates
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import Path as PathMsg
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    n = Node('berth_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
    st, d = {}, {'plans': []}
    g = {'on': False, 'path': 0.0, 'last': None, 'clear': float('inf')}

    def centre():
        name, (ox, oy) = PERSON
        p = st[name]
        th = yaw_of(p.orientation)
        return (p.position.x + ox * math.cos(th) - oy * math.sin(th),
                p.position.y + ox * math.sin(th) + oy * math.cos(th))

    def on_states(m):
        st.update(dict(zip(m.name, m.pose)))
        if g['on'] and 'waffle' in st and PERSON[0] in st:
            p = st['waffle'].position
            if g['last']:
                g['path'] += math.hypot(p.x - g['last'][0], p.y - g['last'][1])
            g['last'] = (p.x, p.y)
            cx, cy = centre()
            g['clear'] = min(g['clear'], math.hypot(p.x - cx, p.y - cy)
                             - args.robot_radius - args.person_radius)

    n.create_subscription(ModelStates, '/gazebo/model_states', on_states, 10)
    n.create_subscription(Costmap, '/global_costmap/costmap_raw',
                          lambda m: d.__setitem__('cm', m),
                          QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(PathMsg, '/plan', lambda m: d['plans'].append(m), 10)
    ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    ac.wait_for_server()

    def go(x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        f = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(n, f)
        r = f.result().get_result_async()
        rclpy.spin_until_future_complete(n, r, timeout_sec=120)
        return {4: 'SUCCEEDED', 6: 'ABORTED'}.get(r.result().status, 'OTHER') if r.done() else 'TIMEOUT'

    def spin(t):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)

    def save_png(path):
        cm, plan = d.get('cm'), (d['plans'][0] if d['plans'] else None)
        if cm is None:
            return
        md = cm.metadata
        img = np.array(cm.data, np.uint8).reshape(md.size_y, md.size_x)
        vis = cv2.applyColorMap(255 - img, cv2.COLORMAP_BONE)
        vis[img == 255] = (90, 90, 90)

        def px(x, y):
            return (int((x - md.origin.position.x) / md.resolution),
                    int((y - md.origin.position.y) / md.resolution))
        if plan:
            pts = [px(p.pose.position.x, p.pose.position.y) for p in plan.poses]
            for a, b in zip(pts, pts[1:]):
                cv2.line(vis, a, b, (0, 0, 255), 1)
        cx, cy = centre()
        cv2.circle(vis, px(cx, cy), 2, (0, 200, 0), -1)
        vis = cv2.resize(cv2.flip(vis, 0), None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST)
        cv2.putText(vis, f'{args.label}: global costmap (raw) + first plan, green = person',
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        out = repo_path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out), vis)

    sx, sy, syaw = (float(v) for v in args.start.split(','))
    gx, gy, gyaw = (float(v) for v in args.goal.split(','))
    out = repo_path(args.csv)
    out.parent.mkdir(parents=True, exist_ok=True)
    new = not out.exists()
    w = csv.writer(open(out, 'a', newline=''))
    if new:
        w.writerow(['label', 'run', 'result', 'min_person_clearance_m', 'path_m'])
    spin(2.0)
    for run in range(1, args.runs + 1):
        go(sx, sy, syaw)
        spin(args.settle)
        g.update(on=True, path=0.0, last=None, clear=float('inf'))
        d['plans'] = []
        res = go(gx, gy, gyaw)
        g['on'] = False
        if run == 1 and args.png:
            save_png(args.png)
        w.writerow([args.label, run, res, round(g['clear'], 3), round(g['path'], 3)])
        print(f'{args.label} run {run}: {res} clearance {g["clear"]:+.3f} m path {g["path"]:.2f} m',
              flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
