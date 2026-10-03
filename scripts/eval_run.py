#!/usr/bin/env python3
"""SemNav evaluation run: drive the waypoint list N times and log per-goal metrics (Phase 8).

Run with system python in a ROS-sourced shell (underlay included, D-20) while semnav.launch.py
runs headless; scripts/run_eval.sh does all of this:

    /usr/bin/python3 scripts/eval_run.py --label semantic_on --runs 5 --csv data/eval/goals.csv

Per goal (CSV row): label, run, goal index, goal x/y, result, time [s, sim], path length [m,
ground truth], min clearance robot->person [m, centre distance minus robot and person radii],
fused-detection error per class [m, mean over the goal, vs ground-truth footprint centre].
Per run: YOLO latency p50/p95 [ms] and fps from the last /metrics message (D-09 layout).
Ground truth comes from /gazebo/model_states.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Coverage waypoints from scripts/auto_map.py (world-derived, 0.5 m clearance), visiting order.
WAYPOINTS = [(-2.00, -0.05), (-1.75, 0.95), (-0.55, 0.55), (0.55, 0.55), (0.50, -0.55),
             (0.55, -1.80), (-0.70, -1.90), (1.75, 0.55), (0.55, 1.80), (-0.55, 1.80)]
# Footprint-centre offsets in each model's frame (from the collision mesh bbox, Phase 5 checks).
OBJECTS = {'person': ('person_1', (-0.005, -0.071)), 'chair': ('chair_1', (-0.019, 0.001))}
# /metrics layout (D-09): [pre p50/p95, infer p50/p95, post p50/p95, total p50/p95, fps, dropped]
M_TOTAL_P50, M_TOTAL_P95, M_FPS = 6, 7, 8


def repo_path(p: str) -> Path:
    path = Path(p).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--label', required=True, help='configuration label, e.g. semantic_on')
    ap.add_argument('--runs', type=int, default=5, help='passes over the waypoint list')
    ap.add_argument('--csv', default='data/eval/goals.csv', help='per-goal CSV (appended)')
    ap.add_argument('--runs-csv', default='data/eval/runs.csv', help='per-run CSV (appended)')
    ap.add_argument('--goals', default='',
                    help='comma-separated waypoint indices to drive, in order (default: all)')
    ap.add_argument('--goal-timeout', type=float, default=120.0, help='per-goal timeout (wall s)')
    ap.add_argument('--robot-entity', default='waffle', help='Gazebo entity name of the robot')
    ap.add_argument('--robot-radius', type=float, default=0.22, help='m, as in nav2_params')
    ap.add_argument('--person-radius', type=float, default=0.35, help='m, class_radius.person')
    args = ap.parse_args()

    import rclpy
    from gazebo_msgs.msg import ModelStates
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from semnav_msgs.msg import SemanticObstacleArray
    from std_msgs.msg import Float32MultiArray

    rclpy.init()
    node = Node('semnav_eval_run', parameter_overrides=[Parameter('use_sim_time', value=True)])
    st = {}
    metrics = {}
    goal_state = {'active': False}
    track_ids = {c: set() for c in OBJECTS}  # distinct published track ids per class, this run
    max_simul = {c: 0 for c in OBJECTS}  # max tracks of a class in a single message, this run

    def on_states(msg):
        st.update(dict(zip(msg.name, msg.pose)))
        if goal_state['active'] and args.robot_entity in st:
            p = st[args.robot_entity].position
            if goal_state['last'] is not None:
                goal_state['path'] += math.hypot(p.x - goal_state['last'][0],
                                                 p.y - goal_state['last'][1])
            goal_state['last'] = (p.x, p.y)
            if 'person_1' in st:
                cx, cy = gt_centre('person')
                clear = math.hypot(p.x - cx, p.y - cy) - args.robot_radius - args.person_radius
                goal_state['min_clear'] = min(goal_state['min_clear'], clear)

    def gt_centre(cls):
        name, (ox, oy) = OBJECTS[cls]
        q = st[name]
        th = yaw_of(q.orientation)
        return (q.position.x + ox * math.cos(th) - oy * math.sin(th),
                q.position.y + ox * math.sin(th) + oy * math.cos(th))

    def on_obstacles(msg):
        for o in msg.obstacles:
            if o.class_name in track_ids:
                track_ids[o.class_name].add(o.id)
        for c in max_simul:
            max_simul[c] = max(max_simul[c], sum(o.class_name == c for o in msg.obstacles))
        if not goal_state['active']:
            return
        for cls in OBJECTS:
            if OBJECTS[cls][0] not in st:
                continue
            gx, gy = gt_centre(cls)
            cand = [o for o in msg.obstacles if o.class_name == cls]
            if cand:
                e = min(math.hypot(o.position.x - gx, o.position.y - gy) for o in cand)
                goal_state['det'][cls].append(e)

    node.create_subscription(ModelStates, '/gazebo/model_states', on_states, 10)
    node.create_subscription(SemanticObstacleArray, '/semantic_obstacles', on_obstacles, 10)
    node.create_subscription(Float32MultiArray, '/metrics',
                             lambda m: metrics.__setitem__('last', list(m.data)), 10)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=60.0):
        print('error: navigate_to_pose not available', file=sys.stderr)
        return 1

    goals_csv = repo_path(args.csv)
    runs_csv = repo_path(args.runs_csv)
    goals_csv.parent.mkdir(parents=True, exist_ok=True)
    g_new = not goals_csv.exists()
    r_new = not runs_csv.exists()
    gf = open(goals_csv, 'a', newline='')
    rf = open(runs_csv, 'a', newline='')
    gw = csv.writer(gf)
    rw = csv.writer(rf)
    if g_new:
        gw.writerow(['label', 'run', 'goal', 'x', 'y', 'result', 'time_s', 'path_m',
                     'min_person_clearance_m', 'person_err_m', 'chair_err_m'])
    if r_new:
        rw.writerow(['label', 'run', 'succeeded', 'goals', 'yolo_total_p50_ms',
                     'yolo_total_p95_ms', 'yolo_fps', 'person_tracks', 'chair_tracks',
                     'max_simultaneous_person', 'max_simultaneous_chair'])

    def mean(v):
        return sum(v) / len(v) if v else float('nan')

    order = [int(i) for i in args.goals.split(',')] if args.goals else list(range(len(WAYPOINTS)))
    for run in range(1, args.runs + 1):
        ok = 0
        for ids in track_ids.values():
            ids.clear()
        for c in max_simul:
            max_simul[c] = 0
        for gi in order:
            x, y = WAYPOINTS[gi]
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.w = 1.0
            goal_state.update(active=True, path=0.0, last=None, min_clear=float('inf'),
                              det={c: [] for c in OBJECTS})
            t0 = node.get_clock().now()
            fut = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, fut)
            handle = fut.result()
            result = 'REJECTED'
            if handle.accepted:
                rfut = handle.get_result_async()
                rclpy.spin_until_future_complete(node, rfut, timeout_sec=args.goal_timeout)
                if rfut.done():
                    result = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(
                        rfut.result().status, str(rfut.result().status))
                else:
                    handle.cancel_goal_async()
                    result = 'TIMEOUT'
            goal_state['active'] = False
            dt = (node.get_clock().now() - t0).nanoseconds * 1e-9
            ok += result == 'SUCCEEDED'
            row = [args.label, run, gi, x, y, result, round(dt, 2), round(goal_state['path'], 3),
                   round(goal_state['min_clear'], 3), round(mean(goal_state['det']['person']), 3),
                   round(mean(goal_state['det']['chair']), 3)]
            gw.writerow(row)
            gf.flush()
            print(' '.join(str(v) for v in row), flush=True)
        m = metrics.get('last', [float('nan')] * 10)
        rw.writerow([args.label, run, ok, len(order), round(m[M_TOTAL_P50], 2),
                     round(m[M_TOTAL_P95], 2), round(m[M_FPS], 2),
                     len(track_ids['person']), len(track_ids['chair']),
                     max_simul['person'], max_simul['chair']])
        rf.flush()
        print(f'run {run}: {ok}/{len(order)} succeeded', flush=True)
    gf.close()
    rf.close()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
