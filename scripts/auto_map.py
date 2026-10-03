#!/usr/bin/env python3
"""Drive the robot through the arena with Nav2 so slam_toolbox maps all free space.

Run with system python in a ROS-sourced shell while mapping + navigation run:

    ros2 launch semnav_bringup mapping.launch.py gui:=false rviz:=false
    ros2 launch semnav_bringup navigation.launch.py
    source /opt/ros/humble/setup.bash && /usr/bin/python3 scripts/auto_map.py

Waypoints come from the world file, not from hand-picked numbers: every collision shape
(cylinders, boxes, mesh triangles) of the world's models and the included object models is
rasterised onto a 2D grid; free space is flood-filled from the robot spawn; cells closer than
--clearance to any obstacle are dropped; a greedy cover picks waypoints --spacing apart and
orders them nearest-neighbour from the spawn.

Per goal it logs the NavigateToPose result, the slam pose error vs Gazebo ground truth
(/gazebo/model_states) and the age of the map->odom transform (a stale map->odom is what
breaks Nav2 with "extrapolation into the future" errors).

--dry-run only plans: prints the waypoints and writes a PNG of the plan; no ROS needed.
"""

import argparse
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
COLLADA_NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def repo_path(p: str) -> Path:
    path = Path(p).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


# ---------------------------------------------------------------- world geometry (no ROS)

def pose_matrix(text):
    """SDF '<pose>x y z r p y</pose>' -> 3x3 planar transform (x, y, yaw only)."""
    v = [float(t) for t in (text or '').split()] + [0.0] * 6
    x, y, yaw = v[0], v[1], v[5]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1]])


def find_model_dir(uri, model_paths):
    name = uri.replace('model://', '').strip('/')
    for base in model_paths:
        d = Path(base) / name
        if (d / 'model.sdf').exists() or (d / 'model.config').exists():
            return d
    raise FileNotFoundError(f'model {uri} not found in GAZEBO_MODEL_PATH {model_paths}')


def model_sdf_file(model_dir):
    """Pick the SDF named in model.config (falls back to model.sdf)."""
    cfg = model_dir / 'model.config'
    if cfg.exists():
        sdfs = ET.parse(cfg).getroot().findall('sdf')
        if sdfs:
            return model_dir / sorted(sdfs, key=lambda e: e.get('version', '0'))[-1].text.strip()
    return model_dir / 'model.sdf'


def mesh_segments(dae_path, scale):
    """Triangle edges of a COLLADA mesh in metres, projected to the xy plane."""
    root = ET.parse(dae_path).getroot()
    unit = root.find('.//c:asset/c:unit', COLLADA_NS)
    metre = float(unit.get('meter', 1.0)) if unit is not None else 1.0
    segs = []
    for mesh in root.iterfind('.//c:mesh', COLLADA_NS):
        arrays = {s.get('id'): np.array(s.find('c:float_array', COLLADA_NS).text.split(), float)
                  for s in mesh.findall('c:source', COLLADA_NS)}
        vert = mesh.find('c:vertices', COLLADA_NS)
        pos_src = vert.find("c:input[@semantic='POSITION']", COLLADA_NS).get('source')[1:]
        pts = arrays[pos_src].reshape(-1, 3)[:, :2] * metre * scale
        for tri in mesh.iterfind('c:triangles', COLLADA_NS):
            stride = len(tri.findall('c:input', COLLADA_NS))
            vin = tri.find("c:input[@semantic='VERTEX']", COLLADA_NS)
            off = int(vin.get('offset', 0))
            idx = np.array(tri.find('c:p', COLLADA_NS).text.split(), int)[off::stride]
            for t in idx.reshape(-1, 3):
                for a, b in ((0, 1), (1, 2), (2, 0)):
                    segs.append((pts[t[a]], pts[t[b]]))
    return segs


def collect_shapes(model_root, model_dir, model_tf, model_paths, shapes, radius_override=None):
    """Append ('circle', centre, r) / ('segment', p, q) shapes of one model in world frame."""
    if radius_override is not None:
        shapes.append(('circle', model_tf[:2, 2].copy(), radius_override))
        return
    for link in model_root.iter('link'):
        link_tf = model_tf @ pose_matrix(link.findtext('pose'))
        for col in link.findall('collision'):
            tf = link_tf @ pose_matrix(col.findtext('pose'))
            geom = col.find('geometry')
            if geom is None:
                continue
            if geom.find('cylinder') is not None:
                r = float(geom.find('cylinder').findtext('radius'))
                shapes.append(('circle', tf[:2, 2].copy(), r))
            elif geom.find('sphere') is not None:
                r = float(geom.find('sphere').findtext('radius'))
                shapes.append(('circle', tf[:2, 2].copy(), r))
            elif geom.find('box') is not None:
                sx, sy, _ = (float(t) for t in geom.find('box').findtext('size').split())
                corners = np.array([[sx, sy], [-sx, sy], [-sx, -sy], [sx, -sy]]) / 2
                w = (tf[:2, :2] @ corners.T).T + tf[:2, 2]
                for i in range(4):
                    shapes.append(('segment', w[i], w[(i + 1) % 4]))
            elif geom.find('mesh') is not None:
                m = geom.find('mesh')
                uri = m.findtext('uri').strip()
                scale = float((m.findtext('scale') or '1 1 1').split()[0])
                rel = uri.replace('model://', '')
                dae = find_model_dir('model://' + rel.split('/')[0], model_paths) / '/'.join(
                    rel.split('/')[1:])
                for p, q in mesh_segments(dae, scale):
                    shapes.append(('segment', tf[:2, :2] @ p + tf[:2, 2],
                                   tf[:2, :2] @ q + tf[:2, 2]))


def world_shapes(world_file, model_paths, object_radius):
    """All obstacle shapes of the world. Models named in object_radius become discs."""
    world = ET.parse(world_file).getroot().find('world')
    shapes = []
    for model in world.findall('model'):
        tf = pose_matrix(model.findtext('pose'))
        collect_shapes(model, None, tf, model_paths, shapes)
        for inc in model.findall('include'):
            d = find_model_dir(inc.findtext('uri').strip(), model_paths)
            sub = ET.parse(model_sdf_file(d)).getroot().find('model')
            collect_shapes(sub, d, tf @ pose_matrix(inc.findtext('pose') or sub.findtext('pose')),
                           model_paths, shapes)
    for inc in world.findall('include'):
        uri = inc.findtext('uri').strip()
        name = inc.findtext('name') or uri.replace('model://', '')
        if uri in ('model://ground_plane', 'model://sun'):
            continue
        d = find_model_dir(uri, model_paths)
        sub = ET.parse(model_sdf_file(d)).getroot().find('model')
        collect_shapes(sub, d, pose_matrix(inc.findtext('pose')), model_paths, shapes,
                       radius_override=object_radius.get(name, object_radius.get('default')))
    return shapes


def plan_waypoints(shapes, start, clearance, spacing, res, half_extent):
    """Return (waypoints [N,2], grid info) covering the free space reachable from start."""
    n = int(2 * half_extent / res)
    occ = np.zeros((n, n), np.uint8)

    def px(p):
        return (int(round((p[0] + half_extent) / res)), int(round((p[1] + half_extent) / res)))

    for s in shapes:
        if s[0] == 'circle':
            cv2.circle(occ, px(s[1]), max(1, int(round(s[2] / res))), 255, -1)
        else:
            cv2.line(occ, px(s[1]), px(s[2]), 255, 1)
    # Free space reachable from the spawn (flood fill stops at obstacle pixels).
    reach = np.zeros((n + 2, n + 2), np.uint8)
    cv2.floodFill(occ.copy(), reach, px(start), 128, flags=4 | cv2.FLOODFILL_MASK_ONLY | (1 << 8))
    reach = reach[1:-1, 1:-1].astype(bool)
    if reach.sum() > 0.9 * n * n:
        raise RuntimeError('free space leaks to the grid border: arena is not closed')
    dist = cv2.distanceTransform((occ == 0).astype(np.uint8), cv2.DIST_L2, 5) * res
    ok = reach & (dist >= clearance)
    ys, xs = np.nonzero(ok)
    cand = np.stack([xs * res - half_extent, ys * res - half_extent], 1)
    # Greedy cover: keep candidates at least `spacing` apart, most-clear first.
    order = np.argsort(-dist[ys, xs])
    chosen = []
    for i in order:
        if all(np.hypot(*(cand[i] - c)) >= spacing for c in chosen):
            chosen.append(cand[i])
    # Nearest-neighbour tour from the spawn.
    tour, cur, left = [], np.asarray(start, float), list(range(len(chosen)))
    while left:
        j = min(left, key=lambda k: np.hypot(*(chosen[k] - cur)))
        tour.append(chosen[j])
        cur = chosen[j]
        left.remove(j)
    return np.array(tour), dict(occ=occ, ok=ok, res=res, half=half_extent)


def draw_plan(path, grid, start, wps):
    img = cv2.cvtColor(255 - grid['occ'], cv2.COLOR_GRAY2BGR)
    img[grid['ok']] = (200, 255, 200)

    def px(p):
        return (int(round((p[0] + grid['half']) / grid['res'])),
                int(round((p[1] + grid['half']) / grid['res'])))
    prev = px(start)
    for i, w in enumerate(wps):
        cv2.line(img, prev, px(w), (255, 0, 0), 1)
        cv2.circle(img, px(w), 3, (0, 0, 255), -1)
        cv2.putText(img, str(i), px(w), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
        prev = px(w)
    cv2.imwrite(str(path), cv2.flip(img, 0))  # +y up


# ---------------------------------------------------------------------------- ROS driving

def drive(args, wps):
    import rclpy
    from gazebo_msgs.msg import ModelStates
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    def yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    rclpy.init()
    node = Node('auto_map', parameter_overrides=[Parameter('use_sim_time', value=True)])
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    truth = {}

    def on_states(msg):
        if args.robot_entity in msg.name:
            truth['pose'] = msg.pose[msg.name.index(args.robot_entity)]
    node.create_subscription(ModelStates, '/gazebo/model_states', on_states, 10)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=30.0):
        print('error: navigate_to_pose action server not available', file=sys.stderr)
        return 1

    def status():
        try:
            mo = tf_buffer.lookup_transform('map', 'odom', Time())
            mb = tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except Exception as e:  # noqa: BLE001 - report any TF failure
            return f'tf unavailable ({e.__class__.__name__})'
        age = (node.get_clock().now() - Time.from_msg(mo.header.stamp)).nanoseconds * 1e-9
        out = f'map->odom age {age:+.2f} s'
        if 'pose' in truth:
            t, g = mb.transform, truth['pose']
            dyaw = yaw_of(t.rotation) - yaw_of(g.orientation)
            dyaw = math.degrees(math.atan2(math.sin(dyaw), math.cos(dyaw)))
            err = math.hypot(t.translation.x - g.position.x, t.translation.y - g.position.y)
            out += f', slam vs truth {err:.3f} m {dyaw:+.1f} deg'
        return out

    results = []
    prev = np.array([args.start_x, args.start_y])
    for i, w in enumerate(wps):
        heading = math.atan2(w[1] - prev[1], w[0] - prev[0])
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(w[0]), float(w[1])
        goal.pose.pose.orientation.z = math.sin(heading / 2)
        goal.pose.pose.orientation.w = math.cos(heading / 2)
        t0 = time.time()
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut)
        handle = fut.result()
        if not handle.accepted:
            results.append('REJECTED')
            print(f'[{i:2d}] ({w[0]:+.2f}, {w[1]:+.2f}) REJECTED', flush=True)
            continue
        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, rfut, timeout_sec=args.goal_timeout)
        if not rfut.done():
            handle.cancel_goal_async()
            res = 'TIMEOUT'
        else:
            code = rfut.result().status
            res = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(code, str(code))
        # Let slam catch up before reporting.
        end = time.time() + args.settle
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        results.append(res)
        print(f'[{i:2d}] ({w[0]:+.2f}, {w[1]:+.2f}) {res:9s} {time.time() - t0:5.1f} s  '
              f'{status()}', flush=True)
        prev = w
    ok = results.count('SUCCEEDED')
    print(f'done: {ok}/{len(results)} SUCCEEDED')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok == len(results) else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--world', default='src/semnav_bringup/worlds/semnav_world.world',
                    help='world file (relative to repo root)')
    ap.add_argument('--start-x', type=float, default=-2.0, help='robot spawn x (m)')
    ap.add_argument('--start-y', type=float, default=-0.5, help='robot spawn y (m)')
    ap.add_argument('--clearance', type=float, default=0.5,
                    help='min distance from waypoint to any obstacle (m)')
    ap.add_argument('--spacing', type=float, default=1.0, help='min distance between waypoints (m)')
    ap.add_argument('--resolution', type=float, default=0.05, help='planning grid resolution (m)')
    ap.add_argument('--half-extent', type=float, default=6.0, help='planning grid half size (m)')
    ap.add_argument('--object-radius', nargs='*',
                    default=['person_1=0.3', 'chair_1=0.35', 'bottle_1=0.05', 'default=0.3'],
                    help='footprint radius per included object name (name=metres)')
    ap.add_argument('--robot-entity', default='waffle', help='Gazebo entity name of the robot')
    ap.add_argument('--goal-timeout', type=float, default=90.0, help='per-goal timeout (wall s)')
    ap.add_argument('--settle', type=float, default=1.0, help='pause after each goal (wall s)')
    ap.add_argument('--plan-image', default='data/auto_map_plan.png',
                    help='PNG of the waypoint plan (relative to repo root)')
    ap.add_argument('--dry-run', action='store_true', help='plan only, do not drive')
    args = ap.parse_args()

    model_paths = [str(REPO_ROOT / 'src/semnav_bringup/models'),
                   str(REPO_ROOT / 'src/semnav_bringup/models_external')]
    model_paths += [p for p in os.environ.get('GAZEBO_MODEL_PATH', '').split(':') if p]
    model_paths.append('/opt/ros/humble/share/turtlebot3_gazebo/models')
    radii = {k: float(v) for k, v in (s.split('=') for s in args.object_radius)}

    start = (args.start_x, args.start_y)
    shapes = world_shapes(repo_path(args.world), model_paths, radii)
    wps, grid = plan_waypoints(shapes, start, args.clearance, args.spacing,
                               args.resolution, args.half_extent)
    img = repo_path(args.plan_image)
    img.parent.mkdir(parents=True, exist_ok=True)
    draw_plan(img, grid, start, wps)
    print(f'{len(shapes)} obstacle shapes, {len(wps)} waypoints (clearance {args.clearance} m, '
          f'spacing {args.spacing} m), plan image {img}')
    for i, w in enumerate(wps):
        print(f'  [{i:2d}] ({w[0]:+.2f}, {w[1]:+.2f})')
    if args.dry_run:
        return 0
    return drive(args, wps)


if __name__ == '__main__':
    sys.exit(main())
