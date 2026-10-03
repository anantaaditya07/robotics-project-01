#!/usr/bin/env python3
"""Remove the semantic objects (person, chair, bottle) from a saved occupancy map (D-27).

The objects stay in the Gazebo world; only the static map loses them, so the static layer and
its inflation no longer cover them and only LiDAR (live) and the SemanticLayer see them.

    /usr/bin/python3 scripts/clear_map_objects.py            # semnav_world -> keeps a backup

Occupied cells within --radius of each object's footprint centre (ground truth from the world
file pose + collision-mesh centre offset) become free, except cells within --protect of a static
cylinder (pillar) of the turtlebot3_world model, so pillars next to an object are never erased.
The original map is copied to <name>_with_objects.{pgm,yaml} first (refuses to overwrite it).
"""

import argparse
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
FREE, OCCUPIED = 254, 0  # trinary PGM values (map_saver, negate 0)


def repo_path(p: str) -> Path:
    path = Path(p).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def pose_xy_yaw(text):
    v = [float(t) for t in (text or '').split()] + [0.0] * 6
    return v[0], v[1], v[5]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--map', default='src/semnav_bringup/maps/semnav_world.yaml')
    ap.add_argument('--world', default='src/semnav_bringup/worlds/semnav_world.world')
    ap.add_argument('--tb3-world-model',
                    default='/opt/ros/humble/share/turtlebot3_gazebo/models/'
                            'turtlebot3_world/model.sdf')
    # name=offset_x,offset_y,clear_radius (offset: footprint centre in the model frame, from the
    # collision-mesh bbox measured in Phase 5; radius covers the footprint plus LiDAR smear)
    ap.add_argument('--object', nargs='*', default=['person_1=-0.005,-0.071,0.40',
                                                    'chair_1=-0.019,0.001,0.35',
                                                    'bottle_1=0.0,0.0,0.15'])
    ap.add_argument('--protect', type=float, default=0.12,
                    help='m kept occupied beyond each static cylinder radius')
    args = ap.parse_args()

    map_yaml = repo_path(args.map)
    meta = yaml.safe_load(open(map_yaml))
    pgm = map_yaml.parent / meta['image']
    backup_yaml = map_yaml.with_name(map_yaml.stem + '_with_objects.yaml')
    backup_pgm = pgm.with_name(pgm.stem + '_with_objects.pgm')
    if backup_yaml.exists() or backup_pgm.exists():
        print(f'error: {backup_yaml.name} already exists; refusing to overwrite the original',
              file=sys.stderr)
        return 1
    shutil.copy(pgm, backup_pgm)
    bmeta = dict(meta)
    bmeta['image'] = backup_pgm.name
    yaml.safe_dump(bmeta, open(backup_yaml, 'w'), sort_keys=False)

    img = cv2.imread(str(pgm), cv2.IMREAD_UNCHANGED)
    h, w = img.shape
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

    # Static cylinders (pillars) of the arena model, world frame (model placed at the origin).
    pillars = []
    for col in ET.parse(args.tb3_world_model).getroot().iter('collision'):
        cyl = col.find('geometry/cylinder')
        if cyl is not None:
            px, py, _ = pose_xy_yaw(col.findtext('pose'))
            pillars.append((px, py, float(cyl.findtext('radius'))))

    world = ET.parse(repo_path(args.world)).getroot().find('world')
    poses = {inc.findtext('name'): pose_xy_yaw(inc.findtext('pose'))
             for inc in world.findall('include') if inc.findtext('name')}
    total = 0
    for spec in args.object:
        name, vals = spec.split('=')
        offx, offy, radius = (float(v) for v in vals.split(','))
        x, y, yaw = poses[name]
        cx = x + offx * math.cos(yaw) - offy * math.sin(yaw)
        cy = y + offx * math.sin(yaw) + offy * math.cos(yaw)
        cleared = 0
        for row in range(h):
            wy = oy + (h - 1 - row + 0.5) * res  # PGM row 0 is the top (max y)
            for c in range(w):
                wx = ox + (c + 0.5) * res
                if img[row, c] != OCCUPIED or math.hypot(wx - cx, wy - cy) > radius:
                    continue
                if any(math.hypot(wx - px, wy - py) <= pr + args.protect
                       for px, py, pr in pillars):
                    continue
                img[row, c] = FREE
                cleared += 1
        print(f'{name}: centre ({cx:+.3f}, {cy:+.3f}), radius {radius} m: {cleared} cells')
        total += cleared
    cv2.imwrite(str(pgm), img)
    print(f'{total} cells cleared; original kept as {backup_yaml.name} / {backup_pgm.name}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
