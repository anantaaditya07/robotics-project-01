#!/usr/bin/env python3
"""Summarise SemNav evaluation CSVs (scripts/eval_run.py) into a markdown results table.

    python3 scripts/eval_summary.py --goals data/eval/goals.csv --runs data/eval/runs.csv \\
        --out docs/results.md
"""

import argparse
import csv
import math
import sys
from datetime import date
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def repo_path(p: str) -> Path:
    path = Path(p).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def num(v):
    try:
        x = float(v)
    except ValueError:
        return None
    return None if math.isnan(x) or math.isinf(x) else x


def mean(v):
    v = [x for x in v if x is not None]
    return sum(v) / len(v) if v else None


def fmt(x, nd=2):
    return '-' if x is None else f'{x:.{nd}f}'


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--goals', default='data/eval/goals.csv', help='per-goal CSV')
    ap.add_argument('--runs', default='data/eval/runs.csv', help='per-run CSV')
    ap.add_argument('--out', default='docs/results.md', help='markdown output')
    args = ap.parse_args()

    goals = list(csv.DictReader(open(repo_path(args.goals))))
    runs = list(csv.DictReader(open(repo_path(args.runs))))
    if not goals:
        print('error: no goal rows', file=sys.stderr)
        return 1
    labels = list(dict.fromkeys(g['label'] for g in goals))

    rows = []
    for lb in labels:
        g = [r for r in goals if r['label'] == lb]
        ok = [r for r in g if r['result'] == 'SUCCEEDED']
        rn = [r for r in runs if r['label'] == lb]
        per_run_min = {}
        for r in g:
            c = num(r['min_person_clearance_m'])
            if c is not None:
                per_run_min[r['run']] = min(per_run_min.get(r['run'], c), c)
        rows.append({
            'label': lb,
            'runs': len({r['run'] for r in g}),
            'success': f"{len(ok)}/{len(g)} ({100.0 * len(ok) / len(g):.0f} %)",
            'time': mean(num(r['time_s']) for r in ok),
            'path': mean(num(r['path_m']) for r in ok),
            'clear_min': min(per_run_min.values()) if per_run_min else None,
            'clear_mean': mean(per_run_min.values()),
            'person_err': mean(num(r['person_err_m']) for r in g),
            'chair_err': mean(num(r['chair_err_m']) for r in g),
            'p50': mean(num(r['yolo_total_p50_ms']) for r in rn),
            'p95': mean(num(r['yolo_total_p95_ms']) for r in rn),
            'fps': mean(num(r['yolo_fps']) for r in rn),
        })

    out = [
        '# SemNav results',
        '',
        f'Generated {date.today().isoformat()} by `scripts/run_eval.sh` (raw CSV in `data/eval/`).',
        'Simulation: Gazebo Classic, TurtleBot3 waffle, `semnav_world`, headless, CPU-only ONNX',
        'Runtime (4 threads), CycloneDDS, tf2 underlay 0.25.24 (D-20). Each run drives the same 10',
        'waypoints (from `scripts/auto_map.py`) with Nav2 + safety gate; only',
        '`semantic_layer.enabled` differs between the two configurations.',
        '',
        '| Configuration | Runs | Goals succeeded | Mean time / goal [s] | Mean path / goal [m] '
        '| Min clearance to person [m] (worst run / mean of runs) | Person pos. error [m] '
        '| Chair pos. error [m] | YOLO total p50 / p95 [ms] | YOLO fps |',
        '|---|---|---|---|---|---|---|---|---|---|',
    ]
    for r in rows:
        out.append(f"| {r['label']} | {r['runs']} | {r['success']} | {fmt(r['time'], 1)} | "
                   f"{fmt(r['path'])} | {fmt(r['clear_min'])} / {fmt(r['clear_mean'])} | "
                   f"{fmt(r['person_err'])} | {fmt(r['chair_err'])} | "
                   f"{fmt(r['p50'], 1)} / {fmt(r['p95'], 1)} | {fmt(r['fps'], 1)} |")
    out += [
        '',
        'Definitions:',
        '- Time and path: successful goals only; path is the ground-truth (Gazebo) path length.',
        '- Clearance: min over the run of (robot centre to person footprint centre) minus robot',
        '  radius 0.22 m and person radius 0.35 m; negative = footprints would overlap.',
        '- Position error: fused /semantic_obstacles track vs ground-truth footprint centre,',
        '  mean over all messages while tracked (D-21 push-out 0.5, D-22 nearest cluster).',
        '- YOLO latency: total per-frame time (pre + inference + post) from /metrics.',
        '',
    ]
    p = repo_path(args.out)
    p.write_text('\n'.join(out))
    print('\n'.join(out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
