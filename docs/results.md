# SemNav results

Generated 2026-10-03 by `scripts/run_eval.sh` (raw CSV in `data/eval/`).
Simulation: Gazebo Classic, TurtleBot3 waffle, `semnav_world`, headless, CPU-only ONNX
Runtime (4 threads), CycloneDDS, tf2 underlay 0.25.24 (D-20). Each run drives the same 10
waypoints (from `scripts/auto_map.py`) with Nav2 + safety gate; only
`semantic_layer.enabled` differs between the two configurations.

| Configuration | Runs | Goals succeeded | Mean time / goal [s] | Mean path / goal [m] | Min clearance to person [m] (worst run / mean of runs) | Person pos. error [m] | Chair pos. error [m] | YOLO total p50 / p95 [ms] | YOLO fps |
|---|---|---|---|---|---|---|---|---|---|
| semantic_on | 5 | 50/50 (100 %) | 18.5 | 1.81 | -0.00 / 0.01 | 0.34 | 0.20 | 34.6 / 40.5 | 15.0 |
| semantic_off | 5 | 50/50 (100 %) | 18.3 | 1.79 | 0.00 / 0.03 | 0.30 | 0.21 | 34.3 / 40.3 | 15.0 |

Definitions:
- Time and path: successful goals only; path is the ground-truth (Gazebo) path length.
- Clearance: min over the run of (robot centre to person footprint centre) minus robot
  radius 0.22 m and person radius 0.35 m; negative = footprints would overlap.
- Position error: fused /semantic_obstacles track vs ground-truth footprint centre,
  mean over all messages while tracked (D-21 push-out 0.5, D-22 nearest cluster).
- YOLO latency: total per-frame time (pre + inference + post) from /metrics.
## Observations (hand-written review, 2026-10-03)

- Both configurations: 100 % goal success, 0 stale-TF errors (D-20 fix holds), YOLO steady at
  15 fps with total latency p50 ~34.5 ms / p95 ~40.5 ms.
- **The semantic layer made no measurable difference in this benchmark.** Paths, times and
  clearances are within run-to-run noise. The minimum person clearance (~0.00 m, i.e. 0.57 m
  centre-to-centre: robot radius 0.22 m + person cost radius 0.35 m, not a collision) comes
  from goal 7, (-0.70, -1.90) -> (1.75, 0.55), which passes beside the person in both
  configurations (path 4.51 vs 4.49 m). Untested hypothesis: a person track expires 2 s (ttl)
  after leaving the camera view, so when that route is planned the person is usually not yet in
  the costmap; by the time it is detected the robot is already alongside. A scenario with the
  person in view at planning time (or a longer-lived obstacle memory) is needed to show the
  intended wider berth. Static costmap evidence of the wider person ring: see CHECKLIST Phase 6.
- **Chair position error is viewpoint dependent.** Median 0.05-0.13 m at most goals, but 0.58 m
  (5/5 runs, both configurations) while driving to goal 2 (-0.55, 0.55), where the LiDAR sector
  of the chair box picks up a different surface than the chair's footprint centre.
