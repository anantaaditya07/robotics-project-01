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

## Targeted A/B after D-25 (negative-evidence track expiry), 2026-10-03

Routes that pass the person only: waypoint sequence 3 -> 4 -> 5 -> 6 -> 7 (`scripts/run_eval.sh 3
--goals 3,4,5,6,7`, `EVAL_NAME=_targeted`), 3 runs per configuration, everything else identical.
Raw CSV: `data/eval/goals_targeted.csv`.

| Goal (route) | Config | Succeeded | Min clearance to person [m] (min / mean) | Path [m] mean (min-max) |
|---|---|---|---|---|
| 4: (0.55, 0.55) -> (0.50, -0.55) | semantic_on | 1/3 | +0.22 / +0.35 | 0.81 (0.36-1.33) |
| | semantic_off | 3/3 | +0.20 / +0.22 | 1.24 (1.20-1.27) |
| 5: -> (0.55, -1.80) | semantic_on | 1/3 | +0.07 / +0.29 | 0.52 (0.00-1.42) |
| | semantic_off | 3/3 | +0.06 / +0.08 | 1.41 (1.38-1.44) |
| 7: (-0.70, -1.90) -> (1.75, 0.55) | semantic_on | 2/3 | +0.08 / +0.28 | 2.02 (0.00-4.19) |
| | semantic_off | 3/3 | +0.01 / +0.07 | 4.40 (4.28-4.47) |

All 5 goals of the sequence: semantic_on 7/15, semantic_off 15/15; 0 stale-TF errors in both.

**Result: with persistent tracks the semantic layer now changes behaviour, but harmfully.** The
higher mean clearances with the layer on mostly come from goals that aborted early (path ~0 m),
not from a wider berth on successful runs. Failures are DWB "No valid trajectories ...
BaseObstacle/Trajectory Hits Obstacle" (343x) and NavFn "failed to create plan" (49x): the robot
or its goal ends up inside lethal semantic discs.

**Cause (probe, one route 3 -> 4):** a single person produced five person tracks - three near the
true footprint centre (1.33, -0.59) and two ghosts at (1.01, -0.16) and (2.60, -0.86), created
when a fused position fell outside the 0.6 m association gate (close-range / partial boxes).
Before D-25 ghosts expired after 2 s; now they persist (up to max_age 120 s) and each paints a
LETHAL core of 0.35 m (0.57 m with the robot radius), e.g. the ghost at (1.01, -0.16) blocks the
gap between the person and pillar (1.1, 0).

Why the layer had no visible effect before D-25: NavFn does weigh it (cell cost = 50 + 0.8 x
costmap cost), but the person was usually not in the costmap when routes were planned (2 s TTL
after leaving view). DWB itself barely reacts to costs below lethal (BaseObstacle scale 0.02 vs
PathAlign/PathDist 32), so the effect has to come from the global plan.

**Next steps (not done):** confirm tracks before publishing (min hits), merge same-class tracks
closer than the class footprint, shorter max_age for low-hit tracks, and re-run this A/B.

## Targeted A/B after D-26 (confirmed/merged tracks, non-lethal semantic cost), 2026-10-03

Same protocol as above (goals 3 -> 7, 3 runs per configuration, `EVAL_NAME=_targeted_d26`).
Clearance and path are over **successful runs only**. Raw CSV: `data/eval/*_targeted_d26.csv`.

| Goal | Config | Succeeded | Min clearance to person [m] (min / mean) | Path [m] mean (min-max) | Time [s] |
|---|---|---|---|---|---|
| 4 | semantic_on | 3/3 | +0.17 / +0.21 | 1.21 (1.16-1.27) | 15.6 |
| 4 | semantic_off | 3/3 | +0.18 / +0.21 | 1.23 (1.21-1.27) | 14.1 |
| 5 | semantic_on | 3/3 | +0.06 / +0.07 | 1.42 (1.38-1.46) | 15.5 |
| 5 | semantic_off | 3/3 | +0.06 / +0.07 | 1.46 (1.41-1.53) | 16.6 |
| 7 | semantic_on | 3/3 | +0.01 / +0.03 | 4.52 (4.49-4.56) | 32.4 |
| 7 | semantic_off | 3/3 | +0.01 / +0.01 | 4.56 (4.47-4.74) | 29.6 |

All goals of the sequence: semantic_on 15/15, semantic_off 15/15; 0 stale-TF errors in both.

Distinct confirmed person track ids per run (one real person): on 3 / 3 / 6, off 3 / 5 / 4
(chair: 1-2). These are ids over a whole ~2 min run, not simultaneous duplicates: a track removed
by negative evidence (D-25) and re-detected later gets a new id. Simultaneous ghosts were not
logged separately in this run.

**Result: D-26 removed the harm (15/15 vs 7/15 before) but the layer still does not measurably
help.** Clearance and path length on these routes are the same with and without the layer,
within run-to-run spread. Likely reasons, not yet verified: (1) the routes past the person are
geometrically constrained (pillar column x = 1.1 m, walls), so NavFn has no materially longer
alternative that is cheaper; (2) a 200-capped ring decaying with k = 3 adds at most
0.8 x 200 = 160 per cell in NavFn's 50 + 0.8 x cost metric, comparable to the static inflation
already around the mapped person; (3) DWB follows the global path (obstacle critic 0.02 vs 32).
Showing the intended "wider berth" needs either an open-floor scenario with a real detour
option or a stronger/longer ring; it is not demonstrated by the current results.

## A/B on a map without the semantic objects (D-27), 2026-10-03

The static map no longer contains person, chair and bottle (they stay in the simulation), so
only live LiDAR and the SemanticLayer see them. D-25/D-26 tracker and non-lethal cost unchanged.
Clearance and path over successful runs only. Raw CSV: `data/eval/*_targeted_d27.csv`,
`data/eval/*_full_d27.csv`.

Targeted routes (goals 3 -> 7), 3 runs per configuration:

| Goal | Config | Succeeded | Min clearance to person [m] (min / mean) | Path [m] mean |
|---|---|---|---|---|
| 4 | semantic_on | 3/3 | +0.19 / +0.22 | 1.22 |
| 4 | semantic_off | 3/3 | +0.19 / +0.22 | 1.29 |
| 5 | semantic_on | 3/3 | +0.05 / +0.09 | 1.42 |
| 5 | semantic_off | 3/3 | +0.10 / +0.11 | 1.35 |
| 7 | semantic_on | 3/3 | -0.04 / -0.02 | 4.59 |
| 7 | semantic_off | 3/3 | -0.02 / -0.01 | 4.52 |

Full 10-goal sequence, 1 run per configuration: 10/10 both; min clearance -0.04 (on) vs -0.05
(off); total path 15.91 m (on) vs 16.08 m (off). 0 stale-TF errors in all runs.

Simultaneous person tracks (max in any single /semantic_obstacles message, one real person):
targeted on 1 / 1 / 1, off 3 / 1 / 2; full run on 2, off 2. Distinct ids per run 1-6. The
tracker is identical in both configurations (only the costmap layer is toggled), so the
difference is run-to-run variation; brief duplicates (2-3) still occur.

**Result: still no measurable benefit.** With the objects removed from the static map, the
semantic layer is the only source of cost beyond the LiDAR obstacle and its 0.55 m inflation,
yet clearance and path length match the layer-off runs within noise, and the closest approach
(goal 7, ~0.53 m centre to centre) is the same. The remaining candidate causes are the planner
and controller, not the map: NavFn's 50 + 0.8 x cost metric with a 200-capped ring that decays
with k = 3 (cost < 100 beyond ~0.6 m from the person centre), the constrained route geometry
around pillar column x = 1.1, and DWB's weak obstacle critic (0.02 vs 32 for path following).
Next diagnostic (not done): plot the global plan and costmap with the layer on vs off on goal 7.
