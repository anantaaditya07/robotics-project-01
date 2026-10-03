# SemNav design notes (interview walkthrough)

Short, talk-through versions of the key decisions; `docs/DECISIONS.md` is the full record with evidence and every option.

---

## D-01 / D-02: the cmd_vel chain

**Problem.** The architecture requires `safety_gate_node` to be the *only* publisher of `/cmd_vel`. Stock Nav2 Humble bringup breaks that in two ways. `velocity_smoother` subscribes `/cmd_vel_nav` and publishes `/cmd_vel` (D-01). `behavior_server` (spin, backup) publishes on relative `cmd_vel`, so recoveries skip the gate entirely (D-02).

**Options.** (A) Write our own navigation launch with no smoother. (B) Keep the smoother: controller -> smoother -> `/cmd_vel_smoothed` -> gate, which adds a topic the PDF doesn't have. (C) Use the stock bringup and override its remaps (fragile). For recoveries: remap them through the gate, or leave two publishers.

**Chosen.** A for both. `semnav_bringup/launch/navigation.launch.py` remaps `cmd_vel -> cmd_vel_nav` on both `controller_server` and `behavior_server` and starts no `velocity_smoother`:

```
controller_server ─┐
                   ├─> /cmd_vel_nav ─> safety_gate_node (+ /scan) ─> /cmd_vel ─> robot
behavior_server  ──┘
```

Why: the topics and nodes match the PDF exactly, and the gate already has its own acceleration limiter, so the smoother would do the same job twice. Routing recoveries through the gate created a new problem, because backup drives in reverse. That led to D-13.

**Verified.** Headless check in commit 91a679b: `/cmd_vel` has exactly one publisher (`safety_gate_node`), and goals succeed. The D-16 `topic_tools` relay is still available, but only as a debug arg that is ignored while the gate runs.

**Next.** Without the smoother, the gate's limiter is the only smoothing, and it only limits *increases* of |v| (D-23). If deceleration jerk turned out to matter on real hardware, I would look at that first.

---

## D-05: camera optical frame convention

**Problem.** Fusion builds rays `((u-cx)/fx, 0, 1)` in a REP-103 *optical* frame (x right, y down, z forward). The stock TurtleBot3 waffle camera plugin has no `frame_name`, so images are stamped `camera_rgb_frame`, which is an x-forward *body* frame. If you rotate optical rays with a TF lookup from the header frame, every bearing comes out 90 degrees wrong. The URDF does provide `camera_rgb_frame -> camera_rgb_optical_frame` (rpy -1.57 0 -1.57).

**Options.** (A) Add a fusion parameter naming the optical frame and use it instead of the header frame. (B) Fix the data at the source: in our copied waffle SDF (already needed for D-06, 640x480 @ 15 Hz), set `<frame_name>camera_rgb_optical_frame</frame_name>`.

**Chosen.** B, so the data describes itself correctly. Three rules came with it:
- `semantic_fusion_node` reads the camera frame from the detection header and the laser frame from the scan header, with no hard-coded frame names.
- If TF can't resolve camera -> laser at the image stamp, the node logs an error and drops the detection. It never guesses a transform.
- `fx` and `cx` always come from `/camera/camera_info`.

In `fusion.hpp` only the rotation is used for bearing: `yaw = atan2(R*ray)`. The few-cm camera-to-LiDAR offset is deliberately ignored, and the code documents the bearing error this causes.

**Verified.** GoogleTests in `test_fusion.cpp` (`FusionBearing.*`): centred bbox gives zero bearing, a bbox right of centre gives negative yaw, and a yawed camera shifts the bearing by 30 degrees. The live accuracy measurements in D-21/D-22 (for example, chair at 1.8 m within 0.09 m) depend on the frame being correct.

**Next.** Use the translation as well as the rotation, which matters at close range.

---

## D-13: safety gate rear cone

**Problem.** PDF section 7.4 specifies the minimum range in a *forward* cone (+/-30 deg, widened with angular speed). Once D-02 sent recoveries through the gate, a backup command would pass the gate unchecked, even with an obstacle right behind the robot.

**Options.** Keep the forward-only cone and accept unchecked reversing, which the D-02 notes originally called acceptable because the watchdog still applies. Or pick the cone direction from the command.

**Chosen.** The cone centre follows the sign of `linear.x`: angle 0 when `linear.x >= 0`, angle pi when `linear.x < 0`. Width and widening rule are unchanged. This is logged as an accepted deviation from the PDF. The logic is in ROS-free `semnav_control/safety_math.hpp` (`cone_for_command`, `cone_min_range`, `gate_step`); the node only wires up ROS. Angles are compared through differences normalised to [-pi, pi]. That way the rear cone works on the TB3 0..2pi scan, on -pi..pi scans where the rear cone straddles both array ends, and with negative increments. The 0.3 m `d_stop` leaves about 0.16 m past the body edge (comment in `safety_gate_params.yaml`).

**Verified.** The required D-13 tests are in `test_safety_math.cpp`:
- `ObstacleBehindReverseGivesZero`: obstacle at 0.3 m behind + reverse command -> zero linear output.
- `ObstacleBehindForwardPassesThrough`: same obstacle + forward command -> passthrough.
- `ObstacleInFrontReversePassesThrough` and `ConeCentreFollowsSignOfLinear`.
- `SafetyGateWrap.*`, which covers the seam cases.

Commit f0207fb records 30 gate tests.

**Next.** No sim test of backup toward an obstacle is recorded yet, so I would add one. A pure rotation (`linear.x = 0`) uses the forward cone and angular always passes through, so the footprint sweep during a spin isn't checked. As the PDF says, in production I would evaluate `nav2_collision_monitor` first.

---

## D-20: the tf2 ABBA deadlock

**Symptom.** With sim + AMCL + Nav2 + perception running, `controller_server` logged "Transform data too old when converting from map to odom" over and over (2,447 errors in one run), and goals aborted. AMCL kept publishing. A fresh `/tf` listener saw map->odom up to date, and only the controller was affected.

**Wrong first hypothesis.** It looked like one DDS reader had stalled, so we switched to CycloneDDS. The stall was still there: a 10-minute soak got 0/7 goals and 5,653 errors. Isolation runs narrowed it down:

| Running | Goals |
|---|---|
| No perception | 10/10 |
| yolo only | 10/10 |
| fusion only | 9/9 |
| yolo + fusion | 2/7 |

A probe showed the local costmap footprint stamp frozen too. So the *whole process* had stopped taking in `/tf`, and the DDS vendor didn't matter. Thread states (futex waits) told us nothing, because a healthy controller looks the same.

**Diagnosis.** I ran the controller under gdb using a launch prefix and took an all-thread backtrace during a stall (`docs/d20_controller_backtrace.txt`):

```
Thread 14: futex_wait(0x555555693b30)
  tf2::BufferCore::addTransformableRequest
  tf2_ros::Buffer::waitForTransform           <- liblayers.so, obstacle layer /scan MessageFilter
Thread 13: futex_wait(0x555555693be0)
  tf2::BufferCore::testTransformableRequests
  tf2::BufferCore::setTransform
  tf2_ros::TransformListener::subscription_callback
```

The listener holds the transformable-requests mutex (`...b30`) and waits for a `tf2_ros::Buffer` mutex (`...be0`) inside the request callback. `waitForTransform` holds `...be0` and waits for `...b30`. That's a classic ABBA lock-order inversion. It's a timing race: the extra perception load and goal transitions make it likely.

**Fix.** Upstream had already fixed it: ros2/geometry2 #982, Humble backport #990 (commit 9997e969), released in tag 0.25.24. apt still ships 0.25.23. The fix collects callbacks and runs them after releasing the mutex. `scripts/setup_underlay.sh` clones the pinned tag into `~/semnav_underlay`, verifies the commit, and builds tf2 + tf2_ros. No local patch. The upstream `wait_for_transform_does_not_deadlock_with_set_transform` test passes.

**Verified.** 10-minute soak with perception on: 36/36 goals; the only 2 stale-TF errors came at startup.

**Next.** Drop the underlay once apt ships 0.25.24. Take a backtrace earlier, before changing infrastructure.

---

## D-22: nearest-cluster range selection

**Problem.** PDF 7.2 step 4 takes the *median* of valid LiDAR ranges in the central 60% of the bbox sector. For a close person this measures the wall. At 1.43 m, the sector had 11 beams: 5 hit the legs at about 1.46 m and 6 passed between them to the wall at 2.68-2.79 m. The median was 2.68 m, so the fused position was 1.36 m too far. At 0.83 m the error was 1.33 m. At 3.3 m the legs fill the sector and the median works.

**Options.**
- (A) Nearest-cluster median, with the gap and minimum beam count as parameters.
- (B) A lower percentile, e.g. the 25th. Simpler, but less robust to a single near outlier.
- (C) Keep the median and document the limitation.

**Chosen.** A, an accepted deviation from the PDF. `nearestClusterMedian` in `fusion.hpp` works like this:
1. Sort the valid ranges.
2. Split them wherever consecutive values differ by more than `cluster_gap` (0.3 m). A gap of exactly 0.3 m stays in one cluster.
3. Take the median of the nearest cluster that has at least `min_cluster_beams` (2) beams, so a single stray beam is skipped.
4. If no cluster qualifies, return `Status::NoCluster` and drop the detection.

Both values are ROS parameters (`perception_params.yaml`).

**Verified.** 7 new `FusionCluster.*` tests (legs + wall, single near outlier skipped, single cluster equals plain median, gap boundary inclusive, NoCluster, bad params); 93 tests green at commit d27379a. Re-measured with teleported poses: person at 1.4 m went from 1.36 to 0.15 m error, and at 0.8 m from 1.33 to 0.25 m. The far person and the chair were unchanged.

**Next.** At 0.8 m YOLO detected the person in only 30/90 frames, so perception is now the close-range bottleneck. The fixed 0.3 m gap would merge an object standing within 0.3 m of the background, so I would tune it per class or make it range-dependent.
