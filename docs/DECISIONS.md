# SemNav decisions log

Each entry: finding (with evidence from this machine), options, recommendation, status.
Status values: NEEDS DECISION (blocks work), INFO (verified, no conflict), ACCEPTED (user approved).
Audit date: 2026-10-03. Installed: ROS 2 Humble, nav2 1.1.20, turtlebot3 2.3.6,
gazebo_ros 3.9.0 / Gazebo 11.10.2, vision_msgs 4.1.1, OpenCV 4.5.4.

---

## D-01 Who publishes /cmd_vel in Nav2 bringup (velocity_smoother)  - ACCEPTED

**PDF:** controller output remapped to /cmd_vel_nav; safety_gate_node is the *only* publisher of
/cmd_vel (sections 4, 6, 8).

**Installed reality** (`/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py`):
- `controller_server` remap `cmd_vel -> cmd_vel_nav` (line 122).
- `velocity_smoother` is launched and lifecycle-managed, remap `cmd_vel -> cmd_vel_nav`,
  `cmd_vel_smoothed -> cmd_vel` (lines 174-183). **It subscribes /cmd_vel_nav and publishes /cmd_vel.**
- Both publish `geometry_msgs/msg/Twist` (not TwistStamped) in Humble - matches the PDF.

So with stock bringup, the safety gate and velocity_smoother would both publish /cmd_vel. Conflict.

**Options**
- A. Own navigation launch in semnav_bringup (copy of nav2_bringup's, adapted): controller ->
  /cmd_vel_nav, **no velocity_smoother**; gate subscribes /cmd_vel_nav, publishes /cmd_vel.
  Literal match to the PDF; the gate already has its own accel limiter, so the smoother is redundant.
- B. Keep velocity_smoother in the chain: controller -> /cmd_vel_nav -> smoother -> /cmd_vel_smoothed
  -> gate -> /cmd_vel. Adds a topic not in the PDF and changes the gate's input topic.
- C. Use stock bringup and remap the smoother output elsewhere via launch override (fragile, still
  needs a custom launch).

**Recommendation:** A. Topics and node list stay exactly as the PDF.

**Final (accepted 2026-10-03):** A. Own navigation launch in semnav_bringup, no velocity_smoother;
controller -> /cmd_vel_nav; safety_gate_node is the only /cmd_vel publisher.

## D-02 behavior_server (spin/backup) bypasses the safety gate  - ACCEPTED (with deviation)

**Finding:** in the same launch file `behavior_server` gets only the tf remaps (line 152), and
`libnav2_spin_behavior.so` publishes on relative `cmd_vel`. So recoveries (spin, backup) publish
straight to **/cmd_vel**, bypassing the gate. The PDF requires default recoveries (spin, backup, wait)
*and* a single /cmd_vel publisher. The PDF does not mention this case.

**Options**
- A. In the semnav navigation launch, remap behavior_server `cmd_vel -> cmd_vel_nav` so recoveries
  also pass through the gate. (Note: backup moves backwards; gate cone is forward-only, so the gate
  passes reverse motion through at full scale; acceptable since the watchdog still applies.)
- B. Leave behaviors on /cmd_vel (two publishers; violates PDF).

**Recommendation:** A (part of the same custom launch as D-01).

**Final (accepted 2026-10-03):** A, plus an addition that is a **deviation from the PDF**:
safety_gate_node picks its cone by the sign of `linear.x` - forward cone when moving forward,
rear cone when reversing - so recoveries such as backup are also gated. The PDF only specifies a
forward cone (section 7.4). Requires a unit test: reverse command with an obstacle close behind ->
output zero; same obstacle behind while moving forward -> passthrough. See D-13.

## D-03 TurtleBot3 Humble Nav2 params file is stale  - ACCEPTED

**PDF:** "Use the TurtleBot3 Nav2 params as the base and change only what you need."

**Finding:** `turtlebot3_navigation2/launch/navigation2.launch.py` loads
`param/humble/waffle.yaml` on Humble. That file is Galactic-era: it configures `recoveries_server`
with `nav2_recoveries/*` plugins (line 317+), and has **no** `behavior_server`, `smoother_server`
or `velocity_smoother` sections, while nav2_bringup 1.1.20 launches `behavior_server`,
`smoother_server`, `velocity_smoother`. Those nodes would run on built-in defaults; the recovery
config in the file is ignored. Not yet run, so failure modes are unverified.
(The non-`humble` `param/waffle.yaml` targets Jazzy: `enable_stamped_cmd_vel`, collision_monitor.)

**Options**
- A. Base = `nav2_bringup/params/nav2_params.yaml` (matches installed Nav2 1.1.20), overlay TB3
  waffle values from `param/humble/waffle.yaml` (robot_radius/footprint, velocity limits, AMCL,
  DWB tuning, scan topic). Frames already agree (`base_link`, `odom`, `map`).
- B. Base = TB3 `param/humble/waffle.yaml`, add the missing behavior_server section by hand.

**Recommendation:** A. Slight deviation from wording ("TurtleBot3 params as the base"), same intent.

**Final (accepted 2026-10-03):** A. Base on `nav2_bringup/params/nav2_params.yaml`, overlay
TurtleBot3 waffle values.

## D-04 Gazebo ground-truth poses  - ACCEPTED

**PDF:** ground truth from `/gazebo/model_states` (section 4), used only by eval.

**Finding:** in ROS 2 Gazebo Classic, `/gazebo/model_states` is **not** published by default.
`gzserver.launch.py` only loads `libgazebo_ros_init.so`, `libgazebo_ros_factory.so`,
`libgazebo_ros_force_system.so`. It comes from the world plugin `libgazebo_ros_state.so`
(installed), which publishes `model_states` / `link_states` (gazebo_msgs/ModelStates, LinkStates)
and serves `get_entity_state` / `set_entity_state`, under the namespace given in `<ros>`
(see `/opt/ros/humble/share/gazebo_ros/worlds/gazebo_ros_state_demo.world`). None of the installed
turtlebot3_gazebo worlds include it. `libgazebo_ros_p3d.so` (per-model odometry) is also installed.

**Options**
- A. Our own world in `semnav_bringup/worlds/` that includes
  `<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"><ros><namespace>/gazebo</namespace></ros><update_rate>N</update_rate></plugin>`
  -> topic `/gazebo/model_states` exactly as the PDF. eval_logger subscribes to it.
- B. Poll the `/gazebo/get_entity_state` service from eval_logger (same plugin needed; no topic).
- C. p3d plugin on each object model (more SDF edits, one topic per model, not in PDF).

**Recommendation:** A. Needs our own world anyway for person/chair/bottle models (section 11).
Adds dependency `gazebo_msgs` to semnav_eval (implied by the PDF topic; confirm).

**Final (accepted 2026-10-03):** A. Own world with `libgazebo_ros_state.so`, namespace `/gazebo`
(-> `/gazebo/model_states`). `gazebo_msgs` dependency approved.

## D-05 Camera image frame_id is not an optical frame  - ACCEPTED

**PDF:** fusion builds rays `((u-cx)/fx, 0, 1)` in the optical frame and asks TF2 for
`camera_optical -> base_scan` at the image stamp. TF tree lists `camera_link -> camera_rgb_optical_frame`.

**Finding** (`turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf` lines 370-419): the
`gazebo_ros_camera` plugin has no `frame_name`, so it defaults to the link name:
**`header.frame_id = camera_rgb_frame`** (x-forward body frame, not optical).
The URDF (robot_state_publisher) does provide
`base_link -> camera_link -> camera_rgb_frame -> camera_rgb_optical_frame` (rpy -1.57 0 -1.57).
SDF and URDF positions agree (0.069, -0.047, 0.107 from base_link). Using the image header
frame directly with optical-frame rays would rotate bearings by 90 deg.
Minor: actual chain has `camera_rgb_frame` between `camera_link` and the optical frame.

**Options**
- A. fusion parameter `camera_optical_frame` (default `camera_rgb_optical_frame`) used for the TF
  lookup instead of the image header frame. No SDF change.
- B. Copy the waffle SDF into semnav_bringup and set `<frame_name>camera_rgb_optical_frame</frame_name>`
  (combine with D-06), then use header frame_id as-is.

**Recommendation:** B if D-06 copies the SDF anyway (data is then self-describing); otherwise A.

**Final (accepted 2026-10-03):** B. In the copied waffle model (D-06) set
`<frame_name>camera_rgb_optical_frame</frame_name>`; done in Phase 2. Additional rules for
semantic_fusion_node:
- Read the camera frame from the image (detection) header; no hard-coded frame names anywhere.
- If TF cannot resolve header frame -> laser frame at the stamp, fail loudly (error log, drop the
  detection; never fall back to a guessed transform).
- fx and cx always come from /camera/camera_info, never from parameters or constants.

## D-06 Camera resolution and rate  - ACCEPTED

**Finding:** waffle camera is **1920x1080 @ 30 Hz**, horizontal_fov 1.02974 rad (~59 deg),
gaussian noise. The PDF mitigation (section 11) is 640x480 @ 15 Hz. Achieving that requires a
modified copy of the robot model (SDF) in semnav_bringup, spawned instead of the stock one.

**Options**
- A. Copy `turtlebot3_waffle` model into `semnav_bringup/models/`, change width/height/update_rate
  (all as launch-configurable values where possible), spawn that copy.
- B. Keep 1920x1080 @ 30 Hz; rely on frame dropping. Higher CPU load, more letterbox cost.

**Recommendation:** A (do it in Phase 1/2 so map and tests use the final sensor).

**Final (accepted 2026-10-03):** A. Copy the waffle model into semnav_bringup, 640x480 @ 15 Hz,
camera frame per D-05. Done in Phase 2 (Phase 1 uses the stock model).

## D-07 TurtleBot3 waffle topic names, frames, sensor limits  - ACCEPTED

Verified from model.sdf / URDF (no namespace):

| Item | Value |
|---|---|
| Image | `/camera/image_raw` (sensor_msgs/Image), frame `camera_rgb_frame` in the stock model; `camera_rgb_optical_frame` in semnav_waffle from Phase 2 (D-05) |
| Camera info | `/camera/camera_info` |
| Scan | `/scan`, frame `base_scan`, 5 Hz, 360 samples, angle 0 .. 6.28 rad, range 0.12 .. 3.5 m |
| Odom | `/odom`, odom TF `odom -> base_footprint` from diff-drive plugin |
| Cmd input | `/cmd_vel` (geometry_msgs/Twist) |
| Joints | `/joint_states`; IMU `/imu` |
| TF | `base_footprint -> base_link -> {base_scan, imu_link, camera_link -> camera_rgb_frame -> camera_rgb_optical_frame}` |

Matches PDF names. Notes:
- **Conflict:** LiDAR max range is **3.5 m**, but the target "median error under 0.3 m in range
  1-4 m" (section 10) cannot be met beyond 3.5 m. Recommendation: evaluate over 1-3.5 m and state
  it in the README, or (needs approval) raise the sim LiDAR max range in the copied SDF (D-06).
- Scan is 5 Hz, so "closest scan" can be up to 100 ms off; consistent with 0.1 s slop.
- Scan angle 0 = forward and wraps at 2*pi; gate cone and fusion sector must handle wrap-around.
- PDF QoS for /camera/image_raw: "depth 5" (section 2) vs "depth 1-2" (section 6). Recommend 1-2
  (sec. 6, more specific), as a parameter.

**Final (accepted 2026-10-03):** as recommended. Evaluate position error over 1-3.5 m and state
the LiDAR limit in the README (sim LiDAR range unchanged). Image subscription QoS depth 1-2 as a
parameter. Scan-angle wrap-around handled in fusion and gate.

## D-08 vision_msgs Detection2D field layout  - ACCEPTED

Installed vision_msgs **4.1.1** (Humble). Layout:
```
Detection2DArray: header, Detection2D[] detections
Detection2D:      header, ObjectHypothesisWithPose[] results, BoundingBox2D bbox, string id
ObjectHypothesisWithPose: ObjectHypothesis hypothesis {string class_id, float64 score},
                          geometry_msgs/PoseWithCovariance pose
BoundingBox2D:    vision_msgs/Pose2D center {Point2D position {x, y}, float64 theta},
                  float64 size_x, size_y
```
Implications (older tutorials are wrong for this version):
- Box center is `bbox.center.position.x/y` (not `bbox.center.x`).
- Class is `results[i].hypothesis.class_id` as a **string**; score is `hypothesis.score`.
- Recommendation: put the COCO class *name* (e.g. "person") in class_id, so fusion maps directly
  to `SemanticObstacle.class_name` without a shared index table. Fill `Detection2D.header` with the
  image header too. No PDF conflict.

**Final (accepted 2026-10-03):** as recommended. `class_id` carries the COCO class name;
`Detection2D.header` copied from the image header.

## D-09 /metrics message type  - ACCEPTED

PDF: "std_msgs/Float32MultiArray or diagnostic_msgs". Must pick one.
- A. `std_msgs/Float32MultiArray` with a fixed, documented layout
  (p50/p95 preprocess, infer, postprocess, total, fps).
- B. `diagnostic_msgs/DiagnosticArray` with key/value pairs (self-describing, works with
  rqt_runtime_monitor, but string values).

**Recommendation:** A (simpler for eval_logger and CSV); layout documented in the message source.

**Final (accepted 2026-10-03):** A. `std_msgs/Float32MultiArray`, layout documented in the code
that publishes it.

## D-10 Small PDF ambiguities  - ACCEPTED

1. **eval_logger language:** PDF says "C++ or Python"; CLAUDE.md says Python only for launch/scripts.
   Recommendation: C++ (`semnav_eval` package), NavigateToPose via rclcpp_action.
2. **"pad by class radius"** (fusion step 5): LiDAR hits the near surface, so recommendation is to
   push the point outward along the ray by the class radius to approximate the object centre.
3. **Global planner:** SmacPlanner2D or NavFn. TB3 params use NavFn. Recommendation: NavFn first
   (fewer moving parts), SmacPlanner2D as a one-block change later.
4. **Local costmap:** TB3/Nav2 defaults use voxel_layer locally; PDF says "obstacle". Recommendation:
   use ObstacleLayer as the PDF says (2D LiDAR only).

**Final (accepted 2026-10-03):** (1) eval_logger in C++, package semnav_eval. (2) Push the point
outward along the ray by the class radius. (3) NavFn first. (4) ObstacleLayer in the local costmap.

## D-11 Workspace layout vs repo  - ACCEPTED

PDF section 5 shows `semnav_ws/` containing CLAUDE.md, docs/, models/, scripts/, src/.
This repo is `robotics-project-01/` with CLAUDE.md and docs/ at the root.

**Recommendation:** treat the repo root as the colcon workspace (`src/<packages>` under it,
models/, scripts/, third_party/ at root). Build/install/log are already gitignored. No rename.

**Final (accepted 2026-10-03):** as recommended. Repo root is the colcon workspace.

## D-12 Missing tooling for later phases  - ACCEPTED

- ONNX Runtime C++ not present (expected: fetched into third_party/ by scripts/setup_ort.sh, Phase 3).
  Need to pick a pinned version then (recommend 1.17.x-1.20.x CPU x64 release tarball).
- Python `onnxruntime` and `ultralytics` not installed; needed for the Day-2 sanity check and
  `export_yolo.py`. Recommend a project `.venv` (already gitignored) with pip; no sudo needed.
  Adds Python tooling deps not named in the PDF beyond "export once" - confirm.
- Not checked: GPU/CUDA availability (use_cuda is optional).

**Final (accepted 2026-10-03):** as recommended. Project `.venv` (gitignored) with `onnxruntime`
and `ultralytics`, created when Phase 2 starts. ORT C++ version pinned in Phase 3.

**Pin (2026-10-03, Phase 3):** ONNX Runtime **1.20.1** CPU x64 release tarball
(`onnxruntime-linux-x64-1.20.1.tgz`, SHA-256
`67db4dc1561f1e3fd42e619575c82c601ef89849afc7ea85a003abbac1a1a105`), fetched and verified by
`scripts/setup_ort.sh` into `third_party/onnxruntime` (gitignored). semnav_perception imports it as
`onnxruntime::onnxruntime` and installs the .so into its lib/. No CUDA provider in this build
(use_cuda falls back to CPU with a warning).

---

## D-13 Safety gate cone direction follows sign of linear.x  - ACCEPTED (deviation from PDF)

**PDF (7.4):** minimum range in a *forward* cone (default +/-30 deg, widened with angular speed).

**Deviation:** the cone is centred forward when `linear.x >= 0` and rearward (scan angle pi)
when `linear.x < 0`. Width and widening rule unchanged. Reason: with D-02, recovery behaviours
(backup) are routed through the gate; a forward-only cone would pass reverse motion unchecked.

**Required test:** obstacle at 0.3 m behind + reverse command -> zero linear output; obstacle at
0.3 m behind + forward command -> passthrough. Implemented in Phase 7.

## D-14 No offline person/chair/bottle Gazebo models  - ACCEPTED

**PDF (section 11):** use Gazebo models with realistic textures (person, chair, bottle) so COCO
YOLO detects them in sim. Task rule: offline models only, ask before any download.

**Finding (2026-10-03):** `~/.gazebo/models` holds only `ground_plane`, `sun`;
`/usr/share/gazebo-11/models` the same; `turtlebot3_gazebo/models` has only TB3 robots/worlds.
A filesystem search found no person/chair/bottle SDF/mesh anywhere. Fuel/model-database models
would be fetched at runtime by gzserver (network), which the rule forbids without approval.

**Options**
- A. Approve a one-time download of specific textured models (e.g. from osrf/gazebo_models or
  Gazebo Fuel: a standing person, a chair, a bottle), vendored into `semnav_bringup/models/`
  with their licenses noted. Realistic textures -> best chance YOLO detects them (PDF intent).
- B. Primitive placeholders (cylinder "person", box "chair", small cylinder "bottle") from SDF
  only. Fully offline, ground-truth plumbing testable, but YOLO will almost certainly not detect
  them -> Phase 2 "YOLO detects at least one class" fails; PDF fallback is a recorded video.
- C. Build the world now with the state plugin only (no objects); add objects after a decision.

**Recommendation:** A (smallest set: person, chair, bottle), with C as the interim step so the
rest of Phase 2a can be built and committed.

**Final (accepted 2026-10-03):** A, but kept **out of git**. `scripts/fetch_models.sh` downloads
pinned versions into `src/semnav_bringup/models_external/` (gitignored, installed if present,
added to GAZEBO_MODEL_PATH by sim.launch.py). Approved sources:
- `person_standing`, `beer` (textured bottle): github.com/osrf/gazebo_models, CC-BY 3.0
- `WoodenChair`: fuel.gazebosim.org OpenRobotics/WoodenChair, CC0
Rejected: Fuel "Water Bottle" (.glb mesh, not loadable by Gazebo Classic 11).
Consequence: a fresh clone needs network once (fetch script) before the world shows objects.

## D-15 Demo classes are person and chair; bottle stays as an undetected obstacle  - ACCEPTED

**PDF (7.1, 11):** class_filter example "person, chair, bottle"; sim objects should be detectable
by COCO YOLO.

**Finding (2026-10-03, Phase 2b sanity check, 20 frames from spawn, conf 0.35 / IoU 0.45):**
chair max 0.917 (20/20 frames), person max 0.842 (20/20); bottle_1 (osrf `beer`, textured can)
not detected even at conf 0.05.

**Final (accepted 2026-10-03):** demo classes are person and chair; yolo_onnx_node
`class_filter` default = `[person, chair]`. bottle_1 stays in the world as an obstacle that
perception misses, demonstrating that LiDAR (obstacle layer, safety gate) still handles the
geometry when perception fails.

## D-16 Interim /cmd_vel_nav -> /cmd_vel relay until safety_gate_node exists  - ACCEPTED

**Task (Phase 2c-1):** navigation.launch.py gets a launch arg (default on) relaying /cmd_vel_nav
to /cmd_vel via topic_tools relay or equivalent, until the gate exists (Phase 7).

**Finding (2026-10-03):** `topic_tools` is not installed (`/opt/ros/humble/share/topic_tools`
missing; apt candidate `ros-humble-topic-tools` 1.1.2 available). No other relay executable in
/opt/ros/humble. CLAUDE.md forbids Python nodes and new dependencies without approval.

**Options**
- A. Install topic_tools (user runs `sudo apt install ros-humble-topic-tools`), add
  `<exec_depend>topic_tools</exec_depend>`; launch arg starts `topic_tools relay`.
  Matches the task literally; /cmd_vel_nav exists in both modes; adds one dependency.
- B. No relay: the launch arg switches the controller/behavior_server remap target between
  /cmd_vel (arg on, interim) and /cmd_vel_nav (arg off, gate). Zero dependencies, zero hops;
  but /cmd_vel_nav does not exist while the arg is on.
- C. Small C++ relay node in semnav_bringup (Twist in -> Twist out). No external dep, but a
  node not in the PDF and throwaway code.

**Final (accepted 2026-10-03):** A. User installs `ros-humble-topic-tools`; semnav_bringup adds
`<exec_depend>topic_tools</exec_depend>`; navigation.launch.py arg `cmd_vel_relay` (default true)
runs `topic_tools relay /cmd_vel_nav /cmd_vel`. Set it false once safety_gate_node exists.

## D-17 Mapping "smear" is an RViz Map-shader failure on the Intel iGPU, not a SLAM fault  - ACCEPTED

**Symptom (user, 2026-10-03):** during `mapping.launch.py`, the map in RViz smeared into a
diagonal band and the live scan did not line up with the map walls.

**Evidence (headless, same build):**
- use_sim_time true on slam_toolbox and robot_state_publisher (and passed to RViz by
  sim.launch.py); slam frames map / odom / base_footprint and scan /scan (frame base_scan) match
  the TF tree; laser pose identical to the stock TB3 waffle SDF.
- /odom vs Gazebo ground truth during a 195 deg in-place turn at 0.3 rad/s: <= 1.5 deg, 3 mm.
- Scripted drives (0.1 m/s + 0.3 rad/s turns; teleop defaults 0.5 m/s + 1.0 rad/s): slam pose
  map->base_footprint vs ground truth <= 0.011 m / 1.3 deg; 74-99 % of scan endpoints within one
  cell of an occupied map cell (lower values = newly seen areas before the next 5 s map update).
- RTF 0.98; all /tf stamps follow /clock; scan age vs /clock < 10 ms.
- No sim time jump: `/reset_simulation` logs "Detected jump back in time" in slam_toolbox; the
  user's slam log has no such line. `/reset_world` does not disturb slam (odom is world-sourced).
- User's RViz log (rviz2_127042): `[ERROR] Vertex Program:rviz/glsl120/indexed_8bit_image.vert
  ... GLSL link result: active samplers with a different type refer to the same texture image
  unit`, ~5 s after start (first /map). indexed_8bit_image is the Map display's shader. The two
  earlier RViz runs on this machine (no Map display) have no such error.
- GPU: Intel iGPU (8086:a7a0, Mesa, GL 4.6) + NVIDIA 25ac with its kernel driver not loaded
  (`nvidia-smi` fails), so RViz renders on Mesa/Intel.

**Conclusion:** SLAM output is correct; the Map display shader fails to link on this GL stack and
RViz cannot draw the occupancy grid correctly on the Intel hardware driver (smeared band in the
user's session; no map at all in the scripted check below).

**Options:** A. launch arg `rviz_software_gl` (default false) setting LIBGL_ALWAYS_SOFTWARE=1 for
RViz only; B. same, default true; C. fix NVIDIA driver + PRIME offload; D. README note only.

**Final (accepted 2026-10-03):** A. sim.launch.py arg `rviz_software_gl` (default false), forwarded
by mapping.launch.py; use `rviz_software_gl:=true` on this machine.

**Verification (2026-10-03, RViz on the user's display, scripted drive 0.1 m/s + 0.3 rad/s turns):**
- `rviz_software_gl:=false` (Intel/Mesa GL 4.6): GLSL link error logged; window grab shows the
  scan but no map.
- `rviz_software_gl:=true` (llvmpipe GL 4.5): the same GLSL error is still logged (Mesa's shared
  GLSL linker), but the map renders and the scan lies on the map walls; slam pose vs ground truth
  <= 0.001 m / 0.2 deg, 63-99 % of scan endpoints on occupied cells (+-1 cell).
- RViz frame rate 14 fps with software GL (31 fps hardware).

## D-18 Phase 5 requested before Phase 3/4 prerequisites exist  - ACCEPTED

**Task (2026-10-03):** Phase 5 semantic_fusion_node in semnav_perception, publishing
/semantic_obstacles (semnav_msgs/SemanticObstacleArray), verified with sim + localization +
yolo node running.

**Finding:** none of the Phase 3 deliverables exist: no `semnav_msgs` package, no
`semnav_perception` package, no ONNX Runtime in third_party/ (`scripts/setup_ort.sh` not written,
ORT version not pinned, D-12), no yolo_onnx_node. Phase 5 needs the message package to publish and
the yolo node (/detections, vision_msgs/Detection2DArray) to verify.

**Options**
- A. Build Phase 3 first (setup_ort.sh with a pinned ORT CPU release, download needs network;
  semnav_msgs exactly as section 6; letterbox/yolo_detector headers + tests; minimal
  yolo_onnx_node), commit, then Phase 5 as requested. Phase 4 hardening stays for later.
- B. Pull only semnav_msgs + the semnav_perception skeleton forward; build Phase 5 fully, but
  verify with a stand-in detector: a script that projects /gazebo/model_states person/chair into
  the camera (camera_info + TF) and publishes Detection2DArray. Real-YOLO verification after Phase 3.
- C. Only the ROS-free fusion.hpp / tracker.hpp + GoogleTests now (in a new semnav_perception
  package); node after Phase 3.

**Final (accepted 2026-10-03):** A, extended: Phase 3 and Phase 4 to full spec first
(setup_ort.sh with pinned ORT CPU, semnav_msgs, letterbox/decode/NMS headers + GoogleTests,
yolo_onnx_node with newest-frame buffer + worker thread, /detections, /detections/image,
/metrics p50/p95 (D-09), class_filter default [person, chair] (D-15)), one commit per green
slice; then Phase 5 as specified, verified with the real YOLO node.

## D-19 Camera images lost in DDS transport (UDP receive buffer overflow)  - ACCEPTED

**Symptom (2026-10-03, Phase 3 live check):** yolo_onnx_node processes 0.2-6 fps although
inference takes ~40 ms (p50) and the camera runs at 15 Hz; 0 frames dropped by the node itself.

**Evidence:**
- Direct rclpy subscriber (SensorData QoS), 6 s windows: /camera/camera_info 15.0 Hz, /scan 5.0 Hz,
  but /camera/image_raw 0.2-3.4 Hz (worse with two image subscribers). Same Gazebo sensor, so the
  camera publishes at 15 Hz; the 921,600-byte images are lost in transport.
- /proc/net/snmp Udp: InErrors 392797 == RcvbufErrors 392797 (kernel dropped datagrams because
  socket receive buffers were full).
- net.core.rmem_max = rmem_default = 212992 bytes. RMW is rmw_fastrtps_cpp (Fast DDS 2.6.12). Its
  default shared-memory segment (512 KB) is smaller than one image, so images go over UDP as
  ~15 fragments each and overflow the 208 KB receive buffer.
- Sim is healthy (RTF 0.99-1.00).

**Options**
- A. Repo Fast DDS XML profile (`semnav_bringup/config/fastdds_profile.xml`) with a larger SHM
  segment (e.g. 10 MB) so same-host images use shared memory; launch files set
  FASTRTPS_DEFAULT_PROFILES_FILE for every process they start. No sudo. Shells running
  `ros2 run`/CLI tools must export it too (document in README).
- B. System UDP buffers (user runs sudo): `sysctl -w net.core.rmem_max=2147483647
  net.core.rmem_default=8388608` (+ /etc/sysctl.d file to persist). Standard ROS 2 advice for
  large messages; machine-level, nothing in the repo.
- C. A + B.
- D. Switch RMW to CycloneDDS (new dependency, not installed).

**Final (accepted 2026-10-03):** C (A + B). Repo profile `semnav_bringup/config/fastdds_profile.xml`
(SHM segment 10 MB + UDPv4, 8 MB socket buffers) exported as FASTRTPS_DEFAULT_PROFILES_FILE by the
SemNav launch files; user raises UDP buffers via /etc/sysctl.d (sudo).

## D-20 Nav2 server stops receiving map->odom from one TF publisher (DDS reader stall)  - FIXED (tf2 0.25.24 underlay)

**Symptom (2026-10-03, Phase 5 verification):** with sim + AMCL + Nav2 + perception running,
controller_server logged "Transform data too old when converting from map to odom" continuously
from sim time 183.7 s to 552 s (2,447 errors); its map->odom stayed at 165.998 s. Goals aborted
("Failed to make progress"). Same pattern as the user's Phase 2c mapping run (planner/behavior
servers: map->odom from slam_toolbox frozen at 120.8 s while the clock reached 399 s; D-17 era,
before the D-19 profile).

**Evidence:**
- AMCL kept publishing: a fresh /tf subscriber at clock 417.8 saw map->odom stamped 418.8
  (= clock + transform_tolerance 1.0). AMCL log has no errors.
- Only controller_server was affected (planner_server 2 transient errors at startup,
  bt_navigator 0). So one writer->reader pair (AMCL -> controller_server /tf) stalled while the
  same writer kept delivering to other readers.
- Not CPU starvation: RTF 0.99-1.00 throughout; 20 cores.
- Fast DDS 2.6.12 uses shared memory for same-host traffic by default (also before D-19).
- Not yet isolated: whether the stall depends on SHM, on the D-19 profile, or on load from the
  perception nodes. Not reproduced in the 10-goal auto_map run (no perception running).

**Options**
- A. UDP-only Fast DDS profile (no SHM) + the D-19 sysctl buffers (needs the user's sudo step,
  not applied yet: rmem_max still 212992). Then soak-test (Nav2 goal loop + perception, 15 min)
  and watch every Nav2 server for stale map->odom.
- B. Switch RMW to CycloneDDS (`sudo apt install ros-humble-rmw-cyclonedds-cpp`,
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp in launch files): new dependency, commonly used with Nav2
  on Humble; same soak test.
- C. Investigate further first (soak with/without SHM, with/without perception) before choosing.

**Final (accepted 2026-10-03):** B. Switch to CycloneDDS (`rmw_cyclonedds_cpp`, user installs
`ros-humble-rmw-cyclonedds-cpp`). SemNav launch files set RMW_IMPLEMENTATION; shells running CLI
tools/scripts export it too. Cyclone uses UDP on the same host (no SHM without iceoryx), so the
D-19 sysctl buffer increase becomes mandatory. Verify with a soak test (Nav2 goal loop +
perception) watching every Nav2 server for stale map->odom, and re-check image rate (15 Hz).

**Update (2026-10-03, after the switch):** CycloneDDS did NOT fix it, so the DDS-reader-stall
diagnosis was wrong; this decision stays open. 10-min soak (Cyclone, sim + AMCL + Nav2 +
perception): 0/7 goals succeeded, controller_server logged 5,653 "Transform data too old" with its
map->odom frozen at 94.598 s, and the local costmap's published_footprint stayed at 93.799 s
(the controller_server process stopped taking in /tf). A separate listener saw map->odom fresh
the whole time (age -1.0..-0.8 s). All controller threads idle in futex waits; no TF/clock
warnings. Isolation, fresh launch each, 150 s, Cyclone:
- A: no perception -> 10/10 SUCCEEDED, 2 startup TF errors
- B: yolo_onnx_node + semantic_fusion_node -> 2/7, 286 stale-TF errors (froze at 67.2 s)
- C: yolo_onnx_node only -> 10/10, 2
- D: semantic_fusion_node only -> 9/9, 2
So the stall needs both perception nodes running (fusion then processes detections and does TF
lookups), independent of the DDS vendor. Root cause not yet found. Cyclone stays (images 15 Hz,
behaviour unchanged). The user's Phase 2c failure (slam, RViz, no perception) may be a different
trigger of the same symptom.

## D-21 Push the fused point out by a fraction of the class radius  - ACCEPTED

**Measurement (2026-10-03, teleported poses, AMCL re-seeded at truth, error <= 0.014 m):** with
the full class radius (D-10), the person at 3.3 m came out +0.31 m too far (radial), i.e. about
one class radius: the LiDAR (scan plane ~0.17 m) hits the legs, which are already near the
person's centre. Chair: 0.05 m at 1.8 m, 0.15 m at 1.0 m.

**Final (accepted by user 2026-10-03):** `push_out_fraction` parameter (default 0.5, range [0,1])
in fusion::Params and semantic_fusion_node; 1.0 reproduces D-10 literally. Re-check with 0.5:
person 3.3 m 0.15 m (+0.13 radial); chair 1.8 m 0.09 m (-0.08 radial); chair 1.0 m 0.11 m.

## D-22 Close-range person: central-sector median ranges the wall between the legs  - ACCEPTED (deviation from PDF 7.2 step 4)

**Measurement (2026-10-03):** person at 1.43 m -> fused 1.36 m too far (LiDAR median 2.68 m); at
0.83 m -> 1.33 m too far (median 2.04 m). Beam dump at 1.43 m, central 60 % of the bbox sector
(-5.4..+4.8 deg, 11 beams): 5 beams hit the legs at ~1.46 m, 6 pass between the legs to the wall
at 2.68-2.79 m, so the median (PDF 7.2 step 4) is a wall range. At 3.3 m the legs fill the sector
and the median is correct.

**Options**
- A. Nearest-cluster range: sort valid ranges, take the median of the nearest cluster (gap
  threshold param, min beams param). Robust to background between/behind thin parts.
- B. Lower percentile instead of median (param, e.g. 25th): simple; less robust to a near outlier.
- C. Keep the median (PDF literal); state the limit in the README.

**Final (accepted by user 2026-10-03):** A, an accepted deviation from PDF 7.2 step 4 ("take the
median"): the range is the median of the NEAREST cluster of valid ranges in the central sector
(clusters split where consecutive sorted ranges differ by more than a gap threshold; a cluster
needs a minimum number of beams, otherwise the next cluster is used). Both are parameters.
Implemented 2026-10-03: `cluster_gap` 0.3 m, `min_cluster_beams` 2 (fusion::Params and node
params); a gap exactly equal to cluster_gap stays in one cluster; no qualifying cluster ->
Status::NoCluster (detection dropped). Re-measure (teleported poses, AMCL re-seeded): person
1.4 m 1.36 -> 0.15 m, person 0.8 m 1.33 -> 0.25 m (YOLO detects it in 30/90 frames at that
range); person 3.3 m 0.15 m, chair 1.8 m 0.09 m, chair 1.0 m 0.11 m unchanged.

**Time-boxed investigation (2026-10-03 19:40-20:21, not solved):**
1. use_sim_time True on all 23 nodes; /tf publishers: amcl, robot_state_publisher,
   turtlebot3_diff_drive; /tf_static: robot_state_publisher; all /tf stamps follow /clock
   (map->odom +1.0 s = AMCL transform_tolerance, others +0.001..+0.099 s). No foreign clock/TF.
2. publish_annotated=false: still stalls (964 stale-TF errors, map->odom frozen at 94.597 s).
   CPU not saturated: ~75 % idle (top), gzserver ~55 %; controller "missed rate" warnings 3-5.
3. fusion tf_timeout=0 (non-blocking lookups): still stalls (951 errors, frozen at 123.4 s).
4. Probe during a stall: controller_server lifecycle "active"; local costmap keeps publishing
   published_footprint at 5 Hz but with the stamp frozen (63.0 s) -> the whole controller_server
   process stopped taking in /tf (odom->base_footprint too, not only map->odom), starting when
   the robot reached its first goal. A healthy controller also shows all threads in futex waits,
   so thread states say nothing about a deadlock.
Next step: backtrace of controller_server during a stall (run it under gdb via a launch prefix,
or temporarily allow ptrace: ptrace_scope is 1), and test without the fusion node's
/semantic_obstacles and /semantic_markers publishers.

**Root cause (2026-10-03 20:36, gdb all-thread backtrace of controller_server during a stall,
controller run under gdb via a launch prefix):** lock-order inversion (ABBA deadlock) inside
tf2_ros::Buffer in the controller_server process:
- TF listener thread: TransformListener::subscription_callback -> BufferCore::setTransform ->
  BufferCore::testTransformableRequests() holds BufferCore's transformable-requests mutex
  (0x555555693b30) and blocks on a tf2_ros::Buffer mutex (0x555555693be0) inside the request
  callback.
- Costmap layer thread (liblayers.so, the obstacle layer's tf2_ros::MessageFilter on /scan):
  tf2_ros::Buffer::waitForTransform holds 0x555555693be0 and blocks in
  BufferCore::addTransformableRequest on 0x555555693b30.
The listener thread never returns, so the process stops inserting /tf: every frame freezes, which
explains map->odom AND the local footprint stamp freezing while AMCL keeps publishing. It is a
timing race: extra load (yolo + fusion) and goal transitions make it likely; without them it was
not observed in 150 s soaks. Not DDS-related (same on Fast DDS and Cyclone). Installed
ros-humble-tf2-ros 0.25.23 is the newest Humble package (no newer candidate).
Backtrace excerpt kept in docs/d20_controller_backtrace.txt.

**Fix options (decision needed)**
- A. Overlay-build tf2_ros (geometry2 humble branch) with the lock-order fix backported, in a
  separate underlay workspace. Fixes the cause for every Nav2 server; adds a vendored dependency.
- B. Mitigate: keep load away from Nav2 (e.g. run perception in a separate CPU set / lower
  priority) and/or reduce waitForTransform traffic (obstacle layer `transform_tolerance`, scan
  rate). Lowers the probability only.
- C. Report upstream and accept the risk for the demo with a restart watchdog.

**Final (accepted 2026-10-03):** A. Upstream already fixed it: ros2/geometry2 PR #982
(https://github.com/ros2/geometry2/pull/982), Humble backport PR #990
(https://github.com/ros2/geometry2/pull/990, commit 9997e969, "Fix ABBA deadlock between
`waitForTransform` and `testTransformableRequests`"), released in tag 0.25.24 (commit 404b7224),
not yet in apt (installed 0.25.23). The only code change in 0.25.24 is tf2/src/buffer_core.cpp
(callbacks are collected and run after releasing transformable_requests_mutex_). No local patch.
`scripts/setup_underlay.sh` clones that tag into ~/semnav_underlay (outside the repo), verifies the
commit, builds tf2 (+ tf2_ros, whose upstream test
`wait_for_transform_does_not_deadlock_with_set_transform` passes: test_buffer 11/11, 21 tests
total). Every shell sources it between /opt/ros/humble and the workspace (README).
Confirmation soak (10 min, Cyclone, sim + AMCL + Nav2 + yolo + fusion, annotated images on):
36/36 goals SUCCEEDED, 2 stale-TF errors (both at startup, before the AMCL initial pose; same
in every healthy run), map->odom age -1.1..-0.8 s, CPU ~74 % idle.
