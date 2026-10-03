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
