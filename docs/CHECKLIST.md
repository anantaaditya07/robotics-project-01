# SemNav build checklist

Derived from docs/architecture.txt, section 12 (build plan), with tasks expanded from sections 5-9.
Legend: `[x]` done and verified, `[ ]` open. A phase is done only when its "Done when" passes.
Open questions blocking a phase are listed in docs/DECISIONS.md (D-xx).

## Phase 0 - Pre-flight (repo + environment audit)
- [x] Repo created, CLAUDE.md, .gitignore (build/, install/, log/, *.onnx, third_party/onnxruntime/)
- [x] docs/architecture.txt present (source of truth)
- [x] Installed-package audit: ROS 2 Humble, Nav2 1.1.20, turtlebot3 2.3.6, gazebo_ros 3.9.0
      (Gazebo 11.10.2), vision_msgs 4.1.1, slam_toolbox 2.6.10, OpenCV 4.5.4, clang-format 14,
      ament_cmake_gtest, launch_testing, message_filters, pluginlib
- [x] Interface checks recorded in DECISIONS.md (D-01 .. D-12)
- [x] User decisions recorded: D-01 .. D-13 ACCEPTED (2026-10-03)

**Done when:** all blocking decisions answered.

## Phase 1 (Day 1) - Simulation baseline
- [x] Workspace layout (D-11): repo root is the colcon workspace; `src/semnav_bringup` builds;
      `colcon test` runs (0 tests yet)
- [x] `ros2 launch semnav_bringup sim.launch.py` starts Gazebo turtlebot3_world with the waffle
      (launch arg `robot_model`, default waffle) - verified headless (`gui:=false rviz:=false`)
- [x] Topics verified headless: /scan 5.0 Hz frame base_scan; /camera/image_raw 1920x1080 rgb8
      frame camera_rgb_frame at ~17 Hz (configured 30 Hz; CPU-limited); /odom ~29 Hz
- [x] TF tree verified with view_frames: odom -> base_footprint -> base_link -> {base_scan,
      camera_link -> camera_rgb_frame -> camera_rgb_optical_frame, ...};
      camera_rgb_optical_frame -> base_scan resolves
- [x] Robot drives on /cmd_vel (scripted Twist 0.15 m/s: odom x -2.00 -> -1.41, stops on zero)
- [x] USER: teleop by keyboard (`ros2 run teleop_twist_keyboard teleop_twist_keyboard`)
      - confirmed by user 2026-10-03
- [x] USER: RViz (launched by sim.launch.py, config rviz/semnav.rviz) shows robot model, TF,
      LaserScan and Camera image - confirmed by user 2026-10-03

**Status: Phase 1 DONE (2026-10-03).**

**Done when:** Teleop works; camera and scan visible in RViz.

## Phase 2 (Day 2) - Map, Nav2, YOLO sanity check
- [x] Map built autonomously (2026-10-03): `scripts/auto_map.py` derives 10 waypoints from the world
      file (0.5 m clearance), 10/10 NavigateToPose SUCCEEDED under mapping + navigation (headless,
      /clock 1 publisher); slam vs ground truth <= 0.034 m / 0.6 deg; map->odom never stale.
      Saved maps/semnav_world.{pgm,yaml}. Map check: outer wall closed (flood fill from spawn stays
      inside), 9/9 pillars (centroid offset <= 0.08 m), occupied cells vs true surfaces median
      0.017 m, p90 0.049 m, 100 % within 0.1 m, best-fit offset 0.26 deg / 0.026 m.
      (For RViz on this machine use rviz_software_gl:=true, D-17)
- [x] Bring up Nav2 + AMCL + map_server on saved map (Mode A): launch/localization.launch.py
      (map_server + amcl, default map maps/semnav_world.yaml). Verified headless 2026-10-03: initial
      pose (-2.0, -0.5) on /initialpose, goal (1.75, 0.55) SUCCEEDED, AMCL map->base_footprint vs
      ground truth 0.081 m / 1.9 deg
- [ ] USER: send a 2D Goal Pose from RViz in Mode A
- [x] Nav2 params based on nav2_bringup nav2_params.yaml + TB3 waffle overlay (D-03), load without errors
      (config/nav2_params.yaml: NavFn, DWB, local obstacle+inflation, global static+obstacle+inflation,
      no velocity_smoother; TB3 humble waffle max_vel_x 0.22) - verified 2026-10-03
- [x] launch/navigation.launch.py (D-01, D-02, D-16): controller/planner/smoother/behavior/bt_navigator/
      waypoint_follower, no velocity_smoother; controller + behavior_server cmd_vel -> /cmd_vel_nav;
      arg cmd_vel_relay (default true) runs topic_tools relay /cmd_vel_nav -> /cmd_vel.
      Verified headless with mapping.launch.py: all 6 nodes active, 0 ERROR lines, /cmd_vel has exactly
      one publisher (cmd_vel_relay), short NavigateToPose goal SUCCEEDED
- [x] launch/mapping.launch.py (sim + slam_toolbox online_async, config/slam_toolbox_online_async.yaml,
      max_laser_range 3.5) + scripts/save_map.sh -> semnav_bringup/maps/; /map published headless
- [x] Copy waffle model into semnav_bringup (models/semnav_waffle): camera 640x480 @ 15 Hz,
      frame_name camera_rgb_optical_frame (D-05, D-06); spawned by sim.launch.py
- [x] Own world worlds/semnav_world.world: TB3 world + libgazebo_ros_state.so (ns /gazebo, 10 Hz)
      + person_1 / chair_1 / bottle_1 in camera view from spawn (D-04, D-14); object models
      fetched by `scripts/fetch_models.sh` into gitignored models_external/
- [x] Verified headless (2026-10-03): /camera/image_raw 640x480 rgb8 frame camera_rgb_optical_frame
      (~8 Hz measured at RTF 0.99, configured 15 Hz; render/CPU-limited); /camera/camera_info
      640x480 fx 565.6 cx 320.5; /scan 4.8 Hz frame base_scan; /gazebo/model_states lists
      ground_plane, turtlebot3_world, person_1, chair_1, bottle_1, waffle;
      TF camera_rgb_optical_frame -> base_scan resolves
- [x] Python ONNX sanity check on sim camera frames (2026-10-03): `scripts/grab_sim_frames.py`
      (system python + ROS, 20 PNGs -> data/sim_frames/) and `scripts/check_yolo_on_frames.py`
      (.venv, onnxruntime + opencv; letterbox 114, decode, per-class NMS, conf 0.35 / IoU 0.45).
      Result on 20 frames from spawn: chair max 0.917 (20/20), person max 0.842 (20/20);
      bottle_1 (textured beer can) NOT detected even at conf 0.05; mean inference 37.6 ms (CPU, 4 threads)
- [x] YOLO export script `scripts/export_yolo.py` (YOLOv8n, 640, opset 12, simplify; defaults match
      existing models/yolov8n.onnx metadata; refuses overwrite without --force; real export not re-run)

**Done when:** Robot reaches a goal; YOLO detects at least one object class in sim.

## Phase 3 (Day 3) - Messages + minimal YOLO node
- [x] `scripts/setup_ort.sh` fetches pinned ONNX Runtime into third_party/onnxruntime (1.20.1 CPU, SHA-256 verified, D-12)
- [x] Hello-world Ort::Session builds and runs before any ROS code (risk mitigation, section 11) - test_ort_session: 3/3 pass
- [x] `semnav_msgs`: SemanticObstacle.msg, SemanticObstacleArray.msg exactly as section 6 (2026-10-03, builds warning-free)
- [ ] `semnav_perception`: header-only letterbox.hpp / yolo_detector.hpp (no ROS)
- [ ] yolo_onnx_node: subscribe /camera/image_raw (SensorData), infer, publish
      /detections (vision_msgs/Detection2DArray, reliable depth 5, stamp copied from image)
      and /detections/image (annotated, best effort)

**Done when:** Boxes visible in RViz.

## Phase 4 (Day 4) - YOLO node hardening + tests
- [ ] Worker thread; callback stores newest frame under mutex + condition variable; drops old frames
- [ ] ORT options: intra_op_num_threads (default 4), ORT_ENABLE_ALL, reused input buffer, use_cuda param
- [ ] Parameters: model_path, conf_threshold (0.35), iou_threshold (0.45), input_size, class_filter (default [person, chair], D-15),
      use_cuda, publish_annotated
- [ ] Stage timing (preprocess/infer/postprocess), rolling p50/p95 logged every 5 s, published on /metrics (D-09)
- [ ] GoogleTest: letterbox round-trip within 1 px; decode on synthetic [1,84,8400] tensor; NMS removes duplicate

**Done when:** p50/p95 reported; unit tests green.

## Phase 5 (Day 5) - semantic_fusion_node
- [ ] Cache intrinsics (fx, cx) from /camera/camera_info
- [ ] Header-only fusion.hpp: bbox u-span -> optical rays -> laser-frame yaw sector; central
      sector_fraction (60%); reject invalid ranges; median; position; class-radius padding (D-10)
- [ ] TF lookup image-header frame -> laser frame at image stamp; no hard-coded frame names;
      unresolved TF fails loudly and drops the detection (D-05)
- [ ] Scan ring buffer, pick closest stamp (or ApproximateTime, slop 0.1 s)
- [ ] Header-only tracker.hpp: NN association per class with gate 0.6 m, smoothing_alpha, ids,
      miss counter, ttl 2 s
- [ ] Publish /semantic_obstacles (frame map, reliable depth 5) and /semantic_markers
- [ ] Parameters: class_radius map (person 0.35, chair 0.25, default 0.3), min/max_range,
      sector_fraction, assoc_gate, ttl, smoothing_alpha
- [ ] GoogleTest for projection / sector / median / tracker

**Done when:** Markers sit on the right objects in RViz.

## Phase 6 (Day 6) - SemanticLayer costmap plugin
- [ ] `semnav_costmap`: nav2_costmap_2d::Layer subclass, PLUGINLIB_EXPORT_CLASS, plugin.xml
- [ ] Prototype: paint one fixed disc; confirm plugin loads
- [ ] Subscription to /semantic_obstacles, mutex snapshot copied at start of updateBounds
- [ ] updateBounds dirty window, updateCosts with LETHAL core + 252*exp(-k*(d-r)) decay to
      class_inflation (person 1.0, chair 0.3, default 0.2), std::max combine, reset() clears cache
- [ ] tf_ conversion map -> costmap global frame for rolling local costmap
- [ ] Added to global (static, obstacle, semantic, inflation) and local (obstacle, semantic, inflation) costmaps
- [ ] GoogleTest for header-only cost math

**Done when:** Costmap visibly changes around a person.

## Phase 7 (Day 7) - safety_gate_node + single launch
- [ ] Header-only gate logic: forward cone min range (+/-30 deg, widened with angular speed),
      s = clamp((d - d_stop)/(d_slow - d_stop), 0, 1), v = s * v_cmd, angular kept
- [ ] 20 Hz timer publish; watchdog (0.5 s cmd, 0.3 s scan) -> zero; accel limiter
- [ ] Cone direction follows sign of linear.x: forward cone forward, rear cone reversing (D-13, deviation)
- [ ] GoogleTest: wall at 0.3 m -> zero; far scan -> passthrough; obstacle behind + reverse -> zero,
      obstacle behind + forward -> passthrough (D-13)
- [ ] Own navigation launch without velocity_smoother; controller and behavior_server remapped to
      /cmd_vel_nav; gate is the only /cmd_vel publisher (D-01, D-02)
- [ ] semnav_bringup: one launch file starts Gazebo, Nav2, perception, fusion, gate, RViz

**Done when:** One command starts the full system.

## Phase 8 (Day 8) - Evaluation
- [ ] World loads libgazebo_ros_state.so in namespace /gazebo -> /gazebo/model_states (D-04)
- [ ] eval_logger_node in C++, package semnav_eval (D-10): latency p50/p95 + FPS from /metrics; position error per class
      vs ground truth; NavigateToPose goal list -> success, time, path length, min clearance to people; CSV
- [ ] A/B runs with semantic layer on/off, same goals
- [ ] `scripts/run_eval.sh`

**Done when:** Results table with real numbers.

## Phase 9 (Day 9) - Polish
- [ ] semnav.rviz: map, both costmaps, plans, scan, footprint, TF, /detections/image, /semantic_markers,
      2D Pose Estimate + 2D Goal Pose; loaded from one launch argument
- [ ] README (GIF, problem statement, diagram, results table, Limitations, AGPL note for Ultralytics weights)
- [ ] docs/design-notes.md (user writes in own words)
- [ ] GitHub Actions: build + unit tests, CI badge
- [ ] Demo video / GIF

**Done when:** A stranger can clone and run it in under 15 minutes.

## Phase 10 (Day 10) - Buffer
- [ ] Bug fixes, optional stretch goal, rehearsal
- [ ] Tag v1.0 (user tags and pushes)

**Done when:** Repo tagged v1.0.

Cut order if behind: (1) MPPI, (2) tracking smoothing, (3) CUDA, (4) eval automation.
Never cut: SemanticLayer plugin, unit tests, latency numbers, demo video.
