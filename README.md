# Robotics Project 01

## Setup notes

### tf2 underlay (required on ROS 2 Humble until apt ships tf2 >= 0.25.24)

`ros-humble-tf2` 0.25.23 has an ABBA deadlock between `tf2_ros::Buffer::waitForTransform` and
`tf2::BufferCore::testTransformableRequests` that freezes a Nav2 server's TF intake under load
(DECISIONS D-20). It is fixed upstream in
[ros2/geometry2#982](https://github.com/ros2/geometry2/pull/982), backported to Humble in
[#990](https://github.com/ros2/geometry2/pull/990) and released in tag `0.25.24`.
Build that tag once into an underlay outside the repo:

```bash
scripts/setup_underlay.sh --test          # clones geometry2 0.25.24, builds tf2 + tf2_ros
```

Then source it in every shell, between ROS and the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/semnav_underlay/install/setup.bash
source install/setup.bash                  # after building the workspace with the underlay sourced
```
