#!/usr/bin/env bash
# Build the SemNav ROS 2 underlay: tf2 / tf2_ros from ros2/geometry2 tag 0.25.24 (D-20).
#
# Why: ros-humble-tf2 0.25.23 (newest apt package as of 2026-10-03) has an ABBA deadlock between
# tf2_ros::Buffer::waitForTransform and tf2::BufferCore::testTransformableRequests that freezes
# a Nav2 server's TF intake under load. Fixed upstream in ros2/geometry2#982, backported to Humble
# in ros2/geometry2#990 (commit 9997e969), released in tag 0.25.24. Once apt ships >= 0.25.24 this
# underlay is no longer needed.
#
# Usage (once; network needed for the clone):
#   scripts/setup_underlay.sh            # clone + build into ~/semnav_underlay
#   scripts/setup_underlay.sh --test     # also run the upstream tf2_ros buffer tests
# Then, in every shell, BEFORE sourcing the SemNav workspace:
#   source /opt/ros/humble/setup.bash
#   source ~/semnav_underlay/install/setup.bash
#   source <repo>/install/setup.bash
set -euo pipefail

UNDERLAY="${SEMNAV_UNDERLAY:-$HOME/semnav_underlay}"
REPO_URL="https://github.com/ros2/geometry2.git"
TAG="0.25.24"
COMMIT="404b7224d623d614f18fa9738dbf1716403d857e"  # tag 0.25.24 (contains 9997e969)
RUN_TESTS=0
[[ "${1:-}" == "--test" ]] && RUN_TESTS=1

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "error: /opt/ros/humble/setup.bash not found" >&2
  exit 1
fi
command -v git >/dev/null || { echo "error: git not found" >&2; exit 1; }

SRC="$UNDERLAY/src/geometry2"
mkdir -p "$UNDERLAY/src"
if [[ ! -d "$SRC/.git" ]]; then
  git clone --quiet --branch "$TAG" "$REPO_URL" "$SRC"
fi
git -C "$SRC" fetch --quiet --tags origin || true
git -C "$SRC" checkout --quiet "$TAG"
actual="$(git -C "$SRC" rev-parse HEAD)"
if [[ "$actual" != "$COMMIT" ]]; then
  echo "error: $SRC is at $actual, expected $COMMIT (tag $TAG)" >&2
  exit 1
fi
git -C "$SRC" merge-base --is-ancestor 9997e969 HEAD \
  || { echo "error: deadlock fix 9997e969 missing" >&2; exit 1; }

# shellcheck disable=SC1091
set +u; source /opt/ros/humble/setup.bash; set -u
cd "$UNDERLAY"
# tf2's own tests need ament_cmake_google_benchmark (not installed); the deadlock regression
# test (test_buffer) is in tf2_ros.
colcon build --packages-select tf2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=0
set +u; source "$UNDERLAY/install/setup.bash"; set -u
colcon build --packages-select tf2_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING="$RUN_TESTS"
if [[ "$RUN_TESTS" == 1 ]]; then
  colcon test --packages-select tf2_ros --ctest-args -R test_buffer
  colcon test-result --verbose
fi
echo "Underlay ready: source $UNDERLAY/install/setup.bash after /opt/ros/humble/setup.bash"
