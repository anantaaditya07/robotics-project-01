#!/usr/bin/env bash
# Save the map built in Mode B (mapping.launch.py) with nav2_map_server map_saver_cli.
# The map is written into the source tree (src/semnav_bringup/maps/) so it can be committed
# and installed by CMake (maps/ is installed to the package share directory).
#
# Usage: scripts/save_map.sh [name] [--force]
#   name     map base name (default: semnav_map) -> maps/<name>.yaml + maps/<name>.pgm
#   --force  overwrite an existing maps/<name>.yaml
# Requires a sourced ROS 2 environment and a running mapping.launch.py publishing /map.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MAPS_DIR="${REPO_ROOT}/src/semnav_bringup/maps"

NAME="semnav_map"
FORCE=0
NAME_SET=0
for arg in "$@"; do
  case "${arg}" in
    --force) FORCE=1 ;;
    -h | --help)
      sed -n '2,9p' "${BASH_SOURCE[0]}"
      exit 0
      ;;
    -*)
      echo "Unknown option: ${arg}" >&2
      exit 2
      ;;
    *)
      if [[ "${NAME_SET}" -eq 1 ]]; then
        echo "Only one map name allowed (got '${NAME}' and '${arg}')" >&2
        exit 2
      fi
      NAME="${arg}"
      NAME_SET=1
      ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found on PATH. Run: source /opt/ros/humble/setup.bash" >&2
  exit 1
fi

OUT="${MAPS_DIR}/${NAME}"
if [[ -e "${OUT}.yaml" && "${FORCE}" -ne 1 ]]; then
  echo "${OUT}.yaml already exists. Use --force to overwrite." >&2
  exit 1
fi

mkdir -p "${MAPS_DIR}"
ros2 run nav2_map_server map_saver_cli -f "${OUT}" --ros-args -p use_sim_time:=true

echo "Saved map files:"
ls -l "${OUT}".*
