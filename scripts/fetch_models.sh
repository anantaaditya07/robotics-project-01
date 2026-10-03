#!/usr/bin/env bash
# Fetch the external Gazebo models used by semnav_world.world (see docs/DECISIONS.md D-14).
# Models are downloaded into src/semnav_bringup/models_external/ (not tracked by git).
# Versions are pinned so every download is reproducible.
#
# Usage: scripts/fetch_models.sh [--force]
#   --force  re-download models that are already present.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEST="${REPO_ROOT}/src/semnav_bringup/models_external"

# osrf/gazebo_models, pinned commit (master as of 2023-07-15).
OSRF_SHA="8163eb4b5e7e21985c6591d1c0bfb56468c0093f"
OSRF_BASE="https://raw.githubusercontent.com/osrf/gazebo_models/${OSRF_SHA}"

# Gazebo Fuel OpenRobotics/WoodenChair, pinned version.
CHAIR_VERSION="1"
CHAIR_BASE="https://fuel.gazebosim.org/1.0/OpenRobotics/models/WoodenChair/${CHAIR_VERSION}/files"

FORCE=0
for arg in "$@"; do
  case "${arg}" in
    --force) FORCE=1 ;;
    -h | --help)
      sed -n '2,8p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown argument: ${arg}" >&2
      exit 2
      ;;
  esac
done

PERSON_FILES=(
  model.config
  model.sdf
  meshes/standing.dae
  materials/textures/eyebrow001-unmodified.png
  materials/textures/eyebrow001.png
  materials/textures/green_eye.png
  materials/textures/jeans01_normals.png
  materials/textures/jeans_basic_diffuse.png
  materials/textures/male02_diffuse_black-unmodified.png
  materials/textures/male02_diffuse_black.png
  materials/textures/teeth.png
  materials/textures/tshirt02_normals.png
  materials/textures/tshirt02_texture.png
  materials/textures/young_lightskinned_male_diffuse.png
)

BEER_FILES=(
  model.config
  model.sdf
  model-1_4.sdf
  materials/scripts/beer.material
  materials/textures/beer.png
)

# Thumbnails are intentionally skipped.
CHAIR_FILES=(
  model.config
  model.sdf
  meshes/WoodenChair.obj
  meshes/WoodenChair.mtl
  meshes/WoodenChair_Col.obj
  meshes/WoodenChair_Col.mtl
  meshes/WoodenChair_Diffuse.png
  meshes/WoodenChair_Rough.png
)

# fetch_model <folder name (must match model:// URIs)> <base url> <files...>
fetch_model() {
  local name="$1" base="$2"
  shift 2
  local target="${DEST}/${name}"
  if [[ -f "${target}/model.sdf" && "${FORCE}" -eq 0 ]]; then
    echo "[skip] ${name} already present (use --force to re-download)"
    return 0
  fi
  echo "[get ] ${name}"
  local tmp="${DEST}/.${name}.partial"
  rm -rf "${tmp}"
  local f
  for f in "$@"; do
    mkdir -p "${tmp}/$(dirname "${f}")"
    curl --fail --silent --show-error --location --retry 3 \
      -o "${tmp}/${f}" "${base}/${f}"
    echo "       ${f}"
  done
  rm -rf "${target}"
  mv "${tmp}" "${target}"
}

mkdir -p "${DEST}"
fetch_model person_standing "${OSRF_BASE}/person_standing" "${PERSON_FILES[@]}"
fetch_model beer "${OSRF_BASE}/beer" "${BEER_FILES[@]}"
fetch_model WoodenChair "${CHAIR_BASE}" "${CHAIR_FILES[@]}"

cat >"${DEST}/LICENSES.txt" <<EOF
External Gazebo models for SemNav (fetched by scripts/fetch_models.sh; not tracked by git).

person_standing
  Source:  https://github.com/osrf/gazebo_models/tree/${OSRF_SHA}/person_standing
  Pinned:  osrf/gazebo_models commit ${OSRF_SHA}
  Author:  Marina Kollmitz (model created with MakeHuman)
  License: Creative Commons Attribution 3.0 (CC BY 3.0), https://creativecommons.org/licenses/by/3.0/

beer
  Source:  https://github.com/osrf/gazebo_models/tree/${OSRF_SHA}/beer
  Pinned:  osrf/gazebo_models commit ${OSRF_SHA}
  License: Creative Commons Attribution 3.0 (CC BY 3.0), https://creativecommons.org/licenses/by/3.0/

WoodenChair
  Source:  https://fuel.gazebosim.org/1.0/OpenRobotics/models/WoodenChair
  Pinned:  Fuel version ${CHAIR_VERSION} (thumbnails not downloaded)
  Author:  Wan Yi Seow, Open Robotics
  License: Creative Commons Zero v1.0 Universal (CC0 1.0), https://creativecommons.org/publicdomain/zero/1.0/
EOF
echo "[done] models in ${DEST}"
