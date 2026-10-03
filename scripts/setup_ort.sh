#!/usr/bin/env bash
# setup_ort.sh - fetch the pinned ONNX Runtime C++ (CPU, linux x64) release into
# <repo>/third_party/onnxruntime (gitignored). Decision record: docs/DECISIONS.md D-12.
#
# Usage:
#   scripts/setup_ort.sh           # install if missing or version differs; no-op otherwise
#   scripts/setup_ort.sh --force   # wipe and reinstall
#
# Result layout:
#   third_party/onnxruntime/include/onnxruntime_cxx_api.h ...
#   third_party/onnxruntime/lib/libonnxruntime.so -> libonnxruntime.so.1 -> libonnxruntime.so.<VER>
#   third_party/onnxruntime/VERSION   (contains ORT_VERSION; used for the idempotency check)
#
# The tarball's SHA-256 is pinned below and verified on every download (mismatch = abort).
# Needs network access and curl or wget. No sudo.
set -euo pipefail

ORT_VERSION="1.20.1"
ORT_SHA256="67db4dc1561f1e3fd42e619575c82c601ef89849afc7ea85a003abbac1a1a105"
ORT_TARBALL="onnxruntime-linux-x64-${ORT_VERSION}.tgz"
ORT_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${ORT_TARBALL}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEST="${REPO_ROOT}/third_party/onnxruntime"

FORCE=0
for arg in "$@"; do
  case "${arg}" in
    --force) FORCE=1 ;;
    -h|--help)
      sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "error: unknown argument '${arg}' (use --force or --help)" >&2
      exit 2
      ;;
  esac
done

if [[ ${FORCE} -eq 0 && -f "${DEST}/VERSION" ]] \
   && [[ "$(cat "${DEST}/VERSION")" == "${ORT_VERSION}" ]] \
   && [[ -e "${DEST}/lib/libonnxruntime.so" && -f "${DEST}/include/onnxruntime_cxx_api.h" ]]; then
  echo "ONNX Runtime ${ORT_VERSION} already installed in ${DEST}"
  exit 0
fi

if command -v curl >/dev/null 2>&1; then
  fetch() { curl -fL --retry 3 -sS -o "$2" "$1"; }
elif command -v wget >/dev/null 2>&1; then
  fetch() { wget -q -O "$2" "$1"; }
else
  echo "error: neither curl nor wget found; install one of them" >&2
  exit 1
fi
command -v sha256sum >/dev/null 2>&1 || { echo "error: sha256sum not found" >&2; exit 1; }
command -v tar >/dev/null 2>&1 || { echo "error: tar not found" >&2; exit 1; }

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

echo "Downloading ${ORT_URL}"
fetch "${ORT_URL}" "${TMP_DIR}/${ORT_TARBALL}"

echo "Verifying SHA-256"
if ! echo "${ORT_SHA256}  ${TMP_DIR}/${ORT_TARBALL}" | sha256sum -c --status -; then
  echo "error: SHA-256 mismatch for ${ORT_TARBALL}" >&2
  echo "  expected: ${ORT_SHA256}" >&2
  echo "  actual:   $(sha256sum "${TMP_DIR}/${ORT_TARBALL}" | cut -d' ' -f1)" >&2
  exit 1
fi

echo "Extracting to ${DEST}"
mkdir -p "${TMP_DIR}/extract"
tar -xzf "${TMP_DIR}/${ORT_TARBALL}" -C "${TMP_DIR}/extract" --strip-components=1
if [[ ! -f "${TMP_DIR}/extract/include/onnxruntime_cxx_api.h" || ! -e "${TMP_DIR}/extract/lib/libonnxruntime.so" ]]; then
  echo "error: unexpected tarball layout (missing include/ or lib/libonnxruntime.so)" >&2
  exit 1
fi
echo "${ORT_VERSION}" > "${TMP_DIR}/extract/VERSION"

rm -rf "${DEST}"
mkdir -p "$(dirname "${DEST}")"
mv "${TMP_DIR}/extract" "${DEST}"

echo "ONNX Runtime ${ORT_VERSION} installed in ${DEST}"
