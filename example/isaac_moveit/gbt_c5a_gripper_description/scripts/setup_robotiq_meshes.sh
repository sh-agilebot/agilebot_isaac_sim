#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

REQUIRED_FILES=(
  "robotiq_arg2f_base_link.stl"
  "robotiq_arg2f_coupling.stl"
  "robotiq_arg2f_140_outer_knuckle.stl"
  "robotiq_arg2f_140_outer_finger.stl"
  "robotiq_arg2f_140_inner_knuckle.stl"
  "robotiq_arg2f_140_inner_finger.stl"
)

SOURCE_DIR=""

usage() {
  cat <<'EOF'
Usage:
  bash scripts/setup_robotiq_meshes.sh <source_dir>

Description:
  Install only the required Robotiq STL files into:
    - meshes/visual/
    - meshes/collision/

Notes:
  - This script only installs files on the local machine.
  - You must prepare the STL files yourself from a lawful source before running it.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ -z "${SOURCE_DIR}" ]]; then
        SOURCE_DIR="$1"
        shift
      else
        echo "Unexpected argument: $1" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

if [[ -z "${SOURCE_DIR}" ]]; then
  usage
  exit 1
fi

if [[ ! -d "${SOURCE_DIR}" ]]; then
  echo "Source directory not found: ${SOURCE_DIR}" >&2
  exit 1
fi

mkdir -p \
  "${REPO_ROOT}/meshes/visual" \
  "${REPO_ROOT}/meshes/collision"

missing_files=()
for file_name in "${REQUIRED_FILES[@]}"; do
  if ! find "${SOURCE_DIR}" -type f -name "${file_name}" | grep -q .; then
    missing_files+=("${file_name}")
  fi
done

if [[ ${#missing_files[@]} -gt 0 ]]; then
  printf 'Missing required STL:\n' >&2
  printf '  %s\n' "${missing_files[@]}" >&2
  exit 1
fi

for file_name in "${REQUIRED_FILES[@]}"; do
  source_file="$(find "${SOURCE_DIR}" -type f -name "${file_name}" | head -n 1)"

  for target_file in \
    "${REPO_ROOT}/meshes/visual/${file_name}" \
    "${REPO_ROOT}/meshes/collision/${file_name}"; do
    if [[ "$(realpath "${source_file}")" != "$(realpath -m "${target_file}")" ]]; then
      cp "${source_file}" "${target_file}"
    fi
  done

  echo "Installed ${file_name}"
done

echo "Robotiq STL setup complete."
