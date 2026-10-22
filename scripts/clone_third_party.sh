#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TP="${ROOT}/third_party"
mkdir -p "${TP}"
cd "${TP}"

clone_pin() {
  local dir="$1" url="$2" sha="$3"
  if [ -d "${dir}/.git" ]; then
    echo "exists ${dir}; fetching..."
    git -C "${dir}" fetch --recurse-submodules origin
  else
    git clone --recurse-submodules "${url}" "${dir}"
  fi
  git -C "${dir}" checkout "${sha}"
  git -C "${dir}" submodule update --init --recursive
  echo "OK ${dir} @ $(git -C "${dir}" rev-parse HEAD)"
}

clone_pin fkie-nbv-planner https://github.com/fkie/fkie-nbv-planner.git 91091f8a58301a69bbf240bce073059092e6f606
clone_pin fkie_ddynamic_reconfigure https://github.com/fkie/fkie_ddynamic_reconfigure.git 7613e5319d815741fedc762cb39d19b3cc3cb6af
clone_pin fkie_environmental_measurements https://github.com/fkie/fkie_environmental_measurements.git 75d331770c4c43fc0247876a192e5dd6809cd759
clone_pin SSMI-Example https://github.com/ExistentialRobotics/SSMI-example.git 750122b8dbb193dcf67af5429be6c8df3b3b462e

# AGX whole-body description (Ranger + CR10 + AG95 + D455); pin to main tip at clone time
if [ -d agx/.git ]; then
  echo "exists agx; fetch..."
  git -C agx fetch origin || true
else
  git clone https://github.com/LiiiiPeggy/agx.git agx
fi
echo "OK agx @ $(git -C agx rev-parse --short HEAD 2>/dev/null || echo unknown)"
ln -sfn ../../third_party/agx/rangerboxcr10lidar_description "${ROOT}/ws_fkie/src/rangerboxcr10lidar_description"

"${ROOT}/scripts/setup_fkie_ws.sh"
echo "third_party ready"
