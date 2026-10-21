#!/usr/bin/env bash
# Create / link FKIE catkin workspace under project root.
# Use relative symlinks so the same tree works on host and in /workspace mount.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROOT}/ws_fkie"
SRC="${WS}/src"
TP_REL="../../third_party"

mkdir -p "${SRC}"
cd "${SRC}"

link_rel() {
  local name="$1"
  local rel="$2"
  ln -sfn "${rel}" "${name}"
  echo "linked ${SRC}/${name} -> ${rel}"
}

link_rel "fkie_ddynamic_reconfigure" "${TP_REL}/fkie_ddynamic_reconfigure"
link_rel "fkie-nbv-planner" "${TP_REL}/fkie-nbv-planner"
link_rel "fkie_measurement_msgs" "${TP_REL}/fkie_environmental_measurements/fkie_measurement_msgs"
link_rel "fkie_measurement" "${TP_REL}/fkie_environmental_measurements/fkie_measurement"
link_rel "fkie_measurement_server" "${TP_REL}/fkie_environmental_measurements/fkie_measurement_server"
link_rel "fkie_measurement_sensor_simulator" "${TP_REL}/fkie_environmental_measurements/fkie_measurement_sensor_simulator"
link_rel "fkie_measurement_tools" "${TP_REL}/fkie_environmental_measurements/fkie_measurement_tools"

# Sanity: resolve from this directory
ls -la
test -f fkie-nbv-planner/package.xml
test -f fkie_ddynamic_reconfigure/package.xml
test -f fkie_measurement_msgs/package.xml

echo "FKIE workspace ready at ${WS}"
