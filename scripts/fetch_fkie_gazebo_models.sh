#!/usr/bin/env bash
# Fetch Gazebo Classic models required by FKIE warehouse/outdoor worlds.
# Source: PX4-SITL_gazebo-classic (shelves_high, big_box3/4) + minimal Floor_room materials.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/third_party/gazebo_models"
TMP="${TMPDIR:-/tmp}/px4_gazebo_models_$$"
mkdir -p "${DEST}"
rm -rf "${TMP}"
git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/PX4/PX4-SITL_gazebo-classic.git "${TMP}"
git -C "${TMP}" sparse-checkout set models/shelves_high models/big_box3 models/big_box4
for m in shelves_high big_box3 big_box4; do
  rm -rf "${DEST}/${m}"
  cp -a "${TMP}/models/${m}" "${DEST}/"
done
rm -rf "${TMP}"

# Floor_room: world only needs materials/scripts (geometry is inline boxes)
mkdir -p "${DEST}/Floor_room/materials/scripts" "${DEST}/Floor_room/materials/textures"
cat > "${DEST}/Floor_room/materials/scripts/floor_room.material" <<'EOF'
material floor_room
{
  technique
  {
    pass
    {
      ambient 0.35 0.35 0.35 1.0
      diffuse 0.45 0.45 0.45 1.0
      specular 0.1 0.1 0.1 1.0 10
    }
  }
}
EOF
cat > "${DEST}/Floor_room/model.config" <<'EOF'
<?xml version="1.0"?>
<model>
  <name>Floor_room</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>Minimal floor material pack for FKIE warehouse world</description>
</model>
EOF
cat > "${DEST}/Floor_room/model.sdf" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="Floor_room">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>1 1 0.001</size></box></geometry>
        <material>
          <script>
            <uri>model://Floor_room/materials/scripts</uri>
            <uri>model://Floor_room/materials/textures</uri>
            <name>floor_room</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF

echo "Models installed under ${DEST}"
ls -la "${DEST}"
echo "Export before Gazebo:"
echo "  export GAZEBO_MODEL_PATH=${DEST}:\$GAZEBO_MODEL_PATH"
