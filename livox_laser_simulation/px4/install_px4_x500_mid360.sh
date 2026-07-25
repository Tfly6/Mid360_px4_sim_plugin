#!/usr/bin/env bash
# Force-install the files needed by: make px4_sitl gz_x500_mid360
set -euo pipefail

[[ $# -eq 1 ]] || {
  echo "Usage: $0 /path/to/PX4-Autopilot" >&2
  exit 2
}

px4_dir=$1
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
airframes_dir="$px4_dir/ROMFS/px4fmu_common/init.d-posix/airframes"
models_dir="$px4_dir/Tools/simulation/gz/models"
plugin_dir="$px4_dir/build/px4_sitl_default/src/modules/simulation/gz_plugins"

[[ -f "$airframes_dir/CMakeLists.txt" && -d "$models_dir" && -d "$plugin_dir" ]] || {
  echo "PX4 path is invalid or PX4 SITL has not been built: $px4_dir" >&2
  exit 1
}
[[ -f "$script_dir/../../../../build/livox_laser_simulation/liblivox_laser_simulation_gz.so" ]] || {
  echo "Plugin is not built; run colcon build first." >&2
  exit 1
}


cp -a "$script_dir/../models/Mid360" "$models_dir/"
cp -a "$script_dir/../models/x500_mid360" "$models_dir/"

install -D -m 0644 "$script_dir/4022_gz_x500_mid360" \
  "$airframes_dir/4022_gz_x500_mid360"
if ! grep -Eq '^[[:space:]]*4022_gz_x500_mid360$' "$airframes_dir/CMakeLists.txt"; then
  sed -i '/^[[:space:]]*4021_gz_x500_flow$/a\	4022_gz_x500_mid360' \
    "$airframes_dir/CMakeLists.txt"
fi

install -m 0644 \
  "$script_dir/../../../../build/livox_laser_simulation/liblivox_laser_simulation_gz.so" \
  "$plugin_dir/liblivox_laser_simulation_gz.so"

echo "Installed: Mid360, x500_mid360, 4022_gz_x500_mid360, and Livox plugin."
