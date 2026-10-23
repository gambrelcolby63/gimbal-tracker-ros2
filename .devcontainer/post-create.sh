#!/usr/bin/env bash
# Build the core library, run its tests, and build the Humble workspace.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  (cd ros2_ws && colcon build --symlink-install)
  touch "$HOME/.bashrc"
  grep -qxF "source /opt/ros/humble/setup.bash" "$HOME/.bashrc" || \
    echo "source /opt/ros/humble/setup.bash" >> "$HOME/.bashrc"
  grep -qxF "source $ROOT/ros2_ws/install/setup.bash" "$HOME/.bashrc" || \
    echo "source $ROOT/ros2_ws/install/setup.bash" >> "$HOME/.bashrc"
fi
