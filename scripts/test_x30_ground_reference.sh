#!/usr/bin/env bash
set -eo pipefail
# Run inside the built Humble container after sourcing install/setup.bash.
x30_test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
x30_test_build="$(mktemp -d /tmp/x30_ground_reference_test.XXXXXX)"
trap 'rm -rf "$x30_test_build"' EXIT
c++ -std=c++17 -O2 "$x30_test_dir/test_x30_ground_reference.cpp" \
  -I "$x30_test_dir/../src/dddmr_perception_3d/include" \
  $(pkg-config --cflags --libs pcl_kdtree) -o "$x30_test_build/test_ground_reference"
"$x30_test_build/test_ground_reference"
python3 "$x30_test_dir/test_x30_ground_goal.py" --obstacles
