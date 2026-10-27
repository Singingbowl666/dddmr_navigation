#!/usr/bin/env bash
set -eo pipefail
# Run inside dddmr_humble_dev after sourcing install/setup.bash.
# The Python runner forces domain 88, localhost DDS and a test-only cmd_vel.
x30_test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for x30_test_case in left right wall stale; do
  python3 "$x30_test_dir/test_x30_obstacle_avoidance.py" --case "$x30_test_case"
done
