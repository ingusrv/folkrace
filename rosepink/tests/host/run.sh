#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
shared_dir="$project_dir/../shared"
common_tests="$project_dir/../sniegbaltite/tests/host"
side_driver="$shared_dir/vendor/VL53L5CX/src"
build_dir=$(mktemp -d /tmp/rosepink-host.XXXXXX)
trap 'rm -rf "$build_dir"' EXIT
# Exercise the port against the same recovery regression cases as Sniegbaltite.
for test in navigation_test stall_test escape_test; do
  c++ -std=c++17 -Wall -Wextra -Werror \
    -I"$shared_dir/src" -I"$project_dir/src" \
    "$common_tests/$test.cpp" -o "$build_dir/$test"
  "$build_dir/$test"
done
c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$project_dir/src" -I"$common_tests/fakes" \
  -I"$shared_dir/src" -I"$side_driver" \
  "$project_dir/tests/host/multizone_test.cpp" -o "$build_dir/multizone_test"
"$build_dir/multizone_test"
c++ -std=c++17 -Wall -Wextra -Werror -Wno-unused-parameter \
  -I"$side_driver" -I"$common_tests/fakes" -I"$shared_dir/src" \
  -I"$project_dir/src" "$common_tests/side_transport_test.cpp" \
  "$side_driver/SparkFun_VL53L5CX_IO.cpp" \
  "$side_driver/SparkFun_VL53L5CX_Library.cpp" \
  "$side_driver/platform.cpp" "$side_driver/vl53l5cx_api.cpp" \
  -o "$build_dir/side_transport_test"
"$build_dir/side_transport_test"
