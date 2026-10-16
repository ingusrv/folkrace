#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
shared_dir="$project_dir/../shared"
host_tests="$project_dir/tests/host"
build_dir=$(mktemp -d /tmp/sniegbaltite-host.XXXXXX)
trap 'rm -rf "$build_dir"' EXIT
c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$host_tests/fakes" -I"$shared_dir/src" \
  "$host_tests/policy_test.cpp" "$shared_dir/src/driver/bmi088_checked.cpp" \
  -o "$build_dir/policy_test"
"$build_dir/policy_test"
c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$host_tests/fakes" -I"$shared_dir/src" \
  "$host_tests/transport_test.cpp" "$shared_dir/src/driver/bmi088_checked.cpp" \
  -o "$build_dir/transport_test"
"$build_dir/transport_test"
c++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only \
  -I"$host_tests/fakes" -I"$shared_dir/src" "$host_tests/layout_test.cpp"
side_driver="$shared_dir/vendor/VL53L5CX/src"
c++ -std=c++17 -Wall -Wextra -Werror -Wno-unused-parameter \
  -I"$side_driver" -I"$host_tests/fakes" -I"$shared_dir/src" \
  -I"$project_dir/../rosepink/src" \
  "$host_tests/side_transport_test.cpp" \
  "$side_driver/SparkFun_VL53L5CX_IO.cpp" \
  "$side_driver/SparkFun_VL53L5CX_Library.cpp" \
  "$side_driver/platform.cpp" "$side_driver/vl53l5cx_api.cpp" \
  -o "$build_dir/side_transport_test"
"$build_dir/side_transport_test"
c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$shared_dir/src" -I"$project_dir/../banana/src" \
  "$host_tests/drive_output_test.cpp" -o "$build_dir/drive_output_test"
"$build_dir/drive_output_test"
c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$shared_dir/src" -I"$project_dir/src" \
  "$host_tests/navigation_test.cpp" -o "$build_dir/navigation_test"
"$build_dir/navigation_test"
c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$shared_dir/src" -I"$project_dir/src" \
  "$host_tests/stall_test.cpp" -o "$build_dir/stall_test"
"$build_dir/stall_test"
c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$shared_dir/src" -I"$project_dir/src" \
  "$host_tests/escape_test.cpp" -o "$build_dir/escape_test"
"$build_dir/escape_test"
c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$shared_dir/src" -I"$project_dir/src" \
  "$host_tests/stall_replay_test.cpp" -o "$build_dir/stall_replay_test"
"$build_dir/stall_replay_test" "$host_tests/fixtures/stall_2026-10-14.txt"
c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$host_tests/fakes" -I"$shared_dir/src" \
  "$host_tests/no_return_replay_test.cpp" -o "$build_dir/no_return_replay_test"
"$build_dir/no_return_replay_test" "$host_tests/fixtures/no_return_2026-10-14.txt"
