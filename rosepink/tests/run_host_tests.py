#!/usr/bin/env python3
"""Exercise geometry/startup with real ULD types; stub only Arduino declarations."""
import configparser
from pathlib import Path
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
config = configparser.ConfigParser()
config.read(root / "platformio.ini")
flags = [flag for flag in shlex.split(config["env:rosepink"]["build_flags"])
         if flag.startswith("VL53L5CX_DISABLE_")]
with tempfile.TemporaryDirectory(prefix="rosepink-tests-") as temporary:
    stub = Path(temporary)
    (stub / "Arduino.h").write_text("#pragma once\n#include <stdint.h>\nusing byte = uint8_t;\n")
    (stub / "Wire.h").write_text("#pragma once\nclass TwoWire {};\nextern TwoWire Wire;\n")
    executable = stub / "geometry_test"
    subprocess.run([
        "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
        *["-D" + flag for flag in flags],
        "-I" + str(stub), "-I" + str(root / "src"),
        "-I" + str(root.parent / "shared/src"),
        "-I" + str(root.parent / "shared/vendor/VL53L5CX/src"),
        str(root / "tests/geometry_test.cpp"), "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
