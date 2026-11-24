# PlatformIO pre-script: C flags for the micro-ROS library build.
#
# The pinned teensy@6.0.0 platform ships GCC 15.2. Two of its defaults break
# the micro-ROS Humble C sources, which were written for GCC 10-era rules:
#
#   - C23 is the default standard (bool/true/false become keywords), and
#   - since GCC 14, an implicit function declaration is an ERROR. rcutils
#     calls clock_gettime() without a prototype; micro_ros_platformio supplies
#     the function itself (platform_code/arduino/clock_gettime.cpp).
#
# CFLAGS is C only, so none of this touches the C++ of this firmware.
# micro_ros_platformio reads env CFLAGS when it writes its CMake toolchain,
# which is how these reach the library build.
Import("env")  # noqa: F821  (provided by PlatformIO's SCons)

env.Append(CFLAGS=[  # noqa: F821
    "-std=gnu11",
    "-Wno-error=implicit-function-declaration",
    "-Wno-error=incompatible-pointer-types",
    "-Wno-error=int-conversion",
])
