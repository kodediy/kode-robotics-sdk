# The plain-C suite: the rules of the module (robotics_plan.c) and the chip's
# truth table (robotics_drv_at8833.c), as they are, built for ESP-IDF's
# `linux` target. They need nothing but Unity: no faked kode-sdk, no
# FreeRTOS. The owner that uses them is tests/host_owner.
#
# `set(COMPONENTS main)` must come BEFORE project(): without it the configure
# walks the whole component registry looking for host implementations that
# do not exist (kode-sdk's tests/host says the same, and why).
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(COMPONENTS main)

project(robotics_host_tests)
