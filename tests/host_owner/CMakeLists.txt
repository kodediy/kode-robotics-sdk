# The owner's suite: robotics_aos.c as it is, on the linux target's FreeRTOS,
# against a simulated module (main/fakes) and an Active Object of robotics_ao.h
# that is only a queue and a task. The plain-C rules it stands on
# (robotics_plan, robotics_drv_at8833) run as they are; everything else is a
# fake. It needs no kode-sdk checkout: the few kode-sdk headers it compiles
# against are fakes (tests/target is where the real ones are checked).
#
# `set(COMPONENTS main)` must come BEFORE project(): without it the configure
# walks the whole component registry looking for host implementations that
# do not exist (kode-sdk's tests/host says the same, and why).
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(COMPONENTS main)

project(robotics_owner_tests)
