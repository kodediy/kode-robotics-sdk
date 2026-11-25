# The standard layer's suite: robotics.c, what a task calls, over an owner that only
# writes down what it is asked. A project of its own because the owner's
# suite links the real owner, and C allows one definition per binary.
#
# `set(COMPONENTS main)` must come BEFORE project(): see tests/host.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(COMPONENTS main)

project(robotics_standard_tests)
