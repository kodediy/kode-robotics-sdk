# The drivers' suite: robotics_drv_stepper.c, the stepper's alarm, over a timer and
# pins that write down what they are asked and fire when told;
# robotics_drv_bridge.c and robotics_drv_servo_pwm.c over an MCPWM that says
# what each pin does across a period; and robotics_drv_i2c_scan.c over a bus
# that answers as told. A project
# of its own because those fakes stand in for ESP-IDF's own headers, which
# no other suite should see.
#
# `set(COMPONENTS main)` must come BEFORE project(): see tests/host.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(COMPONENTS main)

project(robotics_drv_tests)
