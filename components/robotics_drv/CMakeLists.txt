# robotics_drv - the module's drivers. Each one knows a part or a peripheral and
# nothing about who calls it or when: that is robotics_aos's.
#
# robotics_drv_at8833.c is plain C on purpose - no ESP-IDF, no FreeRTOS - because it is
# the chip's truth table and tests/host runs it as it is.
#
# Laid out as kdrv is: one folder per driver, its header in include/ and its
# source in src/. Every include/ is listed below, so an #include names the
# header alone and the folders are on disk only.
idf_component_register(
    SRCS "at8833/src/robotics_drv_at8833.c"
         "bridge/src/robotics_drv_bridge.c"
         "pins/src/robotics_drv_pins.c"
         "servo_pwm/src/robotics_drv_servo_pwm.c"
         "stepper/src/robotics_drv_stepper.c"
         "i2c_scan/src/robotics_drv_i2c_scan.c"
    INCLUDE_DIRS "at8833/include"
                 "bridge/include"
                 "i2c_scan/include"
                 "pins/include"
                 "servo_pwm/include"
                 "stepper/include"
    REQUIRES kdefs robotics_bsp
    PRIV_REQUIRES klog freertos esp_driver_gpio esp_driver_mcpwm esp_driver_gptimer esp_driver_pcnt
                  esp_driver_i2c
                  esp_system
)
