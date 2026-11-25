# The runner, the cases, the drivers under test (the stepper's alarm, the
# bridge, the servo's pulses and the I2C scan) and the ESP-IDF they are given. The same shape as
# tests/host/main: see there for why the cases are globbed, sorted and linked
# whole. The fakes come first on the include path, so driver/gptimer.h,
# driver/mcpwm_prelude.h and the rest are theirs.
set(SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../../..")

file(GLOB CASE_SOURCES "${CMAKE_CURRENT_LIST_DIR}/../cases/test_*.c")
list(SORT CASE_SOURCES)

# kode_status_t: kdefs' one header, from the kode-sdk clone of ../../kode_sdk.cmake
# (kdefs declares esp32p4 only, and the component manager refuses it
# for `linux`).
include(${CMAKE_CURRENT_LIST_DIR}/../../kode_sdk.cmake)

idf_component_register(
    SRCS "runner.c"
         ${CASE_SOURCES}
         "${SDK_DIR}/components/robotics_drv/stepper/src/robotics_drv_stepper.c"
         "${SDK_DIR}/components/robotics_drv/at8833/src/robotics_drv_at8833.c"
         "${SDK_DIR}/components/robotics_drv/i2c_scan/src/robotics_drv_i2c_scan.c"
         "${SDK_DIR}/components/robotics_drv/bridge/src/robotics_drv_bridge.c"
         "${SDK_DIR}/components/robotics_drv/servo_pwm/src/robotics_drv_servo_pwm.c"
         "${SDK_DIR}/components/robotics_drv/pins/src/robotics_drv_pins.c"
         "${CMAKE_CURRENT_LIST_DIR}/fakes/fake_idf.c"
         "${CMAKE_CURRENT_LIST_DIR}/fakes/fake_mcpwm.c"
    INCLUDE_DIRS "${KODE_SDK_DIR}/components/kdefs/include"
                 "${CMAKE_CURRENT_LIST_DIR}/fakes"
                 "."
                 "${SDK_DIR}/components/robotics_drv/at8833/include"
                 "${SDK_DIR}/components/robotics_drv/bridge/include"
                 "${SDK_DIR}/components/robotics_drv/i2c_scan/include"
                 "${SDK_DIR}/components/robotics_drv/pins/include"
                 "${SDK_DIR}/components/robotics_drv/servo_pwm/include"
                 "${SDK_DIR}/components/robotics_drv/stepper/include"
                 "${SDK_DIR}/components/robotics_bsp/board/include"
    REQUIRES unity freertos
    WHOLE_ARCHIVE
)

# The C maths library: sqrt(), pow() and the rest live in libm, which GCC on Linux
# (the CI) does not link by itself. A host compiler may, so a missing line
# here only shows in the CI.
target_link_libraries(${COMPONENT_LIB} PRIVATE m)

# Sanitizers, only when ROBOTICS_SANITIZE is set; see tests/sanitize.cmake.
include(${CMAKE_CURRENT_LIST_DIR}/../../sanitize.cmake)
