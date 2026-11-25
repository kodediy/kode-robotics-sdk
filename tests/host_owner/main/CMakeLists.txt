# The suite is one component: the runner, the cases, and the real SDK
# sources under test (the plain C the owner stands on, and the standard layer, so a
# case can call robotics.h over the real owner; robotics_aos.c itself
# is #included by its case file, so a case reaches its statics). Everything
# else is a fake in fakes/: the drivers and kode-sdk (fake_module.c) and the owner's
# Active Object (fake_robotics_ao.c, at robotics_ao.h). tests/host_drv is where
# the drivers are tested for real, and tests/host_ao where robotics_ao.c is.
#
# The cases are globbed and the sources under test are listed, on purpose:
# adding a case is adding a file, but pulling a production source into the
# host build is a decision about what it drags in, and a decision should cost
# a line somebody reads.
set(SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../../..")

set(SOURCES_UNDER_TEST
    "${SDK_DIR}/components/robotics_drv/at8833/src/robotics_drv_at8833.c"
    "${SDK_DIR}/components/robotics_aos/robotics/src/robotics_plan.c"
    "${SDK_DIR}/components/robotics_aos/robotics/src/robotics_check.c"
    "${SDK_DIR}/components/robotics_aos/robotics/src/robotics_why.c"
    "${SDK_DIR}/components/robotics_aos/robotics/src/robotics_events.c"
    "${SDK_DIR}/components/robotics/src/robotics.c"
)

# Sorted, so every machine and CI run the cases in the same order: a case
# registers itself from a constructor, in the order the linker meets it.
file(GLOB CASE_SOURCES "${CMAKE_CURRENT_LIST_DIR}/../cases/test_*.c")
list(SORT CASE_SOURCES)

# WHOLE_ARCHIVE: a case file that nothing references would otherwise be
# dropped from the link, and Unity would report OK over what survived.
# kode_status_t: kdefs' one header, from the kode-sdk clone of ../../kode_sdk.cmake
# (kdefs declares esp32p4 only, and the component manager refuses it
# for `linux`).
include(${CMAKE_CURRENT_LIST_DIR}/../../kode_sdk.cmake)

idf_component_register(
    SRCS "runner.c"
         ${CASE_SOURCES}
         ${SOURCES_UNDER_TEST}
         "${CMAKE_CURRENT_LIST_DIR}/fakes/fake_module.c"
         "${CMAKE_CURRENT_LIST_DIR}/fakes/fake_robotics_ao.c"
    INCLUDE_DIRS "${KODE_SDK_DIR}/components/kdefs/include"
                 "."
                 "${CMAKE_CURRENT_LIST_DIR}/fakes"
                 "${SDK_DIR}/components/robotics_drv/at8833/include"
                 "${SDK_DIR}/components/robotics_drv/bridge/include"
                 "${SDK_DIR}/components/robotics_drv/i2c_scan/include"
                 "${SDK_DIR}/components/robotics_drv/pins/include"
                 "${SDK_DIR}/components/robotics_drv/servo_pwm/include"
                 "${SDK_DIR}/components/robotics_drv/stepper/include"
                 "${SDK_DIR}/components/robotics/include"
                 "${SDK_DIR}/components/robotics_aos/core/include"
                 "${SDK_DIR}/components/robotics_aos/robotics/src"
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
