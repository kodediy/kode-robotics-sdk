# The suite is one component: the runner, the cases, and the real plain-C
# sources under test.
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
    INCLUDE_DIRS "${KODE_SDK_DIR}/components/kdefs/include"
                 "."
                 "${SDK_DIR}/components/robotics_drv/at8833/include"
                 "${SDK_DIR}/components/robotics_drv/bridge/include"
                 "${SDK_DIR}/components/robotics_drv/servo_pwm/include"
                 "${SDK_DIR}/components/robotics_aos/core/include"
                 "${SDK_DIR}/components/robotics_aos/robotics/src"
                 "${SDK_DIR}/components/robotics_bsp/board/include"
    REQUIRES unity
    WHOLE_ARCHIVE
)

# The C maths library: sqrt(), pow() and the rest live in libm, which GCC on Linux
# (the CI) does not link by itself. A host compiler may, so a missing line
# here only shows in the CI.
target_link_libraries(${COMPONENT_LIB} PRIVATE m)

# Sanitizers, only when ROBOTICS_SANITIZE is set; see tests/sanitize.cmake.
include(${CMAKE_CURRENT_LIST_DIR}/../../sanitize.cmake)
