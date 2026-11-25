# The runner, the cases, the standard layer under test and the owner it is
# given. The same shape as tests/host/main: see there for why the cases are
# globbed, sorted and linked whole. The owner is faked (fakes/), but the rules
# it gates a call with are the real ones (robotics_check.c, robotics_why.c): a
# refusal that happens here is the one the app sees.
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
         "${SDK_DIR}/components/robotics/src/robotics.c"
         "${SDK_DIR}/components/robotics_aos/robotics/src/robotics_check.c"
         "${SDK_DIR}/components/robotics_aos/robotics/src/robotics_why.c"
         "${CMAKE_CURRENT_LIST_DIR}/fakes/fake_robotics_aos.c"
    INCLUDE_DIRS "${KODE_SDK_DIR}/components/kdefs/include"
                 "."
                 "${CMAKE_CURRENT_LIST_DIR}/fakes"
                 "${SDK_DIR}/components/robotics/include"
                 "${SDK_DIR}/components/robotics_aos/core/include"
                 "${SDK_DIR}/components/robotics_aos/robotics/src"
                 "${SDK_DIR}/components/robotics_bsp/board/include"
    REQUIRES unity
    WHOLE_ARCHIVE
)

# Sanitizers, only when ROBOTICS_SANITIZE is set; see tests/sanitize.cmake.
include(${CMAKE_CURRENT_LIST_DIR}/../../sanitize.cmake)
