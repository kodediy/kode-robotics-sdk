# The runner, the cases, the example's Active Object and rows under test, the
# extended layer they call (robotics_sys.c, real), the owner it is given
# (tests/host_sys's fake: it writes down what it is asked) and kodeOS's console
# registry with its rules. The rules the owner gates a call with (robotics_check.c)
# and the words for a reason (robotics_why.c) are the real ones. QP/C is kode-sdk's
# own; the Active Object is started without its task, as tests/host_ao does.
set(SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../../..")
set(KC "${KODE_SDK_DIR}/components")   # ../CMakeLists.txt fetches it
set(SYS_FAKES "${CMAKE_CURRENT_LIST_DIR}/../../host_sys/main/fakes")

# kode_sys.h reaches kqp.h, kode_init.h and the rest of what an Active Object of an
# app uses. None of them includes an ESP-IDF header, so they all compile for `linux`.
file(GLOB KODE_INCLUDES LIST_DIRECTORIES true
     "${KC}/kdefs/include"
     "${KC}/kode/include"
     "${KC}/kode_sys/include"
     "${KC}/kqp/include"
     "${KC}/klog/include"
     "${KC}/kaos/*/include"
     "${KC}/kdrv/*/include"
     "${KC}/khal/*/include"
     "${KC}/kbsp/*/include")

file(GLOB CASE_SOURCES "${CMAKE_CURRENT_LIST_DIR}/../cases/test_*.c")
list(SORT CASE_SOURCES)

# WHOLE_ARCHIVE: a case file that nothing references would otherwise be
# dropped from the link, and Unity would report OK over what survived.
idf_component_register(
    SRCS "runner.c"
         ${CASE_SOURCES}
         "${SDK_DIR}/components/robotics_sys/src/robotics_sys.c"
         "${SDK_DIR}/examples/console_actions/main/console_actions_ao.c"
         "${SDK_DIR}/components/robotics_aos/robotics/src/robotics_check.c"
         "${SDK_DIR}/components/robotics_aos/robotics/src/robotics_why.c"
         "${SYS_FAKES}/fake_owner.c"
         "${CMAKE_CURRENT_LIST_DIR}/fakes/fake_console.c"
    INCLUDE_DIRS "."
                 "${CMAKE_CURRENT_LIST_DIR}/fakes"
                 "${SYS_FAKES}"
                 "${SDK_DIR}/examples/console_actions/main"
                 "${SDK_DIR}/components/robotics_sys/include"
                 "${SDK_DIR}/components/robotics_sys/src"
                 "${SDK_DIR}/components/robotics_aos/core/include"
                 "${SDK_DIR}/components/robotics_bsp/board/include"
                 ${KODE_INCLUDES}
    REQUIRES unity qpc freertos
    WHOLE_ARCHIVE
)

# Sanitizers, only when ROBOTICS_SANITIZE is set; see tests/sanitize.cmake.
include(${CMAKE_CURRENT_LIST_DIR}/../../sanitize.cmake)

# The example starts its Active Object with QACTIVE_START(), which makes a task.
# Here it is test_QActive_start() (cases/test_console.c): the queue, the level and
# the initial transition, and no task, as tests/host_ao does the owner.
target_compile_definitions(${COMPONENT_LIB} PRIVATE QActive_start=test_QActive_start)
