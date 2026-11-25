# The suite is one component: the runner, the cases, and the
# two sources under test - robotics_ao.c, #included by its case file as
# tests/host_owner's test_owner.c includes robotics_aos.c, and kode-sdk's
# kqp_worker.c, the worker the owner hands its waiting work to. What is faked
# is below them: khal's worker task, khal's delay, khal's task_self and
# klog's kode_log() (fakes/).
set(SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../../..")
set(KC "${KODE_SDK_DIR}/components")   # ../CMakeLists.txt fetches it

# kode_sys.h is the extended layer's header, and it reaches kqp.h, kode_init.h
# and the rest of what an Active Object of an app uses. None of them includes
# an ESP-IDF header, so they all compile for `linux`.
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
         "${KC}/kqp/src/kqp_worker.c"
         "${KC}/kqp/src/kqp_self.c"
         "${CMAKE_CURRENT_LIST_DIR}/fakes/fake_kode.c"
    INCLUDE_DIRS "."
                 "${CMAKE_CURRENT_LIST_DIR}/fakes"
                 "${SDK_DIR}/components/robotics_aos/robotics/src"
                 "${SDK_DIR}/components/robotics_aos/core/include"
                 "${SDK_DIR}/components/robotics_bsp/board/include"
                 ${KODE_INCLUDES}
    REQUIRES unity qpc freertos
    WHOLE_ARCHIVE
)

# The image the owner is built for has low power: the rounds and the states are
# signals only then (kqp_signals.h), and the owner subscribes to them.
target_compile_definitions(${COMPONENT_LIB} PRIVATE CONFIG_KODE_LOWPOWER=1)

# Sanitizers, only when ROBOTICS_SANITIZE is set; see tests/sanitize.cmake.
include(${CMAKE_CURRENT_LIST_DIR}/../../sanitize.cmake)
