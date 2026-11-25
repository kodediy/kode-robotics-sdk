# The runner, the cases (test_sys.c builds robotics_sys.c into itself, as tests/host_ao
# does robotics_ao.c: a case reaches its doors and its subscribers) and the owner the
# extended layer is given. The owner is faked (fakes/), but the rules it gates a call with
# (robotics_check.c) and the words for a reason (robotics_why.c) are the real
# ones: a refusal that happens here is the one an Active Object sees. QP/C is
# kode-sdk's own: the pools, the queues and the events the layer posts are real.
set(SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../../..")
set(KC "${KODE_SDK_DIR}/components")   # ../CMakeLists.txt fetches it

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
         "${SDK_DIR}/components/robotics_aos/robotics/src/robotics_check.c"
         "${SDK_DIR}/components/robotics_aos/robotics/src/robotics_why.c"
         "${CMAKE_CURRENT_LIST_DIR}/fakes/fake_owner.c"
    INCLUDE_DIRS "."
                 "${CMAKE_CURRENT_LIST_DIR}/fakes"
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
