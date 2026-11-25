# The extended layer's suite: components/robotics_sys/src/robotics_sys.c on kode-sdk's own
# QP/C (its pools, its queues, its events), over an owner that writes down what it
# is asked and answers when a case says so. A project of its own: the owner's suites
# link the real owner, and C allows one definition per binary.
cmake_minimum_required(VERSION 3.16)

# kode-sdk at the release the apps pin, for its headers and its QP/C: the clone
# tests/host_ao keeps (one clone for both; see there for why not the component
# manager), fetched here when this suite runs first.
set(KODE_SDK_VERSION "v0.1.5")
set(KODE_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../host_ao/kode-sdk-${KODE_SDK_VERSION}")
if(NOT EXISTS "${KODE_SDK_DIR}/components/kode_sys/include/kode_sys.h")
    file(REMOVE_RECURSE "${KODE_SDK_DIR}")
    execute_process(
        COMMAND git clone --quiet --depth 1 --branch "${KODE_SDK_VERSION}"
                "https://github.com/kodediy/kode-sdk.git" "${KODE_SDK_DIR}"
        RESULT_VARIABLE fetched)
    if(NOT fetched EQUAL 0)
        message(FATAL_ERROR "kode-sdk ${KODE_SDK_VERSION} could not be fetched")
    endif()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# QP/C as kode-sdk's own tests/host takes it: its component, as it ships.
set(EXTRA_COMPONENT_DIRS "${KODE_SDK_DIR}/components/qpc")

# Before project(), as in tests/host: only main and what it requires.
set(COMPONENTS main)

project(robotics_sys_tests)
