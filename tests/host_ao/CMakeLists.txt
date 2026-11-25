# The owner's Active Object (components/robotics_aos/robotics/src/robotics_ao.c) on kode-sdk's
# own QP/C and its own worker, built for ESP-IDF's `linux` target.
cmake_minimum_required(VERSION 3.16)

# kode-sdk at the release the apps pin, for its headers, its QP/C and
# kqp_worker.c. Not through the component manager: that fetches every layer
# kodediy/kode depends on and configures them all, and some of what they
# need (esp_eth, for the C5's link) does not exist for `linux`. One shallow
# clone instead, kept beside this file (.gitignore); delete it to fetch again.
set(KODE_SDK_VERSION "v0.1.5")
set(KODE_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/kode-sdk-${KODE_SDK_VERSION}")
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

project(robotics_ao_tests)
