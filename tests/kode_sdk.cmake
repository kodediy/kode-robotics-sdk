# kode-sdk at the release the apps pin, cloned once into tests/host_ao (.gitignore)
# for every host suite that compiles against it. Not through the component
# manager: its components declare esp32p4 only, and the manager
# refuses them for `linux`; kode-sdk's own host suites take its headers from
# the tree the same way. Delete the clone to fetch it again.
#
# Sets KODE_SDK_VERSION and KODE_SDK_DIR.
set(KODE_SDK_VERSION "v0.1.5")
set(KODE_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/host_ao/kode-sdk-${KODE_SDK_VERSION}")
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
