# robotics_sys - the extended layer: what an Active Object includes (robotics_sys.h).
#
# Commands, requests with their answers, and the news, none of them waiting.
# It sits on the owner (robotics_aos) as robotics does, and shares robotics_types.h
# with it (robotics_bsp). It is the only component besides the owner that includes
# kode_sys.h (tools/check_layers.sh, rule 13).
idf_component_register(
    SRCS "src/robotics_sys.c"
    INCLUDE_DIRS "include"
    REQUIRES robotics_bsp kode_sys kqp
    PRIV_REQUIRES kdefs klog robotics_aos
)
