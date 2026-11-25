# robotics - the standard layer: what a task includes (robotics.h) and nothing else.
#
# It names no kode-sdk type and returns no kode_status_t (tools/check_layers.sh,
# rule 11): the owner's component is private, so no app ever sees its types
# (rule 6). robotics_types.h, which both layers share, is in robotics_bsp.
idf_component_register(
    SRCS "src/robotics.c"
    INCLUDE_DIRS "include"
    REQUIRES robotics_bsp
    PRIV_REQUIRES kdefs robotics_aos
)
