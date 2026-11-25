idf_component_register(
    SRCS "servos_main.c" "servos_ao.c"
    INCLUDE_DIRS "."
    REQUIRES kode robotics_sys
    PRIV_REQUIRES kdrv
)
