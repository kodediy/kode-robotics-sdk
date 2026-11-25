idf_component_register(
    SRCS "outputs_main.c" "outputs_ao.c"
    INCLUDE_DIRS "."
    REQUIRES kode robotics_sys
    PRIV_REQUIRES kdrv
)
