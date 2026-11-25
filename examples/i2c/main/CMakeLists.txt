# esp_driver_i2c for the app's own bus on the pair (i2c_ao.c, probe()).
idf_component_register(
    SRCS "i2c_main.c" "i2c_ao.c"
    INCLUDE_DIRS "."
    REQUIRES kode robotics_sys esp_driver_i2c
)
