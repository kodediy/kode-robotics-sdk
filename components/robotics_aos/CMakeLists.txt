# robotics_aos - the owner: one Active Object that finds the module, holds its
# positions and decides when everything happens. robotics_ao.c is the Active
# Object, on kode-sdk's QP/C (kode_sys.h, kode_worker.h); robotics_aos.c is what its signals do.
#
# Laid out as kaos is: what the two layers include in core/include, and each AO
# in a folder of its own with its sources and private headers in src/. There
# is one AO here, AO_Robotics, so there is one folder.
#
# robotics_plan.c, robotics_check.c and robotics_why.c are plain C on purpose - no ESP-IDF, no FreeRTOS, no kode-sdk -
# because it is every rule of the module's behaviour and tests/host runs it
# as it is.
idf_component_register(
    SRCS "robotics/src/robotics_aos.c"
         "robotics/src/robotics_ao.c"
         "robotics/src/robotics_plan.c"
         "robotics/src/robotics_check.c"
         "robotics/src/robotics_why.c"
         "robotics/src/robotics_events.c"
    INCLUDE_DIRS "core/include"
    PRIV_INCLUDE_DIRS "robotics/src"
    REQUIRES kdefs robotics_bsp
    PRIV_REQUIRES kode kode_sys freertos robotics_drv
)
