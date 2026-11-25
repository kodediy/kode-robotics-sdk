robotics_sys
============

The extended layer: the calls an Active Object of the app's own makes, all in robotics_sys.h. No call waits: a command posts and returns, a request is answered once with a signal, and the module's news arrives as signals.

Depends on: robotics_aos, robotics_bsp, and kode_sys, kqp, kdefs and klog from kode-sdk

Docs: https://docs.kode.diy/modules/robotics
