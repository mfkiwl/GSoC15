file name: Auto_rx_conf.cc

---------------------------------------------------------------------------------------------------------------------------------------

This file locates all the satellites that could be tuned in by the receiver.

It sets up the logging system, creates a ControlThread object,makes it run, and releases memory back when the main thread has ended.

The gathered information is used for auto-configuration of receiver.

The Sample Rate and Sample Resolutions are also calculated.

It also finds the bandwidth and center frequency of the signal.

This program also separates out different metadata formats into .xml files.

---------------------------------------------------------------------------------------------------------------------------------------
