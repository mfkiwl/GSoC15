file name: RF_channels.cc

-------------------------------------------------------------------------
This program locates all the satellites that could be tuned in by 
the receiver.

It sets up the logging system, creates a ControlThread object,
makes it run, and releases memory back when the main thread has ended.

The gathered information can be used for auto-configuration of receiver.

It also finds the total number of RF Channels.

-------------------------------------------------------------------------
