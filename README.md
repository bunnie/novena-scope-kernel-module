Novena Scope Drivers
====================

Kernel module for FPGA oscilloscope driver.  This allows for relatively
high-speed reading from the FPGA over EIM, as well as providing automatic
setup for the FPGA firmware and IO pinmux.


Directories
===========

/ - Kernel drivers are located in the root directory.  Simply type "make".

/userspace - Userspace hacking tools, useful in developing the driver.

/firmware - Firmware files.  Place in /lib/firmware/
