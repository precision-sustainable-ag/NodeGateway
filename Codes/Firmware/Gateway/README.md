# Gateway

Hardware and firmware for the gateway portion of the node-gateway system.  The gateway is the system master and is responsible for 
synchronizing logged data from the node units (through LoRa radio), storing that data on a local SD card, and pushing that data to 
a server via modem.

**Note**: Upload and run FONA3G_setbaudAIT.ino.hex first on new units. This sketch sets the cellular modem's baud rate to 4800 and 
only needs to be run once. The microcontroller and modem will not communicate correctly if the hardware baud rate does not match 
what is set in the firmware. 
