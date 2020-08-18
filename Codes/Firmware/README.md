# Node-Gateway - CROWN-V2-AIT
-----------
**NOTE: Users must edit HardwareSerial.h to use configuration strings longer than 64 characters**

HardwareSerial.h is in a path similar to: C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\HardwareSerial.h

Change:
#define SERIAL_TX_BUFFER_SIZE 64 to #define SERIAL_TX_BUFFER_SIZE 256

and

#define SERIAL_RX_BUFFER_SIZE 64 to #define SERIAL_RX_BUFFER_SIZE 256

>> To give yourself permission to edit the file, right click on the filename, go to **Properties > Security > Edit...** and give Users "Full control" of the file. You may need Administrator privileges too.

-----------
The hardware and binary source files have not been changed from the original CROWN branch. Only the .ino files have been edited.

This branch is for me to edit the sketches without yet merging them into a final "V2" branch. 

17-Jan-2020: Gateway and Node sketches use ACReliableMessage.h for sending radio messages. This should fix the problem with sending/receiving data from the CC Nodes.


