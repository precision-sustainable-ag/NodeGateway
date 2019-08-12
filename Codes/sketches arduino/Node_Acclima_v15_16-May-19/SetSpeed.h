/* 
 *  SetSpeed.h
 *  
 *  27-Mar-2019
 *  
 *  For Atmel ATmega1284p MCU with an external oscillator at 16 MHz and VCC set to 3.3 volts:
 *  MCU is not guaranteed to be stable at 16MHz and 3.3 volts...
 *  
 *  Created to implement a prescaler for the MCU -- must happen before Ardiuno setup() function because
 *  other class initialization may rely on proper timing.  In other words, this should be the first object
 *  created!
 *  
 *  At a later date we can modify the bootloader so that it boots at the correct frequency OR more likely we
 *  can run the MCU with an 8 MHz crystal instead of a 16 MHz crystal and the prescaler won't be needed.  At 
 *  that point, this code will no longer be needed.
 *  
 *  NOTE: The F_CPU flag in the boards.txt file MUST BE CHANGED for the proper board to 8000000L or the Arduino 
 *  libraries and your source code will not be "tuned" properly and will likely cause garbled UART / incorrect 
 *  timing.
 *  Path to my boards.txt: "C:\Users\<User Name>\AppData\Local\Arduino15\packages\Moteino\hardware\avr\1.4.0\"
 *  
 *  John Anderson, Acclima Inc.
 *  
 */


class SetSpeed {
  public:
    SetSpeed();
    ~SetSpeed();
};
