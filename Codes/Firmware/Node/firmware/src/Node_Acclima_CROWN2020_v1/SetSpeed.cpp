#include "SetSpeed.h"
#include "Arduino.h"

SetSpeed::SetSpeed() {
  cli();  //disable interrupts
  CLKPR = 0x80;
  CLKPR = 0x1;
  sei();  //enable interrupts
}

SetSpeed::~SetSpeed() {
  //nothing needed here
}
