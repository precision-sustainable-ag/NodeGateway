#include "SIMCOM_Lite.h"
#include <stdlib.h>

#define BaudRatesChecked    6
const uint32_t rates[BaudRatesChecked] = {57600, 115200, 4800, 38400, 19200, 9600}; // check for baud rates in this order.  First index is default.


SimCom::SimCom(HardwareSerial& myPort) {
  hwport = &myPort;
  port = &myPort;
  if ((hwport) && (port)) status = SIMCOM_STATUS_CONSTR;
  else status = 0;
}


bool SimCom::powerOff(bool waitPwrDown) {
  //first ensure power is on so that we don't turn it on accidentally by trying to turn it off  
  if (!digitalRead(pin_SIMCOM_PWR_STATUS))
  return true;   //already off -- exit function without doing anything
  //otherwise, toggle pin as spec'd in SIMCOM SIM5320A hardware document  
  pinMode(pin_SIMCOM_PWR_KEY, OUTPUT);      //Should have already been like this but just to make sure...
  digitalWrite(pin_SIMCOM_PWR_KEY, HIGH);
  delay(180);                               //Wait 180 ms HIGH
  digitalWrite(pin_SIMCOM_PWR_KEY, LOW);    
  delay(600);                               //Wait at least 500 ms LOW
  digitalWrite(pin_SIMCOM_PWR_KEY, HIGH);   //Leave HIGH
  status &= ~(SIMCOM_STATUS_STARTED | SIMCOM_STATUS_GPRS | SIMCOM_STATUS_HTTPS | SIMCOM_STATUS_POSTED);
  //now wait for power down... maximum time specified by SIMCOM_PWRDWN_DELAY but check periodically because it may be faster
  uint32_t waittimer = millis();  
  while ((millis() - waittimer) < SIMCOM_PWRDWN_DELAY) {
    if (!digitalRead(pin_SIMCOM_PWR_STATUS)) return true;
  }
  return false;
}

bool SimCom::powerOn(bool waitPwrUp) {
  //First check to see if radio is already on
  // DBGLN(DBG_INFO, "INFO: Power On Cell modem");
  if (digitalRead(pin_SIMCOM_PWR_STATUS))
  return true; //already running, no need to toggle power key pin to turn on
  //otherwise, toggle pin as spec'd in SIMCOM SIM5320A hardware document
  pinMode(pin_SIMCOM_PWR_KEY, OUTPUT);      //Should have already been like this but just to make sure...
  digitalWrite(pin_SIMCOM_PWR_KEY, HIGH);
  delay(180);                               //Wait 180 ms HIGH
  digitalWrite(pin_SIMCOM_PWR_KEY, LOW);    
  delay(180);                               //Wait 180 ms LOW -- pulling LOW toggles power of radio module
  digitalWrite(pin_SIMCOM_PWR_KEY, HIGH);   //Leave HIGH
  //now wait for boot sequence... maximum time specified by SIMCOM_BOOT_DELAY but check periodically because it may boot faster
  if (waitPwrUp) {
    uint32_t waittimer = millis();
    bool started = false;
    while (((millis() - waittimer) < SIMCOM_BOOT_DELAY) && (!started)) {
      if (digitalRead(pin_SIMCOM_PWR_STATUS)) started = true;
    }
    if (started) {
      // Now wait for the boot trash on the serial port.  Silent for 1 second after data, or no data for 5 seconds.
      // assumes port is already open.
      uint32_t waittimer = millis();
      bool gotsomething = false;
      while (1) {
        while (port->available()) {
          port->read();
          waittimer = millis();
          gotsomething = true;
        }
        if (gotsomething) {
          if ((millis() - waittimer) > 1000) return true;
        }
        else if ((millis() - waittimer) > 5000) return true;
      }
    }    
    return false;
  }
  return true;
}

void SimCom::hardReset() {
  pinMode(pin_SIMCOM_RESET, OUTPUT);
  digitalWrite(pin_SIMCOM_RESET, HIGH);
  delay(100);
  digitalWrite(pin_SIMCOM_RESET, LOW);
  delay(100);
  digitalWrite(pin_SIMCOM_RESET, HIGH); 
  status &= ~(SIMCOM_STATUS_STARTED | SIMCOM_STATUS_GPRS | SIMCOM_STATUS_HTTPS | SIMCOM_STATUS_POSTED);
}



bool SimCom::begin(uint32_t Baudrate = 57600) {
  if ((status & SIMCOM_STATUS_CONSTR) == 0) return false;  
  hwport->begin(Baudrate);
  status |= SIMCOM_STATUS_UART;
  for (int StartAttempts = 0; StartAttempts < 3; StartAttempts++) { // try three times to start up the modem
    if (powerOn(true)) {
      uint16_t N = RecvAT(shortbuf, SIMCOM_SHORT_BUF_LEN, 2500, 1); // try to get something from the UART (startup trash)
      while (N) {
        N = RecvAT(shortbuf, SIMCOM_SHORT_BUF_LEN, 500, 1);     // make sure we get it all -- keep receiving until the modem goes quiet
      }
      uint32_t NewBaud = AutoBaud();                  // AutoBaud - just in case.  This will also verify the port is working.
      if (NewBaud > 0) {
        // The modem is talking via UART

        // Serial.print("Connected at baudrate ");
        // Serial.println(NewBaud);
        
        if (NewBaud != Baudrate) {
          // Serial.print("Changing baud to ");
          // Serial.println(Baudrate);
          if (!ChangeBaud(Baudrate)) {  // Need to change the baud rate
            ChangeBaud(NewBaud);        // Error, wont change!
            Serial.println("Error! Unsuccessful at changing baudrate of modem!");
          }
        }
        status |= SIMCOM_STATUS_STARTED;        
        return true;
      }
      if (!powerOff(true)) hardReset();               // The port is not responding.  Let's Power down and start over.
    }
    // if we get here - the modem did not respond.  Try again.
    if (StartAttempts > 1) hardReset();
    delay(500);
  }  
  return false;   // FAILURE!
}

bool SimCom::end() {
  if ((status & SIMCOM_STATUS_CONSTR) == 0) return false; // not initialized correctly  
  if ((status & SIMCOM_STATUS_STARTED) == 0) return true; // Not powered on yet.  Already in END state
  powerOff(true);
}


void SimCom::SendAT(char *buf) {    // send a null-terminated AT comamnd string.  Opening AT and closing \n should not be included.
  if ((status & SIMCOM_STATUS_UART) == 0) {
    return;
  }
  while (port->available()) port->read(); // Clear any existing data in buffer
  
  port->print("AT");
  port->println(buf);
  port->flush();
}


bool SimCom::GetOK(uint32_t timeout) {
  uint32_t timer = millis();

  char* okay = "OK\r\n";
  char* error = "ERROR\r\n";   // ERROR just used to return faster than timeout parameter if detected

  int8_t idx; // use this index for both ERROR and OK above (negative denotes ERROR, positive OK)

  while (millis() - timer < timeout) {
    if (port->available()) {
      char c = port->read();

      if (idx < 0) {
        // Matching with ERROR
        if (error[-idx] == c) {
          idx--;  // increment progress
          if (-idx == 8) {            
            return false; // Just matched the whole "ERROR\r\n" string
          }
        } else {          
          idx = 0;  // No match.  Reset progress!
        }
      } else if (idx > 0) {
        // Matching with OK
        if (okay[idx] == c) {
          idx++;  // increment progress
          if (idx == 4) {            
            return true; // Just matched the whole "OK\r\n" string
          }
        } else {
          idx = 0;  // No match.  Reset progress!
        }
      } else {
        // Nothing matched yet
        if (c == okay[0]) {
          idx++;
        } else if (c == error[0]) {
          idx--;
        }
      }
    }
  }
  return false; // If we get here then timeout happened.
}



bool SimCom::SendATGetOK(char *buf, uint32_t timeout) { // send a common AT command that expects a simple OK response. No RX buffers are used.
  if ((status & SIMCOM_STATUS_UART) == 0) return false;
  SendAT(buf);
  return GetOK(timeout);
}


uint16_t SimCom::RecvAT(char *buf, uint16_t maxlen, uint32_t timeout, uint8_t NumLF) { // receive the response to an AT command
  if ((status & SIMCOM_STATUS_UART) == 0) return 0;
  if (NumLF == 0) NumLF = 1;
  uint8_t LFCount = 0;
  uint16_t recvlen = 0;
  uint32_t RxATimer = millis();  
  while ((LFCount < NumLF) && ((millis() - RxATimer) < timeout) && (recvlen < maxlen)) {
    if (port->available()) {
      buf[recvlen] = port->read();
      if (buf[recvlen] == _LF) LFCount++;
      recvlen++;
    }
  }
  buf[recvlen] = 0; // add a null terminator
  return recvlen;
}


// Find the Baud Rate of the SimCom5320.  My Baud rate will be set to this on exit, or default to 57600 if unable to auto-baud.
// Baud rate order is set by rate of expected occurance:  (minimize attempts)
uint32_t SimCom::AutoBaud() {
  if ((status & SIMCOM_STATUS_CONSTR) == 0) return 0;
  if ((hwport == 0) || (port == 0)) return 0;
  status |= SIMCOM_STATUS_UART;           // we are going to configure the UART - one way or another.
  for (uint8_t i = 0; i < BaudRatesChecked; i++) {    
    hwport->begin(rates[i]);
    delay(10);
    port->setTimeout(1000);
    for (int j = 0; j < 5; j++) {
      if (SendATGetOK("V1", 1000)) {  // Set verbose mode communicationse        
        return rates[i];  // send just AT\n
      }
    }  
  }
  hwport->begin(57600);
  return 57600;      // 115200 is flaky.  this will need to be changed.
}

// Temporarily and permanently change the baud rate on the SimCom modem
bool SimCom::ChangeBaud(uint32_t BaudRate) {  
  sprintf(shortbuf, "+IPREX=%u", BaudRate);
  SendATGetOK(shortbuf, 500);
  hwport->begin(BaudRate);
  delay(10);
  for (int i = 0; i < 4; i++) {
    if (SendATGetOK("V1", 300)) {      
      return true;
    }
  }
  return false;
}


uint8_t SimCom::getSignalQuality() {
  if ((status & SIMCOM_STATUS_STARTED) && (status & SIMCOM_STATUS_UART) && (status & SIMCOM_STATUS_CONSTR)) {
    SendAT("+CSQ");
    uint16_t len = RecvAT(shortbuf, SIMCOM_SHORT_BUF_LEN, 2500, 2);

    // Code to ignore the command echo:
    uint16_t idx = 0;
    while (idx < len) {
      if (shortbuf[idx] == _LF) {
        idx++;
        break;
      }
      idx++;
    }

    if (idx > 0) {
      char* sigQualText = "+CSQ: ";
      uint8_t idx2 = 0;
      while ((sigQualText[idx2] == shortbuf[idx + idx2]) && (idx2 < 5))  {
        idx2++;
      }

      if (idx2 == 5) {
        // Leading text just verified
        uint8_t qualNum = (uint8_t)strtol(shortbuf + idx + idx2, nullptr, 10);
        return qualNum;
      } else {
        // Leading text did not match!
        return 0;
      }
    }
  } else 
    return 0;
}
