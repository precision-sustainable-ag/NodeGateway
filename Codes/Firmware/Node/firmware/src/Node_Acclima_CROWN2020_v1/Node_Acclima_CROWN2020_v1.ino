/*PSA Node for use with Cellular Gateway

    Main Components:
      - ATMega1284P with MoteinoMEGA core
      - 1 SDI-12 port (D10)
      - DS3231 Precision RTC
      - LoRa radio transceiver

    Functions:
      - Wake at user-specified intervals to take measurements from sensors
      - Save data to Flash chip and send data to Gateway via LoRa radio

    Program Summary:
      - Find active SDI-12 addresses (a-z, A-Z, 0-9)
      - Menu
      - Low-power sleep
      - If time is at measurement interval:
        - DS3231 sends interrupt to wake up node
        - Take and store measurements to Flash
        - Send data to Gateway via LoRa
        - Clear data buffers, set new alarm time, then sleep

   Written by:
   Alondra Thompson, USDA-ARS Sustainable Agricultural Systems Lab
   Justin Ayres, Univeristy of Maryland Computer Science Department
   John Anderson, Acclima Inc.
   David Anderson, Acclima Inc.

   Last edited: August 31, 2021

   - Version History -
   Version 2020.05.06 fixes issue with data string when a sensor is unresponsive
   Version 2020.05.08 changes timeToWait variable in listenRespond() to prevent Node from timing out of
                      initial sync mode before Gateway does
   Version 2020.05.14 fine tunes initial synchronization with Gateway and daily sync check
   Version 2020.05.19 fixes problem with resetting alarm after daily sync check
   Version 2020.05.28 removes #ifdef CONT_MEAS statements, fixes issue with repeated sensor measurements after using
                      "t" menu option, removes changing depths array during sensor scan to avoid pairing addresses with
                      wrong depths
   Version 2020.06.04 fixes -999 for all sensor depths (uncommented recalling depths array from memory)
                      fix menu option "d" to include sensor addresses in depths array
                      Add travel time to TDR sensor output
   Version 2020.06.22 limits loss of data when measurement int = 15 min, and N gets garbled timestamp at measurement interval
                      following daily handshake with G (doesn't fix the garbled timestamp)
                      adds if(nYear - 2000 > 0) in listenRespond()
   Version 2020.07.28 Adds radio frequency to menu header
   Version 2020.08.04 Removes if(buf[0] == GatewayID) in fieldSync --> rejects transmissions if GatewayID = 50 b/c timestamp starts with char "2" = dec 50
   Version 2020.09.24 Minor edits to decodeConfig when delimiting sensor addresses and depths
   Version 2020.10.29 Adds solar current and voltage calcs compatible with new hardware
   Version 2021.01.08 Add 10 minute interval option if RH sensor not connected
   Version 2021.01.25 Remove numMissed loop
   Version 2021.02.03 Add delays in concatenating sensor data
   Version 2021.03.09 Reset both alarms if battV < 3.4
   Version 2021.04.14 Add one minute to listening if batt < 3.4 or numMissed == 3
   Version 2021.06.25 Use struct to save sensor metadata (addr, ID, type, meas command, depth), remove saving sensor IDs to Flash
                      Fix menu option for changing sensor addresses
                      Add option to print last 10 logs to print data menu option
                      In config string: 0 for gateway ID = no gateway, fix saving depths
                      Move numMissed loop to listenRespond
                      Fix extra ~ in data string if last sensor not a TDR
                      Initialize depth array in struct with zeroes
                      Require +/- for depths in config string
                      Only execute decodeConfig if user enters string
   Version 2021.06.28 Make depth -999 if sensor address in config string doesn't match what's found in scan
   Version 2021.07.07 Fixes issue with travel time when sensor address is a number
   Version 2021.07.15 Add number of outputs to sensor struct
   Version 2021.08.03 Add print debug statements option
                      Add yesNo()
                      Add lowInitBatt flag, skips initial fieldSync if battV is too low and come back to it later
                      Edit radio timing parameters  
   Version 2021.08.31 Add option to only print newest logs since last print     
                         
*/

//===================================================================================================

//------------ Libraries --------------------------------------

#include "AcclimaSDI12.h"
#include <SPI.h>                                      // SPI functions for Flash chip
#include <Wire.h>                                     // I2C functions for RTC
#include <EEPROM.h>                                   // built-in EEPROM routines
#include <avr/sleep.h>                                // sleep functions
#include "DS3232RTC.h"                                // RTC library
#include "FlashTools.h"
#include <ACReliableMessage.h>                        // Acclima edits of RadioHead library
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>                                  // controls watchdog timer (we want to turn it off)
#include <stdio.h>
#include "GlobalDefs.h"

//------------- Assign Pins --------------------------------------

#define SDI12Data           10                       // Pin for SDI-12 sensors; handles interrupts
#define SDI12Pwr            0                        // Switches power to SDI-12 sensors on/off
#define LED                 15                       // LED on D15
#define Flash_SS            23
#define baudRate            57600                    // John, 27-Mar-2019: Changed from 115200 to 57600 because when MCU at 8 MHz the sampling resolution is reduced
#define pin_SDIv12          1                        // pin 41 --> D1  -- switch SDI-12 power supply to 12 volts (vs. 7.5)
#define pin_solarVoltage    A5
#define ADC_REF_VOLTAGE     3.3
#define ADC_RESOLUTION      1024
#define pin_solarShort      A4
#define SOLAR_CALIB         1.0                      // This will become a EEPROM constant that is set during factory config – for now just use 1.0
#define MAX_SENSORS         16
//  #define getTT                                        // if defined, queries TDRs for travel time
//  #define keepPEC                                      // if defined, keeps pore water EC in data string, if not, replaces PEC with travel time
#define ADC_MAXVALUE        1023

//------------- Declare Variables ---------------------------------

char VERSION[] = "V2021.08.31";

//-----*** Identifiers ***-----

char  projectID[6];               // 17-Mar-2020: use project ID to filter incoming data from multiple devices
uint8_t  radioID;                 // Active Node radio ID
uint8_t  GatewayID;               // Gateway radio ID
uint8_t  default_radioID;         // radio ID defaults to last 2 digits of SN
FactoryInfo facInfo;              // pull in factory info (serial num, etc) - see GlobalDefs.h

//-----*** for EEPROM ***-----

#define EEPROMSHIFT                 2800
#define EEPROM_LOW_BATT             (1 + EEPROMSHIFT)         // flag for tracking if battV < limit
#define EEPROM_GATEWAYID            (2 + EEPROMSHIFT)         // storage location for gatewayID                     (was gatewayMem)
#define EEPROM_PROJECTID            (18 + EEPROMSHIFT)        // first storage location for projectID array            (was siteIDMem)
#define EEPROM_DEFAULT_RADIO        (3 + EEPROMSHIFT)         // static default radio address (last 2 digits of serial number)
#define EEPROM_ACTIVE_RADIO         (4 + EEPROMSHIFT)         // active radio address, can be changed by user
#define EEPROM_FLAG_RADIO           (5 + EEPROMSHIFT)         // flag to use active address
#define EEPROM_ALRM1_INT            (9 + EEPROMSHIFT)         // storage location for Alarm 1 interval              (was intervalMem)
#define EEPROM_GW_PRESENT           (14 + EEPROMSHIFT)        // store "1" for yes, "0" for node                    (was gatewayPresMem)
#define EEPROM_RH_PRESENT           (16 + EEPROMSHIFT)        //                                                    (was RHPresMem)
#define EEPROM_DEPTHS               (24 + EEPROMSHIFT)        // first storage location for depths array
#define EEPROM_DEBUG                (240 + EEPROMSHIFT)       // store debug setting
#define EEPROM_LASTPRINT            (241 + EEPROMSHIFT)       // LogID of last log printed

#define EEPROM_SERIALNUM            10
#define EEPROM_OPTSRADIO            17
#define EEPROM_verHW                16  // char 
#define EEPROM_optsRadio            17  // char
#define EEPROM_iSolarRes            48
#define EEPROM_verHW                16

bool  gatewayPresent = true;            // flags if user is using a Gateway
bool  lowInitBatt = false;

//The following value is determined by reading EEPROM constant (see method getLoRaFreq())
uint16_t LoRaFREQ;

uint32_t timeSync = 7200000;            // timeout for initial sync, 14May: 2 hours
//uint32_t timeSync = 900000; // 14May for testing 900000 = 15 minutes
uint32_t fieldSyncStart;
uint32_t fieldSyncStop;

//-----*** for Flash ***-----

uint32_t logsBeginAddr = LOG_beginAddr;
uint32_t logsEndAddr = LOG_endAddr;
uint32_t WAC = 0;                   // This is our Write Access Counter -- keeps track of how many things we write to Flash
// Initialize to zero if no record found.  Increment before write.  Persistant - save to log space after erase.
// This should never be written as 0 or 0xFFFFFFFF.
const uint32_t WAC_Sync_Range = 0x7FFFFFFF;   // if the desired WAC is greater by this amount, consider uploading everything

//-----*** for Main Menu ***-----

int   indata;                       // user input data
int   incoming[7];
char  incomingChar[200];
char  charInput[200];
unsigned long serNum;               // serial number
bool skipScan = false;              // tracks if sensor scan was skipped by user
bool debug = false;

//-----*** Data Variables ***-----

//-- SDI-12 addresses and metadata

bool RHPresent = false;                        // tracks presence of RH sensor (RH sensor requires extra routines for heating)
char RHsensor;                                 // holds address of RH sensor
char Tsensor;                                  // holds address of Temp sensor
bool TPresent = false;                         // tracks presence of Temp sensor
char activeSDI12[MAX_SENSORS + 1];             // array holding active sensor addresses (up to 16 sensors)
char sensorList[MAX_SENSORS][36];   // added 4-Mar-2020: 2D array for holding sensor IDs to act as lookup table
byte sensorNum = 0;                 // added 4-Mar-2020: counter for writing to 2D array
byte rowNum = 0;
char depths[MAX_SENSORS][7];
byte sensorTot = 0;

//-- Sensor data

String allData = "";                // String for compiling metadata and sensor data
int measDelay;                      // delay for SDI-12 measurement
boolean SDI12response = false;
char sep = '~';                     // data delimiter

//-- Onboard Temperature sensor

float  boxTemp;

//-- Battery voltage measurement and calc

float multiplier = 0.00322;         // scaling factor for analogRead
float battV;                        // battery voltage
float lowBatt = 3.4;                // low battery limit
bool  battLow = false;              // tracks if battery V fell below limit

//-----*** for Loop ***-----

bool duringInit = true;
bool dataSent = false;
bool updated = false;
bool userinput = false;

//-----*** for RTC ***-----

//-- Time and Date values

byte   secs;                  // time and date values
byte   mins;
byte   hrs;
byte   days;
byte   mnths;
int    yrs;

String timestamp = "";
tmElements_t tm;

//-- Alarms

byte  interval;              // user-defined measurement interval
byte  alarmMins;
byte  NISTmin = 35;           // 25-Mar-2020: edited to 35 minutes to not collide with heaterOff(), 27-Jan-2020 wake up at 12:35 CST (18:36 UTC) for daily sync check
byte  NISThr = 18;

// for testing ////
//byte  NISTmin = 20;           // 27-Jan-2020 wake up at 12:36 CST (18:36 UTC) for daily sync check
//byte  NISThr = 18;
////

boolean timeUpdated = false;

//-----*** LoRa Radio Settings ***-----

#define     TxPower  17     // options: +5 to +17 (default 13), 21Jan21 reduce from 20 to 17
bool radioSwitch;           // tracks if using default radio address or not

/*/ ---------------------------------------------------------------------------------------------------------------------------
  // A BRIEF DISCUSSION ON TIMEOUTS: (added 01/13/2020)
  // NOTE!!  These timeout numbers assume that the radio settings are default as follows:
  //      Bandwidth = 125kHz
  //      Coding Rate = 4/5
  //      Spreading factor = 7
  // These settings favor optimizing for speed.  Longer range is possible but results in much longer message transit times.
  //      With the radio settings above, a full packet (255 bytes) air time is 400ms MINIMUM.  We will call this PacketAirTime.
  //      (With full optimization for Range, PacketAirTime is 192 seconds!)
  //      When transmitting, the default can be used.  This wait only applies to ACK reception (air time = 36ms).  For this discussion, we will call this ACKWait.
  //          The actual wait will be randomized somewhere between ACKWait and ACKWait*2 (By RadioHead Library)
  //          This timeout is set by calling RHReliableDatagram::setTimeout(), and defaults to RH_DEFAULT_TIMEOUT (which is 200 ms at the time of this documentation)
  //          The default is satisfactory.  We could try 100ms to speed things up.
  //      When receiving, there are 2 waits involved:
  //        wait 1 = StartWait:   How long should I hang around waiting to hear from someone?  This is arbitrary and situation specific.
  //                              Listening will be continuous throughout this wait interval
  //        wait 2 = PacketWait:  Once I start getting a message (but the message is not yet complete), how long do I wait before giving up?
  //                              This wait should assume a full packet, and that the sender is retrying.
  //                              This should be at minumim: (((AckWait * 2) + PacketAirTime) * NumberOfRetries)
  // ---------------------------------------------------------------------------------------------------------------------------*/
#define retryNum        3       // 3 retries should be enough, if our wait intervals are correct
#define timeoutACK      300     // This is the wait period after a transmission to receive an ACK for that one packet. (ACKWait)
#define timeoutPacket   2000    // this is the wait period to receive one large packet including retries. (PacketWait)
#define timeoutSmallPkt 1000    // Timeout interval for a small packet (i.e. 20 bytes).  Approx Airtime=timeoutAck.

byte numMissed = 0;             // counter for missed communications with Gateway

// ------- Initialize ----------------------------------------------------

RH_RF95 driverRFM95;                                 // Driver for the RFM95 radio
ACReliableMessage LoRa(driverRFM95, radioID);        // LoRa radio manager

SDI12 SDI12port(SDI12Data);
FlashTools ft;

//===================================================================================================

//------------- Set Up ------------------------------------------------------------------------------

void setup() {

  //--- Initialize ---

  // Serial

  Serial.begin(baudRate);
  delay(100);

  // SPI & I2C

  SPI.begin();
  Wire.begin();
  delay(300);

  // Flash

  if (!ft.init(pin_Flash_CS, 0xC228)) {
    Serial.println("Flash failed to initialize!");
    Serial.println("Erasing flash. This may take a few minutes...");
    ft.chipErase();
    long fiveMin = 1000 * 60 * 5;
    if (!ft.wait(fiveMin)) {
      Serial.println("Five minute timeout expired!");
      // This is an error condition!  Flash did not complete chip erase within timeout.
      // Figure out how to handle this error... maybe report it and halt?
      // Don't remember how long timeout should be but we could measure it
    }
  }

  // RTC

  RTC.begin();

  //--- Pin Settings ---

  //    pinMode(hardSS,OUTPUT);
  pinMode(pin_battV, INPUT);
  pinMode(pin_mBatt, OUTPUT);
  digitalWrite(pin_mBatt, LOW);
  pinMode(SDI12Pwr, OUTPUT);
  digitalWrite(SDI12Pwr, HIGH);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(pin_SDIv12, OUTPUT);
  digitalWrite(pin_SDIv12, HIGH);  // set SDI-12 power supply to 12V (when to set to 7.5v?)
  pinMode(pin_solarVoltage, INPUT);
  pinMode(pin_solarShort, OUTPUT);
  digitalWrite(pin_solarShort, LOW);

  //--- Power Saving

  wdt_disable();                        // turn off watchdog timer
  RTC_osc_off();                         // Turn off 32kHz output

  //--- Read variables from EEPROM ---

  EEPROM.get(EEPROM_SERIALNUM, serNum);  // get the serial number from EEPROM
  default_radioID = serNum % 100;
  EEPROM.update(EEPROM_DEFAULT_RADIO, default_radioID); // set default radio ID
  //    Serial.println(default_radioID);
  EEPROM.update(EEPROM_LOW_BATT, battLow);

  radioSwitch = EEPROM.read(EEPROM_FLAG_RADIO);

  if (radioSwitch) {
    radioID = EEPROM.read(EEPROM_ACTIVE_RADIO);                        // read board ID number from EEPROM
  } else {
    radioID = default_radioID;
  }

  interval = EEPROM.read(EEPROM_ALRM1_INT);
  GatewayID = EEPROM.read(EEPROM_GATEWAYID);
  delay(20);
  gatewayPresent = EEPROM.read(EEPROM_GW_PRESENT);
  delay(20);

  // Print device info
  char* tmpStr = (char*)malloc(100);
  getDeviceIdStr(tmpStr);
  Serial.print("\r\n");
  Serial.println(tmpStr);
  free(tmpStr);
  delay(100);

  //--- Set up Flash space ---

  if (!ft.logSetup(logsBeginAddr, logsEndAddr, LOG_minSize, LOG_maxSize, true)) {
    Serial.println("Erasing flash...");
    ft.chipErase();
    ft.logSetup(logsBeginAddr, logsEndAddr, LOG_minSize, LOG_maxSize, true);
  }

  //--- Radio settings ---

  LoRaFREQ = getLoRaFreq();

  if (!LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, radioID)) {      // initialize radio
    Serial.println("ERROR: radio init failed");
    return;
  }

  //--- Scan SDI-12 Addresses ---

  while (Serial.available()) {
    Serial.read();
  }
  Serial.println("Enter \"S\" to skip SDI-12 bus scan.");
  Serial.print((char)2);    //STX to indicate to UI software that it needs to send 'S' to skip sensor scan
  Serial.print("\r\n");

  long timeout = millis();
  char skipTest;
  while ((millis() - timeout) < 15000)
  {
    if (Serial.available() > 0)                        // if something typed, go to menu
    {
      skipTest = Serial.read();
      if ((skipTest == 's') || (skipTest == 'S'))
        skipScan = true;
      break;
    }
  }

  if (!skipScan) {
    SDI12Scan();
  }

  //--- Go to menu ---

  menu();                                        // display menu on startup

  if (skipScan) {                                // scan for sensors if haven't already
    SDI12Scan();
    depthToStruct();
  }
  if (RHPresent && interval < 15) {               // automatically set interval to minimum 15 minutes if RH sensor found
    interval = 15;
  }


  //--- Time sync in field ---

  delay(50);
  battV = calcbattV();        // check battery voltage
  if (battV <= lowBatt) {     // if battery voltage below low battery limit
    lowInitBatt = true;
  }
  if (gatewayPresent && !lowInitBatt) {
    delay(50);
    fieldSync(timeSync);
    listenRespond();
    duringInit = false;
  }
  digitalWrite(LED, LOW);
  
  //--- Set alarms ----

  // clear alarm registers

  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 0); // set alarm values to 0
  RTC.setAlarm(ALM2_MATCH_MINUTES, 0, 0, 0, 0);
  RTC.alarm(ALARM_1);                    // turn off alarm 1
  RTC.alarm(ALARM_2);                    // turn off alarm 2
  RTC.alarmInterrupt(ALARM_1, true);     // turn on alarm 1 interrupt
  if (gatewayPresent) RTC.alarmInterrupt(ALARM_2, true);     // 27-Jan-2020: turn on alarm 2 interrupt
  RTC.squareWave(SQWAVE_NONE);           // Use SQW pin as interrupt generator

  //--- Set alarm times ---

  readClock();
  RTC.alarm(ALARM_1);
  resetAlarm(interval);

  if (gatewayPresent){
    RTC.alarm(ALARM_2); // 27-Jan-2020: Use Alarm 2 to wake up for daily sync check
    resetAlarm2();
  }
  
}


//===================================================================================================
//===================================================================================================
//===================================================================================================

//-------------*** Main Loop ***---------------------------------------------------------------------

void loop() {

  sleepNow();

  if (RTC.alarm(ALARM_1)) {  // if alarm 1 goes off
    readClock();
    battV = calcbattV();
    battLow = EEPROM.read(EEPROM_LOW_BATT);

    if (battV <= lowBatt) {       // if battery low skip measurements, go to sleep, wake up again at next interval
      resetAlarm(interval);
      if (gatewayPresent) resetAlarm2();
      battLow = true;
      EEPROM.update(EEPROM_LOW_BATT, battLow);
      sleepNow();
    }
    else if (lowInitBatt && gatewayPresent){
      fieldSync(timeSync);
      listenRespond();
      duringInit = false;
      lowInitBatt = false;
      readClock();
      resetAlarm(interval);
      resetAlarm2();
    }    
    else if (battLow && gatewayPresent) { // || numMissed == 3) {   // 22Mar21: added back in; if battery was low but now ok or Node has missed comm w/ G >= 3 consecutive times
      long int_mSecs = (interval + 1) * 60000;
      RTC.alarmInterrupt(ALARM_2, false);   // turn off alarm 2 interrupt
      fieldSync(int_mSecs);  // wait for one interval + 1 min to get timestamp from Gateway
      readClock();
      resetAlarm(interval);
      resetAlarm2();
      //      numMissed = 0;
    }

    // scenario 1: No RH sensor, no Gateway (Bare Node w/o G)

    else if (RHPresent == 0 && gatewayPresent == false) {
      if (mins % interval == 0) {
        digitalWrite(LED, HIGH);
        Timestamp();
        delay(50);
        readSensors();
        saveData();
        resetAlarm(interval);
        digitalWrite(LED, LOW);
      }
    }

    // scenario 2: RH sensor, no Gateway (Cover Crop Node w/o G)

    else if (RHPresent == 1 && gatewayPresent == false) {
      if ((mins + 6) % interval == 0) {
        Serial.println("heaterON");
        heaterON();
        resetAlarm(3);
      }
      else if ((mins + 3) % interval == 0) {
        Serial.println("heaterOFF");
        heaterOFF();
        resetAlarm(3);
      }
      else {       // mins % interval == 0
        Serial.println("Measure sensors");
        digitalWrite(LED, HIGH);
        Timestamp();
        delay(50);
        readSensors();
        saveData();
        byte n = interval - 6;
        resetAlarm(n);
        digitalWrite(LED, LOW);
      }
    }

    // scenario 3: RH sensor, Gateway (Cover Crop Node w G)

    else if (RHPresent == 1 && gatewayPresent == true) {
      if (mins % interval == 1 ) {
        Serial.println("Sending to Gateway");
        digitalWrite(LED, HIGH);
        listenRespond();
        byte n = interval - 7;
        resetAlarm(n);
        digitalWrite(LED, LOW);
      }
      else if ((mins + 6) % interval == 0) {
        Serial.println("heaterON");
        heaterON();
        resetAlarm(3);
      }
      else if ((mins + 3) % interval == 0) {
        Serial.println("heaterOFF");
        heaterOFF();
        resetAlarm(3);
      }
      else if (mins % interval == 0) {      // mins % interval == 0
        Serial.println("Measure sensors");
        digitalWrite(LED, HIGH);
        Timestamp();
        delay(50);
        readSensors();
        saveData();
        resetAlarm(1);
        digitalWrite(LED, LOW);
      }
    }

    // scenario 4: No RH sensor, Gateway (Bare Node w G)

    else {
      if (mins % interval == 1) {
        digitalWrite(LED, HIGH);
        listenRespond();
        resetAlarm(interval);
        digitalWrite(LED, LOW);
      }
      else {
        digitalWrite(LED, HIGH);
        Timestamp();
        delay(50);
        readSensors();
        saveData();
        resetAlarm(1);
        digitalWrite(LED, LOW);
      }
    }

  }   // end if Alarm 1

  else if (RTC.alarm(ALARM_2) && gatewayPresent) {  // 27-Jan-2020: add daily sync check routine
    battV = calcbattV();
    if (battV <= lowBatt) {       // if battery low skip loop, go to sleep
      resetAlarm2();
      resetAlarm(interval);
      battLow = true;
      EEPROM.update(EEPROM_LOW_BATT, battLow);
      sleepNow();
    } else {
      fieldSync(90000);    // wait up 1-1/2 minutes for timestamp from Gateway
      resetAlarm2();
    }
  }

} // end loop

//===================================================================================================
//===================================================================================================
//===================================================================================================
//John: Added below to read LoRa frequency based on factory constants written to EEPROM
uint16_t getLoRaFreq() {
  char optsRadio;
  EEPROM.get(EEPROM_OPTSRADIO, optsRadio);
  byte LoRaCode = (byte)((((byte)optsRadio) >> 2) & 0x07);
  switch (LoRaCode) {
    case 1:
      return 915;
    case 2:
      return 868;
    case 3:
      return 433;
    case 4:
      return 470;
    // cases 0, 5, 6, 7 are undefined
    default:
      return 915;
  }
}
//===================================================================================================

//--------- Find Active SDI-12 Sensor Addresses -----------------------------------------------------

void SDI12Scan() {
  SDI12port.begin();
  delay(100);
  Serial.println(F("Scanning for SDI-12 sensors...")); //digitalWrite(LED, HIGH);
  Serial.println();

  for (byte i = '0'; i <= '9'; i++) if (checkActiveSDI12(i)) setActiveSDI12(i); // scan address space 0-9

  for (byte i = 'a'; i <= 'z'; i++) if (checkActiveSDI12(i)) setActiveSDI12(i); // scan address space a-z

  for (byte i = 'A'; i <= 'Z'; i++) if (checkActiveSDI12(i)) setActiveSDI12(i); // scan address space A-Z

  delay(50);

  printSDI12Info();

  Serial.println();
  Serial.println(F("Done"));
  delay(50);
  SDI12port.end();

  skipScan = false;
}

boolean checkActiveSDI12(char i) {

  String myCommand = "";
  myCommand = "";
  myCommand += (char) i;                 // sends basic 'acknowledge' command [address][!]
  myCommand += "!";

  for (int j = 0; j < 3; j++) {          // goes through three rapid contact attempts
    SDI12port.sendCommand(myCommand);
    delay(100);
    if (SDI12port.available() >= 1) break;
    delay(100);
  }

  if (SDI12port.available() >= 2) {   // if it hears anything it assumes the address is occupied
    SDI12port.clearBuffer();
    //    Serial.println((char) i);
    //    sensorDetected = true;
    return true;
  }
  else {
    SDI12port.clearBuffer();              // otherwise it is vacant.
    return false;
  }

  SDI12port.clearBuffer();

}

// gets identification information from a sensor, and prints it to the serial port
// expects a character between '0'-'9', 'a'-'z', or 'A'-'Z'.
void compileInfoSDI12(char i) {
  SDI12port.clearBuffer();

  int j;
  String Command = "";
  Command += (char) i;
  Command += "I!";
  SDI12port.sendCommand(Command);         // get sensor ID
  delay(565);

  byte buf_length = SDI12port.available();
  byte x = 0;

  for (byte j = 0; j < buf_length; j++) {  // save sensor ID to buffer array and print
    char c = SDI12port.read();
    if (c != 10 && c != 13) {
      sensorList[sensorTot][x] = c;
      delay(8);
    }
    x++;
  }

  // Populate object with metadata
  char oldA = allSensors[sensorTot].addr;
  char newA = sensorList[sensorTot][0];

  if (newA != oldA) {
    char error[] = "-999";
    strncpy(allSensors[sensorTot].depth, error, sizeof(error));
  }

  allSensors[sensorTot].addr = newA;
  strncpy(allSensors[sensorTot].ID, sensorList[sensorTot], buf_length);
  allSensors[sensorTot].ID[buf_length - 2] = 0;
  //  Serial.print("-- Sensor ");
  //  Serial.println(allSensors[sensorTot].addr);
  //  Serial.print("ID: ");
  //  Serial.println(allSensors[sensorTot].ID);

  findSensorType(sensorList[sensorTot]);

  Serial.flush();
}

void printSDI12Info() {         // print sensor IDs
  for (byte a = 0; a < sensorTot; a++) {
    Serial.println(allSensors[a].ID);
    delay(8);
  }
}

void setActiveSDI12(char c) {   // save active sensor addresses
  activeSDI12[sensorTot] = c;
  compileInfoSDI12(c);
  sensorTot++;
}

boolean isActiveSDI12(char c) {
  for (int index = 0; index < MAX_SENSORS + 1; index++) {
    if (activeSDI12[index] == c)
      return true;
  }
  return false;
}

void findSensorType(char fullID[36]) {      // Identify RH, T, or CS655 from sensor ID
  bool typeFound = false;

  for (byte j = 0; j < NUM_TYPES; j++) {
    char * sensor;
    sensor = strstr(sensorList[sensorTot], sensTypes[j][0]);
    //        Serial.println(sensTypes[j][0]);
    //        Serial.println(sensor);
    char sens_buf[5];
    for (byte i = 0; i < 4; i++) {
      sens_buf[i] = sensor[i];
    }
    sens_buf[4] = 0;
    if (strcmp(sens_buf, sensTypes[j][0]) == 0) {
      strncpy(allSensors[sensorTot].type, sensTypes[j][0], 4);
      char fullcmd[5];
      fullcmd[0] = allSensors[sensorTot].addr; delay(10);
      fullcmd[1] = 0;
      strcat(fullcmd, sensTypes[j][1]); delay(10);
      if (fullcmd[2] == '!') {
        fullcmd[3] = 0;
      }
      strncpy(allSensors[sensorTot].cmd, fullcmd, 5); delay(20);
      uint8_t numout;
      numout = 10*(sensTypes[j][2][0] - 48) + (sensTypes[j][2][1] - 48);
//      Serial.print("numout = ");
//      Serial.println(numout);
      allSensors[sensorTot].numOut = numout;
      typeFound = true;

      if (j == 3) {   // if Acclima RH sensor
        RHsensor = allSensors[sensorTot].addr;
        RHPresent = true;
      } else if (j == 2) {   // if Acclima temp sensor
        Tsensor = allSensors[sensorTot].addr;
        TPresent = true;
      }
    }
  }

  if (!typeFound) {
    strncpy(allSensors[sensorTot].type, "UNKN", 4);
    char fullcmd[5];
    fullcmd[0] = allSensors[sensorTot].addr;
    strcat(fullcmd, READ_CMD_DEFAULT);
    strncpy(allSensors[sensorTot].cmd, fullcmd, 5);
    allSensors[sensorTot].numOut = 5;
  }

//    Serial.print("type: ");
//    Serial.println(allSensors[sensorTot].type);
//    Serial.print("command: ");
//    Serial.println(allSensors[sensorTot].cmd);
//    Serial.print("numOut: ");
//    Serial.println(allSensors[sensorTot].numOut);
//    Serial.println();
}

//===================================================================================================

//--------- Recall depths from EEPROM, save to sensor struct ----------------------------------------

void depthToStruct() {

  EEPROM.get(EEPROM_DEPTHS, depths);
  delay(20);

  for (byte i = 0; i < sensorTot; i++) {
    for (byte j = 0; j < sensorTot; j++) {
      if (depths[i][0] == allSensors[j].addr) {
        char dpth[6];
        for (byte c = 0; c < 6; c++) {
          if ((depths[i][c + 1] >= '0' && depths[i][c + 1] <= '9') || (depths[i][c + 1] == '+') || (depths[i][c + 1] == '-') ) {
            dpth[c] = depths[i][c + 1];
          } else {
            dpth[c] = 0;
            break;
          }
        }
        strcpy(allSensors[j].depth, dpth);
      }
    }
  }
}
//===================================================================================================

//------------- Sleep Functions ---------------------------------------------------------------------

void setRTCInterrupt() {
  sei();      // turn on Global Interrupt Enable

  PCMSK0 |= (1 << PCINT2);   //  set pin as PCINT
  PCICR |= (1 << PCIE0);     //  enable interrupts on vector 0
}

void clearRTCInterrupt() {
  PCICR &= (1 << PCIE0);
}

ISR(PCINT0_vect) {
  sleep_disable();
  clearRTCInterrupt();
}

void sleepNow() {
  // Turn off extras to save power
  Serial.end();     // turn off Serial comm
  digitalWrite(LED, LOW);
  digitalWrite(SDI12Pwr, LOW);
  digitalWrite(pin_SDIv12, LOW);
  driverRFM95.sleep();  // put radio to sleep
  analogComp_off(); // turn off analog comparator
  ADC_off();        // turn off ADC
  JTAG_off();       // disable On-Chip Debug system

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);            //Power down completely on sleep
  sleep_enable();
  setRTCInterrupt();
  sleep_mode();                                  // puts MEGA to sleep

  // after wake-up interrupt
  ADC_on();
  Serial.begin(baudRate);
  SPI.begin();
  digitalWrite(SDI12Pwr, HIGH);
  digitalWrite(pin_SDIv12, HIGH);

}

//===================================================================================================

//------------- Power Savers ------------------------------------------------------------------------

void RTC_osc_off() {
  //Turn RTC 32kHz oscillator off to save power:
  uint8_t RTC_SR;
  uint8_t res;
  RTC_SR = RTC.readRTC(0x0F); //First read status register
  res = RTC.writeRTC(0x0F, RTC_SR & ~(1 << 3)); //Now re-write status register bits (bit 3 needs to be set to 0 to disable 32kHz output)
  //res = RTC.writeRTC(0x0F, RTC_SR | (1 << 3)); //Just as a test, this ENABLES the output (which should pull more current)
}

void analogComp_off() {
  //Turn off analog comparator
  ACSR &= ~(1 << 7);
}

void analogComp_on() {
  //Turn on analog comparator
  ACSR |= (1 << 7);
}

void ADC_off() {
  //Turn off ADC  --  this saves about 113 uA
  ADCSRA &= ~(1 << 7);
}

void ADC_on() {
  //Turn on ADC
  ADCSRA |= (1 << 7);
}

void JTAG_off() {     // we don't ever need to turn it on for this application
  cli();
  MCUCR |= (1 << 7);
  //Must be executed twice within 4 clock cycles to disable JTAG!
  MCUCR |= (1 << 7);
  sei();
}

//===================================================================================================

//------------- DS3231 RTC Functions ----------------------------------------------------------------

void readClock()
{
  RTC.read(tm);

  secs = tm.Second;
  mins = tm.Minute;
  hrs = tm.Hour;
  days = tm.Day;
  mnths = tm.Month;
  yrs = tm.Year + 1970;  //tm.Year --> years since 1970

  delay(50);

}

float getBoxT() {
  boxTemp = (float)RTC.temperature() / 4.0;
  return boxTemp;
}

void resetAlarm(byte n) {
  if (n == interval) {
    alarmMins = ((((mins + n) % 60) / n) * n);
  }
  else if (mins + n >= 60) {
    alarmMins = (mins + n) % 60;
  }
  else {
    alarmMins = mins + n;
  }

  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, alarmMins, 0, 0);

  if (debug){
    Serial.print("Next alarm at: ");
    Serial.println(alarmMins);
  }
  delay(50);
}

void resetAlarm2() {
  if(gatewayPresent){
    readClock();
    RTC.setAlarm(ALM2_MATCH_HOURS, 0, NISTmin, NISThr, 0); // for testing 16Jun2020
  
    // Note for Alarm 2:
    // ALM2_MATCH_HOURS -- causes an alarm when the hours and minutes match.
    // ALM2_MATCH_DATE -- causes an alarm when the date of the month and hours and minutes match.
    if (debug){
      Serial.print("Alarm 2 set for: ");
      Serial.print(NISThr);
      Serial.print(":");
      Serial.println(NISTmin);
      delay(50);
    }
  }
}

//===================================================================================================

//--------- Sync time with Gateway in field ---------------------------------------------------------

void fieldSync(uint32_t time2wait) {    // 27-Jan-2020: added time2wait parameter to use function for daily sync check
  uint32_t timeToWait = time2wait;
  if (duringInit) {
    fieldSyncStart = millis();
  }

  if (LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, radioID)) {       // initialize radio

    Serial.println("Waiting to sync with Gateway...");

    uint8_t buf[20];     // array to receive time update from gateway
    uint8_t len = sizeof(buf);
    uint8_t from;
    boolean done = false;
    uint8_t ok[1];
    ok[0] = radioID;

    unsigned long waitStart = millis();

    while (((millis() - waitStart) < timeToWait) && (done == false)) {    // 27-Jan-2020: use timeToWait instead of twoHours
      if (LoRa.recvfromAckTimeout(buf, &len, timeoutSmallPkt, &from)) { // if time update received
        //          for(byte u = 0; u<len;u++){
        //          Serial.print((char)buf[u]);
        //          }
        //          Serial.println();

        if (len > 1) {   // 04Aug2020: && buf[0] != GatewayID) { <-- CAUSES PROBLEM IF Gateway ID = 50 (char "2" in "2020")
          String rID = String((char)radioID);
          LoRa.sendMessage(&rID, GatewayID);   //NOTE: This gives affirmative reception of timestamp without first parsing to ensure it is okay.

          int nYear;
          int nMonth;
          int nDay;
          int nHour;
          int nMin;
          int nSec;
          char *nbuf = (char*)buf;
          //        Serial.println(nbuf);

          if (sscanf(nbuf, "%d/%d/%d|%d:%d:%d", &nYear, &nMonth, &nDay, &nHour, &nMin, &nSec)) {  // evaluate buffer and set clock to new time

            tm.Year = nYear - 1970;
            tm.Month = nMonth;
            tm.Day = nDay;
            tm.Hour = nHour;
            tm.Minute = nMin;
            tm.Second = nSec;
            RTC.write(tm);      // update clock

            Serial.print("Got timestamp from Gateway:");
            Serial.println(nbuf);
            if ((nYear < 2018) || (nYear > 2050)) {
              Serial.println("Bad timestamp");
            }

          }  // end if sccanf
          else {} // 16Jun2020: if timestamp not in correct format, do nothing

          if (duringInit) {
            readClock();
            Timestamp();
            readSensors();
            compileData();
            done = true;  // 14May
          }
          else if (battLow) {
            battLow = false;
            EEPROM.update(EEPROM_LOW_BATT, battLow);
            done = true;
            /*} else if (numMissed >= 3) {
              numMissed = 0;
              done = true;*/
          } else {
            done = true;
          }
        }   // end if(len>0)
      }  // end if transmission received
    } // end while loop
  } else {
    Serial.println("Radio failed");
  }
  if (duringInit) {
    fieldSyncStop = millis();
  }
}

//===================================================================================================

//------------- Listen for timestamp & send data ----------------------------------------------------

void listenRespond() {

  SPI.begin();
  delay(200);

  // Step 1 - turn on radio

  if (!LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, radioID)) {      // initialize radio
    Serial.println("Radio failed");
  }

  // Step 2 - wait to receive time update from Gateway, if received respond with data and update clock

  uint8_t buf[20];     // array to receive time update from gateway
  uint8_t len = sizeof(buf);
  uint8_t from;
  timeUpdated = false;

  Serial.println("Waiting for Gateway...");

  long timeToWait;
  long start = millis();

  if (duringInit) {
    timeToWait = timeSync - (fieldSyncStop - fieldSyncStart) + 1200000; // 15May20 add 20 minutes
    Serial.print("timeToWait = ");
    Serial.println(timeToWait);
  } else if (numMissed == 3) {
    long int_mSecs = (interval + 1) * 60000;
    timeToWait = int_mSecs;
    RTC.alarmInterrupt(ALARM_2, false);     // turn off alarm 2 interrupt just in case time ~ NIST time
  } else {
    timeToWait = 60000; // 13May20: one minute
  }

  while (timeUpdated == false && ((millis() - start) < timeToWait)) { // 13May20
    readClock();

    if (LoRa.recvfromAckTimeout(buf, &len, timeoutPacket, &from)) { // if time update received
      if (len > 0) {
        int nYear;
        int nMonth;
        int nDay;
        int nHour;
        int nMin;
        int nSec;
        char *nbuf = (char*)buf;

        if (sscanf(nbuf, "%d/%d/%d|%d:%d:%d", &nYear, &nMonth, &nDay, &nHour, &nMin, &nSec)) {  // evaluate buffer and set clock to new time
          if ((nYear - 2000 > 0)) {   // 17Jun20: works to reject garbled timestamp
            tm.Year = nYear - 1970;
            tm.Month = nMonth;
            tm.Day = nDay;
            tm.Hour = nHour;
            tm.Minute = nMin;
            tm.Second = nSec;

            RTC.write(tm);      // update clock
            delay(5);
            timeUpdated = true;
            numMissed = 0;      // 08-Jul-2021: only want to track consecutive misses
            Serial.print("Successful time update: ");
            Serial.println(nbuf);

            LoRa.sendMessage(&allData, GatewayID);
            readClock();
          }
        }
      } // end if(len>0)
    }
  }  // end while loop

  if (timeUpdated == false && !duringInit) {   // 22Mar21: Added back in, 13May2020
    if (numMissed >= 3) {
      uint8_t statusReg = RTC.checkAlmStat();    // check if Alarm 2 went off
      if (statusReg == 2) {
        resetAlarm2();
        RTC.alarm(ALARM_2);
        RTC.alarmInterrupt(ALARM_2, true);
      }
      numMissed = 0;
    } else {
      numMissed++;
    }
    Serial.print("numMissed: ");
    Serial.println(numMissed);
  }

}

//===================================================================================================

//------------- Initiate Measurements from Active Sensors -------------------------------------------

void readSensors() {

  //  Serial.println();
  //  Serial.println("Measuring sensors...");
  //  Serial.println();

  //  // Add timestamp before sensor data!!!
  ////  memset(allData,0,sizeof(allData));
  allData = "";
  //  char timestamp[] = "timestamp~";
  ////  strcpy(allData,timestamp);
  //  allData += timestamp; delay(30);

  SDI12port.begin();
  delay(500);

  for (byte i = 0; i < sensorTot; i++) {
    char c = allSensors[i].addr;
    rowNum = i;
    //    Serial.println();
    //    Serial.print(F("--> Measuring sensor "));  delay(20);
    //    Serial.println(allSensors[i].addr);
    measureSDI12(c);
    delay(500);
  }

  //  Serial.println();
  //  Serial.print("Full data string: ");
  //  Serial.println(allData);

  SDI12port.end();
  delay(50);

}

void measureSDI12(char index) {

  if (strcmp(allSensors[rowNum].type, "CLIM") == 0) {  // if ClimaVue
//    numOut = 14;    // specify numOut since parseResp() isn't needed
    SDI12response = true;
    delay(100);
  } else {
    SDI12port.sendCommand(allSensors[rowNum].cmd);
    delay(parseResp());
    SDI12port.clearBuffer();  // <-- DON'T MOVE THIS!
    delay(50);
  }

  getResponseSDI12(index);

}

//------------- Parse response from M! command for delay value --------------------------------------

int parseResp() {
  delay(100);

  byte incomingBytes;

  if (SDI12port.available() > 0) {
    SDI12response = true;
    incomingBytes = SDI12port.available();
    char response[incomingBytes];
    for ( byte i = 0; i < incomingBytes; i++) {
      response[i] = SDI12port.read();
    }

    measDelay = (response[incomingBytes - 4] - 48); // get decimal value of char, convert to milliseconds
    measDelay = measDelay * 1000;
    //    Serial.print("measDelay = ");
    //    Serial.println(measDelay);

    if (strcmp(allSensors[rowNum].type,"UNKN") == 0){
      allSensors[rowNum].numOut = (response[incomingBytes - 3] - 48);  
    //    Serial.print("numOut = ");
    //    Serial.println(numOut);
    }
  } else {
    measDelay = 1000;
    SDI12response = false;
    //    Serial.println("No response");
  }

  return measDelay;

}


//------------- Take Measurement from TDRs --------------------------------------

void getResponseSDI12(char index) {

  char buf[MAX_RESP];
  bool bufAdded = false;

  if (SDI12response == true) {       // if a response to M! was received, send D0!
    char sendData[5];
    sendData[0] = index;
    sendData[1] = 0;
    char snd[4];

    if (strcmp(allSensors[rowNum].type, "CLIM") != 0) {
      strcpy(snd, "D0!");
    } else {
      strcpy(snd, "R7!");
    }
    strcat(sendData, snd);

    SDI12port.sendCommand(sendData);   // send D0! command to retrieve data
    delay(500);                  // 300 works with TDRs

    // after sending D0! command:

    // Step 1 -- create char array to receive data --

    byte incomingbytes = SDI12port.available();
    //    Serial.print("Incoming bytes: "); Serial.println(incomingbytes);

    // Step 2 -- store bytes into array

    byte negNum = 0;

    if (incomingbytes > 3) {
      char responseRaw[SDI12_BUFFER_SIZE];
      responseRaw[SDI12_BUFFER_SIZE - 1] = 0;
      //      Serial.print("Raw response of buffer size: ");
      byte respLen;

      for (byte i = 0; i < (SDI12_BUFFER_SIZE); i++) {
        char c = SDI12port.read(); delay(8);
        responseRaw[i] = c;         // store c in array if not at last byte
        if (c == 13) {
          //          if (strcmp(allSensors[rowNum].type, "TR31") == 0) { // if a TDR and keepPEC, add tilde to end
          //            #ifdef keepPEC
          //              responseRaw[i] = '~';
          //              i++;
          //            #endif
          //          }
          responseRaw[i] = 0;
          respLen = i;
          break;
        } else {
          //          Serial.print(c);
          if (c == '-') {
            negNum++;                 // count number of '-'s to calculate size of array needed to use sep delimiter
          }
        }
        delay(10);
      }    // end for loop
      //      Serial.println(responseRaw);

      SDI12port.clearBuffer();

      //      Serial.print("respLen: "); Serial.println(respLen);

      // Step 3 -- remove address, CR, and replace "+" and "-" with delimiter, add in "-"

      byte len = respLen + negNum;
      char response[len];
      byte j = 1;

      for (byte i = 0; i < len; i++) {
        if (responseRaw[j] == '+') {
          response[i] = sep;
        }
        else if (responseRaw[j] == '-') {
          response[i] = sep;
          i++;
          response[i] = responseRaw[j];
        }
        else {
          response[i] = responseRaw[j];
        }
        j++;
        delay(5);
      }
      response[len - 1] = 0;
      
      if(debug){
        Serial.println();
        Serial.print("Delimited response: ");
        Serial.println(response);
      }

      if (strcmp(allSensors[rowNum].type, "TR31") == 0) { // if a TDR

        bufAdded = getTimeData(rowNum, index, response, len);

      } // end if sensor is a TDR
      else {
        //        strcpy(allSensors[rowNum].data, response);
        if (rowNum != sensorTot - 1) {
          if (allSensors[rowNum].depth[0] != 0) {
            sprintf(buf, "%s~%s%s~", allSensors[rowNum].ID, allSensors[rowNum].depth, response);
          } else {
            char dpth[] = "-999";
            sprintf(buf, "%s~%s%s~", allSensors[rowNum].ID, dpth, response);
          }
        } else {
          if (allSensors[rowNum].depth[0] != 0) {
            sprintf(buf, "%s~%s%s", allSensors[rowNum].ID, allSensors[rowNum].depth, response);
          } else {
            char dpth[] = "-999";
            sprintf(buf, "%s~%s%s", allSensors[rowNum].ID, dpth, response);
          }
        }
      }

    } else {  // ERROR 1: if incomingbytes < 3
      char error[] = "~-999";
      byte numOut = allSensors[rowNum].numOut;
      byte errorLen = numOut * sizeof(error);
      char errString[errorLen];
      strcpy(errString, error);

      if (numOut > 1) {
        for (byte i = 0; i < numOut - 1; i++) {
          strcat(errString, error);
        }
      }

      //      strcpy(allSensors[rowNum].data,errString);
      if(debug){
        Serial.print("Error string: ");
        Serial.println(errString);
      }
      
      if (rowNum != sensorTot - 1) {
        sprintf(buf, "%s~%s%s~", allSensors[rowNum].ID, allSensors[rowNum].depth, errString);
      } else {
        sprintf(buf, "%s~%s%s", allSensors[rowNum].ID, allSensors[rowNum].depth, errString);
      }

    }
  } else {  // if SDI12response = false
    char error[] = "~-999";
    byte numOut = allSensors[rowNum].numOut;
    byte errorLen = numOut * sizeof(error);
    char errString[errorLen];
    strcpy(errString, error);
  
    if (numOut > 1) {
      for (byte i = 0; i < numOut - 1; i++) {
        strcat(errString, error);
      }
    }
    
    if (debug){
      Serial.print("Error string: ");
      Serial.println(errString);
    }
    
    if (rowNum != sensorTot - 1) {
      sprintf(buf, "%s~%s%s~", allSensors[rowNum].ID, allSensors[rowNum].depth, errString);
    } else {
      sprintf(buf, "%s~%s%s", allSensors[rowNum].ID, allSensors[rowNum].depth, errString);
    }
  }
  //  Serial.println(buf);
  delay(50);

  if (!bufAdded) {  // if data string not already concatenated
    allData += buf; delay(50);
  }

  SDI12port.clearBuffer();
  delay(50);

}

//------------- TDRs: Replace PEC with travel time --------------------------------------

bool getTimeData(byte r, char a, char data[80], byte l) {
  byte rowNum = r;
  char index = a;
  byte len = l + 1;
  char response[len];
  char buf[MAX_RESP];

  //  Serial.println(len);
  for (byte j = 0; j < len; j++) {
    response[j] = data[j];
    if (data[j] == 0) {
      response[j] = '~';
    }
    //    Serial.print(j);
  }
  response[len - 1] = 0;
  //  Serial.println(response);

  //--- get time data from TDR

  char sendTT[5];
  sendTT[0] = index;
  sendTT[1] = 0;
  char sndTT[] = "D1!";
  strcat(sendTT, sndTT);
  //          Serial.println(sendTT);

  SDI12port.sendCommand(sendTT);
  delay(500);

  byte incomingTT = SDI12port.available();
  char respTT[incomingTT - 1];   // replace trailing CR with NULL and remove LF

  if (incomingTT > 5) {
    for (byte i = 0; i < (incomingTT - 1); i++) {
      char c = SDI12port.read();
      if (c != 13 && c != 10 && c != '+' && c != '-') {
        respTT[i] = c;         // store c in array if not at last byte
        delay(15);
      }
      else if (c == 13) {
        respTT[i] = 0;
      }
      else if (c == '+' || c == '-') {
        respTT[i] = sep; delay(15);
      }
    }    // end for loop
  }
  
  if (debug){
    Serial.print("Travel time resp: ");
    Serial.println(respTT);
  }

  //--- extract travel time (second to last output)
  
  char * TTlastTilde = strrchr(respTT, '~');
  byte lastTpos = TTlastTilde - respTT;       // position of last tilde in respTT
  byte lenTT = 0;                             // length of char array needed for travel time
  byte lastdigit;                             // position in respTT array where tt begins
  for (byte i = 1; i < lastTpos; i++) {
    if (respTT[lastTpos - i] != '~') {
      lastdigit = lastTpos - i;
      lenTT++;
    } else {
      break;
    }
  }

  char travelT[lenTT + 1];                    // array for holding travel time

  for (byte i = 0; i < lenTT; i++) {
    travelT[i] = respTT[lastdigit + i];
  }
  travelT[lenTT] = 0; delay(100);
  
  if (debug){
    Serial.print("Travel time: ");
    Serial.println(travelT); delay(20);
  }

  // concatenate sensor data and time data
  char resp_TT[len + lenTT]; delay(50);

  strcpy(resp_TT, response); delay(50);
  resp_TT[0] = '~'; delay(50);
  strcat(resp_TT, travelT); delay(50);
  
  if (debug){
    Serial.print("resp_TT: ");        
    Serial.println(resp_TT);
  }
  
  if (rowNum != sensorTot - 1) {
    if (allSensors[rowNum].depth[0] != 0) {
      sprintf(buf, "%s~%s%s~", allSensors[rowNum].ID, allSensors[rowNum].depth, resp_TT);
    } else {
      char dpth[] = "-999";
      sprintf(buf, "%s~%s%s~", allSensors[rowNum].ID, dpth, resp_TT);
    }
  } else {
    if (allSensors[rowNum].depth[0] != 0) {
      sprintf(buf, "%s~%s%s", allSensors[rowNum].ID, allSensors[rowNum].depth, resp_TT);
    } else {
      char dpth[] = "-999";
      sprintf(buf, "%s~%s%s", allSensors[rowNum].ID, dpth, resp_TT);
    }
  }
  //#endif  // end ifdef getTT
  if (debug) Serial.println(buf);
  
  delay(100);
  allData += buf; delay(50);
  return true;

}
//===================================================================================================

//--------------- Calculate Battery Voltage ---------------------------------------------------------

float calcbattV() {
  float result;
  pinMode(pin_mBatt, OUTPUT);
  pinMode(pin_battV, INPUT);

  digitalWrite(pin_mBatt, HIGH);
  delayMicroseconds(500);

  uint16_t vInt;
  vInt = analogRead(pin_battV);

  digitalWrite(pin_mBatt, LOW);

  uint16_t battV100 = (uint16_t)(((uint32_t)vInt * 4 * ADC_REF_VOLTAGE * 100 + (3 * ADC_MAXVALUE / 2)) / (3 * ADC_MAXVALUE));
  // NOTE: The order of operations is important in code above to maintain the greatest precision!
  // NOTE: As needed, terms have been typecasted to uint32_t to avoid overflow for 2 byte integer.
  // Explanation of the terms:
  //  vInt:                             the raw integer from the analog to digital converter (ADC)
  //  4 / 3:                            correction factor due to voltage divider on battery measurement circuit
  //  ADC_REF_VOLTAGE / ADC_MAXVALUE:   convert the raw, 10-bit analog number into a meaningful voltage
  //  100:                              give the result in centi-volts and therefore allow storing a floating point number in 2 bytes instead of using the 4-byte float.
  //  3 * ADC_MAXVALUE / 2:             for rounding to nearest integer value (leave this off to get the truncated value)

  //  Serial.println(battV100);
  result = battV100 / 100 + float((battV100 % 100)) / 100;
  return result;

}

//===================================================================================================

//------------- Compile timestamp  ------------------------------------------------------------------

void Timestamp() {      // compile timestamp

  battV = calcbattV();
  getBoxT();
  //  unsigned int pvCurrent = getSolarCurrent();
  //  float pvVoltage = getSolarVoltage();
  uint16_t pvCurrent = GetISolar();   // using John's code
  float pvVoltage = GetVSolar();


  timestamp = "";
  delay(20);

  timestamp += VERSION;
  timestamp += sep;
  timestamp += projectID;
  timestamp += sep;
  timestamp += serNum;
  timestamp += sep;
  timestamp += battV;
  timestamp += sep;
  timestamp += boxTemp;
  timestamp += sep;
  timestamp += pvCurrent;
  timestamp += sep;
  timestamp += pvVoltage;
  timestamp += sep;
  timestamp += yrs;
  timestamp += '-';
  timestamp += mnths;
  timestamp += '-';
  timestamp += days;
  timestamp += '_';
  timestamp += hrs;
  timestamp += ':';
  if (mins < 10) timestamp += '0';
  timestamp += mins;
  timestamp += ':';
  if (secs < 10) timestamp += '0';
  timestamp += secs;
  timestamp += "_UTC";
  //  timestamp += TMZ;
  timestamp += sep;
  //  Serial.println(timestamp);
  delay(50);

}

//===================================================================================================

//------------- Compile data in String --------------------------------------------------------------

void compileData() {
  //  Serial.println("Compiling data String");

  allData = timestamp + allData; // + '\r' + '\n';
  delay(100);
  Serial.println(allData);
  delay(50);

}

//===================================================================================================

//------------- Save data to Flash ------------------------------------------------------------------

void saveData() {

  compileData();
  uint16_t Len = allData.length(); // + 1; // includes NULL
  //  Serial.print("Len: ");
  //  Serial.println(Len);
  char tmpChars[Len];  // char array to hold data string (will be converted to uint8_t array for sending)

  memcpy(tmpChars, allData.c_str(), Len);

  // Call function to write log
  if (ft.writeLog(tmpChars, Len)) {
    if (debug){
      // The remaining code is just for output to the screen and is not necessary for writing the log:
      uint16_t logSize = ft.getLogSize(ft.lastLogAddress());
      uint16_t logMemorySize = ft.getLogMemorySize(ft.lastLogAddress());
  
      Serial.print("\n\nWrote logId: ");
      Serial.println(ft.logId() - 1);       // Subtracting one here because we are reporting after the write and the log ID has already been incremented
      Serial.print("data: ");
      Serial.println(allData);
      Serial.print("data size: ");
      Serial.print(Len);
      Serial.print("; log size: ");
      Serial.print(logSize);
      Serial.print("; size in flash: ");
      Serial.println(logMemorySize);
      //    Serial.print("Location: 0x");
      //    Serial.println(writeLoc, HEX);
    }
  } else {
    Serial.println("Failed to write log!");
  }

  ft.flash_sleep();

}

//======================================================================================

//--------------- Solar panel ----------------------------------------------------------

//Returns voltage of solar cell in centi-volts (or volts * 100)
float GetVSolar() {
  pinMode(pin_solarVoltage, INPUT);
  float result;
  uint16_t rawSolarV = analogRead(pin_solarVoltage);
  uint16_t calc = (uint16_t)(((uint32_t)rawSolarV * 2 * ADC_REF_VOLTAGE * 100 + (ADC_MAXVALUE / 2)) / ADC_MAXVALUE);
  result = float(calc / 100) + float((calc % 100)) / 100;
  return result;
  // NOTE: The order of operations is important in code above to maintain the greatest precision!
  // NOTE: As needed, terms have been typecasted to uint32_t to avoid overflow for 2 byte integer.
  // Explanation of the terms:
  //  rawSolarV:                        the raw integer from the analog to digital converter (ADC)
  //  2:                                correction factor due to a div-2 voltage divider so that solar voltage does not exceed range of ADC (solar panel up around 6 volts whereas max for ADC is 3.3 volts)
  //  ADC_REF_VOLTAGE / ADC_MAXVALUE:   convert the raw, 10-bit analog number into a meaningful voltage
  //  100:                              give the result in centi-volts and therefore allow storing a floating point number in 2 bytes instead of using the 4-byte float.
  //  ADC_MAXVALUE / 2:                 for rounding to nearest integer value (leave this off to get the truncated value)
}


//Returns short circuit current of PV cell in milliamperes (in effect gives the total available charging current of solar cell)
uint16_t GetISolar() {
  pinMode(pin_solarShort, OUTPUT);

  uint16_t iSolarRes100;
  EEPROM.get(EEPROM_iSolarRes, iSolarRes100);

  if (iSolarRes100 == (uint16_t)0xFFFF) {
    char verHW;
    EEPROM.get(EEPROM_verHW, verHW);

    // WARNING: EEPROM factory constants must always be written from this point forward!
    if (verHW != 'E')
      iSolarRes100 = 1000;    // this is a default of 10.00 ohms!
    else
      iSolarRes100 = 100;     // default of 1.00 ohms for old board rev. E
  }

  digitalWrite(pin_solarShort, HIGH);     // This "shorts" solar cell through shorting resisitor (depending on hardware could be a single 1 ohm resistor or an array of resistors that yields 10 ohms)
  delayMicroseconds(500);                 // Wait 1/2 millisecond before sampling to ensure steady state
  //  uint16_t vSolar100 = GetVSolar();       // Read voltage of shorted solar cell
  float vSolar = GetVSolar();
  digitalWrite(pin_solarShort, LOW);      // Clear short

  uint16_t dec = (uint16_t)(vSolar * 100) % 100;
  uint16_t vSolar100 = 100 * ((uint16_t)vSolar) + dec;

  return (uint16_t)(((uint32_t)vSolar100 * 1000) / iSolarRes100);
  // Explanation:
  // V = I * R  -->  I = V / R
  // NOTE: Both vSolar100 and iSolarRes100 are multiplied by 100 for greater precision but they are divided so that cancels out.
  // NOTE: As needed, terms have been typecasted to uint32_t to avoid overflow for 2 byte integer.
  // Multiplied by 1000 for conversion from amperes to "milliamps".
}

/* float getMOSFETresistance(float temp, float voltSD) {
   //NOTE: This is based on datasheet graphs but has empirical compensation as well
   //returns SSM3K376R MOSFET resistance (in Ohms) as a function of temperature and [pre]source-drain voltage (assuming 3-volt gate-source voltage)

   float tempCompensation = (float)((0.2053 * temp + 42.158) / 1000.0);
   float adjusted;
   if (voltSD > 0.10) {
     adjusted = tempCompensation + (-1.122 * voltSD * voltSD + 0.8759 * voltSD - 0.0843);
   } else {
     adjusted = tempCompensation + (-42.672 * voltSD * voltSD + 9.4514 * voltSD - 0.5258);
   }

   if (adjusted < 0)
     adjusted = 0;
   return adjusted;
  }
*/
/*
  //Returns voltage of solar cell in volts (battery load may affect voltage)
  float getSolarVoltage() {
  uint16_t rawSolarV;
  rawSolarV = analogRead(pin_solarVoltage);
  return (float)((ADC_REF_VOLTAGE * (float)rawSolarV / (float)ADC_RESOLUTION) * 2.0);   //multiplied by two because there is a voltage divider to keep the Arduino pin from getting 6+ volts from solar cell
  }

  //Returns short circuit current of PV cell in milliamperes (in effect gives the total available charging current of solar cell)
  unsigned int getSolarCurrent() {
  //Add the following lines to header and setup function:
  //  #define pin_solarShort      A4
  //  #define SOLAR_CALIB      1.0          //This will become a EEPROM constant that is set during factory config – for now just use 1.0
  //  pinMode(pin_solarShort, OUTPUT);

  float solarV;
  digitalWrite(pin_solarShort, HIGH);   //this "shorts" solar cell through a 1 Ohm resisitor
  delay(150);                           //wait to settle -- if this delay is too short the current reading will be high
  solarV = getSolarVoltage();           //read voltage of shorted solar cell
  digitalWrite(pin_solarShort, LOW);    //clear short
  return (uint16_t)((solarV / (SOLAR_CALIB)) * 1000);
  //return (uint16_t)((solarV / (SOLAR_CALIB + getMOSFETresistance((float)RTC.temperature()/4.0, solarV))) * 1000);         //solar current is voltage / resistance (which is 1 Ohm in this case); multiply by 1000 to convert to milliamperes
  }
*/
//======================================================================================

//--------------- RH Sensor Heater Functions -------------------------------------------

void heaterON() {
  SDI12port.begin();
  delay(100);

  String heatOn = "";
  heatOn += RHsensor;
  heatOn += "M7!";
  SDI12port.sendCommand(heatOn);
  delay(300);

  while (SDI12port.available()) SDI12port.read();
  delay(20);
  SDI12port.end();
  delay(20);
}

void heaterOFF() {
  SDI12port.begin();
  delay(100);

  String heatOff = "";
  heatOff += RHsensor;
  heatOff += "M8!";
  SDI12port.sendCommand(heatOff);
  delay(300);

  while (SDI12port.available()) SDI12port.read();
  delay(20);
  SDI12port.end();
  delay(20);
}

//======================================================================================

//--------------- Get Device info  -----------------------------------------------------

void getFactoryInfoFromEEPROM() {

  uint32_t sernum = EEPROM.get(10, sernum);
  uint16_t mfgDate = EEPROM.get(14, mfgDate);
  char verHW = EEPROM.get(16, verHW);
  char optsRadio = EEPROM.get(17, optsRadio);

  if (sernum == 0)
    sernum = 1024;            //Acclima default serial number 1024 is invalid
  if (mfgDate == 0)
    mfgDate = 9857;           //this default corresponds to 01-Apr-2019
  if (verHW == 0)
    verHW = '?';
  if (optsRadio == 0)
    optsRadio = '?';

  facInfo.sernum = sernum;
  facInfo.mfgDate = mfgDate;
  facInfo.verHW = verHW;
  facInfo.optsRadio = optsRadio;
}

void getDeviceIdStr(char* idStr) {  //TODO: Add serial number, hardware rev, mfgDate, get factory set stuff from EEPROM or reserved flash
  getFactoryInfoFromEEPROM();
  uint8_t idx;
  char* tmpStr = (char*)malloc(12);

  strcpy(idStr, "Acclima");                 //Acclima
  strcat(idStr, " ");                       //Acclima
  strcat(idStr, facInfo.model);             //Acclima Node
  strcat(idStr, "_");                       //Acclima Node_

  idx = strlen(idStr);
  idStr[idx] = facInfo.verHW;
  idStr[idx + 1] = 0;                       //Acclima Node_E

  strcat(idStr, " (");                      //Acclima Node_E (

  itoa(facInfo.verFW_maj, tmpStr, 10);

  strcat(idStr, tmpStr);                    //Acclima Node_E (1
  strcat(idStr, ".");                       //Acclima Node_E (1.

  itoa(facInfo.verFW_min, tmpStr, 10);

  strcat(idStr, tmpStr);                    //Acclima Node_E (1.5
  strcat(idStr, " build ");                 //Acclima Node_E (1.5 build

  itoa(facInfo.verFW_bld, tmpStr, 10);

  strcat(idStr, tmpStr);                    //Acclima Node_E (1.5 build 200
  strcat(idStr, ")");                       //Acclima Node_E (1.5 build 200)

  idx = strlen(idStr);
  idStr[idx] = facInfo.flavor;
  idStr[idx + 1] = 0;                       //Acclima Node_E (1.5 build 200)A

  strcat(idStr, "; SN ");                   //Acclima Node_E (1.5 build 200)A; SN

  ltoa(facInfo.sernum, tmpStr, 10);

  strcat(idStr, tmpStr);                    //Acclima Node_E (1.5 build 200)A; SN 1024
  strcat(idStr, "; radioOp ");              //Acclima Node_E (1.5 build 200)A; SN 1024; radioOp

  idx = strlen(idStr);
  idStr[idx] = facInfo.optsRadio;
  idStr[idx + 1] = 0;                       //Acclima Node_E (1.5 build 200)A; SN 1024; radioOp D

  strcat(idStr, "; UI ");                   //Acclima Node_E (1.5 build 200)A; SN 1024; radioOp D; UI

  idx = strlen(idStr);
  idStr[idx] = facInfo.verUI;
  idStr[idx + 1] = 0;                       //Acclima Node_E (1.5 build 200)A; SN 1024; radioOp D; UI 2

  strcat(idStr, "; ");                      //Acclima Node_E (1.5 build 200)A; SN 1024; radioOp D; UI 2;

  mfgDateToStr(facInfo.mfgDate, tmpStr);

  strcat(idStr, tmpStr);                    //Acclima Node_E (1.5 build 200)A; SN 1024; radioOp D; UI 2; 01-Apr-2019

  free(tmpStr);
}

//Pass in a 16-bit number AND a pointer to char with 12 bytes pre-allocated
void mfgDateToStr(uint16_t dateNumIn, char* dateStrOut) {
  uint16_t year;
  uint8_t month;
  uint8_t day;

  year = ((dateNumIn & 0xFE00) >> 9) + 2000;
  month = ((dateNumIn & 0x01E0) >> 5);
  day = (dateNumIn & 0x1F);

  char* tmpStr = (char*)malloc(4);

  itoa(day, tmpStr, 10);
  if (day < 10) {
    strcpy(dateStrOut, "0");
    strcat(dateStrOut, tmpStr);
  } else {
    strcpy(dateStrOut, tmpStr);
  }

  strcat(dateStrOut, "-");
  numToMonthStrCat(month, dateStrOut);
  strcat(dateStrOut, "-");
  itoa(year, tmpStr, 10);
  strcat(dateStrOut, tmpStr);
  free(tmpStr);
}

void numToMonthStrCat(uint8_t monthValIn, char* dateStrOut) {
  switch (monthValIn) {
    case 1:
      strcat(dateStrOut, "Jan");
      break;
    case 2:
      strcat(dateStrOut, "Feb");
      break;
    case 3:
      strcat(dateStrOut, "Mar");
      break;
    case 4:
      strcat(dateStrOut, "Apr");
      break;
    case 5:
      strcat(dateStrOut, "May");
      break;
    case 6:
      strcat(dateStrOut, "Jun");
      break;
    case 7:
      strcat(dateStrOut, "Jul");
      break;
    case 8:
      strcat(dateStrOut, "Aug");
      break;
    case 9:
      strcat(dateStrOut, "Sep");
      break;
    case 10:
      strcat(dateStrOut, "Oct");
      break;
    case 11:
      strcat(dateStrOut, "Nov");
      break;
    case 12:
      strcat(dateStrOut, "Dec");
      break;
    default:
      strcat(dateStrOut, "MMM");
      break;
  }
}
//===================================================================================================
//===================================================================================================
//===================================================================================================

//------------- Menu Routine ------------------------------------------------------------------------

void menu()
{
  bool userIn = false;
  byte toDo;
  
  if (Serial.available() > 0)
  {
    Serial.read();       // clear serial input buffer
  }

  depthToStruct();
  EEPROM.get(EEPROM_PROJECTID, projectID);

  if (!radioSwitch) {
    radioID = EEPROM.read(EEPROM_DEFAULT_RADIO);
  } else {
    radioID = EEPROM.read(EEPROM_ACTIVE_RADIO);
  }

  GatewayID = EEPROM.read(EEPROM_GATEWAYID);
  interval = EEPROM.read(EEPROM_ALRM1_INT);
  gatewayPresent = EEPROM.read(EEPROM_GW_PRESENT);
  delay(50);

  battV = calcbattV();

  //  Serial.println();
  Serial.println(F("Soil Water Data Network - Wireless SDI-12 Datalogger Node"));               // print out board info
  Serial.print(F("Version "));
  Serial.println(VERSION);
  Serial.println();
  Serial.print(F("Serial Number: "));
  Serial.println(facInfo.sernum);
  Serial.print(F("LoRa Radio Freq (MHz): "));
  Serial.println(LoRaFREQ);
  Serial.print(F("Project ID: "));
  Serial.println(projectID);
  Serial.print(F("Radio ID: "));
  Serial.println(radioID);
  Serial.print(F("Gateway Radio ID: "));
  if (gatewayPresent) {
    Serial.println(GatewayID);
  } else {
    Serial.println("-");
  }
  Serial.print(F("Measurement Interval: "));
  Serial.println(String(interval) + " mins");

  delay(50);
  readClock();

  Serial.print(F("Current date & time: "));
  Serial.print(mnths);                               // date
  Serial.print('-');
  Serial.print(days);
  Serial.print('-');
  Serial.print(yrs);
  Serial.print(' ');
  Serial.print(hrs);                                 // time
  Serial.print(':');
  if (mins < 10)
  {
    Serial.print('0');
  }
  Serial.print(mins);
  Serial.print(':');
  if (secs < 10)
  {
    Serial.print('0');
  }
  Serial.print(secs);
  Serial.println(" UTC");
  //  Serial.println(TMZ);
  Serial.print(F("Current battery voltage:  "));
  Serial.print(battV);
  Serial.println(" V");

  if (battV <= lowBatt) {
    Serial.println("WARNING: BATTERY VOLTAGE TOO LOW!! PLEASE CHARGE!!");
  }

  Serial.println();

  Serial.println(F("Menu options: "));
  Serial.println(F("   0  <--  Enter configuration string"));       // NEW 24-Feb-2020: enter variables all at once
  Serial.println(F("   d  <--  Enter sensor depths"));              // 4-Mar-2020
  Serial.println(F("   c  <--  Set clock"));                        // change month, day, year, hour, minute
  Serial.println(F("   i  <--  Enter project ID"));                 // enter project ID
  Serial.println(F("   g  <--  Enter Gateway radio ID"));
  Serial.println(F("   r  <--  Change Node radio ID"));
  Serial.println(F("   m  <--  Set measurement interval"));         // choose how often to take measurements from sensors
  Serial.println(F("   n  <--  Scan for devices on SDI-12 bus"));   // added 01/13/2020
  Serial.println(F("   a  <--  Change SDI-12 sensor addresses"));
  Serial.println(F("   t  <--  Test sensors"));                     // takes three measurements from sensors
  //  Serial.println(F("   S  <--  Synchronize Gateway & Node clocks"));  // get time from Gateway, update clock
  Serial.println(F("   p  <--  Print data to screen"));         // print data to Serial Monitor
  Serial.println(F("   e  <--  Erase all data"));                   // delete all data from Flash
  Serial.println(F("   b  <--  Turn debug statements on/off"));    
  Serial.println(F("   x  <--  Exit menu"));                        // exit
  Serial.print(F("Enter choice: "));
  byte   menuinput;                                    // user input to menu prompt
  long  timeout;                                      // length of time to wait for user

  timeout = millis(); // + 30000;                                      // wait 30 secs for input, edited 01/13/2020
  while ((millis() - timeout) < 30000)
  {
    menuinput = 120;
    if (Serial.available() > 0)                                    // if something typed, go to menu
    {
      //      userinput = true;                        // added 01/13/2020
      menuinput = Serial.read();               // get user input
      Serial.println(char(menuinput));
      while (Serial.available() > 0)
      {
        Serial.read();
      }
      break;
    }
  }

  switch (menuinput)
  {
    case 48:                    // ------ 0 - Enter config string ---------------------------------------------
      Serial.println();
      Serial.println(F("REMINDER: Use '+' or '-' to indicate above or below soil surface for depths!"));
      Serial.print(F("Enter configuration string: "));
      Serial.println();
      charinput();
      if (charInput[0]) {
        decodeConfig(charInput);
      }
      Serial.println();
      delay(500);

      menu();
      break;

    case 100: case 68:           // ------ d - Enter sensor depths ---------------------------------------------
      clearDepths();    // 28May2020

      Serial.println();
      if (skipScan) {
        SDI12Scan();
      }

      Serial.println(F("Enter sensor depth in cm. Use '+' or '-' to indicate above or below soil surface:"));

      for (byte i = 0; i < sensorTot; i++) {
        depths[i][0] = allSensors[i].addr;

        Serial.print(allSensors[i].addr);
        Serial.print(": ");
        charinput();

        if (charInput[0]) {
          while (charInput[0] != '-' && charInput[0] != '+') {
            Serial.println("ERROR: Use '+' or '-' to indicate above or below soil surface");
            Serial.print(allSensors[i].addr);
            Serial.print(": ");
            charinput();
          }
          userIn = true;
          strcpy(allSensors[i].depth, charInput);

          for (byte j = 0; j < 7; j++) {  // populate depths array (for saving to EEPROM)
            if (charInput[j]) {
              depths[i][j + 1] = charInput[j]; delay(8);
            } else {
              depths[i][j + 1] = 0;
              break;
            }
          }
        }
      }

      Serial.println();
      if (userIn) {
        EEPROM.put(EEPROM_DEPTHS, depths);
      } else {};
      delay(500);

      menu();
      break;


    case 99: case 67:           // ------ c - Set clock ---------------------------------------------

      setRTCTime();
      Serial.println();
      menu();
      break;

    case 'N': case 'n':         // ------ n - Scan for SDI-12 devices ---------------------------------------------
      SDI12Scan();
      menu();
      Serial.println();
      break;

    case 'g': case 'G':          // ---------- g - Enter Gateway radio ID ---------------------------------------------------

      setIDs();
      menu();                                      // go back to menu
      Serial.println();
      break;


    case 114: case 82:         // ---------- r - Change node radio ID from default ---------------------------------------------------
      changeDefault();
      menu();
      Serial.println();
      break;

    case 109: case 77:         //--------- m - Set measurement interval ----------------------------

      measureInt();
      menu();
      Serial.println();
      break;

    /*case 83:   //--------- S - Synchronize Gateway & Node clocks ----------------------------

      Serial.println();
      Serial.println(F("Waiting for time from Gateway..."));
      syncTime();
      delay(500);
      menu();
      break;
    */
    case 97: case 65:           // ------ a - Change sensor address ---------------------------------------------

      sensorAddress();
      Serial.println();
      //      delay(500);
      menu();
      break;

    case 116: case 84:          // ------ t - Test sensors ---------------------------------------------

      Serial.println(F("Test measurements:"));     // take 3 readings to check sensors
      //      Serial.println();
      delay(10);
      if (skipScan) {
        SDI12Scan();
      }

      for (byte g = 1; g < 4; g++) {
        readClock();
        Timestamp();         // compile timestamp
        delay(50);
        readSensors();           // read all sensors
        compileData();
        //        Serial.println(allData);
        delay(200);
      }
      delay(500);
      Serial.println();
      menu();
      break;

    case 112: case 80:          // ------ p - Print node data to screen ---------------------------------------------

      Serial.println(F("Print data to screen: "));     // download all data in Flash
      delay(100);
      Serial.println(F("  a --> Print all the data"));
      Serial.println(F("  t --> Print latest logs"));
      Serial.print(F("Enter choice: "));
      charinput();

      if (charInput[0] == 'a' || charInput[0] == 'A') {
        ft.printData();
      } else if (charInput[0] == 't' || charInput[0] == 'T') {
        /*uint32_t lastLog = ft.logId() - 1;
        //        Serial.println(lastLog);
        uint32_t toLog;
        if (lastLog > 10) {
          toLog = lastLog - 10;
        } else {
          toLog = 0;
        }
        for (uint32_t j = lastLog; j > toLog; j--) {
          ft.printLog(ft.logAddrFromId(j));
        }*/
        printLatestLogs();
      } else {
        Serial.println();
        Serial.print(F("Invalid input. Returning to menu.")); delay(1000);
      }
      Serial.println();

      menu();
      break;

    case 101: case 69:          // ------ e - Erase Flash ---------------------------------------------

      Serial.print(F("Are you sure you want to erase all data stored in memory? (y/n): "));
      toDo = yesNo();
    
      if (toDo == 1) {
        Serial.print("Erasing data...");
        ft.eraseLogs();
        Serial.println(" done.");
        ft.logSetup(logsBeginAddr, logsEndAddr, LOG_minSize, LOG_maxSize, true);
      }

      Serial.println();
      menu();
      break;

    case 'b': case 'B':          // ------ b - Turn debug on/off ----------------------------------------------
      if(debug){
        Serial.println(F("Turn debug statements off? (y/n): "));
        toDo = yesNo();
        if (toDo == 1){
          debug = false;
          EEPROM.update(EEPROM_DEBUG,debug);
          Serial.println(F("Debug statements off"));
        } else if (toDo == 2){
          Serial.println(F("Debug statements on"));
        }
      } else {
        Serial.println(F("Turn debug statements on? (y/n): "));
        toDo = yesNo();
        if (toDo == 1){
          debug = true;
          Serial.println(F("Debug statements on"));
        } else if (toDo == 2){
          Serial.println(F("Debug statements off"));
        }         
      }
      EEPROM.update(EEPROM_DEBUG,debug);
      Serial.println();
      delay(500);
      menu();
      break;
 
    case 120: case 88:          // ------ x ----------------------------------------------
      Serial.println(F("Exit"));                           // exit
      Serial.println();
      delay(10);
      break;

    case 'i': case 'I':         // ------ i - Enter project ID ---------------------------------------------
      Serial.println();
      Serial.print(F("  Enter a Project ID (up to 5 characters, ex: PSA): "));    // set project ID
      charinput();

      if (charInput[0]) {
        byte i = 0;
        for (i = 0; i < 5; i++) {
          if (charInput[i] != 0) {
            projectID[i] = charInput[i];
          } else {
            break;
          }
        }
        projectID[i] = 0;

        EEPROM.put(EEPROM_PROJECTID, projectID);
        delay(10);
      }

      Serial.println();
      delay(500);

      menu();
      break;
  }

  //  if (userinput == true) {
  //    digitalWrite(LED, LOW);
  //  }
}

//======================================================================================
//======================================================================================
//======================================================================================

//-------------- Decode config string --------------------------------

void decodeConfig(char config_string[200]) {
  /* Config string format: projectID,sernum,radio ID,gateway ID,measInt,sensorTot,addr1,depth1,...
    ** gateway ID == '0' --> no gateway
  */
  clearDepths();  // 28May2020

  char buf[200];

  for (byte i = 0; i < 200; i++) {
    buf[i] = config_string[i];   // put config string into a buffer
  }

  bool configOK = true;
  byte configSize;

  for (byte i = 0; i < 200; i++) {
    if (buf[i] == 0) {              // find the length of the config string
      configSize = i + 1;
      break;
    }
  }

  char configN[configSize];           // create new array of correct size
  for (byte i = 0; i < configSize; i++) {
    configN[i] = buf[i];
  }

  //--- Find comma positions

  uint8_t commaPos[36];
  byte commaTot = 0;
  for (byte i = 0; i < configSize ; i++) {
    if (configN[i] == ',') {
      commaPos[commaTot] = i;
      commaTot++;
    }
  }
  commaTot++; // add virtual comma at end of string
  commaPos[commaTot - 1] = sizeof(configN);

  //--- Check for project ID
  byte firstComma = 1;

  if (commaPos[0] > 5) {
    Serial.println("ERROR: Project ID missing!");
    configOK = false;
  }

  if (configOK) {
    byte i = 0;
    for (i = 0; i < commaPos[0]; i++) {
      projectID[i] = configN[i];
    }
    projectID[i] = 0;
    Serial.print(F("projectID: "));
    Serial.println(projectID);

    //--- Match serial num

    uint8_t SerNum[8];

    for (byte i = 0; i < 8; i++) {
      SerNum[i] = configN[i + commaPos[0] + 1] - 48;
      //        Serial.print(SerNum[i]);
    }

    uint32_t serNumCheck = 0;

    for (byte i = 0; i < 8; i++) {
      long baseTen = 1;
      for (byte j = 0; j < (7 - i); j++) {
        baseTen = baseTen * 10;
      }
      serNumCheck += (SerNum[i] * baseTen);
    }

    Serial.println(serNumCheck);

    if (serNumCheck != serNum) {
      Serial.println("ERROR: Serial numbers do not match!");
      configOK = false;
    }

    //--- get radio ID

    if ((commaPos[firstComma + 1] - commaPos[firstComma + 0]) == 2) {  // radio ID is one digit
      radioID = configN[commaPos[firstComma + 0] + 1] - 48;
    } else {
      radioID = 10 * (configN[commaPos[firstComma + 0] + 1] - 48) + (configN[commaPos[firstComma + 1] - 1] - 48);
    }

    Serial.print("radio ID: ");
    Serial.println(radioID);

    //--- get Gateway ID

    if ((commaPos[firstComma + 2] - commaPos[firstComma + 1]) == 2) {  // radio ID is one digit
      GatewayID = configN[commaPos[firstComma + 1] + 1] - 48;
    } else {
      GatewayID = 10 * (configN[commaPos[firstComma + 1] + 1] - 48) + (configN[commaPos[firstComma + 2] - 1] - 48);
    }

    Serial.print("Gateway ID: ");
    
    if (GatewayID != 0) {       // allow user to enter '0' if not using gateway
      gatewayPresent = true;
      Serial.println(GatewayID);
    } else {
      gatewayPresent = false;
      Serial.println("N/A");
    }


    //--- Get measurement interval

    interval = 10 * (configN[commaPos[firstComma + 2] + 1] - 48) + (configN[commaPos[firstComma + 2] + 2] - 48);

    Serial.print("Measurement interval: ");
    Serial.println(interval);

    if (interval != 2 && interval != 10 && interval != 15 && interval != 20 && interval != 30 && interval != 60) {
      Serial.println(F("ERROR: Invalid interval (every 10, 15, 20, 30, or 60 mins)"));
      configOK = false;
    }

    //--- Get number of sensors

    byte sensorNum;

    if ((commaPos[firstComma + 4] - commaPos[firstComma + 3]) == 2) {
      sensorNum = configN[commaPos[firstComma + 3] + 1] - 48;
    } else {
      sensorNum = 10 * (configN[commaPos[firstComma + 3] + 1] - 48) + (configN[commaPos[firstComma + 4] - 1] - 48);
    }

    Serial.print("Number of sensors: ");
    Serial.println(sensorNum);

    //--- Get sensor addresses and depths

    for (byte r = 0; r < sensorNum; r++) {
      byte L = 4 + 2 * r; // comma before address
      byte R = 4 + 2 * r + 1; // comma after address
      byte x = commaPos[firstComma + R + 1] - commaPos[firstComma + R];
      //     Serial.print(commaPos[firstComma + R]);
      //     Serial.print(commaPos[firstComma + L]);
      //     Serial.print(x);

      if (r == (sensorNum - 1) && R != (commaTot - 3)) {    // 9/24/2020: check for correct number of sensors first
        //      Serial.println(R);
        //      Serial.println(commaTot);
        Serial.println("ERROR: Incorrect number of sensor depths");
        configOK = false;
      } else {
        depths[r][0] = configN[commaPos[firstComma + L] + 1];
        Serial.print(depths[r][0]);
        allSensors[r].addr = depths[r][0];

        bool missingSign = false;
        char dpth[6];
        byte dLen = 0;
        for (byte c = 1; c < x; c++) {
          char digit = configN[commaPos[firstComma + R] + c];
          if (c == 1 && digit != '-' && digit != '+') {
            missingSign = true;
            configOK = false;
          } else {
            if ((digit > 47 && digit < 58) || digit == '-' || digit == '+' || digit == '.') {
              depths[r][c] = digit;
              dpth[c - 1] = digit;
              dLen++;
            }
          }
          Serial.print(depths[r][c]);
        }
        Serial.println();
//        Serial.print("dLen: ");
//        Serial.println(dLen);
        depths[r][x] = 0;
        dpth[x - 1] = 0;
        strcpy(allSensors[r].depth, dpth);
        //                Serial.println(allSensors[r].addr);
        //        Serial.println();
        //        Serial.println(allSensors[r].depth);
        for (byte j = 0; j < 6; j++) {
          //          Serial.print(allSensors[r].depth[j],DEC);
        }

        Serial.println();
        if (missingSign) Serial.println(F("ERROR: Missing +/- before depth!"));
        delay(5);
      }
    }
  }

  if (!configOK) {
    Serial.println("!!! FAIL: Invalid configuration !!!");
  } else {
    Serial.println("Configuration complete");
    EEPROM.put(EEPROM_PROJECTID, projectID);
    EEPROM.update(EEPROM_GATEWAYID, GatewayID);
    EEPROM.update(EEPROM_GW_PRESENT, gatewayPresent);
    if (radioID != default_radioID) {
      radioSwitch = 1;
      EEPROM.update(EEPROM_ACTIVE_RADIO, radioID);
    } else {
      radioSwitch = 0;
    }
    EEPROM.update(EEPROM_FLAG_RADIO, radioSwitch);
    LoRa.setThisAddress(radioID);
    EEPROM.update(EEPROM_ALRM1_INT, interval);
    EEPROM.put(EEPROM_DEPTHS, depths);
  }
  delay(400);   // Added 9/24/2020
}

void setRTCTime() {

  Serial.println(F("Enter the date and time in Coordinated Universal Time (UTC): "));

  Serial.print(F("  input month: "));
  getinput();
  mnths = indata;
  tm.Month = mnths;
  Serial.print(F("  input day: "));
  getinput();
  days = indata;
  tm.Day = days;
  Serial.print(F("  input year: "));
  getinput();
  yrs = indata;
  tm.Year = yrs - 1970;
  Serial.print(F("  input hour: "));
  getinput();
  hrs = indata;
  tm.Hour = hrs;
  Serial.print(F("  input minute: "));
  getinput();
  mins = indata;
  tm.Minute = mins;

  if (indata) {
    RTC.write(tm);
  }
  delay(50);
}

void setIDs() {
  Serial.println();
  Serial.print(F("Are you using a Gateway? (y/n): "));

  byte toDo = yesNo();
  if (toDo == 1){ 
    gatewayPresent = true;   
    if (gatewayPresent == 1) {
      Serial.print(F("    Enter Gateway radio ID: "));
      getinput();
      GatewayID = indata;
      EEPROM.update(EEPROM_GATEWAYID, GatewayID);
      delay(10);
    }
  }
  else if (toDo == 2){
    gatewayPresent = false;
    Serial.println(F("Data logging only mode"));
  }
  EEPROM.update(EEPROM_GW_PRESENT, gatewayPresent);
  delay(500);
}

void changeDefault() {
  Serial.println();
  Serial.print(F("Would you like to change the radio ID from the default value? (y/n): "));
  byte toDo = yesNo();
  if (toDo == 1){
    radioSwitch = true;
    EEPROM.update(EEPROM_FLAG_RADIO, radioSwitch);
      Serial.print(F(" Enter new radio ID: "));
      getinput();
      radioID = indata;
      EEPROM.update(EEPROM_ACTIVE_RADIO, radioID);
      Serial.println(F("Radio ID updated"));
    } else {
      radioID = default_radioID;
      Serial.println(F("Default radio ID accepted"));
    }
    LoRa.setThisAddress(radioID);
    Serial.println();
}

void measureInt() {
  Serial.println();
  Serial.print(F("Enter measurement interval (every 10, 15, 20, 30, or 60 mins): "));
  Serial.flush();
  getinput();
  if (indata) {
    while (indata != 2 && indata != 5 && indata != 10 && indata != 15 && indata != 20 && indata != 30 && indata != 60) {
      Serial.print(F("Invalid interval. Enter measurement interval (every 10, 15, 20, 30, or 60 mins): "));
      Serial.flush();
      getinput();
    }

    while (RHPresent && indata < 15) {
      Serial.println(F("ERROR: minimum 15 minute interval required with RH sensor."));
      Serial.print(F("Enter new measurement interval(every 10, 15, 20, 30, or 60 mins): "));
      getinput();
    }

    interval = indata;
    EEPROM.update(EEPROM_ALRM1_INT, interval);
  }
}

void printDepths() { // added 28May2020
  //  EEPROM.get(EEPROM_DEPTHS, depths); delay(100);
  Serial.println("Printing depths...");
  for (byte r = 0; r < sensorNum; r++) {
    for (byte c = 0; c < 10; c++) {
      Serial.print(depths[r][c]);
      delay(5);
    }
    Serial.println();
  }
  Serial.println();
}

void clearDepths() {  // added 28May2020
  EEPROM.get(EEPROM_DEPTHS, depths); delay(100);
  memset(depths, 0, sizeof(depths));

}

void printLatestLogs(){
  uint32_t lastPrint;
  lastPrint = EEPROM.read(EEPROM_LASTPRINT);
  uint32_t lastLog = ft.logId();

  uint32_t i;
  if (lastPrint != 0){
    i = lastPrint;
  } else {
    i = 0;
  }
  
  for (i; i < lastLog; i++){
    ft.printLog(ft.logAddrFromId(i));
    delay(10);
  }

  EEPROM.update(EEPROM_LASTPRINT,lastLog);
  
}

//===================================================================================================
//===================================================================================================
//===================================================================================================

//------------- Get User Input ----------------------------------------------------------------------

void getinput()
{
  long  timeout;                                      // length of time to wait for user
  timeout = millis() + 10000;         // time to wait for user to input something

  byte numincoming;
  int input;

  indata = 0;                                      // initialize
  while (millis() < timeout)                       // wait for user to input something
  {
    if (Serial.available() > 0)                    // something came in to serial buffer
    {
      delay(100);                                  // give time for everything to come in
      numincoming = Serial.available();            // number of incoming bytes
      for (byte i = 1; i <= numincoming; i++)           // read in everything
      {
        incoming[i] = Serial.read();               // read from buffer
        input = incoming[i] - 48;                // convert ASCII value to decimal
        indata = indata * 10 + input;            // assemble to get total value
      }
      Serial.println(indata);
      break;                                       // exit before timeout
    }
  }
  delay(10);

}

//======================================================================================

//-------------- Get User Input for character variables --------------------------------

void charinput() {
  memset(charInput, 0, sizeof(charInput)); delay(50);
  long  timeout;                                      // length of time to wait for user
  timeout = millis() + 10000;         // time to wait for user to input something

  byte numincoming;

  //indata = 0;                                      // initialize
  while (millis() < timeout)                       // wait for user to input something
  {
    if (Serial.available() > 0)                    // something came in to serial buffer
    {
      delay(100);                                  // give time for everything to come in
      numincoming = Serial.available();            // number of incoming bytes
      //             Serial.println(numincoming);
      for (byte i = 0; i <= numincoming; i++)           // read in everything
      {
        incomingChar[i] = Serial.read();
        if (incomingChar[i] == 13 || incomingChar[i] == 10)   // ignore CR or LF
        {
        }
        else                                       // otherwise
        {
          charInput[i] = incomingChar[i];
        }
      }
      charInput[numincoming] = 0;
      break;
    }
  }
  Serial.println(charInput);
}


//======================================================================================

//-------------- Change sensor address -------------------------------------------------

void sensorAddress() {
  
  SDI12port.begin();
  delay(500);
  boolean found = false;
  if (skipScan) {
    SDI12Scan();
  }

  SDI12port.begin();
  delay(500);

  for (byte j = 0; j < sensorTot; j++) {
    char oldAddr = allSensors[j].addr;
    Serial.print(F("Current address: "));
    Serial.println(oldAddr);

    Serial.print("Enter new address: ");                             // prompt for a new address
    while (!Serial.available());
    char newAdd = Serial.read();

    // wait for valid response
    while ( ((newAdd < '0') || (newAdd > '9')) && ((newAdd < 'a') || (newAdd > 'z')) && ((newAdd < 'A') || (newAdd > 'Z'))) {
      if (!(newAdd == '\n') || (newAdd == '\r') || (newAdd == ' ')) {
        Serial.println("Not a valid address. Please enter '0'-'9', 'a'-'A', or 'z'-'Z'.");
      }
      while (!Serial.available());
      newAdd = Serial.read();
    }
    Serial.println(newAdd);
    if (!changeAddress(oldAddr, newAdd)) Serial.println(F("Address change failed")); else Serial.println(F("Success"));
    SDI12port.clearBuffer(); delay(30);
  }

  SDI12port.end();
  delay(500);

}

bool changeAddress(char oldA, char newA) {
  char oldAddress = oldA;
  char newAddress = newA;
  String myCommand = "";
  bool success = false;
//  SDI12port.clearBuffer();
  
  Serial.print("Readdressing sensor...");
  myCommand = "";
  myCommand += oldAddress;
  myCommand += "A";
  myCommand += newAddress;
  myCommand += "!";
//  Serial.println(myCommand);
  
  SDI12port.sendCommand(myCommand); delay(400);
  
  byte numIn = SDI12port.available(); 
//  Serial.println(numIn);
  char resp[numIn]; 

  for(byte a = 0; a < numIn; a++){
    char b = SDI12port.read();
    resp[a] = b;
//    Serial.print(resp[a],DEC);
  }
  delay(20);
//  Serial.println();
  
//  Serial.flush();
  
  Serial.println(resp[0]);
  
  if(resp[0] == newAddress){
    success = true;
    for (byte i = 0; i < sensorTot; i++) {    // find sensor in struct and update info
    if (oldAddress == allSensors[i].addr) {
      updateInfo(newAddress, oldAddress);
      SDI12port.clearBuffer();     
     }
    }
  } else {
    success = false;
  }
  SDI12port.clearBuffer(); delay(30);
  if(success == true) return true; else return false;
  
}

void updateInfo(char a, char old) {
  char addr = a;
  char oldA = old;

  // Populate object with metadata
  for (byte i = 0; i < sensorTot; i++) {    // find sensor in struct and update info
    if (oldA == allSensors[i].addr) {
      allSensors[i].addr = addr;
      allSensors[i].ID[0] = addr;
      allSensors[i].cmd[0] = addr;
      delay(20);
      //        Serial.println(allSensors[i].addr);
      //        Serial.println(allSensors[i].ID);
      //        Serial.println(allSensors[i].cmd);
    }
    if (oldA == depths[i][0]) {      // update addresses is depths array
      addr = depths[i][0];
    }
  }

  depthToStruct();

}

byte yesNo(){
  charinput();
  if(charInput[0]){
    if (charInput[0] == 'y' || charInput[0] == 'Y') {
      return 1;
    } 
    else if (charInput[0] == 'n' || charInput[0] == 'N') {
      return 2;
    } else {
      Serial.println(F("Invalid response"));
      return 3; 
    }
  } else {
    Serial.println(F("No user response"));
    return 3;
  }
}
