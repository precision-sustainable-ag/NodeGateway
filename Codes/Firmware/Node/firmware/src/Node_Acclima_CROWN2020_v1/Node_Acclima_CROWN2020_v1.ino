/* CROWN-V2-AIT Node --> Edits for CROWN 2020 that have not been merged into main branch

  Acclima Cooperative - Node for use with Cellular Gateway
    Components:
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

   Last edited: August 4, 2020

   Version 2020.05.06 fixes issue with data string when a sensor is unresponsive
   Version 2020.05.08 changes timeToWait variable in listenRespond() to prevent Node from timing out of
                      initial sync mode before Gateway does
   Version 2020.05.14 fine tunes initial synchronization with Gateway and daily sync check
   Version 2020.05.19 fixes problem with resetting alarm after daily sync check
   Version 2020.05.28 removes #ifdef CONT_MEAS statements, fixes issue with repeated sensor measurements after using 
                      "t" menu option, removes changing depths array during sensor scan to avoid pairing addresses with 
                      wrong depths
                      Add travel time to TDR sensor output
   Version 2020.06.04 fixes -999 for all sensor depths (uncommented recalling depths array from memory)
                      fix menu option "d" to include sensor addresses in depths array
   Version 2020.06.22 limits loss of data when measurement int = 15 min, and N gets garbled timestamp at measurement interval
                      following daily handshake with G (doesn't fix the garbled timestamp)                 
                      adds if(nYear - 2000 > 0) in listenRespond()
   Version 2020.07.28 Adds radio frequency to menu header
   Version 2020.08.04 Removes if(buf[0] == GatewayID) in fieldSync --> rejects transmissions if GatewayID = 50 b/c timestamp starts with char "2" = dec 50
*/

//===================================================================================================

//------------ Libraries --------------------------------------

#include "AcclimaSDI12.h"
#include <SPI.h>                                      // SPI functions for Flash chip
#include <Wire.h>                                     // I2C functions for RTC
#include <EEPROM.h>                                   // built-in EEPROM routines
#include <avr/sleep.h>                                // sleep functions
#include <DS3232RTC.h>                                // RTC library
//#include "Flash_Func_AITRev.h"                        // replaces SPIFlash lib
#include "FlashTools.h"
#include <ACReliableMessage.h>                        // Acclima edits of RadioHead library
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>                                  // controls watchdog timer (we want to turn it off)
#include <stdio.h>
#include "SetSpeed.h"
#include "GlobalDefs.h"

//------------- Assign Pins --------------------------------------

#define SDI12Data 10                                 // Pin for SDI-12 sensors; handles interrupts
#define SDI12Pwr 0                                   // Switches power to SDI-12 sensors on/off
#define LED 15                                       // LED on D15
#define Flash_SS 23
#define baudRate 57600                               //John, 27-Mar-2019: Changed from 115200 to 57600 because when MCU at 8 MHz the sampling resolution is reduced
#define pin_SDIv12 1                                 //pin 41 --> D1  -- switch SDI-12 power supply to 12 volts (vs. 7.5)
#define pin_solarVoltage    A5
#define ADC_REF_VOLTAGE     3.3
#define ADC_RESOLUTION      1024
#define pin_solarShort      A4
#define SOLAR_CALIB      1.0          //This will become a EEPROM constant that is set during factory config – for now just use 1.0
#define MAX_SENSORS 16
#define getTT
#define keepPEC

  char VERSION[] = "V2020.08.04";


//------------- Declare Variables ---------------------------------


//-----*** Identifiers ***-----

char  projectID[6];         // 17-Mar-2020: use project ID to filter incoming data from multiple devices
uint8_t  radioID;                                    // node radio ID
uint8_t  GatewayID;                                  // Gateway radio ID
uint8_t  default_radioID;
FactoryInfo facInfo;      // pull in factory info (serial num, etc) - see GlobalDefs.h


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

#define EEPROM_SERIALNUM            10
#define EEPROM_OPTSRADIO            17
#define EEPROM_verHW                16  //char 
#define EEPROM_optsRadio            17  //char

//The following is a field/variable, NOT a EEPROM location
bool  gatewayPresent = true;

//The following value is determined by reading EEPROM constant (see method getLoRaFreq())
uint16_t LoRaFREQ;

uint32_t timeSync = 7200000; // 14May 2 hours
//uint32_t timeSync = 900000; // 14May for testing 900000 = 15 minutes
uint32_t fieldSyncStart;
uint32_t fieldSyncStop;

//-----*** for Flash ***-----

uint32_t logsBeginAddr = LOG_beginAddr;
uint32_t logsEndAddr = LOG_endAddr;

const uint32_t IDaddr = 0x001000;   // we will write a list of IDs here (first sector)
uint32_t lastIDaddr;                // This shows where the ID List ends

uint32_t WAC = 0;           // This is our Write Access Counter -- keeps track of how many things we write to Flash
// Initialize to zero if no record found.  Increment before write.  Persistant - save to log space after erase.
// This should never be written as 0 or 0xFFFFFFFF.
const uint32_t WAC_Sync_Range = 0x7FFFFFFF;   // if the desired WAC is greater by this amount, consider uploading everything


//-----*** for Main Menu ***-----

int   indata;                                       // user input data
int   incoming[7];
char  incomingChar[200];
char  charInput[200];
boolean firsttime = true;
unsigned long serNum;
bool skipScan = false;

//-----*** Data Variables ***-----

//-- SDI-12 addresses and metadata

char oldAddress = '!';
char RHsensor;
char CS655addr;
char activeSDI12[MAX_SENSORS + 1];                                // array holding active sensor addresses (up to 16 sensors)
byte registerSDI12[MAX_SENSORS] = {                          // holds addresses of active SDI12 addresses
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
  0B00000000,
};

String sensorIDs = "";
char sensorList[MAX_SENSORS][36];   // added 4-Mar-2020: 2D array for holding sensor IDs to act as lookup table
byte sensorNum = 0;             // added 4-Mar-2020: counter for writing to 2D array
byte rowNum = 0;
char depths[MAX_SENSORS][10];
char depthsScan[MAX_SENSORS];
bool sensorDetected = false;    // added 4-Mar-2020: flag to initiate overwrite of sensor ID memory space

//-- Sensor data

String SDI12data = "";                              // String for compiling data from SDI12 sensors
String allData = "";
int measDelay;                                      // delay for SDI-12 measurement
boolean SDI12response = false;
bool RHPresent = false;
char sep = '~';                                     // data delimiter

char Tsensor;
bool TPresent = false;
bool start_RHT = true;

//-- Onboard Temperature sensor

float  boxTemp;

//-- Battery voltage measurement and calc

float multiplier = 0.00322;                         // scaling factor for analogRead
//float resist = 1.333; // for voltage divider: R1=10k, R2=30k, resist =  (R1+R2)/R2 --> use as local variable instead 13-Jan-2020
float battV;                                        // battery voltage
float lowBatt = 3.4;                                // low battery limit
bool  battLow = false;

//-----*** for Loop ***-----

bool duringInit = true;
boolean dataSent = false;
boolean updated = false;
boolean userinput = false;
//  boolean IDsuccess = false;  // for sending sensor IDs -- function removed 13-Jan-2020


//-----*** for RTC ***-----

//-- Time and Date values

byte   secs;                  // time and date values
byte   mins;
byte   hrs;
byte   days;
byte   mnths;
int    yrs;
char   TMZ[] = "UTC";         // Time Zone

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

#define     TxPower  20     // options: +5 to +23 (default 13)
bool radioSwitch;           // tracks if using default radio address or not
//  unsigned int  radioTimeout = RH_DEFAULT_TIMEOUT;   // changed from 300 ms to 200ms(default)
//  uint8_t  retryNum = 10; //RH_DEFAULT_RETRIES;            // changed from 20 to 3 (default)

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
#define timeoutACK      120     // This is the wait period after a transmission to receive an ACK for that one packet. (ACKWait)
#define timeoutPacket   2000    // this is the wait period to receive one large packet including retries. (PacketWait)
#define timeoutSmallPkt 1000    // Timeout interval for a small packet (i.e. 20 bytes).  Approx Airtime=timeoutAck.

byte numMissed = 0;             // counter for missed communications with Gateway

// ------- Initialize ----------------------------------------------------

// Note:  changed the name of the radio objects to be sure to look at every use
RH_RF95 driverRFM95;                                 // Driver for the RFM95 radio
ACReliableMessage LoRa(driverRFM95, radioID);            // LoRa radio manager

SDI12 SDI12port(SDI12Data);
FlashTools ft;

//SPIFlash flash(Flash_SS, 0xC228);                  // C228 for Macronix 64 Mbit MX25R6435F Flash chip
//  SetSpeed ss;
//===================================================================================================

//------------- Set Up ------------------------------------------------------------------------------

void setup() {
  Serial.begin(baudRate);
  delay(100);
  //  Serial.println("Hello");

  //--- Initialize

  // I2C
  SPI.begin();
  Wire.begin();                                   // begin I2C
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

  //--- Pin Settings

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


  //--- Read variables from EEPROM

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

  if (!ft.logSetup(logsBeginAddr, logsEndAddr, LOG_minSize, LOG_maxSize, true)) {
    Serial.println("Erasing flash...");
    ft.chipErase();
    ft.logSetup(logsBeginAddr, logsEndAddr, LOG_minSize, LOG_maxSize, true);
  }

  //--- Radio settings

  LoRaFREQ = getLoRaFreq();   //John

  if (!LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, radioID)) {                      // initialize radio
    Serial.println("ERROR: radio init failed");
    return;
  }


  //--- Scan SDI-12 Addresses

  while (Serial.available()) {
    Serial.read();
  }
  Serial.println("Enter \"S\" to skip SDI-12 bus scan.");
  Serial.print((char)2);    //STX to indicate to UI software that it needs to send 'S' to skip sensor scan
  Serial.print("\r\n");

  long timeout = millis();
  char skipTest;
  while ((millis() - timeout) < 5000)
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

  //--- Go to menu

  menu();                                        // display menu on startup

  if (skipScan) {                                // scan for sensors if haven't already
    SDI12Scan();
  }

  //----- set alarms -----

  // clear alarm registers

  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 0); // set alarm values to 0
  RTC.setAlarm(ALM2_MATCH_MINUTES, 0, 0, 0, 0);
  RTC.alarm(ALARM_1);                    // turn off alarm 1
  RTC.alarm(ALARM_2);                    // turn off alarm 2
  RTC.alarmInterrupt(ALARM_1, true);     // turn on alarm 1 interrupt
  RTC.alarmInterrupt(ALARM_2, true);     // 27-Jan-2020: turn on alarm 2 interrupt
  RTC.squareWave(SQWAVE_NONE);           // Use SQW pin as interrupt generator


  //--- Time sync in field

  //  if (userinput == false && gatewayPresent == 1) {
  if (gatewayPresent) {
    //      Serial.end();
    delay(50);
    //      digitalWrite(LED,HIGH);
    fieldSync(timeSync); // 14May20 15 minutes
    listenRespond();
    duringInit = false;
    digitalWrite(LED, LOW);
  }

  //--- Set alarm times

  readClock();
  RTC.alarm(ALARM_1);
  resetAlarm(interval);

  RTC.alarm(ALARM_2); // 27-Jan-2020: Use Alarm 2 to wake up for daily sync check
  resetAlarm2();

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
      battLow = true;
      EEPROM.update(EEPROM_LOW_BATT, battLow);
      sleepNow();
    }

    else if (battLow == true || numMissed >= 3) {   // if battery was low but now ok or Node has missed comm w/ G >= 3 consecutive times
      fieldSync(3605000);  // wait for 1 hr and 5 minutes to get timestamp from Gateway
      readClock();
      resetAlarm(interval);
    }

    // scenario 1: No RH sensor, no Gateway (Bare Node w/o G)

    else if (RHPresent == 0 && gatewayPresent == false) {

      //      Serial.println("RHPresent == 0 && gatewayPresent == 0");
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
      //      Serial.println("RHPresent == 1 && gatewayPresent == 0");

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
      //      Serial.println("RHPresent == 1 && gatewayPresent == 1");
      if (mins % interval == 1 ) {
        Serial.println("Sending to Gateway");
        digitalWrite(LED, HIGH);
        listenRespond();
        byte n = interval - 7;
        resetAlarm(n);
        digitalWrite(LED, LOW);
      }
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
      //      Serial.println("RHPresent == 0 && gatewayPresent == 1");
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
  else if (RTC.alarm(ALARM_2)) {  // 27-Jan-2020: add daily sync check routine
    battV = calcbattV();
    if (battV <= lowBatt) {       // if battery low skip loop, go to sleep
      resetAlarm2();
      battLow = true;
      EEPROM.update(EEPROM_LOW_BATT, battLow);
      sleepNow();
    } else {
      fieldSync(90000);    // wait up 1-1/2 minutes for timestamp from Gateway
      resetAlarm2();
    }
  }

}
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


  for (int index = 0; index < 9; index++) activeSDI12[index] = '\0'; delay(50);

  for (byte i = '0'; i <= '9'; i++) if (checkActiveSDI12(i)) setActiveSDI12(i); // scan address space 0-9

  for (byte i = 'a'; i <= 'z'; i++) if (checkActiveSDI12(i)) setActiveSDI12(i); // scan address space a-z

  for (byte i = 'A'; i <= 'Z'; i++) if (checkActiveSDI12(i)) setActiveSDI12(i); // scan address space A-Z

  delay(20);



  // scan address space 0-9
  for (char i = '0'; i <= '9'; i++) {
    if (isActiveSDI12(i)) {
      compileInfoSDI12(i);

    }
  }

  // scan address space a-z
  for (char i = 'a'; i <= 'z'; i++) {
    if (isActiveSDI12(i)) {
      compileInfoSDI12(i);
    }
  }

  // scan address space A-Z
  for (char i = 'A'; i <= 'Z'; i++) {
    if (isActiveSDI12(i)) {
      compileInfoSDI12(i);
    }
  }

  //    if(sensorDetected){    // added 4-Mar-2020: rewrite sensor mem space only if a sensor detected (keep previous IDs stored)
  saveSDI12Info();
  //    }

  delay(50);

  //    digitalWrite(SDI12Pwr, LOW);  // turn off SDI12 sensors

  Serial.println();
  Serial.println();
  Serial.println(F("Done"));
  delay(50);
  SDI12port.end();

//  bool newSensors = false;    // removed 28May2020


/*  printDepths();  // 28May2020

//     for(byte r = 0; r < sensorNum; r++){
//      for(byte c = 0; c < 10; c++){
//        Serial.print(depths[r][c]);
//      }
//      Serial.println();
//     }
//
//  for (byte i = 0; i < sensorNum; i++) {
//    for (byte j = 0; j < sensorNum; j++) {
//      if (depthsScan[i] != depths[j][0]) {
//        newSensors = true;
//        depths[i][0] = depthsScan[i];
//      }
//    }
//  }

   printDepths();  // 28May2020

  if (newSensors) {
//     for(byte r = 0; r < sensorNum; r++){
//      for(byte c = 0; c < 10; c++){
//        Serial.print(depths[r][c]);
//      }
//      Serial.println();
//     }
    EEPROM.put(EEPROM_DEPTHS, depths);
  }*/
  skipScan = false;
}

boolean checkActiveSDI12(char i) {

  String myCommand = "";
  myCommand = "";
  myCommand += (char) i;                 // sends basic 'acknowledge' command [address][!]
  myCommand += "!";

  for (int j = 0; j < 3; j++) {          // goes through three rapid contact attempts
    SDI12port.sendCommand(myCommand);
    delay(100);  //John, 26-Mar-2019: This wasn't long enough at 30 -- changed to 40
    if (SDI12port.available() >= 1) break;
    delay(100);  //John, 26-Mar-2019: This wasn't long enough at 30 -- changed to 40
  }

  if (SDI12port.available() >= 2) {   // if it hears anything it assumes the address is occupied
    SDI12port.clearBuffer();
    //    Serial.println((char) i);
    sensorDetected = true;
    return true;
  }
  else {
    SDI12port.clearBuffer();              // otherwise it is vacant.
    return false;
  }

  SDI12port.clearBuffer();

}

boolean isTakenSDI12(byte i) {
  i = charToDec(i); // e.g. convert '0' to 0, 'a' to 10, 'Z' to 61.
  byte j = i / 8;   // byte #
  byte k = i % 8;   // bit #
  return registerSDI12[j] & (1 << k); // return bit status
}

boolean setTakenSDI12(byte i) {
  boolean initStatusSDI12 = isTakenSDI12(i);
  i = charToDec(i); // e.g. convert '0' to 0, 'a' to 10, 'Z' to 61.
  byte j = i / 8;   // byte #
  byte k = i % 8;   // bit #
  registerSDI12[j] |= (1 << k);
  return !initStatusSDI12; // return false if already taken
}

byte charToDec(char i) {
  if ((i >= 'a') && (i <= 'z')) return i - 'a' + 10;
  if ((i >= 'A') && (i <= 'Z')) return i - 'A' + 37;
}

// gets identification information from a sensor, and prints it to the serial port
// expects a character between '0'-'9', 'a'-'z', or 'A'-'Z'.
char compileInfoSDI12(char i) {
  SDI12port.clearBuffer();

  int j;
  String Command = "";
  Command += (char) i;
  Command += "I!";
  SDI12port.sendCommand(Command);
  delay(565);

  byte buf_length = SDI12port.available();
  char sensorID[36];
  byte x = 0;

  for (byte j = 0; j < buf_length; j++) {  // save sensor ID to buffer array and print
    char c = SDI12port.read();
    sensorIDs += c;
    delay(8);
    //        sensorID[x] = c;
    if (c != 10 && c != 13) {
      sensorList[sensorNum][x] = c;
      delay(8);
    }
    x++;
  }

  //Serial.println(sensorIDs);
  //  for( byte u=0; u<50; u++){
  ////  Serial.print("sensorID[");
  ////  Serial.print(u);
  ////  Serial.print("]: ");
  //    if(sensorID[u] == 10){
  //      break;
  //    }
  //  Serial.print(sensorID[u]);
  //  delay(8);
  //
  //  }
  //  Serial.println();

  char RHsens[] = "THum";
  char Tsens[] = "Temp";
  char CSid[] = "CS65";

  if (findSensorType(sensorList[sensorNum], RHsens)) {
    //  if(findSensorType(sensorID,RHsens)){
    //    Serial.print("Thum at address ");
    //    Serial.println((char) i);
    RHPresent = true;
    RHsensor = (char) i;
  }
  else if (findSensorType(sensorList[sensorNum], Tsens)) {
    //  else if(findSensorType(sensorID, Tsens)){
    TPresent = true;
    Tsensor = (char) i;  
    
 }
  else if (findSensorType(sensorID, CSid)) {
    CS655addr = (char) i;
    //    Serial.print("CS655 at address ");
    //    Serial.println(CS655addr);
  }
  else {}
  sensorNum++;
  Serial.flush();
}

void saveSDI12Info() {
  ft.flash_wake();
  delay(50);

  //  if (!isBlank(IDaddr)) {
  ft.eraseSector(IDaddr);
  //  }
  delay(50);
  //   Serial.println(sensorIDs);
  int ids_length = sensorIDs.length();
  char ids[ids_length];

  sensorIDs.toCharArray(ids, ids_length);

  ft.writeBytes(IDaddr, ids, ids_length);
  delay(300);

  lastIDaddr = IDaddr + ids_length; delay(10);

  printSDI12Info();
  delay(20);
  ft.flash_sleep();

}

void printSDI12Info() {
  //  Serial.flush();
  for (uint32_t a = IDaddr; a < lastIDaddr; a++) {
    Serial.print(char(ft.readByte(a)));
    delay(8);
  }
}


void setActiveSDI12(char c) {
  for (int index = 0; index < MAX_SENSORS + 1; index++) {
    if (activeSDI12[index] == '\0') {
      activeSDI12[index] = c;
      depthsScan[index] = c;
      return;
    }
  }
}

boolean isActiveSDI12(char c) {
  for (int index = 0; index < MAX_SENSORS + 1; index++) {
    if (activeSDI12[index] == c)
      return true;
  }
  return false;
}

boolean findSensorType(char fullID[36], char type[5]) {
  boolean found = false;
  for (byte i = 0; i < 36; i++) {
    //    Serial.print("fullID[");
    //      Serial.print(i);
    //      Serial.print("] = ");
    //      Serial.println(fullID[i]);
    if (fullID[i] == type[0]) {
      byte j = 1;
      byte k = i;
      while (j < 4) {
        if (fullID[k + j] != type[j]) {
          found = false;
          break;
        }
        else {
          //         Serial.print(j);
          //         Serial.print(": ");
          //         Serial.print(fullID[k+j]);
          //          Serial.print(": ");
          //          Serial.println(type[j]);
          //          delay(10);
          found = true;
          j++;
        }
      }
    }
  }
  if (found == true) {
    return true;
  }
  else {
    return false;
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
  //  digitalWrite(LED,HIGH);
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

//------------- Read DS3231 RTC -------------------------

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
  //  readClock();

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

  Serial.print("Next alarm at: ");

  Serial.println(alarmMins);
  delay(50);
}

void resetAlarm2() {
  readClock();
  // 29Jun20: try removing nextday variable and calculation
  /*byte nextDay; 

  if ((mnths == 4) || (mnths == 6) || (mnths == 9) || (mnths == 11)) {  // to rollover into a new month
    if (days == 30) {
      nextDay = 1;
    } else {
      nextDay = days + 1;
    }
  }
  else if (mnths == 2) {
    if (days == 29) {   // check for leap year
      nextDay = 1;
    } else if ((days == 28) && (yrs % 4 != 0)) {
      nextDay = 1;
    } else {
      nextDay = days + 1;
    }
  }
  else if (days == 31) {
    nextDay = 1;
  }
  else {
    nextDay = days + 1;
  }

  RTC.setAlarm(ALM2_MATCH_DATE, 0, NISTmin, NISThr, nextDay);   // 15Jun2020
*/
//  RTC.setAlarm(ALM2_MATCH_HOURS, 0, NISTmin, NISThr, nextDay);
  RTC.setAlarm(ALM2_MATCH_HOURS, 0, NISTmin, NISThr, 0); // for testing 16Jun2020

// Note for Alarm 2:
// ALM2_MATCH_HOURS -- causes an alarm when the hours and minutes match.
// ALM2_MATCH_DATE -- causes an alarm when the date of the month and hours and minutes match.

  Serial.print("Alarm 2 set for: ");
//  Serial.print(mnths);
//  Serial.print('/');
//  Serial.print(nextDay);
//  Serial.print(' ');
  Serial.print(NISThr);
  Serial.print(":");
  Serial.println(NISTmin);
  delay(50);
}

//===================================================================================================

//--------- Sync time with Gateway in field ----------------------------------------------

void fieldSync(uint32_t time2wait) {    // 27-Jan-2020: added time2wait parameter to use function for daily sync check
  uint32_t timeToWait = time2wait;
  if(duringInit){
    fieldSyncStart = millis();
  }
  
  if (!LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, radioID)) {      // 01/14/2020                // initialize radio
    Serial.println("Radio failed");
  }

  Serial.println("Waiting to sync with Gateway...");

  uint8_t buf[20];     // array to receive time update from gateway
  uint8_t len = sizeof(buf);
  uint8_t from;
  boolean done = false;
  uint8_t ok[1];
  ok[0] = radioID;

 //  unsigned long twoHours = 7200000; // 2 hrs in milliseconds // 27-Jan-2020: commented out, use timeToWait instead
  //  unsigned long timeToWait = millis(); // + twoHours;
  unsigned long waitStart = millis();

  //  while(millis() < timeToWait && done == false){  // wait for 2 hours or until receive confirmation from Gateway that sync is done

  while (((millis() - waitStart) < timeToWait) && (done == false)) {    // 27-Jan-2020: use timeToWait instead of twoHours
    if (LoRa.recvfromAckTimeout(buf, &len, timeoutSmallPkt, &from)) { // if time update received
//          for(byte u = 0; u<len;u++){
//          Serial.print((char)buf[u]);
//          }
//          Serial.println();

      if (len > 1){    // 04Aug2020: && buf[0] != GatewayID) { <-- CAUSES PROBLEM IF Gateway ID = 50 (char "2" in "2020")
        //      Serial.println(ok[0]);
        //      LoRa.sendtoWait(ok,sizeof(ok),GatewayID);
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
//          break;        // 14May
        }
        else if (battLow) {
          battLow = false;
          EEPROM.update(EEPROM_LOW_BATT, battLow);
          done = true;
//          break;  // don't wait for ok from Gateway
        } else if (numMissed >= 3) {
          numMissed = 0;
          done = true;
//          break;
        } else {
          done = true;
//          break;
      }
//      else if (buf[0] == GatewayID) {  // Gateway done syncing times 14May20
//        done = true;    // set flag to break out of while loop
//        Serial.println("Done");
      }   // end if(len>0) 
    }  // end if transmission received
  } // end while loop
  //    sendIDs();
if(duringInit){
  fieldSyncStop = millis();
}

}

//===================================================================================================

//------------- Listen for timestamp & send data ---------------------------------------------------------------------

void listenRespond() {
 
  SPI.begin();
  delay(200);

  // Step 1 - turn on radio

  if (!LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, radioID)) {      // 01/14/2020                // initialize radio
    Serial.println("Radio failed");
  }

  // Step 3 - wait to receive time update from Gateway, if received respond with data and update clock

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
//    timeToWait = 600000;  // 13May20: ten minutes
//    timeToWait = mins + 2;
//    timeToWait = mins + 10; // 13May20: to give Gateway time to send Hologram message and account for Ns getting turned on before G
  } else {
    timeToWait = 60000; // 13May20: one minute
  }

//  while (timeUpdated == false && mins <= timeToWait){
  while (timeUpdated == false && ((millis() - start) < timeToWait)) { // 13May20
    readClock();

    if (LoRa.recvfromAckTimeout(buf, &len, timeoutPacket, &from)) { // if time update received
//    if (LoRa.recvfromAckTimeout(buf, &len, timeoutPacket, GatewayID)) { // 18Jun20
      if (len > 0) {
         
//        LoRa.sendMessage(&allData, GatewayID);

        int nYear;
        int nMonth;
        int nDay;
        int nHour;
        int nMin;
        int nSec;
        char *nbuf = (char*)buf; 
//        delay(5); //18Jun20
//        Serial.println(nbuf);   // 16Jun20
        
        if (sscanf(nbuf, "%d/%d/%d|%d:%d:%d", &nYear, &nMonth, &nDay, &nHour, &nMin, &nSec)) {  // evaluate buffer and set clock to new time
//          Serial.println(nYear); // 17Jun20

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
            numMissed = 0;      // 25-Mar-2020: only want to track consecutive misses 
            Serial.print("Successful time update: ");
            Serial.println(nbuf);
  
            LoRa.sendMessage(&allData, GatewayID);
        }
       }
      } // end if(len>0)
    }  
    //  else { Serial.println(secs);}
  }  // end while loop

//  if (timeUpdated == false) {   // no comm with Gateway
  if (timeUpdated == false && !duringInit) {   // 13May2020
    numMissed++;
    Serial.print("numMissed: ");
    Serial.println(numMissed);
  }
  
}


//===================================================================================================

//------------- Initiate Measurements from Active Sensors -------------------------------------------

void readSensors() {

  //  digitalWrite(SDI12Pwr, HIGH);
  EEPROM.get(EEPROM_DEPTHS, depths);    // 04Jun2020
  delay(20);
//  printDepths();
  
  SDI12port.begin();
  delay(500);
  allData = "";

  Serial.println(allData);
  /*
    //  get measurements from address space 0-9

    for (char j1 = '0'; j1 <= '9'; j1++) if (isActiveSDI12(j1)) {
        measureSDI12(j1);
      }

    //  get measurements from address space a-z

    for (char j1 = 'a'; j1 <= 'z'; j1++) if (isActiveSDI12(j1)) {
        measureSDI12(j1);
      }

    //  get measurements from address space A-Z
    for (char j1 = 'A'; j1 <= 'Z'; j1++) if (isActiveSDI12(j1)) {
        measureSDI12(j1);
      }
  */

  for (byte i = 0; i < sensorNum; i++) {
    char c = sensorList[i][0];
    rowNum = i;
    measureSDI12(c);
  }

  //remove trailing separator
  char td[allData.length()];
  delay(50);
  allData.toCharArray(td, allData.length());
  delay(50);
  td[allData.length()] = '\0';
  delay(50);

  allData = String(td);

  delay(50);
  //    Serial.println(allData);
  SDI12port.end();  delay(50);

  //  digitalWrite(SDI12Pwr, LOW);
  delay(50);

}

void measureSDI12(char index) {

// Add metadata to data string

  SDI12data = "";         // clear String if not already
  delay(20);
  SDI12data += sensorList[rowNum];    // 4-Mar-2020: use sensor ID instead of address
  char depth[9];
  byte addrPos;
  bool depthFound = false;

//  printDepths();  // 28May2020

  for (byte j = 0; j < sensorNum; j++) {  // added 5-Mar-2020: scroll through depths to find sensor addr (in case row order doesn't match with sensorList)
    if (depths[j][0] == index) {
//      Serial.print(index);
//      Serial.print(',');
//      Serial.println(j);
      addrPos = j;
      depthFound = true;
    }
  }

  for (byte i = 0; i < 9; i++) {
    if (depths[addrPos][i + 1] != 0) {
      depth[i] = depths[addrPos][i + 1];
    } else {
      depth[i] = 0;
      break;
    }
  }

  SDI12data += sep;

  if (depthFound) {
    //        if (depths[addrPos][1] != '+' && depths[addrPos][1] != '-'){
    //         SDI12data += "-999";    // added 5-Mar-2020: throw error if depth not found
    //        } else {
    SDI12data += depth;
  }
   else {
    SDI12data += "-999";    // added 5-Mar-2020: throw error if depth not found
  }

//  Serial.println(SDI12data);

  String readData = "";
  readData += index;
  readData += "M!";                    // returns address, VWC, Soil T, Permittivity, EC, CEC from Acclima TDRs

  SDI12port.sendCommand(readData);

  delay(parseResp());

  SDI12port.clearBuffer();
  delay(50);

  getResponseSDI12(index);
  allData += SDI12data + sep;
  delay(100);

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
      //      Serial.print(response[i],DEC);
    }

    measDelay = (response[3] - 48); // get decimal value of char, convert to milliseconds
    measDelay = measDelay * 1000;

  } else {
    measDelay = 1000;
    SDI12response = false;
  }

  // Serial.println();
  // Serial.println(measDelay);
  return measDelay;

}


//------------- Take Measurement from TDRs --------------------------------------

void getResponseSDI12(char index) {

  if (SDI12response == true) {       // if a response to M! was received, send D0!

    String sendData = "";
    sendData += index;
    sendData += "D0!";
    //    Serial.println(sendData);

    SDI12port.sendCommand(sendData);   // send D0! command to retrieve data
    delay(500);                  // 300 works with TDRs

    // after sending D0! command:

    // Step 1 -- create char array to receive data --

    byte incomingbytes = SDI12port.available();
    //         Serial.println(incomingbytes);
    char response[incomingbytes];   // replace trailing CR with NULL and remove LF

    // Step 3 -- store bytes into array

    if (incomingbytes > 5) {
      for (byte i = 0; i < (incomingbytes); i++) {
        char c = SDI12port.read();
        response[i] = c;         // store c in array if not at last byte
        //               Serial.print(c);
        delay(15);
      }    // end for loop

      // Steps 3 & 4 -- 3) take out "address CR LF" response from TDRs; 4) convert array to String, replacing "+" and "-" with delimiter, add in "-"


      //      char RHTresponse[incomingbytes-2];
      char TDRresponse[incomingbytes];              // remove first three chars (i<CR><LF>) and last two (<CR><LF>) from array

      if (index == RHsensor || index == Tsensor) {
        for (byte i = 0; i < incomingbytes; i++) {
          TDRresponse[i] = response[i + 1]; // 4-Mar-2020: skip address, replaced with sensor ID
          //            Serial.print(TDRresponse[i]);
          delay(15);
        }
        //          Serial.println();
      } else {
        /*
          //        for (byte i = 0; i <= incomingbytes-5; i++){
          //          TDRresponse[i] = response[i+3];
          //            Serial.print(TDRresponse[i]);*/
        for (byte i = 0; i < incomingbytes - 2; i++) {   // was -2
          TDRresponse[i] = response[i + 1]; // 4-Mar-2020: skip address, replaced with sensor ID
          //            Serial.print(TDRresponse[i],DEC);
          delay(20);
        }
        //        Serial.println();
        delay(20);
      }

      byte j = sizeof(TDRresponse) - 1;
      for (byte i = 0; i < j; i++) {           // put response from TDR into String
        if (TDRresponse[i] != 10 && TDRresponse[i] != 13 && TDRresponse[i] > 42  && TDRresponse[i] < 123) { // don't want to save LF or CR or out of range chars
          if ((TDRresponse[i] == '+') || (TDRresponse[i] == '-')) {  // check for + and -, replace with delimiter
            SDI12data += sep;
            delay(20);        // from John's demo: 11 ms ensures that there has been enough time to receive another byte
            if (TDRresponse[i] == '-') {
              SDI12data += '-';                  // add '-' back in when needed
              delay(20);
            }
          } else {
            SDI12data += TDRresponse[i];         // add char to String
            delay(25);
          }
        }
        else {}
      }  // end for loop
  
  #ifdef getTT
    if(index != Tsensor && index != RHsensor && index != CS655addr){  // if a TDR, get time info
      getTimeData(index);
    }
  #endif

    } else {  // ERROR 1: if incomingbytes < 5
      if (index == CS655addr) {
        SDI12data += "~-999~-999~-999";
      }
      else if (index == Tsensor || index == RHsensor) {
          if (index == Tsensor) {
            SDI12data += "~-999";
          } else {
            SDI12data += "~-999~-999";
          }
      }
      else {    // if a TDR sensor
        SDI12data += "~-999~-999~-999~-999~-999";
      }   
    }
  }
  else {   // ERROR 2: if there was no response to M!
    if (index == CS655addr) {
      SDI12data += "~-999~-999~-999";
    }
    else if (index == Tsensor || index == RHsensor) {
        if (index == Tsensor) {
          SDI12data += "~-999";
        } else {
          SDI12data += "~-999~-999";
        }
    }   
    else {    // if a TDR
      SDI12data += "~-999~-999~-999~-999~-999";
    }   
  }

  delay(300);
  //  Serial.print("SDI12data: ");
  //  Serial.println(SDI12data);
  SDI12port.clearBuffer();
  delay(50);

  SDI12port.clearBuffer();
  delay(50);

}

//------------- TDRs: Replace PEC with travel time --------------------------------------

void getTimeData(char index){

//  Serial.println(SDI12data);

//--- Remove pEC value from SDI12data
#ifndef keepPEC  
  uint8_t lastTilde = SDI12data.lastIndexOf('~');
//  Serial.println(lastTilde);

  uint8_t toRemove = SDI12data.length() - lastTilde;

  SDI12data.remove(lastTilde, toRemove);

//  Serial.println(SDI12data);
  #endif
//--- get time data from TDR

  String sendData = "";
  sendData += index;
  sendData += "D1!";

  SDI12port.sendCommand(sendData);   
  delay(500);                  

  byte incomingbytes = SDI12port.available();
  char response[incomingbytes - 2];   // replace trailing CR with NULL and remove LF

  if (incomingbytes > 5) {
    for (byte i = 0; i < (incomingbytes - 1); i++) {
      char c = SDI12port.read();
      if(c != 13 && c != 10){
        response[i] = c;         // store c in array if not at last byte
//        Serial.print(response[i],DEC);
        delay(15);
      }
    }    // end for loop
  }

  String timeData = "";
  
    for (byte i = 1; i < incomingbytes - 2; i++){
      if(response[i] != '+' ){
        timeData += response[i];
      } else {
        timeData += sep;  
      }
      delay(20);
    }
//  Serial.println(timeData);

//--- delete all values except for travel time from timeData

  uint8_t last = timeData.lastIndexOf('~');

  uint8_t firstDelete = timeData.length() - last;

  timeData.remove(last, firstDelete);

  uint8_t secondlast = timeData.lastIndexOf('~');

  timeData.remove(0, secondlast);

//  Serial.println(timeData);

// concatenate SDI12data and timeData

  SDI12data += timeData;
  delay(20);
//  Serial.println(SDI12data);
  
}

//===================================================================================================

//--------------- Calculate Battery Voltage ---------------------------------------------------------

float calcbattV() {
  float result;
  analogComp_on();
  digitalWrite(pin_mBatt, HIGH);  // turn on voltage divider
  delay(50);

  int Aout;
  Aout = analogRead(pin_battV); // analogRead returns an integer between 0 and 1023
  result = Aout * multiplier * 1.3333;  // for voltage divider: R1=10k, R2=30k, resist =  (R1+R2)/R2

  digitalWrite(pin_mBatt, LOW);   // turn off voltage divider
  return result;
}

//===================================================================================================

//------------- Compile timestamp  --------------------------------------------------------------

void Timestamp() {      // compile timestamp

  battV = calcbattV();
  getBoxT();
  unsigned int pvCurrent = getSolarCurrent();
  float pvVoltage = getSolarVoltage();

  timestamp = "";
  delay(20);

  timestamp += VERSION;
  timestamp += sep;
  //  if(use_siteID){
  timestamp += projectID;
  timestamp += sep;
  //  }
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
  timestamp += '_';
  timestamp += TMZ;
  timestamp += sep;
  //  Serial.println(timestamp);
  delay(50);

}

//===================================================================================================

//------------- Compile data in String --------------------------------------------------------------

void compileData() {
  //  Serial.println("Compiling data String");
  //  int8_t rssi = driverRFM95.lastRssi();


  allData = timestamp + allData; // + '\r' + '\n';

  delay(100);

      Serial.println(allData);
  delay(50);
}

//===================================================================================================

//------------- Save data to Flash --------------------------------------------------------------

void saveData() {

  compileData();
  //  String saveString = allData; // + '+'; // 14-Feb-2020: don't need anymore becuase of printData()
  uint16_t Len = allData.length(); // + 1; // includes NULL
  //  Serial.print("Len: ");
  //  Serial.println(Len);
  char tmpChars[Len];  // char array to hold data string (will be converted to uint8_t array for sending)

  memcpy(tmpChars, allData.c_str(), Len);

  // Call function to write log
  if (ft.writeLog(tmpChars, Len)) {
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
  } else {
    Serial.println("Failed to write log!");
  }
  /*
    //  Serial.println("Package: ");
    //  for (byte i=0; i<Len; i++){
    //    Serial.print(i); Serial.print(": "); Serial.println(Package[i],DEC);
    //  }
    // Serial.println();

    uint16_t len = strlen(Package) + 1;
    Serial.print("len: ");
    Serial.println(len);

    char* saveStr = malloc(len);
    strcpy(saveStr, Package);
    Serial.println("saveStr: ");
    Serial.println(saveStr);
    //     for (byte i=0; i<len; i++){
    //      Serial.print(i); Serial.print(": "); Serial.print(saveStr[i],DEC); Serial.print(",");Serial.println(saveStr[i]);
    //    }
    //  Serial.print("len= ");
    //  Serial.println(len);

    ft.flash_wake();
    delay(50);

    NodeLog* nLog = malloc(sizeof(NodeLog));
    lastLogId++;
    nLog->logId = lastLogId;
    nLog->logSize = len + 6;

    if (writeLogEntry(nLog, saveStr, len)) {
      Serial.print("Successful save!");
    } else {
      Serial.print("Failed to save!");
    }

    free(saveStr);
    free(nLog);
  */
  ft.flash_sleep();

  /* uint32_t Len;
    Len = allData.length()+ 2;                  // size array, add space for "+"

    uint32_t spaceNeeded;
    spaceNeeded = startAddr + Len;    // +1?

    if ((spaceNeeded > firstAddrInNextSector) && (spaceNeeded <= maxAddr)){  // if 1) not enough space in current sector for entire data string and 2) last sector not full
      zeroOut();
      Serial.println("not enough space in sector");

      if (!isBlank(firstAddrInNextSector)){    // if next sector to write to already has data
         Serial.println("Next sector not blank, erasing...");
         flash.sectorErase4K(firstAddrInNextSector);   // erase next sector before writing to it
         delay(100);
      }

      ptr_startAddr = firstAddrInNextSector;   // update start address to first address in next sector
     Serial.println(*ptr_startAddr);
      ptr_firstAddrInNextSector = firstAddrInNextSector + sectorLength;    // update for next sector
     Serial.println(*ptr_firstAddrInNextSector);
     delay(50);

     writeData();

      ptr_startAddr += Len;

    } else if (spaceNeeded > maxAddr){   // if last sector is full
    //  Serial.println(" > maxAddr");
     zeroOut();
      ptr_startAddr = firstDataAddr;    // start at beginning of data storage space
     flash.sectorErase4K(startAddr);    // erase sector
      ptr_firstAddrInNextSector = firstDataAddr + sectorLength;    //update firstAddrInNextSector
     writeData();
      ptr_startAddr += Len;
    } else {
    //Serial.println("within sector");
     writeData();
      ptr_startAddr = spaceNeeded;    // update startAddr for next empty byte
     Serial.println(startAddr);
    }

    saveAddrPtrs();
    delay(200);

    //  ft.flash_sleep();

    //  Serial.print("Updated startAddr: ");
    //  Serial.println(startAddr, DEC);*/

}

/*
  void writeData(){
    uint32_t i=0;
    uint32_t Len;
    Len = allData.length()+ 2;                  // size array, add space for NULL and "+" measurement delimiter

    char dataArray[Len];
    allData.toCharArray(dataArray,Len);
    dataArray[Len-1] = 43;  // "+"; to separate measurements

    for (addr = *ptr_startAddr; addr < (*ptr_startAddr + Len); addr++){    // write data array
    flash.writeByte(addr, dataArray[i]);
    delay(5);
    i++;
  }
  }

  void zeroOut(){   // fill in rest of sector with zeroes
  for (addr = startAddr; addr < firstAddrInNextSector; addr++){   // fill in rest of sector with zeroes
    flash.writeByte(addr,0x00);
    delay(5);
  }
  delay(50);
  }

  void saveAddrPtrs(){
  //Serial.println("Saving updated addresses");
  flash.sectorErase4K(FlashHeader_Sector2);
  delay(20);

  // Decomposition from a long to 4 bytes using bitshift
  // buf[0] = Most significant --> buf[3] = least significant byte
  byte buf[4];
  byte buf2[4];

  buf[3] = (startAddr & 0xFF);
  buf[2] = ((startAddr >> 8) & 0xFF);
  buf[1] = ((startAddr >> 16) & 0xFF);
  buf[0] = ((startAddr >> 24) & 0xFF);

  buf2[3] = (*ptr_firstAddrInNextSector & 0xFF);
  buf2[2] = ((*ptr_firstAddrInNextSector >> 8) & 0xFF);
  buf2[1] = ((*ptr_firstAddrInNextSector >> 16) & 0xFF);
  buf2[0] = ((*ptr_firstAddrInNextSector >> 24) & 0xFF);

  flash.writeBytes(startAddrLoc,buf,4);
    delay(20);
  flash.writeBytes(firstAddrLoc,buf2,4);
    delay(20);
  }

  void recallAddrPtrs(){
  ft.flash_wake();
  delay(50);

  byte buf[4];
  flash.readBytes(startAddrLoc, buf, 4);
  delay(20);
  uint32_t test = ((buf[3] << 0) & 0xFF) + ((buf[2] << 8) & 0xFFFF) + ((buf[1] << 16) & 0xFFFFFF) + ((buf[0] << 24) & 0xFFFFFFFF);

  if (test > firstDataAddr){
     ptr_startAddr = test;
  }
  delay(20);

  //      Serial.println(*ptr_startAddr);

  byte buf2[4];
  flash.readBytes(firstAddrLoc, buf2, 4);
    delay(20);
  uint32_t test2 = ((buf2[3] << 0) & 0xFF) + ((buf2[2] << 8) & 0xFFFF) + ((buf2[1] << 16) & 0xFFFFFF) + ((buf2[0] << 24) & 0xFFFFFFFF);
  if (test2 > 0x004000){
      ptr_firstAddrInNextSector = test2;
  }
  delay(20);

  //      Serial.println(*ptr_firstAddrInNextSector);

  ft.flash_sleep();
  delay(50);
  }*/

//======================================================================================

//--------------- Solar panel ---------------------------------------------

// float getMOSFETresistance(float temp, float voltSD) {
//   //NOTE: This is based on datasheet graphs but has empirical compensation as well
//   //returns SSM3K376R MOSFET resistance (in Ohms) as a function of temperature and [pre]source-drain voltage (assuming 3-volt gate-source voltage)

//   float tempCompensation = (float)((0.2053 * temp + 42.158) / 1000.0);
//   float adjusted;
//   if (voltSD > 0.10) {
//     adjusted = tempCompensation + (-1.122 * voltSD * voltSD + 0.8759 * voltSD - 0.0843);
//   } else {
//     adjusted = tempCompensation + (-42.672 * voltSD * voltSD + 9.4514 * voltSD - 0.5258);
//   }

//   if (adjusted < 0)
//     adjusted = 0;
//   return adjusted;
// }

//Returns voltage of solar cell in volts (battery load may affect voltage)
float getSolarVoltage() {
  //Add the following lines to header and setup function:
  //  #define pin_solarVoltage    A5
  //  #define ADC_REF_VOLTAGE     3.3
  //  #define ADC_RESOLUTION      1024
  //  pinMode(pin_solarVoltage, INPUT);

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

//--------------- Get Device info  -------------------------------------------

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
  userinput = false;    // added 01/13/2020
  if (Serial.available() > 0)
  {
    Serial.read();       // clear serial input buffer
  }

  EEPROM.get(EEPROM_PROJECTID, projectID);

  if (!radioSwitch) {
    radioID = EEPROM.read(EEPROM_DEFAULT_RADIO);
  } else {
    radioID = EEPROM.read(EEPROM_ACTIVE_RADIO);
  }
  GatewayID = EEPROM.read(EEPROM_GATEWAYID);
  interval = EEPROM.read(EEPROM_ALRM1_INT);
  gatewayPresent = EEPROM.read(EEPROM_GW_PRESENT);

//  printDepths();

  delay(50);

  battV = calcbattV();

  //  Serial.println();
  Serial.println(F("Soil Water Data Network - Wireless SDI-12 Datalogger Node"));               // print out board info
  Serial.print(F("Version "));
  Serial.println(VERSION);
  Serial.println();
  Serial.print(F("Serial Number: "));
  Serial.println(serNum);
  Serial.print(F("LoRa Radio Freq (MHz): "));
  Serial.println(LoRaFREQ);
  //  if(use_siteID){
  Serial.print(F("Project ID: "));
  Serial.println(projectID);
  //  }

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

  Serial.print(F("Current date & time:  "));
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
  Serial.print(' ');
  Serial.println(TMZ);
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
  Serial.println(F("   a  <--  Change SDI-12 sensor address"));
  Serial.println(F("   t  <--  Test sensors"));                     // takes three measurements from sensors
  Serial.println(F("   S  <--  Synchronize Gateway & Node clocks"));  // get time from Gateway, update clock
  Serial.println(F("   p  <--  Print data to screen"));         // print data to Serial Monitor
  Serial.println(F("   e  <--  Erase all data"));                   // delete all data from Flash
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
      userinput = true;                        // added 01/13/2020
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
      Serial.print(F("Enter configuration string: "));
      charinput();
      Serial.println();
      decodeConfig(charInput);
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

      for (byte i = 0; i < sensorNum; i++) {
        depths[i][0] = sensorList[i][0];
        
        Serial.print(sensorList[i]);
        Serial.print(": ");
        charinput();

        while (charInput[0] != '-' && charInput[0] != '+') {
          Serial.println("ERROR: Use '+' or '-' to indicate above or below soil surface");
          Serial.print(sensorList[i]);
          Serial.print(": ");
          charinput();
        }

            for (byte j = 0; j < 8; j++) {
          if (charInput[j]) {
            depths[i][j + 1] = charInput[j]; delay(8);
          } else {
            depths[i][j + 1] = 0;
            break;
          }
        }
      }

      // for testing ---

//      printDepths();
      
      Serial.println();
      EEPROM.put(EEPROM_DEPTHS, depths);
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

    case 83:   //--------- S - Synchronize Gateway & Node clocks ----------------------------

      Serial.println();
      Serial.println(F("Waiting for time from Gateway..."));
      syncTime();
      delay(500);
      menu();
      break;

    case 97: case 65:           // ------ a - Change sensor address ---------------------------------------------

      sensorAddress();
      Serial.println();
      delay(500);
      menu();
      break;

    case 116: case 84:          // ------ t - Test sensors ---------------------------------------------

      Serial.println(F("Test measurements:"));     // take 3 readings to check sensors
//      Serial.println();
      delay(10);
      if(skipScan){
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

      Serial.println(F("Print all data to screen: "));     // download all data in Flash
      delay(100);

      ft.printData();
      Serial.println();
      
      menu();
      break;

    case 101: case 69:          // ------ e - Erase Flash ---------------------------------------------

      Serial.println(F("Are you sure you want to erase all data stored in memory?"));
      Serial.print(F("Enter 'y' for yes or 'n' for no: "));
      charinput();

      if (charInput[0] == 'y' || charInput[0] == 'Y') {
        ft.eraseLogs();
      }

      ft.logSetup(logsBeginAddr, logsEndAddr, LOG_minSize, LOG_maxSize, true);
      Serial.println();
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
    
    byte i = 0;
    for (i = 0; i < 5; i++) {
      if(charInput[i] != 0){
        projectID[i] = charInput[i];
      } else {
        break;
      }        
    }
    projectID[i] = 0;
    
    EEPROM.put(EEPROM_PROJECTID, projectID);
    delay(10);

    Serial.println();
    delay(500);

    menu();
    break;
  }
  
  if (userinput == true) {
    digitalWrite(LED, LOW);
  }
}

//======================================================================================
//======================================================================================
//======================================================================================

//-------------- Decode config string --------------------------------

void decodeConfig(char config_string[200]) {
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

  //--- Check for site ID
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
    //   firstComma = 1;
    //  }

    //  if(!use_siteID){

    //--- Match serial num

    uint8_t SerNum[8];

    //  if(!use_siteID){
    //    for (byte i = 0; i < 8; i++){
    //      SerNum[i] = configN[i] - 48;
    ////      Serial.print(SerNum[i]);
    //    }
    //  } else {
    for (byte i = 0; i < 8; i++) {
      SerNum[i] = configN[i + commaPos[0] + 1] - 48;
      //        Serial.print(SerNum[i]);
    }
    //  }
    //  Serial.println();

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

    gatewayPresent = true;
    Serial.print("Gateway ID: ");
    Serial.println(GatewayID);

    //--- Get measurement interval

    interval = 10 * (configN[commaPos[firstComma + 2] + 1] - 48) + (configN[commaPos[firstComma + 2] + 2] - 48);

    Serial.print("Measurement interval: ");
    Serial.println(interval);

    if (interval != 15 && interval != 20 && interval != 30 && interval != 60) {
      Serial.println(F("ERROR: Invalid interval (every 15, 20, 30, or 60 mins)"));
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
    //  byte x;
    for (byte r = 0; r < sensorNum; r++) {
      byte L = 4 + 2 * r; // comma before address
      byte R = 4 + 2 * r + 1; // comma after address
      byte x = commaPos[firstComma + R + 1] - commaPos[firstComma + R];
      //     Serial.print(commaPos[firstComma + R]);
      //     Serial.print(commaPos[firstComma + L]);
      //     Serial.print(x);
      depths[r][0] = configN[commaPos[firstComma + L] + 1];
      Serial.print(depths[r][0]);

      for (byte c = 1; c < x; c++) {
        depths[r][c] = configN[commaPos[firstComma + R] + c];
        Serial.print(depths[r][c]);
      }
      depths[r][x] = 0;
      Serial.println();
      delay(5);
      //    if(!use_siteID && r == (sensorNum - 1) && R != (commaTot - 2)){
      ////      Serial.println(R);
      ////      Serial.println(commaTot);
      //      Serial.println("ERROR: Incorrect number of sensor depths");
      //    } else
      if (r == (sensorNum - 1) && R != (commaTot - 3)) {
        //      Serial.println(R);
        //      Serial.println(commaTot);
        Serial.println("ERROR: Incorrect number of sensor depths");
      }
    }
//    printDepths();
  }
  if (!configOK) {
    Serial.println("!!! FAIL: Invalid configuration !!!");
  } else {
    Serial.println("Configuration complete");
    //    if(use_siteID){
    //      EEPROM.update(EEPROM_SITEID_FLAG, use_siteID);
    EEPROM.put(EEPROM_PROJECTID, projectID);
    //    }
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

}

void setRTCTime() {

  Serial.println(F("Enter the date and time in Coordinated Universal Time (UTC): "));

  Serial.print(F("  input month:  "));
  getinput();
  mnths = indata;
  tm.Month = mnths;
  Serial.print(F("  input day:    "));
  getinput();
  days = indata;
  tm.Day = days;
  Serial.print(F("  input year:   "));
  getinput();
  yrs = indata;
  tm.Year = yrs - 1970;
  Serial.print(F("  input hour:   "));
  getinput();
  hrs = indata;
  tm.Hour = hrs;
  Serial.print(F("  input minute: "));
  getinput();
  mins = indata;
  tm.Minute = mins;

  RTC.write(tm);
  delay(50);
}

void setIDs() {

  Serial.print(F("Are you using a Gateway? Enter 'y' for yes, 'n' for no: "));
  charinput();
  gatewayPresent = (charInput[0] == 'y' || charInput[0] == 'Y' ? 1 : 0);
  EEPROM.update(EEPROM_GW_PRESENT, gatewayPresent);

  if (gatewayPresent == 1) {
    Serial.print(F("    Enter Gateway radio ID: "));
    getinput();
    GatewayID = indata;
    EEPROM.update(EEPROM_GATEWAYID, GatewayID);
    delay(10);
  }

  delay(100);
}

void changeDefault() {
  Serial.println(F("Would you like to change the radio ID from the default value?"));
  Serial.print(F(" Enter 'y' for yes, 'n' for no: "));
  charinput();
  radioSwitch = (charInput[0] == 'y' || charInput[0] == 'Y' ? 1 : 0);
  EEPROM.update(EEPROM_FLAG_RADIO, radioSwitch);
  if (radioSwitch) {
    //    Serial.println();
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
  Serial.print(F("Enter measurement interval (every 15, 20, 30, or 60 mins): "));
  Serial.flush();
  getinput();
  while (indata != 2 && indata != 5 && indata != 15 && indata != 20 && indata != 30 && indata != 60) {
    Serial.print(F("Invalid interval. Enter measurement interval (every 15, 20, 30, or 60 mins): "));
    Serial.flush();
    getinput();
  }
  interval = indata;
  EEPROM.update(EEPROM_ALRM1_INT, interval);
}

/*void addRH(){
  Serial.print(F("Do you want to log data from a humidity sensor? Enter 'y' for yes, 'n' for no: "));
  charinput();
  RHPresent = (charInput[0] == 'y' || charInput[0] == 'Y'? 1:0);
  EEPROM.update(EEPROM_RH_PRESENT, RHPresent);
  delay(10);
  }*/

void printDepths(){ // added 28May2020
//  EEPROM.get(EEPROM_DEPTHS, depths); delay(100); 
  Serial.println("Printing depths...");
     for(byte r = 0; r < sensorNum; r++){
      for(byte c = 0; c < 10; c++){
        Serial.print(depths[r][c]);
        delay(5);
      }
      Serial.println();
     }
    Serial.println();
}

void clearDepths(){   // added 28May2020
  EEPROM.get(EEPROM_DEPTHS, depths); delay(100);
  memset(depths,0,sizeof(depths));

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
      break;                                       // exit before timeout
    }
  }
  Serial.println(indata); delay(10);

}

//======================================================================================

//-------------- Get User Input for character variables --------------------------------

void charinput() {
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
      //       Serial.println(numincoming);
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

//-------------- Sync G & N Time --------------------------------

void syncTime() {
  //   if (!LoRa.init()) {                      // initialize radio
  //    Serial.println("Radio failed");
  //  }
  //  driver.setFrequency(LoRaFREQ);  //John: 07-May-2019, after call to LoRa.init() the radio frequency and power must be reset to desired value (otherwise defaults to 915 MHz)
  //  driver.setTxPower(TxPower);
  //  LoRa.setRetries(retryNum);

  if (!LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, radioID)) {      // 01/14/2020                // initialize radio
    Serial.println("Radio failed");
  }



  uint8_t buf[19];
  uint8_t len = sizeof(buf);
  uint8_t from;
  boolean timeSynced = false;
  uint8_t hello[] = {1, 1, 1, 1};

  readClock();
  long timeout = millis();// + 45000; 01/14/2020    // 45 seconds

  //  while ((millis() < timeout)  && (timeSynced == false)){
  while (((millis() - timeout) < 45000)  && (timeSynced == false)) {    // 01/14/2020

    LoRa.sendtoWait(hello, 4, GatewayID);

    /* if(LoRa.recvfromAckTimeout(buf, &len, timeoutPacket, &from)){  // if time update received
      if (len > 0) {

       int nYear;
       int nMonth;
       int nDay;
       int nHour;
       int nMin;
       int nSec;
       char *nbuf = (char*)buf;

       if(sscanf(nbuf, "%d/%d/%d|%d:%d:%d", &nYear, &nMonth, &nDay, &nHour, &nMin, &nSec)){    // evaluate buffer and set clock to new time

       tm.Year = nYear - 1970;
       tm.Month = nMonth;
       tm.Day = nDay;
       tm.Hour = nHour;
       tm.Minute = nMin;
       tm.Second = nSec;

       RTC.write(tm);      // update clock
       timeSynced = true;

      }
      }  // end if(len>0)
      }  // end if transmission received */
  }
  /*
    if (timeSynced == false){
      Serial.println("Synchronization failed. Return to menu to try again.");
    } else {
      Serial.println();
      Serial.println("Gateway & Node successfully synced");
    }*/
}



//======================================================================================

//-------------- Change sensor address --------------------------------

void sensorAddress() {
  oldAddress = '!';
  //    digitalWrite(SDI12Pwr,HIGH);                    // turn on SDI12 sensors
  SDI12port.begin();
  delay(500);
  boolean found = false;
  if(skipScan){
    SDI12Scan();
  }

  for (char j1 = '0'; j1 <= '9'; j1++) if (isActiveSDI12(j1)) {
      found = true;
      oldAddress = j1;
      changeAddress();
      Serial.println();
    }

  for (char j1 = 'a'; j1 <= 'z'; j1++) if (isActiveSDI12(j1)) {
      found = true;
      oldAddress = j1;
      changeAddress();
      Serial.println();
    }

  for (char j1 = 'A'; j1 <= 'Z'; j1++) if (isActiveSDI12(j1)) {
      found = true;
      oldAddress = j1;
      changeAddress();
      Serial.println();
    }

  if (!found) {
    Serial.println("No sensor detected. Check physical connections."); // couldn't find a sensor. check connections..
  }


}

void changeAddress() {
  //  char oldAddress = '!';
  String myCommand = "";

  Serial.print(F("Current address: "));

  Serial.println(oldAddress);

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

  Serial.println("Readdressing sensor.");
  myCommand = "";
  myCommand += (char) oldAddress;
  myCommand += "A";
  myCommand += (char) newAdd;
  myCommand += "!";
  SDI12port.sendCommand(myCommand);

  /* wait for the response then throw it away by
    clearing the buffer with clearBuffer()  */
  delay(300);
  if (SDI12port.available()) {
    while (SDI12port.available()) {
      Serial.write(SDI12port.read());
    }
    Serial.println("Success.") ;
  }
}
