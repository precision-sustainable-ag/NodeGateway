 
/*PSA Cellular Gateway - LPWA module

   Main Components:
      - ATMega1284P with MoteinoMEGA core
      - DS3231 Precision RTC
      - LoRa radio transceiver
      - SimCom 7070G cellular module
      - microSD slot
 
    Main Functions:
      - Wake at user-specified intervals to request data from networked Nodes by sending a timestamp
      - Receive incoming data from networked nodes via LoRa radio transceivers
      - Store received data onto microSD card; if microSD fails to open, immediately send the data to Hologram
        otherwise wait until user-specified upload interval to send data to Hologram Cloud web host via TCP using cellular module

    Supplemental Functions:
      - Synchronize time with NIST time server on startup and once daily
      - Daily sync check with Nodes (after daily NIST time update)
      - User-interface menu

    Program Summary: dictated by two alarms
      - Alarm 1: wakes up 1 minute after Nodes take measurements
        - Query Nodes sequentially with timestamp, receive data response (over LoRa)
        - Save received data to microSD card or send to Hologram if SD fails
      - Alarm 2: wake up at 4 minutes past upload interval
        - if time for time update: get time from NIST server
        - if time to send data to Hologram: read in data from microSD card, send to Hologram


   Written by:
   Alondra Thompson, USDA-ARS Sustainable Agricultural Systems Lab
   Justin Ayres, Univeristy of Maryland Computer Science Department
   John Anderson, Acclima Inc.

   Last edited: April 20, 2022

   - Version History -

   Version 2020.05.06 fixes the retries for NISTtime() during fieldSync()
   Version 2020.05.08 fixes initialization message when Gateway fails to sync with at least one Node
   Version 2020.05.14 fine tunes initial synchronization and daily sync check
   Version 2020.06.22 For initial fieldSync, send unsent data saved on microSD first (to not delay sync w Nodes)
   Version 2020.07.28 Adds radio frequency to menu header
   Version 2020.08.06 Adds scout mode for measuring cellular signal
   Version 2020.10.29 Adds cell module baud rate detection, sets to 57600 instead of 4800
                      Adds solar current and voltage calcs compatible with new hardware
   Version 2020.11.16 Remove autobaud commands (need to set baud rate on new hardware to 4800 with separate sketch)
   Version 2021.01.06 Fix boxTemp_whole and _dec to account for negative temps
   Version 2021.01.08 Add 10 min interval option, add cellular SS to Gateway data string
   Version 2021.01.27 Remove 3 upload retries per string, add resetAlarm1() to Alarm 2 loop, increase netopen timeout to 30000
   Version 2021.02.01 Change CIP timeouts to match sendATcommand timeouts (netopen, cipopen, cipsend)
   Version 2021.02.04 Add uploadMin variable, set to 3 instead of 4
   Version 2021.03.01 Reset both alarms in low batt loops, check network status after each uploaded string to prevent lockup if network suddenly fails
                      Turn off Alarm 1 interrupt during upload, reset Alarm 1 if it went off, turn on interrupt after upload
   Version 2021.03.15 Remove command to turn on (or off! <= PROBLEM) Alarm 1 at end of senddataSD that was leftover from testing
   Version 2021.04.14 Use statusReg bit status for checking if alarms triggered during sendDataSD; add function checkAlmStat to DS3232RTC library
   Version 2021.06.28 Revise menu, add option to clear SIM FPLMN list
                      Increase timeoutPacket to 9500 from 2000 to accomodate max sensor scenario (approx. packet len = 1950, air time = 2878 ms)
                      Increase number of tries for getting data from each node from 3 to numNodes*3
                      Add getGPRSNetworkStatus, increase NETOPEN timeout to 120000 ms
   Version 2021.07.08 Add Receiver mode and debug  options, check for sim card during pwr_cellular                      
   Version 2021.07.13 Edit sendDataSD so that the dump file doesn't get deleted if one upload fails, won't send junk chars                           
   Version 2021.08.02 Skip fieldSync if battV is low on startup
                      Increase startTimeout in GetNodeData from 500 to 1000
   Version 2021.08.30 Print gateway data string in Receiver Mode 
                      Set defaults for debug and recvMode if there is no info in EEPROM
   Version 2021.09.02 Print gateway data every upload interval instead of measurement interval
   Version 2021.09.09 Remove uploadInt = 0 as indication for Receiver Mode from config string
                      In config string: Device key = 0 = Receiver Mode
   Version 2022.01.07 Remove disabling of watchdog timer
                      Move uploadInterval into if statement (within menu), give 1 min for user
                      input for menu
                      Toggle recvMode depending on if GPRS_on is successful
                      Move checking recvMode flag in various places 
                      Add separate forceRecv flag to keep G in receiver mode at all times
   Version 2022.01.19 Change menu routine to show/hide configuration options
   Version 2022.02.11 Move clear forbidden networks option from config options to user options
   Version 2022.04.20 Replace Adafruit_FONA.h with SIMCOM_7070G.h
                      Edit cellular functions to be capatible with SIM7070G LPWA module:
                      sendInit, NISTtime, sendNIST, sendDataSD, sendDataArray, scoutMode

*/

//===================================================================================================

// ------- Libraries ----------------------------------------------------

#include <SD.h>                                    // SD card library
#include <SPI.h>                                   // SPI library for microSD breakout
#include <Wire.h>                                  // standard I2C library
#include <EEPROM.h>                                // ATmega1284P internal non-volatile memory
#include "avr/sleep.h"                             // sleep functions
#include "DS3232RTC.h"                             // RTC library 
#include <ACReliableMessage.h>                     // replaced radiohead libraries with this one, which uses RadioHead
//#include "Adafruit_FONA.h"                         // for Adafruit FONA 3G cellular breakout
#include "SIMCOM_7070G.h"
#include <PCAL6416.h>
#include "avr/io.h"
#include "avr/interrupt.h"
#include "avr/wdt.h"                               // controls watchdog timer (we want to turn it off)


// ------- Assign Pins ----------------------------------------------------

#define LED                 15       // MoteinoMEGA LED
#define SD_CS               3        // D3 for SD card ChipSelect 
#define hardSS              4
#define FONA_RX             11       // communication w/ FONA if using software serial, hardware Serial1 is the same
#define FONA_TX             10
#define BattV               24       // A0, for calculating Vin from battery
#define Mbatt               14       // Switches voltage divider on/off
#define CELL_RADIO_BAUD     57600    // SIM7070G baud rate
#define baudRate            57600    // Serial baud rate
#define maxNodes            8        // AIT, 14-Jan-2020: Changed from 10
#define FONA_MSG_SIZE       1000     // AIT 06-Mar-2020: Changed to accomodate longer data strings using sensors IDs, was: 350   // max length for sending to Cloud
#define pin_SD_OFF          1        // physical pin 41  D1
#define pin_MOSI            5        // physical pin 1  D5
#define pin_solarVoltage    A5
#define ADC_REF_VOLTAGE     3.3
#define ADC_RESOLUTION      1024
#define pin_solarShort      A4
#define SOLAR_CALIB         1.0          //This will become a EEPROM constant that is set during factory config – for now just use 1.0
#define ADC_MAXVALUE        1023

// ------- Declare Variables -----------------------------------------------

char VERSION[] = "V2022.04.20";

//-----*** Site/Gateway Identifier ***-----

char projectID[6];      // AIT, 17-Mar-2020: use project ID to filter incoming data from multiple devices
unsigned long serNum;   // serial number

//-----*** for EEPROM ***-----

#define EEPROMSHIFT                 2800
#define EEPROM_PROJECTID            (32 + EEPROMSHIFT)        // first storage location for projectID array            
#define EEPROM_DEFAULT_RADIO        (2 + EEPROMSHIFT)         // static default radio address (last 2 digits of serial number)
#define EEPROM_ACTIVE_RADIO         (3 + EEPROMSHIFT)         // active radio address, can be changed by user
#define EEPROM_FLAG_RADIO           (4 + EEPROMSHIFT)         // flag to use active address
#define EEPROM_NODE_COUNT           (5 + EEPROMSHIFT)         // storage location for numNodes                      (was nodesMem)
#define EEPROM_ALRM1_INT            (6 + EEPROMSHIFT)         // storage location for Alarm 1 interval              (was intervalMem)
#define EEPROM_NODEIDS              (7 + EEPROMSHIFT)         // first storage location for nodeIDs array           (was nodeIDsMem)
#define EEPROM_DEVKEY               (20 + EEPROMSHIFT)        // first storage location for device key array        (was devkeyMem)
#define EEPROM_ALRM2_INT            (30 + EEPROMSHIFT)        // storage location for uploadInt                     (was alarm2Mem)  
#define EEPROM_FORCERECV            (70 + EEPROMSHIFT)
#define EEPROM_DEBUG                (71 + EEPROMSHIFT)
#define EEPROM_OPTSRADIO            17
#define EEPROM_SERIALNUM            10
#define EEPROM_iSolarRes            48
#define EEPROM_verHW                16

uint16_t LoRaFREQ;                            // This value is determined by reading EEPROM constant (see method getLoRaFreq())

//-----*** for Main Menu ***-----

int       menuinput;                             // user input in menu
int       numincoming;                           // number of bytes coming from user input
long      timeout;                               // msec to wait for user input
int       indata;                                // integer values entered as user unput
int       input;                                 // holder for converting ASCII value to decimal
int       incoming[7];                           // holder for integer user input (getinput())
char      charInput[20];                         // holder for character user input (charinput())
char      incomingChar[20];                      // holder for incoming char or byte values
uint8_t   u_indata;                              // holder for incoming uint8_t value (byteInput())
bool      clockMenu = false;                     // don't send time update message if clock update initiated form Main Menu
bool      viewConfig = false;

//-----*** for battery voltage calculation ***-----

float   battV;                                   
float   lowBatt = 3.40;                           // low battery limit

//---*** Counters and Flags ***---

byte    i;                                     // counter for user input functions
bool    init1 = true;                          // flag for initial synchronization (fieldSync() on startup)
bool    duringInit = true;                     // controls sending init message to Hologram in syncNodes() and daily sync check in NISTtime()
bool    radioSwitch;                           // flag for using default radio ID or active radio ID
bool    recvMode = false;                      // will skip cellular functions if true
bool    debug = true;
bool    sim_detect = true;
bool    lowInitBatt = false;
bool    forceRecv = false;

//-----*** for LoRa Radio ***-----

//-- Identifiers, Metadata

uint8_t   GatewayID;                            // Gateway radio ID
uint8_t   default_radioID;                      // Default radio ID (last 2 digits of SN)
uint8_t   NodeIDs[maxNodes];                    // networked Node IDs
byte      numNodes;                             // number of nodes connected to Gateway

//-- Radio Settings

#define retryNum        3       // 3 retries should be enough, if our wait intervals are correct
#define timeoutACK      300     // was 120 This is the wait period after a transmission to recieve an ACK for that one packet. (ACKWait)
#define timeoutPacket   9500    // this is the wait period to receive one large packet including retries. (PacketWait)
#define timeoutSmallPkt 1000    // Timeout interval for a small packet (i.e. 20 bytes).  Approx Airtime=timeoutAck.
#define TxPower         17      // default 13 dBm, range: +2 to +17, 21Jan21 changed from 20 to 17

// ---------------------------------------------------------------------------------------------------------------------------
// A BRIEF DISCUSSION ON TIMEOUTS:
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
// ---------------------------------------------------------------------------------------------------------------------------


//-- for Data Storage

String nodeData[maxNodes];
String sep = "~";

//-----*** for RTC ***-----

//-- Time and Date values

byte    secs;                                 // time and date values
byte    mins;
byte    hrs;
byte    days;
byte    mnths;
int     yrs;

tmElements_t tm;
String Timestamp = "";                        // for compiling timestamp into YYYY_MM_DD_hh:mm:ss_UTC format

//-- for Alarms

byte  interval;                               // user-selected loop interval (minutes), interval for Alarm 1
byte  alarmMins;                              // minutes value at which Alarm 1 goes off
byte  uploadInt;                              // # hours between sending data to cloud (Alarm 2)
byte  NISTmin = 35;                           // wake-up to receive sensor IDs @ 12:36am Central time (for Alarm 2)
byte  NISThr = 18;
byte  uploadMins = 3;                         // 4Feb21

// FOR TESTING
//  byte  NISTmin = 20;                            // wake-up to receive sensor IDs @ 12:36am Central time (for Alarm 2)
//  byte  NISThr = 18;
///

bool  timeSuccess = false;                  // tracks success of syncing to NIST server

//-----*** for microSD ***-----

//-- File names

char  datafile[] = "G00.txt";                   // microSD text data file name
char  dumpfile[] = "dump.txt";                  // dump file name (contains only unsent data)

//-- Boolean

boolean dataSaved = false;                      // flag for confirming successful save

//-- for erasing SD card

byte   DeletedCount = 0;                        // number of files deleted
byte   FolderDeleteCount = 0;                   // number of deleted folders
byte   FailCount = 0;                           // delete fails
String rootpath = "/";                          // indicates path

//-----*** for SIM7070G ***-----

//-- Comm with module

//char    response[100];                            // holder for responses from cellular module and network
uint8_t answer;                                   // flag for matching expected response to actual response (sendATcommand())
//char    aux_str[FONA_MSG_SIZE];                   // buffer for compiling AT commands
//boolean fonaON = false;                           // flag for cellular module power status
uint32_t uartBaud = CELL_RADIO_BAUD;
uint8_t printCMRX = 0;                            //debug -- set to zero as default?

//-- Network settings

uint8_t gStatus;                                  // GPRS status
uint8_t n;
char hologramIP[] = "cloudsocket.hologram.io"; 
char hologramPort[] = "9999"; 

//-- for fieldSync

boolean userinput = false;
char toSend[FONA_MSG_SIZE];
unsigned long startTime;

//-- for posting to Hologram Cloud

char    devicekey[9];                             // Hologram device key - authorizes device to send data to Hologram server

// ------- Initialize ----------------------------------------------------

RH_RF95 driverRFM95;                                 // Driver for the RFM95 radio
ACReliableMessage LoRa(driverRFM95, GatewayID);        // LoRa radio manager

SIMCOM_7070G cellRad;    //This is the blank initializer method but requires setting port, baud, and running commInit before use (some functionality such as turning off the modem power is available immediately)

File myfile;    // initialize object for sensor data file
File dump;      // initialize object for dump file
File IDfile;    // initialize object for sensor list file
File root;      // initialize dummy root file

//===========================================================================================

//-------------- Setup ----------------------------------------------------------------------
void setup()
{
  Serial.begin(baudRate);
  delay(100);

  //--- Pin settings

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(SD_CS, OUTPUT);                         // set ChipSelect pin as output
  pinMode(hardSS, OUTPUT);
  digitalWrite(hardSS, HIGH);
//  pinMode(Fona_Key, OUTPUT);
//  digitalWrite(Fona_Key, HIGH);                   // set HIGH so toggle() works
//  pinMode(Fona_PS, INPUT);                        // input; reads HIGH or LOW
  pinMode(BattV, INPUT);                          // reads A0 to calculate SLA battery V
  pinMode(pin_solarVoltage, INPUT);
  pinMode(pin_solarShort, OUTPUT);
  pinMode(pin_SD_OFF, OUTPUT);

  randomSeed(analogRead(4));                      // to randomize IP choices in NISTtime

  if (digitalRead(pin_SIMCOM_PWR_KEY) == HIGH) {
    pwr_cellular(0);    // if cellular is on, turn it off
  }

  //--- Initialize
  // SIM7070G
  
  cellRad.setRXEventHandler(&getModemRX);
  cellRad.setPort(&Serial1);
  cellRad.setBaudrate(uartBaud);
  cellRad.commInit(); 

  // I2C

  Wire.begin();                                   // begin I2C
  delay(300);

  // RTC

  RTC.begin();

  debug = EEPROM.read(EEPROM_DEBUG);
  if (debug != 1 && debug != 0){          // if nothing stored in EEPROM
    debug = true;
    EEPROM.update(EEPROM_DEBUG,debug);
  }  debug = EEPROM.read(EEPROM_DEBUG);

  forceRecv = EEPROM.read(EEPROM_FORCERECV);
  if (forceRecv != 1 && forceRecv != 0){    // if nothing stored in EEPROM
    forceRecv = false;
    EEPROM.update(EEPROM_FORCERECV,forceRecv);
    uploadInt = 1;
    EEPROM.update(EEPROM_ALRM2_INT,uploadInt);  
  }
  
  // SD

  if (debug && !SD.begin(SD_CS)) {
    Serial.println("ERROR: SD init failed");
  }

  //--- Power saving

//  wdt_disable();                                 // turn off watchdog timer
  RTC_osc_off();                                 // Turn off 32kHz output

  //--- Initialize radio settings

  LoRaFREQ = getLoRaFreq();

  EEPROM.get(EEPROM_SERIALNUM, serNum);  // get the serial number from EEPROM
  default_radioID = serNum % 100;
  EEPROM.update(EEPROM_DEFAULT_RADIO, default_radioID); // set default radio ID
  radioSwitch = EEPROM.read(EEPROM_FLAG_RADIO);

  if (radioSwitch) {
    GatewayID = EEPROM.read(EEPROM_ACTIVE_RADIO);                        // read board ID number from EEPROM
  } else {
    GatewayID = default_radioID;
  }

  if (debug && !LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, GatewayID)) {                      // initialize radio
    Serial.println("ERROR: radio init failed");
    return;
  }

  //--- Go to main menu for setup

  MainMenu();
  if (forceRecv) debug = false;

 //--- Time sync in field

  delay(50);
  battV = calcbattV();        // check battery voltage
  if (battV <= lowBatt) {     // if battery voltage below low battery limit
    lowInitBatt = true;
  } else {
    fieldSync();
    getData();
    saveData(); delay(1000);             // if microSD fails, G will send data to Hologram immediately (sendDataArray())
    if(!recvMode){
      sendDataSD();
    }
    resetArrays();
    init1 = false;
    lowInitBatt = false;
  }
  digitalWrite(LED, LOW);

  //--- Initialize alarms

  // clear alarm registers

  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 0); // set alarm values to 0
  RTC.setAlarm(ALM2_MATCH_MINUTES, 0, 0, 0, 0);
  RTC.alarm(ALARM_1);                         // turn off alarm 1
  RTC.alarm(ALARM_2);                         // turn off alarm 2

  // turn alarm interrupts on

  RTC.alarmInterrupt(ALARM_1, true);          // turn on alarm 1 interrupt
  RTC.alarmInterrupt(ALARM_2, true);          // turn on alarm 2 interrupt
  RTC.squareWave(SQWAVE_NONE);                // Use SQW pin as interrupt generator

  readClock();

  // set alarms
  RTC.alarm(ALARM_1);     // turn on Alarm 1
  RTC.alarm(ALARM_2);     // turn on Alarm 2
  setAlarm1();            // set time for Alarm 1
  setAlarm2();            // set time for Alarm 2
  duringInit = false;     // moved 16Jun2020 to set NIST handshake for next day after init
  delay(50);

}

//======================================================================================
//======================================================================================
//======================================================================================

//-----------------*** Main Loop ***----------------------------------------------------

void loop()
{
  sleepNow();

  if (RTC.alarm(ALARM_1)) {     // ALARM 1 controls waking up to get data from Nodes
    battV = calcbattV();        // check battery voltage
    if (battV <= lowBatt) {     // if battery voltage below low battery limit
      setAlarm1();              // set Alarm 1 to one minute after the "measurement" interval, go to sleep
      setAlarm2();              // 23Feb21: set both alarms to stay updated during extended sleeps
      sleepNow();
    } else if (lowInitBatt){
      fieldSync();
      getData();
      saveData(); delay(1000);             // if microSD fails, G will send data to Hologram immediately (sendDataArray())
      if(!forceRecv){
        sendDataSD();
      }
      resetArrays();
      init1 = false;
      lowInitBatt = false; 
    } else {
      digitalWrite(LED, HIGH);
      getData();                // request data from Nodes (received as String objects)
      saveData();               // save received data to SD card
      resetArrays();            // erase contents of String buffer
      digitalWrite(LED, LOW);
      setAlarm1();              // set Alarm 1 to one minute after the "measurement" interval
    }
  }   // end if alarm 1

  else if (RTC.alarm(ALARM_2)) { // ALARM 2 controls waking up to upload data to Hologram and daily NIST sync
    battV = calcbattV();
    if (battV <= lowBatt) {     // Again, G will go to sleep if battV too low
      setAlarm1();              // 23Feb21
      setAlarm2();
      sleepNow();
    } else if (lowInitBatt){
      fieldSync();
      getData();
      saveData(); delay(1000);             // if microSD fails, G will send data to Hologram immediately (sendDataArray())
      if(!forceRecv){
        sendDataSD();
      }
      resetArrays();
      init1 = false;
    }
    else {
      readClock();
      if (hrs == NISThr && mins == NISTmin) { // time to get time from NIST
        digitalWrite(LED, HIGH);
        pwr_cellular(1);
        delay(3000);
        timeSuccess = NISTtime();           // Get time from NIST, send to Nodes
        pwr_cellular(0);
        digitalWrite(LED, LOW);
        delay(1000);
      }
      else {
        if(!forceRecv) {
          digitalWrite(LED, HIGH);
          sendDataSD();                       // upload data from DUMP file to Hologram
          digitalWrite(LED, LOW);
        } else {
          gatewayInfo();
        }
      }
      setAlarm2();
    }
  }   // end if Alarm 2
 
}
//======================================================================================
//======================================================================================
//======================================================================================
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
//======================================================================================

//------------- Sleep Functions --------------------------------------------------------

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
  Serial.end();         // turn off Serial comm
  driverRFM95.sleep();  // put radio to sleep
  analogComp_off();     // turn off analog comparator
  ADC_off();            // turn off ADC
  JTAG_off();           // disable On-Chip Debug system
  SD_off();             // turn off SD card

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);           //Power down completely on sleep
  sleep_enable();
  setRTCInterrupt();
  sleep_mode();                                  // puts MEGA to low-power sleep

  // after wake-up interrupt

  ADC_on();                 // Starts here after wakeup
  SD_on();
  Serial.begin(baudRate);
  delay(1000);

}


//======================================================================================

//------------- Power Savers -----------------------------------------------------------

void RTC_osc_off() {
  //Turn RTC 32kHz oscillator off to save power:
  uint8_t RTC_SR;
  uint8_t res;
  RTC_SR = RTC.readRTC(0x0F); //First read status register
  res = RTC.writeRTC(0x0F, RTC_SR & ~(1 << 3)); //Now re-write status register bits (bit 3 needs to be set to 0 to disable 32kHz output)
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

void SD_off() {
  digitalWrite(pin_SD_OFF, HIGH);

  SPCR &= 0B10111111;  //disable SPI  -- I prefer to use this instead of spi.end() because spi.end won’t disable hardware SPI if there is more than one instance in use

  pinMode(pin_MOSI, OUTPUT);
  digitalWrite(pin_MOSI, LOW);
}

void SD_on() {
  digitalWrite(pin_SD_OFF, LOW);
  SPCR |= 0B01000000;  //enable SPI
}



//======================================================================================

//---------- Sync time with Nodes after field install, get sensor lists ----------------

void fieldSync() {

  if (duringInit && !recvMode && !forceRecv) {
    sendDataSD(); //29Jun20: send data in dump file
    startTime = millis();
  }

  Serial.println();
  Serial.println("fieldSync(): Syncing time with Nodes...");

  //--- Step 1: Get time from NIST

  pwr_cellular(1);
  delay(3000);
  if (sim_detect) {
    timeSuccess = NISTtime();
    pwr_cellular(0);
    delay(5000);
    if (!timeSuccess && recvMode) {
      Serial.println("NIST clock update failed");
    }
    else if (!recvMode){
      pwr_cellular(1);
      sendNIST();         // 21-Apr-2020: send confirmation to Hologram/Slack
      pwr_cellular(0);
    } else if (timeSuccess && recvMode){
      Serial.println("NIST clock update successful");
    }
  }

  //--- Step 2: Send timestamp to Nodes, try for up to 2 hrs

  if (!LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, GatewayID)) {           // initialize radio
    return;
  }

  unsigned long totalTime = 7200000; // 13May20
  //  unsigned long totalTime = 900000; // 14May20 TESTING: 15 minutes
  unsigned long timeStart = millis();
  unsigned long waitTime = totalTime - (startTime + timeStart);
  
  syncNodes(totalTime);  
  delay(5000);

  //--- Step 3: Send init message to Hologram
  if(!recvMode){
    pwr_cellular(1);
    delay(3000);
  
    if (gStatus == 1) { // if cell module on and connected to network
      if (!sendInit()) {                                // send initialization message to Hologram, if fails, save to SD card for later upload
        delay(20);
        dump = SD.open(dumpfile, FILE_WRITE);           // only saves to dump file for upload, not to data file
        delay(50);
        if (dump) {
          dump.println(toSend);
        }
        delay(50);
        dump.close();
      } else {
        pwr_cellular(0);
        delay(1000);
      }
    }
  } else {
    delay(20000);
  }
  
  if (!recvMode) Serial.println("Done");
}

// ------------------------ GetNodeData --------------------------------
// Gather data from a single radio node

bool GetNodeData(String* msg, uint8_t NodeAddr) {
  readClock();

  *msg = "";                  // Build timestamp
  *msg += String(yrs);
  *msg += "/";
  *msg += String(mnths);
  *msg += "/";
  *msg += String(days);
  *msg += "|";
  *msg += String(hrs);
  *msg += ":";
  *msg += String(mins);
  *msg += ":";
  *msg += String(secs);
  //  Serial.println(*msg);

  uint8_t len_resp = (*msg).length() + 1;
  uint8_t resp[len_resp];
  for (byte i = 0; i <= len_resp; i++) {             // convert timestamp to uint8_t for sending
    resp[i] = (*msg)[i];
  }

  *msg = "";

  if (LoRa.sendtoWait(resp, len_resp, NodeAddr)) {   // send timestamp, which should trigger a data response
    uint8_t from;
    if (LoRa.recvMessage(msg, timeoutSmallPkt, timeoutPacket, &from)) {  // 14Jan20. 21Jan21 set back to 500ms
      return true;
    } // end successful reception
  } // end successful transmission
  *msg = "";

  return false;
}

//---------- Sync time with Nodes ----------------------------------

void syncNodes(uint32_t time2wait) {  // 22-Jan-2020: add parameter time2wait
  uint32_t timeToWait = time2wait;
  unsigned long StartTimeToWait = millis();
  
  if(debug && !recvMode){
    Serial.println();
    Serial.println("syncNodes(): Querying Nodes with timestamp...");
  }
  String synced = "";
  String unsynced = "";

  uint8_t unsyncedNodes[numNodes];    // keep track of unsynced nodes, synced ones get set to 0

  for (byte f = 0; f < numNodes; f++) {
    unsyncedNodes[f] = NodeIDs[f];
  }

  boolean done = false;
  String nodeResponse;
  uint8_t numAttempts;

  while (((millis() - StartTimeToWait) < timeToWait) && (done == false)) {    // 27-Jan-2020: change twoHours to timeToWait
    done = true;                            // if we can make it through the list, we will be done
    for (byte y = 0; y < numNodes; y++) {   // iterate through all networked Nodes
      uint8_t to = unsyncedNodes[y];
      if (to == 0) continue;                // this node has synced already - skip to the next
      if(debug){
        Serial.print("Pinging Node ");
        Serial.println(to);
      }
      for (numAttempts = 0; numAttempts < 3; numAttempts++) {   // try a node 3 times
        if (GetNodeData(&nodeResponse, to)) {                            // get data from node
          unsyncedNodes[y] = 0;                                         // mark this one as complete
          Serial.print("Received response from ");
          Serial.println(to);
          break;
        }
      }
      if (numAttempts >= 3) {
        done = false;                       // oops - we missed one!
      }
    } // end for each node
  } // End of Syncing all nodes

  if (init1) { // only send message to Hologram during fieldSync, not during daily sync check
    for (byte i = 0; i < numNodes; i++) { // list out synced and unsynced radio IDs    if(missedSyncs == true){
      if (unsyncedNodes[i] == 0) {
        synced += NodeIDs[i];
        synced += ' ';
      } else {
        unsynced += unsyncedNodes[i];
        unsynced += ' ';
      }
    }

//    Serial.print("synced: ");
//    Serial.println(synced);

    byte syncLen = synced.length() + 1;
    char syncSend[syncLen];
    synced.toCharArray(syncSend, syncLen);
    syncSend[syncLen - 1] = 0;

    readClock();
    if (done == false) {
      byte unsyncLen = unsynced.length() + 1;
      char unsyncSend[unsyncLen];
      unsynced.toCharArray(unsyncSend, unsyncLen);
      unsyncSend[unsyncLen - 1] = 0;
      delay(50);
      sprintf(toSend, "%lu(%d) Gateway awake at %d/%d/%d %d:%d. Synced to Nodes: %s Missed Nodes: %s.\0", serNum, GatewayID, mnths, days, yrs, hrs, mins, syncSend, unsyncSend);
      delay(100);
    } else {
      sprintf(toSend, "%lu(%d) Gateway awake at %d/%d/%d %d:%d. Synced to Nodes: %s Missed Nodes: none.\0", serNum, GatewayID, mnths, days, yrs, hrs, mins, syncSend);
      delay(50);
    }
    if(debug) Serial.println(toSend);

  } else {    // during daily sync check, give Nodes the ok to stop listening
    uint8_t ok[2];
    uint8_t okLen = 2;
    uint8_t node;
    uint8_t stopNow[1];

    stopNow[0] = GatewayID;

    for (byte y = 0; y < numNodes; y++) {
      uint8_t to = NodeIDs[y];
      if (LoRa.sendtoWait(stopNow, 1, to)) { // send message to Nodes to stop listening and send data
        LoRa.recvfromAckTimeout(ok, &okLen, timeoutACK, &node); // receive ack from Node
      }
    }
  }
}

//---------- Send init confirmation to Hologram ----------------------------------

boolean sendInit() {
  if (debug){ 
    Serial.println();
    Serial.println("sendInit(): Sending init info to Hologram...");
  }
  
  if (gStatus == 1) {
    int init_len;
    for (int i = 0; i < sizeof(toSend); i++){
      if (toSend[i] == 0){
        init_len = i;
        break;
      }
    }

    char init_msg[init_len];
    for (int i = 0; i < init_len; i++){
      init_msg[i] = toSend[i];
    }
    
    int msg_len = sizeof(devicekey) + init_len + 8 + 31;
    char msg[msg_len];
    sprintf(msg, "{\"k\":\"%s\",\"d\":\"%s\",\"t\":[\"%lu\",\"init\"]}\r", devicekey, init_msg, serNum); // compile json
    delay(60);
//    Serial.println(msg[msg_len - 3],DEC);
//    Serial.println(msg[msg_len - 2],DEC);
//    Serial.println(msg[msg_len - 1],DEC);
    
    
    //--- Open TCP socket and send data
    
    uint16_t IPlen = sizeof(hologramIP);
    uint16_t portLen = sizeof(hologramPort);
    
    answer = cellRad.openTCP(hologramIP,IPlen,hologramPort,portLen);
    delay(60);

    if (answer == 1) {
      delay(60);
      answer = cellRad.sendMsg(msg, msg_len, 10000);  // send Gateway data

      answer = cellRad.closeTCP();                     // close TCP socket
    }
  }
  else {
    Serial.println("ERROR: Failed to connect to Hologram");
    return false;
  }
  delay(1000);

}

//---------- Send message of successful NIST contact ----------------------------------

void sendNIST() {

  if(debug){
    Serial.println();
    Serial.println("sendNIST(): Sending confirmation to Hologram...");
  }

  if (gStatus == 1) {

    //--- Prepare message, Open TCP socket and send data
    
    uint16_t IPlen = sizeof(hologramIP);
    uint16_t portLen = sizeof(hologramPort);
        
    delay(60);

    timestamp();
    byte len = Timestamp.length() + 1;
    char timeSend[len];
    Timestamp.toCharArray(timeSend, len); delay(20);
    int nist_len = len + sizeof(devicekey) + 8 + 46;
    char nist_send[nist_len];

    sprintf(nist_send, "{\"k\":\"%s\",\"d\":\"Successful time update %s\",\"t\":[\"%lu\"]}\r", devicekey, timeSend, serNum); // Compile message to send
    delay(60);
//    Serial.println(nist_send[nist_len - 3], DEC);
//    Serial.println(nist_send[nist_len - 2], DEC);
//    Serial.println(nist_send[nist_len - 1], DEC);

    answer = cellRad.openTCP(hologramIP,IPlen,hologramPort,portLen);
    
    if (answer == 1) {
      delay(60);
      answer = cellRad.sendMsg(nist_send, nist_len, 10000);  // send Gateway data

      answer = cellRad.closeTCP();                     // close TCP socket
    }
  }
  else {
    if(debug) Serial.println("ERROR: Failed to connect to Hologram");
  }
  delay(1000);

}
//======================================================================================

//---------- Read DS3231 RTC -----------------------------------------------------------

void readClock() {
  RTC.read(tm);

  secs = tm.Second;
  mins = tm.Minute;
  hrs = tm.Hour;
  days = tm.Day;
  mnths = tm.Month;
  yrs = tm.Year + 1970;   //tm.Year --> years since 1970
  delay(50);
}

float getBoxT() {         // get enclosure temp from temp sensor on RTC
  float boxTemp;
  boxTemp = (float)RTC.temperature() / 4.0;
  return boxTemp;
}

//======================================================================================

//---------- Compile timestamp String --------------------------------------------------

void timestamp() {
  Timestamp = "";
  readClock();

  Timestamp += yrs;
  Timestamp += '-';
  Timestamp += mnths;
  Timestamp += '-';
  Timestamp += days;
  Timestamp += '_';
  Timestamp += hrs;
  Timestamp += ':';
  if (mins < 10) Timestamp += '0';
  Timestamp += mins;
  Timestamp += ':';
  if (secs < 10) Timestamp += '0';
  Timestamp += secs;
  Timestamp += "_UTC";
  delay(50);
}

//======================================================================================

//---------- Set Alarms ----------------------------------------------------------------

//---------- Set Alarm 1  -----------------------

void setAlarm1() {
  readClock();

  alarmMins = ((((mins + interval) % 60) / interval) * interval) + 1; // set alarm 1 to 1 minute after measurement interval

  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, alarmMins, 0, 0);

  if(debug && !recvMode){
    Serial.print("Alarm 1 set for: ");
    Serial.println(alarmMins);
  }
  delay(50);
}

//---------- Set Alarm 2  -----------------------

void setAlarm2() {

  readClock();

  byte alarm2Mins;
  byte alarm2Hrs;
  bool alarmSet = false;

  // 29Jun20: remove nextday variable and calc
//  if(!recvMode){  
    if (uploadInt == 4) {                         // if upload interval = 4
      if (hrs + 2 == NISThr) {   // if next alarm should be time to get NIST time
        alarm2Hrs = NISThr % 24;
        alarm2Mins = NISTmin;
      }
      else if (hrs == (NISThr % 24)) {            // alarm after getting NIST time
        alarm2Hrs = (hrs + 2) % 24;
        alarm2Mins = uploadMins;
      }
      else {
        alarm2Hrs = (hrs + uploadInt) % 24;
        alarm2Mins = uploadMins;
      }
    } else if (uploadInt == 1) {                   // if upload interval is every hour
      if (hrs == NISThr && mins < NISTmin) {       // if next alarm should be time to get NIST time
        alarm2Hrs = NISThr;
        alarm2Mins = NISTmin;
      } else if (hrs == NISThr && mins >= NISTmin) { // alarm after getting NIST time
        alarm2Hrs = (NISThr % 24) + 1;
        alarm2Mins = uploadMins;
      } else {
        alarm2Hrs = (hrs + uploadInt) % 24;
        alarm2Mins = uploadMins;
      }
    } 
//  } else {
//    alarm2Hrs = NISThr;
//    alarm2Mins = NISTmin;
//  }

  // Note for Alarm 2:
  // ALM2_MATCH_HOURS -- causes an alarm when the hours and minutes match.
  // ALM2_MATCH_DATE -- causes an alarm when the date of the month and hours and minutes match.

  if (!alarmSet) {
    RTC.setAlarm(ALM2_MATCH_HOURS, 0, alarm2Mins, alarm2Hrs, 0);
  }

  if(debug && !recvMode){
    Serial.print("Alarm 2 set for: ");
    Serial.print(alarm2Hrs);
    Serial.print(":");
    Serial.println(alarm2Mins);
  }
  delay(50);

}

//=======================================================================================

//--------------- Calculate Battery Voltage ---------------------------------------------

float calcbattV() {
  float   resist = 1.333;         // for voltage divider: R1=10k, R2=30k, resist =  (R1+R2)/R2
  float   multiplier = 0.00322;   // analog input resolution (3.3V / 1024 units)
  float result;
  analogComp_on();
  digitalWrite(Mbatt, HIGH);      // turn on voltage divider
  delay(50);

  int Aout;
  Aout = analogRead(BattV);       // analogRead returns an integer between 0 and 1023
  result = Aout * multiplier * resist;

  digitalWrite(Mbatt, LOW);       // turn off voltage divider
  return result;
}

//======================================================================================

//---------- Get data from Nodes -------------------------------------------------------

void getData() {
  if(!recvMode){
    Serial.println();
    Serial.println("getData(): Querying Nodes for data...");
  }
//  gatewayInfo();
  dataSaved = false;

  // Step 1 - turn on radio

  if (debug && !LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, GatewayID)) {     // initialize radio
    Serial.println("radio failed");
  }

  delay(2000);

  // Step 2 - Send timestamp to Ns one by one

  uint8_t nodeIdlist[maxNodes];             // array for holding Node radio IDs
  numNodes = EEPROM.read(EEPROM_NODE_COUNT);
  EEPROM.get(EEPROM_NODEIDS, nodeIdlist);

  if(debug && !recvMode){
    for (byte i = 0; i < numNodes; i++)       // populate array with saved radio IDs
    {
      Serial.print("nodeIdlist[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.println(nodeIdlist[i]);
    }
  }

  uint8_t to;             // radio ID of Node to ping
  bool missedOne = true;  // flag if didn't data from Node, triggers a retry
  int  tries;             // number of times to ping each Node

  if (duringInit) {
    tries = 30;           // give more time during initial synchronization
  } else {
    tries = numNodes * 3; // 17Jun21: increase from 3
  }

  for (int n = 0; ((n < tries) && (missedOne == true)); n++) {   // try each node "tries" times
    missedOne = false;
    for (byte y = 0; y < numNodes; y++) {                        // iterate through all networked Nodes
      to = nodeIdlist[y];
      if (to == 0) continue;                                     // skip anything that is empty or complete
      String msg;
      if (GetNodeData(&msg, to)) {
        int8_t rssi = driverRFM95.lastRssi();
        nodeData[y] = msg + sep + String(rssi);                  // save the data
        nodeIdlist[y] = 0;                                       // mark as complete
        Serial.println(nodeData[y]);
        //getFreeRAM(); // 21Jan21
      }

      else missedOne = true;
    }   // end for loop
  } // end of attempts
  if (!recvMode) Serial.println("Done");
}

//======================================================================================

//---------- Get time from NIST server, update RTC -------------------------------------

boolean NISTtime() {

  if (sim_detect){    
    unsigned long nistStart = millis(); //14May20
    Serial.println();
    Serial.println("NISTtime(): Getting time from NIST...");
    timeSuccess = false;
  
    char IPs[4][12] = {"129.6.15.28", "129.6.15.29", "129.6.15.30", "129.6.15.27"}; // NIST has 4 IPs to try to not overload any of them
    char ipaddr[12];
  
    long y = random(0, 4);
  
    for (byte u = 0; u < 12; u++) {
      ipaddr[u] = IPs[y][u];      // pick a random IP address of the four
    }
    uint16_t IPsize = sizeof(ipaddr);
    char port[] = "13";
    uint16_t portSize = sizeof(port);
  
    if (gStatus == 1) {
      bool tcp_open = cellRad.openTCP(ipaddr,IPsize,port,portSize);
      char getTime[] = "AT+CARECV=0,100\r";
      uint16_t nistLen = cellRad.sendAT(getTime,500,3000);
      char nistResp[nistLen];
//      Serial.println(nistLen);
      for(byte i = 0; i < nistLen; i++){
         nistResp[i]=Serial1.read();
         if (debug) Serial.print(nistResp[i]);
      }
      delay(500);
      parseResponse(nistResp);
      delay(500);
      cellRad.closeTCP();     
      timeSuccess = true;
    } else {
      delay(500);
      if(debug) Serial.println("ERROR: No connection to network. Network time update unsuccessful.");
      timeSuccess = false;
    }

    Serial.println("Done");
  
    if (!duringInit) {
      unsigned long timeNow = millis();   // 14May20
      unsigned long totalTime = 90000;
      unsigned long timeSync = totalTime - (timeNow - nistStart);
      syncNodes(timeSync);     // 27-Jan-2020: daily sync check with Nodes
    }
  
    return timeSuccess;
  }
}

//---------- Decode response from NIST & Update clock  -----------------------

void parseResponse(char response[100]) {
  int i = 0;
  while (response[i] != '-') i++;

  if (debug && i >= 100) {
    Serial.println("ERROR: Could not parse response");
    return;
  }

  int yr = (10 * (response[i - 2] - 48) + (response[i - 1] - 48)); // comes as a 2-digit year
  byte mnth = 10 * (response[i + 1] - 48) + (response[i + 2] - 48);
  byte dy = 10 * (response[i + 4] - 48) + (response[i + 5] - 48);
  i += 7;
  byte hr = 10 * (response[i] - 48) + (response[i + 1] - 48);
  byte mins = 10 * (response[i + 3] - 48) + (response[i + 4] - 48);
  byte sec = 10 * (response[i + 6] - 48) + (response[i + 7] - 48);

  if (debug){
    Serial.print("Timestamp: ");
    Serial.println(String(dy) + "-" + String(mnth) + "-" + String(yr) + " " + String(hr) + ":" + String(mins) + ":" + String(sec));
  }
  
  if ((yr < 16) || (yr > 50) || (mnth > 12) || (mnth < 1) || (dy > 31) || (dy < 1)) {   // check for invalid time variables
    if (debug) Serial.println("ERROR: Invalid timestamp");
    timeSuccess = false;
    return;
  } else {    // accept timestamp, update RTC
    tm.Year = yr + 30;   // netY + 2000 - 1970
    tm.Month = mnth;
    tm.Day = dy;
    tm.Hour = hr;
    tm.Minute = mins;
    tm.Second = sec;

    RTC.write(tm);
    delay(50);

    timeSuccess = true;
  }
}

//======================================================================================

//--------------- Save Data to SD ------------------------------------------------------

void saveData() {

  if (!SD.begin(SD_CS)) {}
  
  if(!recvMode){
    Serial.println();
    Serial.println("saveData(): Saving data to SD card...");
  }
  dataSaved = false;
  delay(50);

  myfile = SD.open(datafile, FILE_WRITE);   // write received transmissions to SD
  delay(100);
  dump = SD.open(dumpfile, FILE_WRITE);
  delay(100);

  if (dump) {
    for (byte j = 0; j < numNodes; j++) {   // write data into both SD text files
      myfile.println(nodeData[j]);
      delay(100);
      dump.println(nodeData[j]);
      delay(100);
      if(debug && !recvMode){
        Serial.println("----Data----");
        Serial.println(nodeData[j]);
        delay(100);
        Serial.println("----Data----");
      }
    }

    myfile.close();
    delay(200);
    dump.close();
    delay(200);
    dataSaved = true;

  } else {
//    if(!recvMode){
      if(debug) Serial.println("ERROR: Failed to Open SD. Sending Data Now...");
      sendDataArray();        // upload data immediately to Hologram if SD fails
//    }
    setAlarm2();      // 22Feb21
  }
  if (!recvMode) Serial.println("Done");
}

//======================================================================================

//--------------- Clear data array ----------------------------------------------

void resetArrays() {
  for (byte index = 0; index < numNodes; index++) {
    nodeData[index] = "";
  }
}

//======================================================================================

//--------------- Compile Gateway Data ----------------------------------------------

void gatewayInfo(){
  uint16_t pvCurrent = GetISolar();   // using John's code
  float pvVoltage = GetVSolar();

  char aux_str2[FONA_MSG_SIZE];

  timestamp();                                        // compile timestamp String
  byte len1 = Timestamp.length() + 1;
  char timeSave[len1];
  Timestamp.toCharArray(timeSave, len1); delay(20);   // convert String to char array

  battV = calcbattV();                                // get Li-ion battery voltage

  float boxTemp = getBoxT();                          // get enclosure temperature
  
  //--- Convert floats to arrays
  
  String str_battV = String(battV);
  uint8_t bvLen = str_battV.length() + 1;
  char bv[bvLen];
  str_battV.toCharArray(bv,bvLen);
  bv[bvLen - 1] = 0;

  String solarV = String(pvVoltage);
  uint8_t pvLen = solarV.length() + 1;
  char pv[pvLen];
  solarV.toCharArray(pv,pvLen);
  pv[pvLen -1] = 0;

  String boxT = String(boxTemp);
  uint8_t bTlen = boxT.length();
  char bT[bTlen]; 
  boxT.toCharArray(bT,bTlen);
  bT[bTlen - 1] = 0;

  String pvC = String(pvCurrent); 
  uint8_t pvcLen = pvC.length() + 1;
  char pvc[pvcLen];
  pvC.toCharArray(pvc,pvcLen);
  pvc[pvcLen - 1] = 0; 
  byte battV_whole = battV / 1;
  int battV_dec = (battV * 100);
  battV_dec = battV_dec % 100;

  uint16_t gData_len = sizeof(VERSION) + sizeof(projectID) + 8 + bvLen + bTlen + pvcLen + pvLen + sizeof(timeSave);
  char gData[gData_len];                           // array for Gateway data, compiled below
  sprintf(gData, "%s~%s~%lu~%s~%s~%s~%s~%s", VERSION, projectID, serNum, bv, bT, pvc, pv, timeSave);
  if (debug) Serial.println(gData);
  
  if (!SD.begin(SD_CS)) {}
  delay(20);
  
  myfile = SD.open(datafile, FILE_WRITE);   // save power data to SD
  delay(100);
  if (myfile) {
    myfile.print(gData);
    myfile.print("~"); delay(50);
    myfile.close();
    delay(200);
  }

}

//======================================================================================

//--------------- Send data to Hologram Cloud ----------------------------------------------

//--------------- from SD card

void sendDataSD() {

  if (!init1) {
    RTC.alarmInterrupt(ALARM_1, false);          // turn off alarm interrupts
  }
  if (debug) Serial.println("sendDataSD(): Sending data from SD...");    // Send node data

  uint16_t pvCurrent = GetISolar();   // using John's code
  float pvVoltage = GetVSolar();

  timestamp();                                        // compile timestamp String
  byte len1 = Timestamp.length() + 1;
  char timeSave[len1];
  Timestamp.toCharArray(timeSave, len1); delay(20);   // convert String to char array

  battV = calcbattV();                                // get Li-ion battery voltage

  float boxTemp = getBoxT();                          // get enclosure temperature
  

  //--- Convert floats to arrays

  String str_battV = String(battV);
  uint8_t bvLen = str_battV.length() + 1;
  char bv[bvLen];
  str_battV.toCharArray(bv,bvLen);
  bv[bvLen - 1] = 0;

  String solarV = String(pvVoltage);
  uint8_t pvLen = solarV.length() + 1;
  char pv[pvLen];
  solarV.toCharArray(pv,pvLen);
  pv[pvLen -1] = 0;

  String boxT = String(boxTemp);
  uint8_t bTlen = boxT.length() + 1;
  char bT[bTlen]; 
  boxT.toCharArray(bT,bTlen);
  bT[bTlen - 1] = 0;

  String pvC = String(pvCurrent); 
  uint8_t pvcLen = pvC.length() + 1;
  char pvc[pvcLen];
  pvC.toCharArray(pvc,pvcLen);
  pvc[pvcLen - 1] = 0; 


  //--- Compile gateway data string to save
  
  uint16_t gData_len = sizeof(VERSION) + sizeof(projectID) + 8 + bvLen + bTlen + pvcLen + pvLen + sizeof(timeSave);
  char gData[gData_len];                           // array for Gateway data, compiled below
  sprintf(gData, "%s~%s~%lu~%s~%s~%s~%s~%s", VERSION, projectID, serNum, bv, bT, pvc, pv, timeSave);
  delay(50);
//  Serial.println(gData);
 
  if (!SD.begin(SD_CS)) {}
  delay(20);

  myfile = SD.open(datafile, FILE_WRITE);   // save power data to SD
  delay(100);
  if (myfile) {
    myfile.print(gData);
    myfile.print("~"); delay(50);         // add 1 to gData_len
    myfile.close();
    delay(200);
  }

  //--- Send data to Hologram

  pwr_cellular(1);
  delay(1000);

  int8_t dBm;

  char c[1];
  int pos = 0;
  c[0] = 0;
  int eof = 0;

  if (gStatus == 1 && !recvMode) {    // added 05Jan22, without recvMode part G still tries to connect
    boolean tcpSent = true;   // 12Jul21 want to know when send fails    

    //--- Get and save RSSI
    
    dBm = cellRad.getRSSIdBm();

    uint8_t div10 = dBm / -10;
    uint8_t digits;
    if (div10 > 9){
      digits = 3;
    } else if (div10 == 1){
      digits = 2;
    } else if (div10 == 0){
      digits = 1;
    }

    String rs_dbm = String(dBm);
    uint8_t dbm_len = rs_dbm.length() + 1;
    char rssi[dbm_len];
    rs_dbm.toCharArray(rssi,dbm_len);
    rssi[dbm_len - 1] = 0;
    
    if (!SD.begin(SD_CS)) {}
    delay(20);

    myfile = SD.open(datafile, FILE_WRITE);   // save RSSI to SD
    delay(50);
    if (myfile) {
      myfile.println(dBm);
      myfile.close();
      delay(200);
    }
       
    //--- Compile gData message
    
    uint16_t gData_len = sizeof(VERSION) + sizeof(projectID) + 8 + bvLen + bTlen + pvcLen + pvLen + sizeof(timeSave);   // have to do it again, previous one is out of scope
    char gData[gData_len];                           // array for Gateway data, compiled below
    sprintf(gData, "%s~%s~%lu~%s~%s~%s~%s~%s", VERSION, projectID, serNum, bv, bT, pvc, pv, timeSave); delay(20);
    if (debug) Serial.println(gData);
       
    uint16_t gMsgLen = gData_len + dbm_len + 8 + 47; //48; <-- DON'T INCLUDE NULL IN MSG
    char gMsg[gMsgLen];
    sprintf(gMsg,"{\"k\":\"%s\",\"d\":\"%s~%s\",\"t\":[\"%lu\",\"GATEWAY_DATA\"]}\r",devicekey, gData, rssi, serNum);
//    Serial.println(gMsg);
//    Serial.println(gMsg[gMsgLen - 3],DEC);
//    Serial.println(gMsg[gMsgLen - 2],DEC);  
//    Serial.println(gMsg[gMsgLen - 1],DEC);

    //--- Open TCP socket and send data
    
    uint16_t IPlen = sizeof(hologramIP);
    uint16_t portLen = sizeof(hologramPort);
    
    answer = cellRad.openTCP(hologramIP,IPlen,hologramPort,portLen);
    delay(60);

    if (answer == 1) {
      delay(60);
      answer = cellRad.sendMsg(gMsg, gMsgLen, 10000);  // send Gateway data

      answer = cellRad.closeTCP();                     // close TCP socket
      

    //--- Read and send node data from SD card

    
    while (eof != -1) {    
      if (n == 5) {   //25Feb21: added n==5, 02Mar21: moved to if statement
        if (!SD.exists(dumpfile)) {
          if (debug) Serial.println("ERROR: No Data in SD to Send");
          delay(50);
          return;
        }
        if (!(dump = SD.open(dumpfile, FILE_READ))) {
          if (debug) Serial.println("ERROR: File-open Failed");
          delay(50);
          return;
        }

        char dataToSend[FONA_MSG_SIZE];
        memset(dataToSend, 0, sizeof(dataToSend));
        
        // Jumps to current position in file 
        if (!dump.seek(pos)) {
          eof = -1;
          dump.close();
          break;
        }
        eof = dump.read(c, 1);
        if (eof == -1) {
          dump.close();
          break;
        }
        int ind = 0;
        delay(50);
        
        int len = 0;      // calculate exact length of data string

        while (eof != -1 && c[0] != '\n' && c[0] != 0) {    // populate array with row of data from DUMP file
          if((c[0] > 31 && c[0] < 127) || c[0] == 13 || c[0] == 10){            // 13Jul21: don't send characters that Hologram will reject
            dataToSend[ind++] = c[0];
            if (c[0] == '\r'){     
              len = ind - 1 ;
            }
          }
          if ((eof = dump.read(c, 1)) == -1) break;  
        }

        delay(50);
        if (c[0] == '\n' || c[0] == '\r')     // not sure if this is throwing things off 
          pos += ind + 1;
        if (c[0] == 0)
          eof = -1;
        if(debug){ 
          Serial.println("Data to send from SD:");
          Serial.println("-------Start-------");
          delay(50);
          Serial.print((int)(dataToSend[0]));
          Serial.print(" : ");
          Serial.println(eof);
          Serial.println("--------End--------");
          delay(100);
        }

        // Do not send anything if blank
        //--- Compile data into a json string
        
            char Nmsg[len+1];         // new array of correct length

            for (uint16_t i = 0; i < len; i++){
              Nmsg[i] = dataToSend[i];
//              Serial.print(Nmsg[i]);
            }
            Nmsg[len] = 0;

            uint16_t nMsgLen = len + 8 + 44; // <-- DON'T INCLUDE NULL IN MSG

        if (dataToSend[0] != 0 && dataToSend[0] != '\n' && dataToSend[0] != '\r') {

          answer = cellRad.openTCP(hologramIP,IPlen,hologramPort,portLen);          // open TCP socket
          if (answer == 1) {

            char nMsg[nMsgLen];
            sprintf(nMsg,"{\"k\":\"%s\",\"d\":\"%s\",\"t\":[\"%lu\",\"NODE_DATA\"]}\r", devicekey, Nmsg, serNum);

            answer = cellRad.sendMsg(nMsg, nMsgLen, 5000);
            answer = cellRad.closeTCP();
 
          } else {
            if(debug) Serial.println("ERROR: Failed to connect to Hologram");
          }

        }   // end if dataToSend[0] != 0

        c[0] = 0;
        dump.close();
        delay(100);

      } else {
        break;
      }
      n = cellRad.getNetworkStatus();   // check if we're still connected

    }  // end while !eof loop

    delay(100);
      
      if (eof == -1 && tcpSent == true) {    // 13Apr21: only delete dump file if successfully sent
        if (SD.remove(dumpfile)){
         if(debug) Serial.println("File Cleared");
        } else {
          if(debug) Serial.println("File Clear Failed");
        }
      } else if (tcpSent == false){
        if(debug) Serial.println("Upload failed. File not cleared.");
      }
    } else {
      if (debug) Serial.println("Failed to connect to Hologram");
    }
  } // end if gStatus == 1

  delay(3000);
  pwr_cellular(0);

  if (recvMode) gatewayInfo();  // 05Jan22 save gateway info to SD if G not connected to network

  // Check Alarms
  if (!duringInit) {
    uint8_t statusReg = RTC.checkAlmStat();

    if (statusReg == 1) {
      setAlarm1();
      RTC.alarm(ALARM_1); // reset statusReg to 0
    }
    else if (statusReg == 2) {
      setAlarm2();
      RTC.alarm(ALARM_2);
    }

    RTC.alarmInterrupt(ALARM_1, true);          // turn on alarm 1 interrupt
    RTC.alarmInterrupt(ALARM_2, true);
  }

  if (debug) Serial.println("Done sendDataSD()");
  
}

//--------------- from array if SD fails

void sendDataArray() {

  timestamp();                                        // compile timestamp String
  byte len1 = Timestamp.length() + 1;
  char timeSave[len1];
  Timestamp.toCharArray(timeSave, len1); delay(20);   // convert String to char array
  timeSave[len1 - 1] = 0;

  int8_t dBm;

  pwr_cellular(1);
  delay(3000);

  if (gStatus == 1 && !recvMode) {
    if (debug) Serial.println("sendDataArray(): Sending data from array...");
     
     // Get metadata
      uint16_t pvCurrent = GetISolar();   // using John's code
      float pvVoltage = GetVSolar();
      float boxTemp = getBoxT();    // added 14-Feb-2020
            
      //--- Convert floats to arrays
    
      String str_battV = String(battV);
      uint8_t bvLen = str_battV.length() + 1;
      char bv[bvLen];
      str_battV.toCharArray(bv,bvLen);
      bv[bvLen - 1] = 0;
            
      String solarV = String(pvVoltage);
      uint8_t pvLen = solarV.length() + 1;
      char pv[pvLen];
      solarV.toCharArray(pv,pvLen);
      pv[pvLen - 1] = 0;
            
      String boxT = String(boxTemp);
      uint8_t bTlen = boxT.length() + 1;
      char bT[bTlen]; 
      boxT.toCharArray(bT,bTlen);
      bT[bTlen - 1] = 0;

      String pvC = String(pvCurrent); 
      uint8_t pvcLen = pvC.length() + 1;
      char pvc[pvcLen];
      pvC.toCharArray(pvc,pvcLen);
      pvc[pvcLen - 1] = 0; 

      delay(100);

    //--- Get RSSI
    
    dBm = cellRad.getRSSIdBm();

    uint8_t div10 = dBm / -10;
    uint8_t digits;
    if (div10 > 9){
      digits = 3;
    } else if (div10 == 1){
      digits = 2;
    } else if (div10 == 0){
      digits = 1;
    }

    String rs_dbm = String(dBm);
    uint8_t dbm_len = rs_dbm.length() + 1;
    char rssi[dbm_len];
    rs_dbm.toCharArray(rssi,dbm_len);
    rssi[dbm_len - 1] = 0;
   
    //--- Compile gData message
    
    uint16_t gData_len = sizeof(VERSION) + sizeof(projectID) + 8 + bvLen + bTlen + pvcLen + pvLen + sizeof(timeSave);   // have to do it again, previous one is out of scope
    char gData[gData_len];                           // array for Gateway data, compiled below
    sprintf(gData, "%s~%s~%lu~%s~%s~%s~%s~%s", VERSION, projectID, serNum, bv, bT, pvc, pv, timeSave); delay(100);
//       Serial.println(gData);

    uint16_t gMsgLen = gData_len + 1 + dbm_len + 8 + 46; //47; //48; <-- DON'T INCLUDE NULL IN MSG
    char gMsg[gMsgLen];
    sprintf(gMsg,"{\"k\":\"%s\",\"d\":\"%s~%s\",\"t\":[\"%lu\",\"GATEWAY_DATA\"]}\r",devicekey, gData, rssi, serNum);
//    Serial.println(gMsg);
    delay(60);

    
    //--- Open TCP socket and send data
    
    uint16_t IPlen = sizeof(hologramIP);
    uint16_t portLen = sizeof(hologramPort);
    
    answer = cellRad.openTCP(hologramIP,IPlen,hologramPort,portLen);

    if (answer == 1) {
      delay(60);
      answer = cellRad.sendMsg(gMsg, gMsgLen, 10000);  // send Gateway data

      answer = cellRad.closeTCP();                     // close TCP socket

    //--- Send node data
    
      for (byte index = 0; index < numNodes; index++) {

        // Do not send anything if blank
        boolean tcpSent = false;
 
        if (nodeData[index][0] != 0) {
          int len = nodeData[index].length() + 1;
//          if(debug) {
//            Serial.print("len = ");
//            Serial.println(len);
//          }
          char dataToSend[len];
          nodeData[index].toCharArray(dataToSend, len);
          if(debug) {
            Serial.print("dataToSend: ");
            Serial.println(dataToSend);
          }

        for (byte i = 1; i <= 3; i++){    // try to send data max 3 times
          if (tcpSent == false) {
            
            // need to calculate length of full json msg

            uint16_t json_len = len + 8 + 44; //45;
            char n_msg[json_len];
            
            sprintf(n_msg, "{\"k\":\"%s\",\"d\":\"%s\",\"t\":[\"%lu\",\"NODE_DATA\"]}\r", devicekey, dataToSend, serNum);
            delay(60);
            if(debug) {
              Serial.println(n_msg);
            }

            answer = cellRad.openTCP(hologramIP,IPlen,hologramPort,portLen);

            if (answer == 1){
              delay(60);
              answer = cellRad.sendMsg(n_msg, json_len, 10000);  // send node data

              answer = cellRad.closeTCP();                     // close TCP socket
            }

            if(answer == 0){ 
              tcpSent = false;
              Serial.println("Send to Hologram failed");
            } else {
              tcpSent = true;
            }

          }  // end if tcpSent == false

          else {
            break;
          }

         } // end for loop that tries to send 3 times

        }   // end if nodeData[index][0] != 0

      }  // end loop through nodeData

      delay(100);
    } else {
      if (debug) Serial.println("Failed to connect to Hologram");
    }
  } // end if gStatus == 1

  delay(3000);
  pwr_cellular(0);

  if (debug) Serial.println("Done");

}


//======================================================================================

//--------------- Initialize SIM7070G ------------------------------------------------------
bool pwr_cellular(bool onoff) {
  bool cellOn;

  // check current power status
  
  if (digitalRead(pin_SIMCOM_PWR_KEY) == LOW){
    cellOn = false;
  } else {
    cellOn = true;
  }
//  Serial.println(digitalRead(pin_SIMCOM_PWR_STATUS));
//  Serial.println(digitalRead(pin_SIMCOM_PWR_KEY));
//  Serial.println(cellOn);

  // compare to onoff input, execute pwr toggle accordingly

  if (cellOn == true && onoff == 0){                        // cell on, want to turn off
    if (debug){
      Serial.print("Turning cellular radio OFF...");
    }    
    if (cellRad.powerOff(true)){                            // turn radio off
      if (debug)Serial.println("Success!");
      cellOn = false;
    } else {
      if (debug) Serial.println("Failure!");
    }    
  } 
  else if (cellOn == false && onoff == 1){                  // cell off, want to turn on
    if (debug){
      Serial.print("Turning cellular radio ON...");
    }
    if (cellRad.begin()){                                   // turn radio on
      if (debug)Serial.println("Success!");
      cellOn = true;
      cellRad.echo(1);                                      // turn off cellular command line echo
    } else {
      if (debug) Serial.println("Failure!");
    }      
  } 
  else {
    // do nothing, pwr state matches onoff input
  }
  
  if(cellOn){    
//    delay(200);
    if(cellRad.detectSIM()){                                // check for SIM
      sim_detect = true;                                    // all good, continue normal function  
      if (debug) Serial.println("SIM detected");    
      n = cellRad.getNetworkStatus();
      cellRad.closeTCP();
      cellRad.closeIP();
      gStatus = cellRad.startIP();                          // start internet protocol  
      if (gStatus == 1){
        recvMode = false;
      } else {
        recvMode = true;
      }
    } else {
      sim_detect = false;                                   // SIM missing, go to Receiver mode
      recvMode = true;
      if (debug) Serial.println("SIM not detected");
    }
  }
  return cellOn;

}

void printNetStat() {
  Serial.print(F("Network status "));
  Serial.print(n);
  Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));
  delay(100);
}

bool isSubstring(char substr[100], char str[100]) {
  for (int i = 0; i < 100; i++) {
    if (substr[i] == '\0')
      return true;
    if (str[i] != substr[i])
      return false;
  }
  return true;
}

//======================================================================================

//--------------- Solar panel ---------------------------------------------

//Returns voltage of solar cell in volts (battery load may affect voltage)
float GetVSolar() {
  //unsigned int GetVSolar() {
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
  //unsigned int GetISolar() {
  pinMode(pin_solarShort, OUTPUT);
  uint16_t iSolarRes100;
  EEPROM.get(EEPROM_iSolarRes, iSolarRes100);

  if (iSolarRes100 == (uint16_t)0xFFFF) {
    char verHW;
    EEPROM.get(EEPROM_verHW, verHW);

    // WARNING: EEPROM factory constants must always be written from this point forward!
    if (verHW != '5')
      iSolarRes100 = 1000;    // this is a default of 10.00 ohms!
    else
      iSolarRes100 = 100;     // default of 1.00 ohms for old board rev. E
  }

  digitalWrite(pin_solarShort, HIGH);     // This "shorts" solar cell through shorting resistor (depending on hardware could be a single 1 ohm resistor or an array of resistors that yields 10 ohms)
  delayMicroseconds(500);                 // Wait 1/2 millisecond before sampling to ensure steady state
  float vSolar = GetVSolar();       // Read voltage of shorted solar cell
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

//======================================================================================
//======================================================================================
//======================================================================================

//--------------- Main Menu ------------------------------------------------------------

void MainMenu()
{
  if (Serial.available() > 0) {
    Serial.read();       // clear serial input buffer
  }
  uint8_t loraKey;
  uint8_t cellKey;
  PCAL6416* RadioDetect = new PCAL6416(false);
  if (RadioDetect->begin(true)) {
    RadioCardID_t radioType = RadioDetect->readID();
    loraKey = radioType.LoRa;
    cellKey = radioType.Cell;
  } else {
    Serial.println("Could not detect radio!");
  }
  delete RadioDetect;  

  //--- fill in data filename
  
  datafile[1] = (GatewayID / 10) + 48;
  datafile[2] = (GatewayID % 10) + 48;

  delay(50);
  
  //--- read variables stored in memory
  
  numNodes = EEPROM.read(EEPROM_NODE_COUNT);                        

  if (!radioSwitch) {
    GatewayID = EEPROM.read(EEPROM_DEFAULT_RADIO);
  } else {
    GatewayID = EEPROM.read(EEPROM_ACTIVE_RADIO);
  }

  uploadInt = EEPROM.read(EEPROM_ALRM2_INT);
  EEPROM.get(EEPROM_DEVKEY, devicekey);

  EEPROM.get(EEPROM_PROJECTID, projectID);

  EEPROM.get(EEPROM_NODEIDS, NodeIDs);
  interval = EEPROM.read(EEPROM_ALRM1_INT);

  forceRecv = EEPROM.read(EEPROM_FORCERECV);
  debug = EEPROM.read(EEPROM_DEBUG);
  delay(30);

  battV = calcbattV();
 
  //  Serial.println();
  Serial.println(F("Soil Water Data Network - Cellular Gateway"));
  if(forceRecv){
    Serial.println(F("--- Receiver Only Mode ---"));
  }
  Serial.print(F("Version "));
  Serial.println(VERSION);
  Serial.println();

  Serial.print(F("Serial Number: "));
  Serial.println(serNum);
  Serial.print(F("LoRa Radio Freq (MHz): "));
  Serial.println(loraKey);
  Serial.print(F("Cellular Module: "));
  Serial.println(cellKey);
  Serial.print(F("Project ID: "));
  Serial.println(projectID);
  Serial.print(F("Gateway Radio ID: "));
  Serial.println(GatewayID);
  if(!forceRecv){
    Serial.print(F("Device key: "));
    Serial.println(devicekey);
  }
  Serial.print(F("# of nodes: "));
  Serial.println(numNodes);
  Serial.print(F("Networked Node IDs: "));
  for (byte i = 0; i < numNodes; i++) {
    if (numNodes > 1 && i < (numNodes - 1)) {
      Serial.print(NodeIDs[i]);
      Serial.print(", ");
    }
    else if (numNodes > 1 && i == (numNodes - 1)) {
      Serial.println(NodeIDs[i]);
    } else {    // only one node
      Serial.println(NodeIDs[i]);
    }
  }
//  Serial.println();
  Serial.print(F("Data file: "));
  Serial.println(datafile);
  Serial.println();

  Serial.print(F("Measurement Interval: "));
  Serial.println(String(interval) + " mins");
  if(!forceRecv){
    Serial.print(F("Upload interval (hrs): "));    
  } else {
    Serial.print(F("Gateway data interval (hrs): "));  
  }
  Serial.println(uploadInt);
  readClock();                                       // get current date, time
  delay(20);
  Serial.print(F("Current date & time: "));
  Serial.print(mnths);                             // date
  Serial.print(F("-"));
  Serial.print(days);
  Serial.print(F("-"));
  Serial.print(yrs);
  Serial.print(' ');
  Serial.print(hrs);                               // time
  Serial.print(F(":"));
  if (mins < 10) Serial.print(F("0"));
  Serial.print(mins);
  Serial.print(F(":"));
  if (secs < 10) Serial.print(F("0"));
  Serial.print(secs);
  Serial.println(" UTC");

  Serial.print(F("Current battery voltage:  "));
  Serial.print(battV);
  Serial.println(" V");
  if(debug) Serial.println(F("Debug on"));
  else Serial.println(F("Debug off"));  
  Serial.flush();

  if (battV <= lowBatt) {
    Serial.println();
    Serial.println();   // added 26Jun2020: try to make message more obvious
    Serial.println("!!! WARNING: BATTERY VOLTAGE TOO LOW!! PLEASE CHARGE!!! ");
    Serial.println();
    Serial.println();
    Serial.println("!!! WARNING: BATTERY VOLTAGE TOO LOW!! PLEASE CHARGE!!! ");
    Serial.println();
    Serial.println();
  }

  Serial.println();
  Serial.println(F("User Options:"));
  if(!forceRecv) Serial.println(F("  1  <--  Cellular Signal Scouting Mode")); // 06Aug20
  Serial.println(F("  p  <--  Print node data to screen"));
  Serial.println(F("  r  <--  Receiver Mode on/off"));
  Serial.println(F("  f  <--  See list of saved files")); 
  Serial.println(F("  o  <--  Debug statements on/off"));

  if(!viewConfig){ 
    Serial.println(F("  2  <--  Show configuration options"));
  } else {
    Serial.println();
    Serial.println(F("Configuration Options:"));
    Serial.println(F("  0  <--  Enter configuration string"));    // 25-Feb-2020: enter config info all at once 
    Serial.println(F("  i  <--  Enter project ID"));                      // set siteID
    Serial.println(F("  g  <--  Change Gateway radio ID"));
    if(!forceRecv) Serial.println(F("  d  <--  Enter Hologram Device Key"));
    Serial.println(F("  n  <--  Enter Node radio IDs"));
     if(!forceRecv) {
      Serial.println(F("  u  <--  Set data upload to cloud interval"));
    } else {
      Serial.println(F("  u  <--  Set gateway data interval"));
    }
    Serial.println(F("  m  <--  Set measurement interval"));    
    Serial.println(F("  c  <--  Set clock"));    // set clock to NIST time
    Serial.println(F("  e  <--  Erase microSD card"));
    Serial.println(F("  2  <--  Hide configuration options"));
    Serial.println();
  }
  Serial.println(F("  x  <--  Exit menu"));             // exit
  Serial.println();

  Serial.print(F("Enter choice: "));
  Serial.flush();

  timeout = millis() + 60000;                         // wait 60 secs for input
  while (millis() < timeout)
  {
    menuinput = 120;
    if (Serial.available() > 0)                        // if something typed, go to menu
    {
      menuinput = Serial.read();               // get user input
      while (Serial.available() > 0)
      {
        Serial.read();
      }
      break;
    }
  }
  Serial.println(char(menuinput));

  switch (menuinput)                               // take action based on user input
  {
    case 49:                  // ------ 1 - Enter scouting mode ---------------------------------------------
      Serial.println();
      Serial.println("Entering signal scouting mode...");
      scoutMode();
      
      MainMenu();
      break;

    case '2':                 // ------ 2 - Show/hide config options ---------------------------------------------
      delay(500);
      if (!viewConfig){
        Serial.println("Activating configuration options...");
        viewConfig = true;
      } else {
        Serial.println("Hiding configuration options...");
        viewConfig = false;
      }
      Serial.println();
      delay(1000);
      
      MainMenu();
      break;

    case 48:                   // ------ 0 - Enter config string ---------------------------------------------
      Serial.println();
      Serial.print(F("Enter configuration string: "));
      Serial.println();
      charinput();
      if (charInput[0]) {
        decodeConfig(charInput);
      }
      Serial.println();
      delay(500);

      MainMenu();
      break;

    case 99: case 67:          // ------ c - Set clock ---------------------------------------------
      Serial.println();
      clockMenu = true;
      NISTsync();
      clockMenu = false;
      MainMenu();
      break;

    case 103: case 71:         // ---------- g - Enter Gateway radio ID ---------------------------------------------------
      Serial.println();
      changeDefault();
      Serial.println();
      
      MainMenu();
      break;

    case 100: case 68:         // ---------- d - Enter Hologram Device Key ---------------------------------------------------
      Serial.println();
      devKey();
      Serial.println();

      MainMenu();
      break;

    case 110: case 78:         // ---------- n - Enter Node radio IDs ---------------------------------------------------
      Serial.println();
      nodeIDs();
      Serial.println();
      
      MainMenu();                                      // go back to menu
      break;

    case 117:   // ------ u - Set upload interval ---------------------------------------------
      Serial.println();
      uploadInterval();
      Serial.println();
      MainMenu();
      break;

    case 109: case 77:        //--------- m - Set measurement interval----------------------------
      Serial.println();
//      if (!measureInt() && !recvMode) {
//        Serial.println(F("ERROR: upload interval must be entered first. Returning to menu..."));
//      }
      measureInt();
      Serial.println();
      delay(500);
      MainMenu();
      break;

    case 102: case 70:          // ------ f - See list of saved files ---------------------------------------------
      Serial.println();
      Serial.println(F("Saved files: "));     // list all files on SD card
      delay(10);

      if (!SD.begin(SD_CS)) {
        Serial.println("ERROR: SD fail!");
      }

      root = SD.open("/");
      printDirectory(root, 0);
      Serial.println();
      delay(500);
      MainMenu();
      break;

    case 112: case 80:          // ------ p - Print node data to screen ---------------------------------------------
      Serial.println();
      Serial.println(F("Print data files: "));

      delay(20);
      if (!SD.begin(SD_CS)) {
        Serial.println("ERROR: SD fail!");
      }

      myfile = SD.open(datafile);
      if (myfile) {
        while (myfile.available()) {
          Serial.write(myfile.read());
        }
      }
      myfile.close();
      Serial.println();
      delay(500);
      
      MainMenu();
      break;

    case 101: case 69:         // ------ e - Erase SD card ---------------------------------------------
      Serial.println();
      Serial.println(F("Erase SD card: "));

      //      SPI.begin();
      delay(20);
      if (!SD.begin(SD_CS)) {
        Serial.println("ERROR: SD fail!");
      }

      root = SD.open("/");
      delay(2000);

      rm(root, rootpath);

      if ( !DeletedCount && !FailCount && !FolderDeleteCount ) {

      }
      else {
        Serial.print("Deleted ");
        Serial.print(DeletedCount);
        Serial.print(" file");
        if ( DeletedCount != 1 ) {
          Serial.print("s");
        }
        Serial.print(" and ");
        Serial.print(FolderDeleteCount);
        Serial.print(" folder");
        if ( FolderDeleteCount != 1 ) {
          Serial.print("s");
        }
        Serial.println(" from SD card.");
        if ( FailCount > 0 ) {
          Serial.print("Failed to delete ");
          Serial.print(FailCount);
          Serial.print(" item");
          if ( FailCount != 1 ) {
            Serial.print("s");
          }
        }
      }
      delay(500);

      MainMenu();
      break;

    case 'o': case 'O': // ------ o - Turn debug statements on/off --------------------------------------
      if(debug){
        Serial.print(F("Would you like to turn debug statements off? y/n: "));
        charinput();
        if (charInput[0] == 'y' || charInput[0] == 'Y') {
          debug = false;
        }
      } else {
        Serial.print(F("Would you like to turn debug statements on? y/n: "));
        charinput();
        if (charInput[0] == 'y' || charInput[0] == 'Y') {
          debug = true;
        }
      }
  
      EEPROM.update(EEPROM_DEBUG,debug);
      delay(100);
      Serial.println();
      delay(500);
      MainMenu();
      break;
           
    case 120: case 88:         // ------ x - Exit ---------------------------------------------
      Serial.println(F("Exit"));                           // exit
      Serial.println();
      delay(10);
      break;                                       // exit switch(case) routine

    case 'i': case 'I':        // ------ i - Set Project ID ---------------------------------------------
      Serial.println();
      Serial.print(F("  Enter a Project ID (up to 5 characters, ex: PSA): "));    // set project ID
      charinput();

      while (numincoming > sizeof(projectID)) {
        Serial.println(F("  ERROR: Project ID exceeds 5 characters!"));
        Serial.print(F("  Enter a Project ID (up to 5 characters, ex: PSA): "));    // set project ID
        charinput();
      }

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
      }
      EEPROM.put(EEPROM_PROJECTID, projectID);
      delay(10);

      Serial.println();
      delay(500);

      MainMenu();
      break;
      
    case 'r': case 'R':        // ------ r - Set to Receiver Mode ---------------------------------------------
      Serial.println();
      Serial.println(F("NOTE: Receiver only mode will not upload data to Hologram"));
      Serial.print(F("Would you like to use Receiver Mode? y/n: "));
      charinput();

      if(charInput[0] == 'y' || charInput[0] == 'Y'){
        forceRecv = true;       
        Serial.println(F("  Receiver Mode activated"));
        uploadInt = 1;
        EEPROM.update(EEPROM_ALRM2_INT, uploadInt);
      } else {
        forceRecv = false;
        Serial.println(F("  Receiver Mode deactivated"));
        uploadInterval();
      }
           
      EEPROM.update(EEPROM_FORCERECV, forceRecv);
      Serial.println();
      delay(500);

       MainMenu();
       break;
  }

  Serial.println(F("Exit"));                       // exit
  Serial.println();
  delay(100);
  if (userinput == true) {
    digitalWrite(LED, LOW);
  }
}

//======================================================================================
//======================================================================================
//======================================================================================

//-------------- Decode config string --------------------------------

void decodeConfig(char config_string[55]) {

/* config string format: projectID,serial number,device key,gateway ID,number of nodes,node 1 radio ID,...,meas interval,upload int
    ** device key = 0 --> Receiver mode
*/
  bool configOK = true;
  byte configSize;

  char buf[55];

  for (byte i = 0; i < 55; i++) {
    buf[i] = config_string[i];   // put config string into a buffer
  }

  for (byte i = 0; i < 55; i++) {
    if (buf[i] == 0) {              // find the length of the config string
      configSize = i + 1;
      break;
    }
  }

  char configG[configSize];        // create new array of correct size
  for (byte i = 0; i < configSize; i++) {
    configG[i] = buf[i];
  }
//  Serial.println(configG);


  //--- Find comma positions

  uint8_t commaPos[15];
  byte commaCount = 0;

  for (uint8_t i = 0; i < configSize ; i++) {
    if (configG[i] == ',') {
      commaPos[commaCount] = i;
      commaCount++;
      //      Serial.println(commaCount);
    }
  }

  if (commaCount < 7) {
    Serial.println("ERROR: Incomplete configuration string!");
    configOK = false;
  }

  //  Serial.print("Commas at positions ");
  //  for(byte q = 0; q < commaCount; q++){
  //    Serial.print(commaPos[q]);
  //    Serial.print(" ");
  //  }
  //  Serial.println();

  //--- Get Project ID

  byte firstComma = 1;

  if (commaPos[0] > 5) {
    Serial.println("ERROR: Project ID missing!");
    configOK = false;
  }

  if (configOK) {
    byte i = 0;
    for (i = 0; i < commaPos[0]; i++) {
      projectID[i] = configG[i];
    }
    projectID[i] = 0;
    Serial.print(F("Project ID: "));
    Serial.println(projectID);

    //--- Match serial num

    uint8_t SerNum[8];

    //  if(!use_siteID){
    //    for (byte i = 0; i < 8; i++){
    //      SerNum[i] = configG[i] - 48;
    //    }
    //  } else {
    for (byte j = 0; j < 8; j++) {
      SerNum[j] = configG[j + commaPos[0] + 1] - 48;
      //      Serial.print(SerNum[j]);
    }
    //  }

    uint32_t serNumCheck = 0;

    for (byte i = 0; i < 8; i++) {
      long baseTen = 1;
      for (byte j = 0; j < (7 - i); j++) {
        baseTen = baseTen * 10;
      }
      serNumCheck += (SerNum[i] * baseTen);
    }
    Serial.print("Serial number: ");
    Serial.println(serNumCheck);

    if (serNumCheck != serNum) {
      Serial.println("ERROR: Serial numbers do not match!");
      configOK = false;
    }

    //--- get device key

    byte devLen = commaPos[firstComma + 1] - commaPos[firstComma] + 1;
//    Serial.print(devLen);
    if (devLen != 10 && devLen != 3) {
      configOK = false;
      Serial.println("ERROR: Invalid device key!");
    } else if (devLen == 10) {
      byte y = 0;
      for (byte i = commaPos[firstComma] + 1; i < commaPos[firstComma + 1]; i++) {
        devicekey[y] = configG[i];
        y++;
      }
      devicekey[8] = 0;
      Serial.print("Device key: ");
      Serial.println(devicekey);
      forceRecv = false;
    } else if (devLen == 3){
      if(configG[commaPos[firstComma] + 1] == '0'){
        Serial.println("Receiver only mode activated");
        forceRecv = true;
      }
    }

    //--- get Gateway ID

    if ((commaPos[firstComma + 2] - commaPos[firstComma + 1]) == 2) {  // radio ID is one digit
      GatewayID = configG[commaPos[firstComma + 1] + 1] - 48;
    } else {
      GatewayID = 10 * (configG[commaPos[firstComma + 1] + 1] - 48) + (configG[commaPos[firstComma + 2] - 1] - 48);
    }

    Serial.print("Gateway ID: ");
    Serial.println(GatewayID);

    //--- get # Nodes and IDs

    byte pos = commaPos[firstComma + 2] + 1;
    numNodes = configG[pos] - 48;

    Serial.print("Number of nodes: ");
    Serial.println(numNodes);

    byte t = 0;
    for (byte i = 0; i < numNodes; i++) {
      if (((3 + t) != (commaCount - 1)) && ((4 + t) != (commaCount - 1))) {
        if ((commaPos[firstComma + 4 + t] - commaPos[firstComma + 3 + t]) == 2) {
          NodeIDs[i] = configG[commaPos[firstComma + 3 + t] + 1] - 48;
        } else {
          NodeIDs[i] = 10 * (configG[commaPos[firstComma + 3 + t] + 1] - 48) + configG[commaPos[firstComma + 3 + t] + 2] - 48;
        }
        t++;
      }
      Serial.print("Node ID ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(NodeIDs[i]);
    }

    //  if(!use_siteID){
    //    if((commaCount - 5) != numNodes){
    //      Serial.println("ERROR: Incorrect number of radio IDs"); // too many or too few radio IDs listed
    //      configOK = false;
    //    }
    //  } else {
    if ((commaCount - 6) != numNodes) {
      Serial.println("ERROR: Incorrect number of radio IDs"); // too many or too few radio IDs listed
      configOK = false;
    }
    //  }

    //--- get measurement interval

    byte lastComma = commaCount - 1;
    interval = 10 * (configG[commaPos[lastComma - 1] + 1] - 48) + (configG[commaPos[lastComma - 1] + 2] - 48);

    Serial.print("Measurement interval: ");
    Serial.println(interval);

    if (interval != 10 && interval != 15 && interval != 20 && interval != 30 && interval != 60) {
      Serial.println(F("ERROR: Invalid interval (every 10, 15, 20, 30, or 60 mins)"));
      configOK = false;
    }

    //--- get upload interval

    uploadInt = configG[commaPos[lastComma] + 1] - 48;
    if(forceRecv){
      Serial.print("Gateway data interval: ");
    } else {
      Serial.print("Upload interval: ");
    }
    Serial.println(uploadInt);

    if (uploadInt != 1 && uploadInt != 4) {
      Serial.println(F("ERROR: Invalid interval (every 1 or 4 hours)"));
      configOK = false;
    }

  }

  if (!configOK) {
    Serial.println("!!! FAIL: Invalid configuration !!!");
  } else {
    Serial.println("Configuration complete");
    //    EEPROM.update(EEPROM_SITEID_FLAG, use_siteID);
    EEPROM.put(EEPROM_PROJECTID, projectID);
    if (GatewayID != default_radioID) {
      radioSwitch = 1;
      EEPROM.update(EEPROM_ACTIVE_RADIO, GatewayID);
    } else {
      radioSwitch = 0;
    }
    EEPROM.update(EEPROM_FLAG_RADIO, radioSwitch);
    LoRa.setThisAddress(GatewayID);
    EEPROM.update(EEPROM_FORCERECV,forceRecv);
    if(!forceRecv) EEPROM.put(EEPROM_DEVKEY, devicekey);
    EEPROM.update(EEPROM_NODE_COUNT, numNodes);                           // save number of reps to EEPROM
    EEPROM.put(EEPROM_NODEIDS, NodeIDs);
    EEPROM.update(EEPROM_ALRM1_INT, interval);
    EEPROM.update(EEPROM_ALRM2_INT, uploadInt);

  }

}

void changeDefault() {
  Serial.println(F("Would you like to change the radio ID from the default value? y/n: "));
  charinput();
  radioSwitch = (charInput[0] == 'y' || charInput[0] == 'Y' ? 1 : 0);
  EEPROM.update(EEPROM_FLAG_RADIO, radioSwitch);
  if (radioSwitch) {
    Serial.println();
    Serial.print(F("Enter new radio ID: "));
    getinput();
    GatewayID = indata;
    EEPROM.update(EEPROM_ACTIVE_RADIO, GatewayID);
    Serial.println(F("Radio ID updated"));
  } else {
    Serial.println(F("Default radio ID accepted"));
    GatewayID = default_radioID;
  }
  LoRa.setThisAddress(GatewayID);
}

void devKey() {
  Serial.print(F("Enter the Hologram device key for this Gateway: "));
  bool devkeyOK = true;
  charinput();
  char devkeyTemp[9];

  for (byte i = 0; i < 8; i++) {
    if (charInput[i] != 0) {
      devkeyTemp[i] = charInput[i];
    } else {
      devkeyOK = false;
    }
  }
  devkeyTemp[8] = 0;

  if (devkeyOK) {
    for (byte i = 0; i < 9; i++) {
      devicekey[i] = devkeyTemp[i];
    }
    EEPROM.put(EEPROM_DEVKEY, devicekey);
    delay(10);
  } else {
    Serial.println("ERROR: Invalid device key!");
  }
  delay(500);
}

void nodeIDs() {
  Serial.print(F("How many nodes are at this site (max. "));
  Serial.print(maxNodes);
  Serial.print(")? ");
  getinput();
  numNodes = indata;
  long utimeout = millis() + 10000;
  while (numNodes > maxNodes || numNodes <= 0 && millis() < utimeout) {
    Serial.println(F("Invalid number of nodes.  How many nodes are at this site (max."));
    Serial.print(maxNodes);
    Serial.println(F(")?"));
    getinput();
    numNodes = indata;
  }

  EEPROM.update(EEPROM_NODE_COUNT, numNodes);                           // save number of reps to EEPROM
  delay(20);

  //      Serial.println();

  for (byte n = 1; n <= numNodes; n++) {
    Serial.print(F("Enter the radio ID for Node "));
    Serial.print(n);
    Serial.print(": ");
    byteInput();
    NodeIDs[n - 1] = u_indata;
  }
  delay(500);

  EEPROM.put(EEPROM_NODEIDS, NodeIDs);
}

void uploadInterval() {
  Serial.print(F("Set"));
  if (!forceRecv){
    Serial.print(F(" upload"));
  } else {
    Serial.print(F(" gateway data"));
  }
  Serial.println(F(" interval to 1 or 4 hours."));
  Serial.print(F("Enter \"1\" or \"4\": "));
  byteInput();
  uploadInt = u_indata;
  //    Serial.println(uploadInt,DEC);
  long utimeout = millis() + 10000;

  while ((uploadInt != 1 && uploadInt != 4) && millis() < utimeout) {
    Serial.println(F("Invalid interval. Please enter 1 or 4."));
    byteInput();
  }

  EEPROM.update(EEPROM_ALRM2_INT, uploadInt);
  delay(20);
}

void measureInt() {
  Serial.print(F("Enter measurement interval (10, 15, 20, 30, or 60 mins): "));
  Serial.flush();
  boolean intSet = false;
  long utimeout = millis() + 10000;
  while (intSet == false && millis() < utimeout) {
    getinput();
    if (indata != 10 && indata != 15 && indata != 20 && indata != 30 && indata != 60) {
      Serial.print(F("Invalid interval. Enter measurement interval (10, 15, 20, 30, or 60): "));
      Serial.flush();
    }
    else {
      intSet = true;
      interval = indata;
      EEPROM.update(EEPROM_ALRM1_INT, interval);
      delay(20);
    }
  }
}

void NISTsync() {
  if(sim_detect){
    Serial.println(F("Synchronizing clock to NIST server ... "));
    pwr_cellular(1);
    delay(3000);
    timeSuccess = NISTtime();
    if (timeSuccess == false) {
      Serial.println(F("Synchronization failed. Please enter time manually in UTC: "));
  
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
      setAlarm1();
      setAlarm2();
    } else if (!clockMenu) {
      sendNIST();
      delay(1000);
      pwr_cellular(0);
      delay(1000);
    }
  }
}

//======================================================================================
//======================================================================================
//======================================================================================

//-------------- Get User Input for numerical variables --------------------------------

void getinput()
{
  timeout = millis() + 15000;        // allow user time to input
  numincoming = 0;
  indata = 0;
  while (millis() < timeout)                       // loop while waiting for input
  {
    if (Serial.available() > 0)                    // something came in
    {
      delay(100);                                  // give time for everything to come in
      numincoming = Serial.available();            // number of incoming bytes
      for (i = 1; i <= numincoming; i++)           // read in everything
      {
        incoming[i] = Serial.read();                  // read from buffer
        if (incoming[i] == 13 || incoming[i] == 10)   // if CR or LF, ignore
        {
        }
        else                                       // otherwise
        {
          input = incoming[i] - 48;                // convert ASCII value to decimal
          indata = indata * 10 + input;            // assemble to get total value
        }
      }
      break;                                       // exit before timeout
    }
  }
  Serial.println(indata); delay(10);
}

//======================================================================================

//-------------- Get User Input for byte variables -------------------------------------

void byteInput()
{
  timeout = millis() + 15000;        // allow user time to input
  numincoming = 0;
  uint8_t u_input;
  u_indata = 0;                                      // initialize
  while (millis() < timeout)                       // wait for user to input something
  {
    if (Serial.available() > 0)                    // something came in to serial buffer
    {
      delay(100);                                  // give time for everything to come in
      numincoming = Serial.available();            // number of incoming bytes
      //      Serial.println(numincoming);
      for (byte i = 0; i < numincoming; i++)           // read in everything
      {
        incomingChar[i] = Serial.read();
        u_input = incomingChar[i] - 48;
        u_indata = u_indata * 10 + u_input;
      }

      break;
    }
  }
  Serial.println(u_indata);
}
//======================================================================================

//-------------- Get User Input for character variables --------------------------------

void charinput() {
  long  timeout;                                      // length of time to wait for user
  timeout = millis() + 15000;         // time to wait for user to input something
  numincoming = 0;

  //indata = 0;                                      // initialize
  while (millis() < timeout)                       // wait for user to input something
  {
    if (Serial.available() > 0)                    // something came in to serial buffer
    {
      delay(100);                                  // give time for everything to come in
      numincoming = Serial.available();            // number of incoming bytes
      //      Serial.println(numincoming);
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

//--------------- Print SD Card Directory ----------------------------------------------

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

//======================================================================================

//--------------- Erase SD -------------------------------------------------------------

void rm(File dir, String tempPath) {
  while (true) {
    File entry =  dir.openNextFile();
    String localPath;

    Serial.println("");
    if (entry) {
      if ( entry.isDirectory() )
      {
        localPath = tempPath + entry.name() + rootpath + '\0';
        char folderBuf[localPath.length()];
        localPath.toCharArray(folderBuf, localPath.length() );
        rm(entry, folderBuf);


        if ( SD.rmdir( folderBuf ) )
        {
          Serial.print("Deleted folder ");
          Serial.println(folderBuf);
          FolderDeleteCount++;
        }
        else
        {
          Serial.print("Unable to delete folder ");
          Serial.println(folderBuf);
          FailCount++;
        }
      }
      else
      {
        localPath = tempPath + entry.name() + '\0';
        char charBuf[localPath.length()];
        localPath.toCharArray(charBuf, localPath.length() );

        if ( SD.remove( charBuf ) )
        {
          Serial.print("Deleted ");
          Serial.println(localPath);
          DeletedCount++;
        }
        else
        {
          Serial.print("Failed to delete ");
          Serial.println(localPath);
          FailCount++;
        }

      }
    }
    else {
      // break out of recursion
      break;
    }
  }
}

//======================================================================================

//--------------- Cellular Signal Strength Scout Mode ----------------------------------

void scoutMode() {  // 06Aug20
  Serial.println(F("Turning cell module on..."));
  bool cellOn = pwr_cellular(1);
//  n = cellRad.getNetworkStatus();
  
  if (cellOn) {
    if (n == 5) {
      Serial.println(F("Enter 'x' at any time to stop"));
      while (Serial.read() != 'x') {

        int8_t dBm = cellRad.getRSSIdBm();
        //    Serial.println(response);

        // AT+CSQ Query Signal quality command:
        // returns <rssi>,<ber>
        // <rssi> = received signal strength indication
        // 0      --> -115 dBm or less
        // 1      --> -111 dBm
        // 2...30 --> -110...-54 dBm
        // 31     --> -52 dBm or greater
        // 99     --> not known or not detectable
        // <ber> = channel bit error rate

        //        String rssi = response.substring(response.length()-2);
        //        int rssiNum = rssi.toInt();
//        uint8_t dBm = 113 - 2 * rssi;
        Serial.print(F("RSSI (dBm): "));
        Serial.print(dBm);
       
        // LTE rssi ratings (from Khan, et al. 2019 Table 2)
        // Module in CAT-M1 system mode (not NB-IoT)
        // > -65      dBm EXCELLENT
        // -65 to -75 dBm GOOD
        // -75 to -85 dBm FAIR
        // -85 to -95 dBm POOR
        // < -95      dBm DISCONNECT --> leave as poor bc module will return lower values while still connected
               
        if (dBm >= -65) {
          Serial.println("  EXCELLENT");
        } else if (dBm < -65 && dBm >= -75) {
          Serial.println("  GOOD");
        } else if (dBm < -75 && dBm >= -85) {
          Serial.println("  FAIR");
        } else if (dBm == -99) {
          Serial.println("  UNKNOWN");
        } else {
          Serial.println("  POOR");
        }
        delay(1000);
      }
      Serial.println(F("Turning cell module off..."));
      pwr_cellular(0);
    } 
    else {
      Serial.println("ERROR: could not connect to network");
    }
  } else {
    Serial.println("ERROR: could not turn on cellular module");
  }
}

//======================================================================================

//--------------- Decode LoRa and cellular radio types ---------------------------------

void printLoraFreq(uint8_t lorakey){
  // LoRa types
  if (lorakey == 0){
    Serial.println("Not installed");
  } 
  else if (lorakey == 1){
    Serial.println("915");
  }
  else if (lorakey == 2){
    Serial.println("868");
  }
  else if (lorakey == 3){
    Serial.println("433");
  }
  else if (lorakey == 4){
    Serial.println("472");
  }
}

void printCellType(uint8_t cellkey){
    // Cellular types
  if (cellkey == 0){
    Serial.println("Not installed");
  } 
  else if (cellkey == 1){
    Serial.println("SIM5320A");
  }
  else if (cellkey == 2){
    Serial.println("SIM5320E");
  }
  else if (cellkey == 3){
    Serial.println("SIM5320J");
  }
  else if (cellkey == 4){
    Serial.println("SIM7070G");
  }
}

void getModemRX(uint8_t rxCnt, uint16_t unreadCnt) {
  if (printCMRX > 0) {
    char* buf;
    uint16_t len;
    //Serial.print(cellRad.dumpRXBuff());    
    len = cellRad.getRX(unreadCnt, &buf);
    while (len > 0) {
      if (buf[rxCnt-len] == 0)
        break;
      Serial.print(buf[rxCnt-len]);
      len--;
    }    
//    strcat(buf, "");
//    Serial.print(buf);  //TODO: this should print actual chars... fix later
  }
}
