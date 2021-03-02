
/*PSA Cellular Gateway  
  
   Main Components:
      - ATMega1284P with MoteinoMEGA core
      - DS3231 Precision RTC
      - LoRa radio transceiver
      - SimCom 5320A cellular module
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
   
   Last edited: March 2, 2021

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
#include "Adafruit_FONA.h"                         // for Adafruit FONA 3G cellular breakout
#include "avr/io.h"
#include "avr/interrupt.h"
#include "avr/wdt.h"                               // controls watchdog timer (we want to turn it off)
//#include "SIMCOM_Lite.h"                           // Autobaud 28Oct20


// ------- Assign Pins ----------------------------------------------------

  #define LED                 15       // MoteinoMEGA LED
  #define SD_CS               3        // D3 for SD card ChipSelect 
  #define hardSS              4
  #define Fona_Key            0        // turn FONA on/off
  #define Fona_RST            13       // FONA hard reset - toggle low for 100ms to reset
  #define FONA_RX             11       // communication w/ FONA if using software serial, hardware Serial1 is the same
  #define FONA_TX             10
  #define Fona_PS             25       // FONA Power Status - LOW when off, HIGH when on
  #define BattV               24       // A0, for calculating Vin from battery
  #define Mbatt               14       // Switches voltage divider on/off
  #define baudRate            57600    // John, 27-Mar-2019: Changed from 115200 to 57600 because when MCU at 8 MHz the sampling resolution is reduced
  #define maxNodes            6        // AIT, 14-Jan-2020: Changed from 10
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
  
  char VERSION[] = "V2021.03.01";

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
//  #define EEPROM_BAUDCHECK            (40 + EEPROMSHIFT)        // store whether or not cell baud rate has already been checked and set to 57600
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
  
//-----*** for battV (12V) calculation ***-----

  float   battV;                                   // battery voltage (V) --> max 12V with voltage regulator
  float   lowBatt = 3.4;                           // low battery limit

//---*** Counters and Flags ***---

  byte    i;                                     // counter for user input functions
  bool    init1 = true;                          // flag for initial synchronization (fieldSync() on startup)
//  byte    retries = 0;                           // counts number or retries for getting NIST time during fieldSync()
  bool    duringInit = true;                     // controls sending init message to Hologram in syncNodes() and daily sync check in NISTtime()
  bool    radioSwitch;                           // flag for using default radio ID or active radio ID
  
//-----*** for LoRa Radio ***-----

  //-- Identifiers, Metadata
  
  uint8_t   GatewayID;                            // Gateway radio ID 
  uint8_t   default_radioID;                      // Default radio ID (last 2 digits of SN)
  uint8_t   NodeIDs[maxNodes];                    // networked Node IDs
  byte      numNodes;                             // number of nodes connected to Gateway

  //-- Radio Settings

  #define retryNum        3       // 3 retries should be enough, if our wait intervals are correct
  #define timeoutACK      120     // This is the wait period after a transmission to recieve an ACK for that one packet. (ACKWait)
  #define timeoutPacket   2000    // this is the wait period to receive one large packet including retries. (PacketWait)
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
  String Timestamp="";                          // for compiling timestamp into YYYY_MM_DD_hh:mm:ss_UTC format
  
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

//-----*** for SIM5320 ***-----

  //-- Comm with module
  
  char    response[100];                            // holder for responses from cellular module and network
  uint8_t answer;                                   // flag for matching expected response to actual response (sendATcommand())
  char    aux_str[FONA_MSG_SIZE];                   // buffer for compiling AT commands
  boolean fonaON = false;                           // flag for cellular module power status
//  uint16_t setbaudrate = 57600;
//  uint32_t oldbaudrate;
  
  //-- Network settings
  
  char  APN[] = "hologram";                         // cellular provider APN
  uint8_t gStatus;                                  // FONA GPRS status
  uint8_t n;
//  uint32_t netopen_timeout;                       // might not be necessary
//  uint32_t cipopen_timeout;
//  uint32_t cipsend_timeout;

  //-- for fieldSync
  
  boolean userinput = false;
  char toSend[FONA_MSG_SIZE];
  unsigned long startTime;
  
  //-- for posting to Hologram Cloud
  
  char    devicekey[9];                             // Hologram device key - authorizes device to send data to Hologram server

// ------- Initialize ----------------------------------------------------

  RH_RF95 driverRFM95;                                 // Driver for the RFM95 radio
  ACReliableMessage LoRa(driverRFM95, GatewayID);        // LoRa radio manager

  HardwareSerial *fonaSerial = &Serial1;              // initialize serial port for comm to SIM5320    
  Adafruit_FONA_3G fona = Adafruit_FONA_3G(Fona_RST); // initialize cellular module object
  
//  SimCom *CellModem;  // autobaud lib 28Oct20
  
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
  pinMode(Fona_Key, OUTPUT); 
  digitalWrite(Fona_Key, HIGH);                   // set HIGH so toggle() works                     
  pinMode(Fona_PS, INPUT);                        // input; reads HIGH or LOW   
  pinMode(BattV, INPUT);                          // reads A0 to calculate SLA battery V
  pinMode(pin_solarVoltage, INPUT);
  pinMode(pin_solarShort, OUTPUT);
  pinMode(pin_SD_OFF, OUTPUT); 

  randomSeed(analogRead(4));                      // to randomize IP choices in NISTtime 

/*  //--- Check cell baud rate if haven't already
  
//  byte baudChecked = EEPROM.read(EEPROM_BAUDCHECK);
  
//  if(baudChecked != 1){
    Serial.print("Checking cell modem baud rate... ");
    CellModem = new SimCom(Serial1);  // 28Oct20
    oldbaudrate = CellModem->AutoBaud();
    Serial.println(oldbaudrate);
  
    if(oldbaudrate != setbaudrate){
      Serial.print("Setting baud rate... ");
      if(!CellModem->ChangeBaud(setbaudrate)){
        Serial.println("Unable to change baud rate");
      } else {
//        baudChecked = 1;
//        EEPROM.update(EEPROM_BAUDCHECK, baudChecked);
        Serial.println(" done.");
      }
    } else {
      Serial.println("OK");
//      baudChecked = 1;
//      EEPROM.update(EEPROM_BAUDCHECK, baudChecked);
    }
//  }
*/ 

  if (digitalRead(Fona_PS) == HIGH) {
    toggle();    // if Fona is on, turn it off
  }
    
  //--- Initialize
   
    // I2C
    
    Wire.begin();                                   // begin I2C
    delay(300);

    // RTC
    
    RTC.begin();

    // SD
    
    if (!SD.begin(SD_CS)) {Serial.println("ERROR: SD init failed");}

  //--- Power saving
  
    wdt_disable();                                 // turn off watchdog timer 
    RTC_osc_off();                                 // Turn off 32kHz output

  //--- Initialize radio settings

    LoRaFREQ = getLoRaFreq();
    
    EEPROM.get(EEPROM_SERIALNUM, serNum);  // get the serial number from EEPROM
    default_radioID = serNum % 100;
    EEPROM.update(EEPROM_DEFAULT_RADIO, default_radioID); // set default radio ID
    radioSwitch = EEPROM.read(EEPROM_FLAG_RADIO);

    if(radioSwitch){
      GatewayID = EEPROM.read(EEPROM_ACTIVE_RADIO);                        // read board ID number from EEPROM
    } else {
      GatewayID = default_radioID;
    }
    
    if (!LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, GatewayID)) {                      // initialize radio
      Serial.println("ERROR: radio init failed");
      return;
    }
  
  //--- Go to main menu for setup
    
    MainMenu();

  //--- Time sync in field 

    delay(50);
    fieldSync();
    getData();
    
    saveData();             // if microSD fails, G will send data to Hologram immediately (sendDataArray())
    sendDataSD();
    resetArrays();
    init1 = false;
    digitalWrite(LED,LOW);

  //--- Initialize alarms
   
    // clear alarm registers
    
    RTC.setAlarm(ALM1_MATCH_MINUTES,0,0,0,0);   // set alarm values to 0
    RTC.setAlarm(ALM2_MATCH_MINUTES,0,0,0,0);
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

  if (RTC.alarm(ALARM_1)){      // ALARM 1 controls waking up to get data from Nodes
    battV = calcbattV();        // check battery voltage
    if(battV <= lowBatt){       // if battery voltage below low battery limit
      setAlarm1();              // set Alarm 1 to one minute after the "measurement" interval, go to sleep  
      setAlarm2();              // 23Feb21: set both alarms to stay updated during extended sleeps
      sleepNow();
    } else {
      digitalWrite(LED, HIGH);
      getData();                // request data from Nodes (received as String objects)
      saveData();               // save received data to SD card
      resetArrays();            // erase contents of String buffer
      digitalWrite(LED,LOW);
      setAlarm1();              // set Alarm 1 to one minute after the "measurement" interval    
    } 
  }   // end if alarm 1
  
  else if (RTC.alarm(ALARM_2)){ // ALARM 2 controls waking up to upload data to Hologram and daily NIST sync
    battV = calcbattV(); 
    if(battV <= lowBatt){       // Again, G will go to sleep if battV too low
      setAlarm1();              // 23Feb21
      setAlarm2();      
      sleepNow();
    } 
    else { 
      readClock();
      if(hrs == NISThr && mins == NISTmin){ // time to get time from NIST
        digitalWrite(LED,HIGH); 
        fonaOn();
        delay(3000);
        timeSuccess = NISTtime();           // Get time from NIST, send to Nodes
        fonaOff();
        digitalWrite(LED,LOW);
        delay(1000);
      } 
      else {
        digitalWrite(LED,HIGH);
        sendDataSD();                       // upload data from DUMP file to Hologram       
        digitalWrite(LED,LOW);
      }
//      setAlarm1();                        // 02Mar21: is within sendDataSD(), not needed here; 27Jan21: just in case upload overlaps interval
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

void setRTCInterrupt(){
  sei();      // turn on Global Interrupt Enable

  PCMSK0 |= (1 << PCINT2);   //  set pin as PCINT
  PCICR |= (1 << PCIE0);     //  enable interrupts on vector 0  
}

void clearRTCInterrupt(){
  PCICR &= (1 << PCIE0);
}

ISR(PCINT0_vect){
  sleep_disable();
  clearRTCInterrupt();
}

void sleepNow(){
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
  ACSR &= ~(1<<7);
}

void analogComp_on() {
  //Turn on analog comparator
  ACSR |= (1<<7);
}

void ADC_off() {
  //Turn off ADC  --  this saves about 113 uA
  ADCSRA &= ~(1<<7);
}

void ADC_on() {
  //Turn on ADC
  ADCSRA |= (1<<7);
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

//------------- Reset function --------------------------------//
void(* resetFunc) (void) = 0;   // 24-Jan-2020

//======================================================================================

//---------- Sync time with Nodes after field install, get sensor lists ----------------

void fieldSync(){
  if(duringInit){
    sendDataSD(); //29Jun20: send data in dump file
    startTime = millis();
  }
 
  Serial.println();
  Serial.println("fieldSync(): Syncing time with Nodes...");

  //--- Step 1: Get time from NIST

  for (byte retries = 0; retries < 3; retries++){  
    fonaOn();
    delay(3000);
//    Serial.print("Retry ");
//    Serial.println(retries);
    
    
    if (digitalRead(Fona_PS) == HIGH){
      timeSuccess = NISTtime();
      fonaOff();
      delay(5000);        
      if (!timeSuccess){} 
      else {
        fonaOn();
        sendNIST();         // 21-Apr-2020: send confirmation to Hologram/Slack
        fonaOff();
        break;
      }      
    }
  }

  //--- Step 2: Send timestamp to Nodes, try for up to 2 hrs
  
  if (!LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, GatewayID)) {           // initialize radio
    return;
  }
  
//  syncNodes(7200000);   // 27-Jan-2020: add time2wait parameter (2 hours)

  unsigned long totalTime = 7200000; // 13May20
//  unsigned long totalTime = 900000; // 14May20 TESTING: 15 minutes
  unsigned long timeStart = millis();
  unsigned long waitTime = totalTime - (startTime + timeStart);
  Serial.print("waitTime = ");
  Serial.println(waitTime);
  syncNodes(totalTime);  // 14May20 for testing
  delay(5000);

  //--- Step 3: Send init message to Hologram
  
  fonaOn();
  delay(3000);

  if(digitalRead(Fona_PS) == HIGH && gStatus == 1) {  // if cell module on and connected to network
    if(!sendInit()){                                  // send initialization message to Hologram, if fails, save to SD card for later upload
      delay(20);
      dump = SD.open(dumpfile, FILE_WRITE);           // only saves to dump file for upload, not to data file
      delay(50);
      if(dump){
        dump.println(toSend);
      }
      delay(50);
      dump.close();
    } else {
      fonaOff();    
      delay(1000);
    }
  } 

  Serial.println("Done"); 
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
  
//  for(byte j = 0; j <= len_resp; j++){
////    Serial.print(resp[j]);
//  }
  
  
//  getFreeRAM(); // 21Jan21
//  Serial.println("GetNodeData: prepare timestamp,");
  
  *msg = ""; 
  
  if (LoRa.sendtoWait(resp, len_resp, NodeAddr)) {   // send timestamp, which should trigger a data response    
    uint8_t from;
    if (LoRa.recvMessage(msg, 500, timeoutPacket, &from)) {  // 14Jan20. 21Jan21 set back to 500ms    
      return true;
    } // end successful reception
  } // end successful transmission
  *msg = "";

  
//  getFreeRAM(); // 21Jan21
//  Serial.println("GetNodeData: recvd msg from node");
  return false;
}

//---------- Sync time with Nodes ----------------------------------

void syncNodes(uint32_t time2wait){   // 22-Jan-2020: add parameter time2wait
  uint32_t timeToWait = time2wait;
  unsigned long StartTimeToWait = millis();
  
  Serial.println();
  Serial.println("syncNodes(): Querying Nodes with timestamp...");
  String synced="";
  String unsynced="";
  
  uint8_t unsyncedNodes[numNodes];    // keep track of unsynced nodes, synced ones get set to 0

  for(byte f = 0; f < numNodes; f++){
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
      Serial.print("Pinging Node ");
      Serial.println(to);
      for (numAttempts = 0; numAttempts < 3; numAttempts++) {   // try a node 3 times
        if (GetNodeData(&nodeResponse, to)) {                            // get data from node
          unsyncedNodes[y] = 0;                                         // mark this one as complete
          Serial.println(nodeResponse);
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

  if(init1){  // only send message to Hologram during fieldSync, not during daily sync check
    for(byte i = 0; i< numNodes; i++){    // list out synced and unsynced radio IDs    if(missedSyncs == true){
      if(unsyncedNodes[i]== 0){  
        synced += NodeIDs[i];
        synced += ' ';
      } else { 
        unsynced += unsyncedNodes[i];
        unsynced += ' ';
      }
    }

    Serial.print("synced: ");
    Serial.println(synced);
  
    byte syncLen = synced.length()+1;
    char syncSend[syncLen];
    synced.toCharArray(syncSend,syncLen);  
    syncSend[syncLen - 1] = 0;
//    Serial.println(syncSend); 
  
    readClock();  
    if(done == false){     
      byte unsyncLen = unsynced.length()+1;
      char unsyncSend[unsyncLen];
      unsynced.toCharArray(unsyncSend,unsyncLen);
      unsyncSend[unsyncLen - 1] = 0;
//      Serial.println(unsyncSend);
      
      delay(50);
      sprintf(toSend,"%lu(%d) Gateway awake at %d/%d/%d %d:%d. Synced to Nodes: %s Missed Nodes: %s.",serNum,GatewayID,mnths,days,yrs,hrs,mins,syncSend,unsyncSend);
      delay(100);
    } else{
      sprintf(toSend,"%lu(%d) Gateway awake at %d/%d/%d %d:%d. Synced to Nodes: %s Missed Nodes: none.",serNum,GatewayID,mnths,days,yrs,hrs,mins,syncSend);
      delay(50);
    }
      Serial.println(toSend);
  
  } else {    // during daily sync check, give Nodes the ok to stop listening
    uint8_t ok[2];
    uint8_t okLen = 2;
    uint8_t node;
    uint8_t stopNow[1];
        
    stopNow[0] = GatewayID; 
    
    for (byte y = 0; y < numNodes; y++){       
      uint8_t to = NodeIDs[y];
      if (LoRa.sendtoWait(stopNow, 1, to)) { // send message to Nodes to stop listening and send IDs
        LoRa.recvfromAckTimeout(ok, &okLen, timeoutACK, &node); // receive ack from Node
      }
    }
  }  
}

//---------- Send init confirmation to Hologram ----------------------------------

boolean sendInit(){
  char ctrlZ[2];
  ctrlZ[0] = 0x1A;
  ctrlZ[1] = 0x00;
    
  memset(aux_str,0,sizeof(aux_str));
  delay(500);
  Serial.println();
  Serial.println("sendInit(): Sending init info to Hologram...");
 
  if (gStatus == 1) {
    
    char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"cloudsocket.hologram.io\",9999";   // Open TCP socket to Hologram (connect to Hologram)
    Serial.println(tcpinit); 
    answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",10000);   
    
    char sendtcp[] = "AT+CIPSEND=0,";   
    Serial.println();
    Serial.print(">> ");
    Serial.println(sendtcp);
    answer = sendATcommand(sendtcp,">",10000); 
    
    sprintf(aux_str, "{\"k\":\"%s\",\"d\":\"%s\",\"t\":[\"%lu\",\"init\"]}%s\r\n\r\n",devicekey,toSend,serNum,ctrlZ);   // compile json
    delay(60);
    Serial.println();
    Serial.print(">> ");
    Serial.println(aux_str);
    answer = sendATcommand(aux_str,"OK\r\n\r\n+CIPSEND:0,",10000);    // send json to Hologram

    char closesocket[] = "AT+CIPCLOSE=0";
    Serial.println();
    Serial.print(">> ");
    Serial.println(closesocket);
    answer = sendATcommand(closesocket,"OK/r/n+CIPCLOSE:0,0",5000);   // close TCP socket (disconnect from Hologram)
    return true;
  } 
  else {
    Serial.println("ERROR: Failed to connect to Hologram");
    return false;
  }
  delay(1000);
  
}

//---------- Send message of successful NIST contact ----------------------------------

void sendNIST(){
  char ctrlZ[2];
  ctrlZ[0] = 0x1A;
  ctrlZ[1] = 0x00;
    
  memset(aux_str,0,sizeof(aux_str));
  delay(500);
  Serial.println();
  Serial.println("sendNIST(): Sending confirmation to Hologram...");
 
  if (gStatus == 1) {
    
    char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"cloudsocket.hologram.io\",9999";    // open TCP socket
    Serial.println(tcpinit); 
    answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",10000);     

    char sendtcp[] = "AT+CIPSEND=0,";           // request to send data of unknown length
    Serial.println();
    Serial.print(">> ");
    Serial.println(sendtcp);
    answer = sendATcommand(sendtcp,">",10000); 
    
    timestamp();
    byte len = Timestamp.length()+1;
    char timeSend[len]; 
    Timestamp.toCharArray(timeSend,len); delay(20);
    
    sprintf(aux_str, "{\"k\":\"%s\",\"d\":\"Successful time update %s\",\"t\":[\"%lu\"]}%s\r\n\r\n",devicekey,timeSend,serNum,ctrlZ);  // Compile message to send 
    delay(60);
    Serial.println();
    Serial.print(">> ");
    Serial.println(aux_str);
    answer = sendATcommand(aux_str,"OK\r\n\r\n+CIPSEND:0,",10000);    // send message

    char closesocket[] = "AT+CIPCLOSE=0";
    Serial.println();
    Serial.print(">> ");
    Serial.println(closesocket);
    answer = sendATcommand(closesocket,"OK/r/n+CIPCLOSE:0,0",5000);   // close TCP socket
  } 
  else {
    Serial.println("ERROR: Failed to connect to Hologram");
  }
  delay(1000);
  
}
//======================================================================================

//---------- Read DS3231 RTC -----------------------------------------------------------

void readClock(){
  RTC.read(tm);

  secs = tm.Second;
  mins = tm.Minute;
  hrs = tm.Hour;
  days = tm.Day;
  mnths = tm.Month;
  yrs = tm.Year + 1970;   //tm.Year --> years since 1970
  delay(50);
}

float getBoxT(){          // get enclosure temp from temp sensor on RTC
  float boxTemp;
  boxTemp = (float)RTC.temperature()/4.0;
  return boxTemp;
}

//======================================================================================

//---------- Compile timestamp String --------------------------------------------------

void timestamp(){
  Timestamp= "";
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

void setAlarm1(){
  readClock();

  alarmMins = ((((mins + interval)%60)/interval)*interval)+1;   // set alarm 1 to 1 minute after measurement interval
    
  RTC.setAlarm(ALM1_MATCH_MINUTES,0,alarmMins,0,0);
  
  Serial.print("Alarm 1 set for: ");
  Serial.println(alarmMins);
  delay(50);
}

//---------- Set Alarm 2  -----------------------

void setAlarm2(){  

  readClock();

  byte alarm2Mins;
  byte alarm2Hrs;
  bool alarmSet = false;

  // 29Jun20: remove nextday variable and calc

  if (uploadInt == 4){                          // if upload interval = 4
    if (hrs + 2 == NISThr){    // if next alarm should be time to get NIST time
        alarm2Hrs = NISThr%24;
        alarm2Mins = NISTmin;  
      }
      else if (hrs == (NISThr%24)){               // alarm after getting NIST time
        alarm2Hrs = (hrs + 2)%24;
        alarm2Mins = uploadMins;
      }
      else {                                       
        alarm2Hrs = (hrs + uploadInt) % 24;
        alarm2Mins = uploadMins;  
      }
  } else if (uploadInt == 1){                    // if upload interval is every hour
    if (hrs == NISThr && mins < NISTmin){        // if next alarm should be time to get NIST time
        alarm2Hrs = NISThr;
        alarm2Mins = NISTmin;     
    } else if (hrs == NISThr && mins >= NISTmin){ // alarm after getting NIST time
        alarm2Hrs = (NISThr%24)+1;
        alarm2Mins = uploadMins;
    }
    else {                                   
      alarm2Hrs = (hrs + uploadInt) % 24;
      alarm2Mins = uploadMins;        
    }
  }

 
// Note for Alarm 2:
// ALM2_MATCH_HOURS -- causes an alarm when the hours and minutes match.
// ALM2_MATCH_DATE -- causes an alarm when the date of the month and hours and minutes match.
  
  if(!alarmSet){
    RTC.setAlarm(ALM2_MATCH_HOURS, 0, alarm2Mins, alarm2Hrs, 0); 
  }
  
  Serial.print("Alarm 2 set for: ");
  Serial.print(alarm2Hrs);
  Serial.print(":");
  Serial.println(alarm2Mins);
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

void getData(){
  Serial.println();
  Serial.println("getData(): Querying Nodes for data...");
  dataSaved = false;

  // Step 1 - turn on radio

  if (!LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, GatewayID)) {     // initialize radio
    Serial.println("radio failed");
  }
  
  delay(2000);
  
  // Step 2 - Send timestamp to Ns one by one  
     
  uint8_t nodeIdlist[maxNodes];             // array for holding Node radio IDs  
  numNodes = EEPROM.read(EEPROM_NODE_COUNT);   
  EEPROM.get(EEPROM_NODEIDS, nodeIdlist);   
  
  for (byte i = 0; i < numNodes; i++)       // populate array with saved radio IDs
  {
    Serial.print("nodeIdlist[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(nodeIdlist[i]);
  }
  
  uint8_t to;             // radio ID of Node to ping
  bool missedOne = true;  // flag if didn't data from Node, triggers a retry
  int  tries;             // number of times to ping each Node
  
  if (duringInit){
    tries = 30;           // give more time during initial synchronization
  } else {
    tries = 3;
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
}

//======================================================================================

//---------- Get time from NIST server, update RTC -------------------------------------

boolean NISTtime(){
  unsigned long nistStart = millis(); //14May20
  Serial.println();
  Serial.println("NISTtime(): Getting time from NIST...");
  timeSuccess = false;

  char IPs[4][12] ={"129.6.15.28","129.6.15.29","129.6.15.30","129.6.15.27"};   // NIST has 4 IPs to try to not overload any of them
  char ipaddr[12];
   
  long y = random(0,4);  

  for(byte u = 0; u < 12; u++){
    ipaddr[u]=IPs[y][u];        // pick a random IP address of the four
  }

  if(gStatus == 1){

    char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",";
    sprintf(aux_str,"%s\"%s\",13",tcpinit,ipaddr);
    Serial.println();
    Serial.print(">> ");
    Serial.println(aux_str);
    answer = sendATcommand(aux_str,"OK\r\n\r\n+CIPOPEN: 0,0",10000);

    char gettime[] = "AT+CIPRXGET=0";
    Serial.println();
    Serial.print(">> ");
    Serial.println(gettime);
    char commandTime[256];
    sprintf(commandTime, "OK\r\n\r\nRECV FROM:%s UTC(NIST)", ipaddr);
    sendATcommand(gettime,commandTime,10000, response);  
    parseResponse(response);      // decode response from NIST and update clock

//    getFreeRAM();
//    Serial.println("NISTtime: recvd and decoded timestamp");
  } else { 
    delay(500);
    Serial.println("ERROR: No connection to network. Network time update unsuccessful.");
    timeSuccess = false;
  }


  if(!duringInit){  
    unsigned long timeNow = millis();   // 14May20
    unsigned long totalTime = 90000;
    unsigned long timeSync = totalTime - (timeNow - nistStart);
    syncNodes(timeSync);     // 27-Jan-2020: daily sync check with Nodes 
  }
  
  return timeSuccess;
}

//---------- Decode response from NIST & Update clock  -----------------------

void parseResponse(char response[100]) {    
  int i = 0;
  while (response[i] != '-') i++;
  
  if (i >= 100) {
    Serial.println("ERROR: Could not parse response");
    return;
  }
  
  int yr = (10 * (response[i-2] - 48) + (response[i-1] - 48));    // comes as a 2-digit year 
  byte mnth = 10 * (response[i+1] - 48) + (response[i+2] - 48);
  byte dy = 10 * (response[i+4] - 48) + (response[i+5] - 48);
  i += 7;
  byte hr = 10 * (response[i] - 48) + (response[i+1] - 48);
  byte mins = 10 * (response[i+3] - 48) + (response[i+4] - 48);
  byte sec = 10 * (response[i+6] - 48) + (response[i+7] - 48);
 
  Serial.print("Timestamp: ");
  Serial.println(String(dy)+"-"+String(mnth)+"-"+String(yr)+" "+String(hr)+":"+String(mins)+":"+String(sec));

  if ((yr < 16) || (yr > 50) || (mnth > 12) || (mnth < 1) || (dy > 31) || (dy < 1)){    // check for invalid time variables
    Serial.println("ERROR: Invalid timestamp");
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

  if(!SD.begin(SD_CS)){}      
  Serial.println();
  Serial.println("saveData(): Saving data to SD card...");
//  getFreeRAM(); // 21Jan21
  dataSaved = false;
  delay(50);

  myfile = SD.open(datafile, FILE_WRITE);   // write received transmissions to SD
  delay(100);
  dump = SD.open(dumpfile, FILE_WRITE);
  delay(100);

  if (dump){
    for (byte j = 0; j < numNodes; j++){    // write data into both SD text files
      myfile.println(nodeData[j]);
      delay(100);
      dump.println(nodeData[j]);
      delay(100);
      Serial.println("----Data----");
      Serial.println(nodeData[j]);
      delay(100);
      Serial.println("----Data----");
//      getFreeRAM();
//      Serial.println("saveData: saved string to dump & data files");
    }
      
    myfile.close();
    delay(200);
    dump.close();
    delay(200);  
    dataSaved = true;   
    
//    getFreeRAM(); // 21Jan21
//    Serial.println("saveData: closed SD card");
    
  } else {
    Serial.println("ERROR: Failed to Open SD. Sending Data Now...");
    sendDataArray();        // upload data immediately to Hologram if SD fails
    setAlarm2();      // 22Feb21
  }
}

//======================================================================================

//--------------- Clear data array ----------------------------------------------

void resetArrays(){
  for (byte index = 0; index < numNodes; index++){
    nodeData[index] = "";
  }    
}

//======================================================================================

//--------------- Send data to Hologram Cloud ----------------------------------------------

//--------------- from SD card

void sendDataSD() {

  if(!init1){
    RTC.alarmInterrupt(ALARM_1, false);          // turn on alarm 1 interrupt 
  }
//  unsigned int pvCurrent = getSolarCurrent();         // get photovoltaic current
//  float pvVoltage = getSolarVoltage();                // get photovoltaic voltage  
  uint16_t pvCurrent = GetISolar();   // using John's code
  float pvVoltage = GetVSolar();
  
  char aux_str2[FONA_MSG_SIZE];

  timestamp();                                        // compile timestamp String
  byte len1 = Timestamp.length()+1;
  char timeSave[len1]; 
  Timestamp.toCharArray(timeSave,len1); delay(20);    // convert String to char array
  
  battV = calcbattV();                                // get Li-ion battery voltage

  float boxTemp = getBoxT();                          // get enclosure temperature
  
  // separate floats into whole number and decimal parts to be able to put decimal value in sprintf
  
  byte battV_whole = battV / 1;
  int battV_dec = (battV * 100);
  battV_dec = battV_dec % 100;

  char bv_dec[3];
  if(battV_dec < 10){
    bv_dec[0] = '0';
    bv_dec[1] = battV_dec + 48;
  } else {
    bv_dec[0] = (battV_dec / 10) + 48;
    bv_dec[1] = (battV_dec % 10) + 48;
  }
  bv_dec[2] = 0;
  
  int boxTemp_whole = boxTemp / 1;   // using byte data type loses negative values!!! changed to int on 01/06/2021
  int boxTemp_dec;                   // need to account for negative boxTemp values, boxTemp_dec should always be pos
  if (boxTemp < 0){
    boxTemp_dec = -(boxTemp * 100);  
  } else {
    boxTemp_dec = (boxTemp * 100);  
  }
  
  boxTemp_dec = boxTemp_dec % 100;

  char bt_dec[3];                   // to add leading 0 if needed
  if(boxTemp_dec < 10){   
    bt_dec[0] = '0';
    bt_dec[1] = boxTemp_dec + 48;
  } else {
    bt_dec[0] = (boxTemp_dec / 10) + 48;
    bt_dec[1] = (boxTemp_dec % 10) + 48;
  } 
  bt_dec[2] = 0;
  
  byte pvV_whole = pvVoltage / 1;
  int pvV_dec = (pvVoltage * 100);
  pvV_dec = pvV_dec % 100;

  char pv_dec[3];
  if(pvV_dec < 10){
    pv_dec[0] = '0';
    pv_dec[1] = pvV_dec + 48;
  } else {
    pv_dec[0] = (pvV_dec / 10) + 48;
    pv_dec[1] = (pvV_dec % 10) + 48;
  }
  pv_dec[2] = 0;

  char gData[65];                           // array for Gateway data, compiled below 
  sprintf(gData,"%s~%s~%lu~%d.%s~%d.%s~%d~%d.%s~%s", VERSION,projectID, serNum, battV_whole, bv_dec, boxTemp_whole, bt_dec, pvCurrent, pvV_whole, pv_dec, timeSave);

  if(!SD.begin(SD_CS)){}   
  delay(20);
  
  myfile = SD.open(datafile, FILE_WRITE);   // save power data to SD
    delay(100);  
  if(myfile){
    myfile.print(gData); 
    myfile.print("~");delay(50);
    myfile.close();
    delay(200);
  } 
//  getFreeRAM(); // 21Jan21 
//  Serial.println("sendDataSD: saved gateway string to SD");
  
  fonaOn();
  delay(3000);
//  getFreeRAM(); // 21Jan21 
//  Serial.println("sendDataSD: turned cell module on");
  
  uint8_t rssi; 
  uint8_t dBm; 
   
  char ctrlZ[2];
       ctrlZ[0] = 0x1A;
       ctrlZ[1] = 0x00;
      
  memset(aux_str,0,sizeof(aux_str));
  delay(50);

  if (gStatus == 1) {
    Serial.println("sendDataSD(): Sending data from SD...");    // Send node data 
    rssi = fona.getRSSI();     // 08Jan21
    dBm = 113 - 2*rssi;
    Serial.print("RSSI: -");
    Serial.println(dBm);

    if(!SD.begin(SD_CS)){}   
    delay(20);
  
    myfile = SD.open(datafile, FILE_WRITE);   // save power data to SD
      delay(50);  
    if(myfile){
      myfile.println(dBm); 
      myfile.close();
      delay(200);
    }   

//    getFreeRAM(); // 21Jan21
//    Serial.println("sendDataSD: save RSSI to SD");
    
    char c[1];
    int pos = 0;
    c[0] = 0;
    int eof = 0;

    char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"cloudsocket.hologram.io\",9999";
    Serial.println(tcpinit); 
    answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",10000); 

    if (answer == 1){   
 
      char sendtcp[] = "AT+CIPSEND=0,";   // unknown data string length
      Serial.println();
      Serial.print(">> ");
      Serial.println(sendtcp);
      answer = sendATcommand(sendtcp,">",10000); 

      // compile json with Gateway data to send to Hologram
      sprintf(aux_str, "{\"k\":\"%s\",\"d\":\"%s~%s~%lu~%d.%s~%d.%s~%d~%d.%s~%s~-%d\",\"t\":[\"%lu\",\"GATEWAY_DATA\"]}%s\r\n\r\n",devicekey,VERSION,projectID,serNum, battV_whole, bv_dec, boxTemp_whole, bt_dec, pvCurrent, pvV_whole, pv_dec, timeSave,dBm,serNum,ctrlZ);    
    
      delay(60);
      Serial.println();
      Serial.print(">> ");
      Serial.println(aux_str);
      answer = sendATcommand(aux_str,"OK\r\n\r\n+CIPSEND: 0,",5000);    // send Gateway data
      memset(aux_str,0,sizeof(aux_str));                                // reset aux_str array

      char closesocket[] = "AT+CIPCLOSE=0";
      Serial.println();
      Serial.print(">> ");
      Serial.println(closesocket);
      answer = sendATcommand(closesocket,"OK\r\n\r\n+CIPCLOSE: 0,",5000);

//      getFreeRAM();
//      Serial.println("sendDataSD: upload gateway data string (not read from SD)");
      
      delay(20);
      while (eof != -1) {       
        if(n == 5){     //25Feb21: added n==5, 02Mar21: moved to if statement 
          if (!SD.exists(dumpfile)) {
            Serial.println("ERROR: No Data in SD to Send");
            delay(50);
            return;
          }
          if (!(dump = SD.open(dumpfile, FILE_READ))) {
            Serial.println("ERROR: File-open Failed");
            delay(50);
            return;
          }
  
  //        getFreeRAM();
  //        Serial.println("sendDataSD: open dump file");
          
          char dataToSend[FONA_MSG_SIZE];
          memset(dataToSend, 0, sizeof(dataToSend));
          /* Jumps to current position in file */
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
        
          while (eof != -1 && c[0] != '\n' && c[0] != 0) {    // populate array with row of data from DUMP file
            dataToSend[ind++] = c[0];
            if ((eof = dump.read(c, 1)) == -1)
              break;
          }
        
          dataToSend[ind-1] = '\0'; // REMOVE CARRIAGE RETURN AT END OF ARRAY
          delay(50);
          if (c[0] == '\n' || c[0] == '\r')
            pos += ind + 1;
          if (c[0] == 0)
            eof = -1;
          Serial.println("Data to send from SD:");
          Serial.println("-------Start-------");
          delay(50);
          Serial.print((int)(dataToSend[0]));
          Serial.print(" : ");
          Serial.println(eof);
          Serial.println("--------End--------");
          delay(100);
          
          // Do not send anything if blank
          boolean tcpSent = false;
          char successfulSend[] = "\r\nOK\r\n\r\n+CIPSEND: 0,";
          
          if (dataToSend[0] != 0 && dataToSend[0] != '\n' && dataToSend[0] != '\r') {
  //          for (byte i = 1; i <= 3; i++){                                            // try to send data max 3 times, removed 27Jan21: takes too long and interferes with faster interval
              if (tcpSent == false){  
                answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",10000);      // open TCP socket
                memset(aux_str,0,sizeof(aux_str));
  
              if (answer == 1){
                char sendtcp[] = "AT+CIPSEND=0,";                                     // unknown data string length
                Serial.println();
                Serial.print(">> ");
                Serial.println(sendtcp);
                answer = sendATcommand(sendtcp,">",5000);                             // send request to send data
  
                // compile data into a json string
                sprintf(aux_str2, "{\"k\":\"%s\",\"d\":\"%s\",\"t\":[\"%lu\",\"NODE_DATA\"]}%s\r\n\r\n",devicekey,dataToSend,serNum,ctrlZ);
                delay(60);
                Serial.println();
                Serial.print(">> ");
                Serial.println(aux_str2);
                answer = sendATcommand(aux_str2,"OK\r\n\r\n+CIPSEND: 0,", 5000);      // send data
                memset(aux_str2,0,sizeof(aux_str));                                   // reset array for next line of data
  //              Serial.print("answer = ");
  //              Serial.println(answer);
  
                for (byte y = 0; y < sizeof(successfulSend); y++){
                  if(response[y] == successfulSend[y]){                               // check response from Hologram confirming receipt
                    tcpSent = true;
                  } 
                }
  
  //              getFreeRAM();
  //              Serial.println("sendDataSD: send node string while socket open");
  //              Serial.print("tcpSent = ");
  //              Serial.println(tcpSent);
              
                char closesocket[] = "AT+CIPCLOSE=0"; 
                Serial.println();
                Serial.print(">> ");
                Serial.println(closesocket);
                answer = sendATcommand(closesocket,"OK\r\n\r\n+CIPCLOSE: 0,",5000);
  //
  //              getFreeRAM();
  //              Serial.println("sendDataSD: socket closed");
                
                } else {
                  Serial.println("ERROR: Failed to connect to Hologram");
                }
             }  // end if tcpSent == false
             else {}  // move to next attempt            
  //          } // end for loop that tries to send 3 times
          }   // end if dataToSend[0] != 0
              
        c[0] = 0;
        dump.close();
        delay(100);

  //      getFreeRAM();
  //      Serial.println("sendDataSD: query network status");
        } else {
          break;
        }
        n = fona.getNetworkStatus(); // 22Feb21
      }  // end while !eof loop
      
      delay(100);
      if (SD.remove(dumpfile)){
        Serial.println("File Cleared");
//        getFreeRAM();
//        Serial.println("sendDataSD: dump file removed");
       } else {
         Serial.println("File Clear Failed");
         delay(50);
       } 
      } else {
         Serial.println("Failed to connect to Hologram");
      }
  } // end if gStatus == 1
   
  delay(3000); 
  fonaOff();

  // Check Alarm 1
  if(!init1){
    if(RTC.alarm(ALARM_1)) setAlarm1(); 
    RTC.alarmInterrupt(ALARM_1, true);          // turn on alarm 1 interrupt 
  }
  Serial.println("Done sendData");
//  getFreeRAM(); // 21Jan21
//  Serial.println("sendDataSD: cell module off");
}

//--------------- from array if SD fails

void sendDataArray() {  

  //unsigned int pvCurrent = getSolarCurrent();
  uint16_t pvCurrent = GetISolar();   // using John's code
  float pvVoltage = GetVSolar();
    
//  float pvVoltage = getSolarVoltage();
//  unsigned int pvVoltageSend = pvVoltage * 1000; // convert float (V) to int (mV)
    float boxTemp = getBoxT();    // added 14-Feb-2020
  
  char aux_str2[FONA_MSG_SIZE];
        
  battV = calcbattV();                            // get Li-ion battery voltage
  byte battV_whole = battV / 1;
  int battV_dec = (battV * 100);
  battV_dec = battV_dec % 100;

  char bv_dec[3];
  if(battV_dec < 10){
    bv_dec[0] = '0';
    bv_dec[1] = battV_dec + 48;
  } else {
    bv_dec[0] = (battV_dec / 10) + 48;
    bv_dec[1] = (battV_dec % 10) + 48;
  }
  bv_dec[2] = 0;
  
  int boxTemp_whole = boxTemp / 1;   // using byte data type loses negative values!!! changed to int on 01/06/2021
  int boxTemp_dec;                   // need to account for negative boxTemp values, boxTemp_dec should always be pos
  if (boxTemp < 0){
    boxTemp_dec = -(boxTemp * 100);  
  } else {
    boxTemp_dec = (boxTemp * 100);  
  }
  
  boxTemp_dec = boxTemp_dec % 100;

  char bt_dec[3];
  if(boxTemp_dec < 10){
    bt_dec[0] = '0';
    bt_dec[1] = boxTemp_dec + 48;
  } else {
    bt_dec[0] = (boxTemp_dec / 10) + 48;
    bt_dec[1] = (boxTemp_dec % 10) + 48;
  } 
  bt_dec[2] = 0;
  
  byte pvV_whole = pvVoltage / 1;
  int pvV_dec = (pvVoltage * 100);
  pvV_dec = pvV_dec % 100;

  char pv_dec[3];
  if(pvV_dec < 10){
    pv_dec[0] = '0';
    pv_dec[1] = pvV_dec + 48;
  } else {
    pv_dec[0] = (pvV_dec / 10) + 48;
    pv_dec[1] = (pvV_dec % 10) + 48;
  }
  pv_dec[2] = 0;
    
  char ctrlZ[2];
       ctrlZ[0] = 0x1A;
       ctrlZ[1] = 0x00;
      
  memset(aux_str,0,sizeof(aux_str));
  delay(50);
  uint8_t rssi; 
  uint8_t dBm;

  fonaOn();
  delay(3000);
  
  if (gStatus == 1) {
    Serial.println("sendDataArray(): Sending data from array...");
    rssi = fona.getRSSI();     // 08Jan21
    dBm = 113 - 2*rssi;
    
    // Send node data          
    char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"cloudsocket.hologram.io\",9999";
     answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",10000); // needs longer timeout?
        memset(aux_str,0,sizeof(aux_str));

    if (answer == 1){

     char sendtcp[] = "AT+CIPSEND=0,";   // unknown data string length --> WORKS! COMMA IS 
          Serial.println();
     Serial.print(">> ");
     Serial.println(sendtcp);
     answer = sendATcommand(sendtcp,">",10000); 
      
  timestamp();
  byte len1 = Timestamp.length()+1;
  char timeSave[len1]; 
  Timestamp.toCharArray(timeSave,len1); delay(20);

//      if(!use_siteID){
//        sprintf(aux_str, "{\"k\":\"%s\",\"d\":\"%s~%lu~%d.%s~%d.%s~%d~%d.%s~%s\",\"t\":[\"%lu\",\"GATEWAY_DATA\"]}%s\r\n\r\n",devicekey,VERSION,serNum, battV_whole, bv_dec, boxTemp_whole, bt_dec, pvCurrent, pvV_whole, pv_dec, timeSave,serNum,ctrlZ);   // 14-Feb-2020: use string order consistent with Node
//      } else {
        sprintf(aux_str, "{\"k\":\"%s\",\"d\":\"%s~%s~%lu~%d.%s~%d.%s~%d~%d.%s~%s~-%d\",\"t\":[\"%lu\",\"GATEWAY_DATA\"]}%s\r\n\r\n",devicekey,VERSION,projectID,serNum, battV_whole, bv_dec, boxTemp_whole, bt_dec, pvCurrent, pvV_whole, pv_dec, timeSave,dBm,serNum,ctrlZ);    
//      }      
      delay(60);
          Serial.println();
     Serial.print(">> ");
     Serial.println(aux_str);
     answer = sendATcommand(aux_str,"OK\r\n\r\n+CIPSEND: 0,",5000);

     char closesocket[] = "AT+CIPCLOSE=0";
          Serial.println();
     Serial.print(">> ");
     Serial.println(closesocket);
     answer = sendATcommand(closesocket,"OK\r\n\r\n+CIPCLOSE: 0,",5000);

     for (byte index = 0; index < numNodes; index++){  
               
      // Do not send anything if blank
        boolean tcpSent = false;
        char successfulSend[] = "\r\nOK\r\n\r\n+CIPSEND: 0,";
        
        if (nodeData[index][0] != 0) {
          int len = nodeData[index].length() + 1;
          Serial.print("len = ");
          Serial.println(len);
          char dataToSend[len];
          nodeData[index].toCharArray(dataToSend,len);
          Serial.print("dataToSend: ");
          Serial.println(dataToSend);
          
//          for (byte i = 1; i <= 3; i++){    // try to send data max 3 times, removed 27Jan21
            if (tcpSent == false){  
              Serial.print("Sending to Hologram attempt ");
              Serial.println(i);
              
//              char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",";   
//              sprintf(aux_str,"%s\"%s\",9999",tcpinit,ipaddr);
              
              char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"cloudsocket.hologram.io\",9999";
              answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",10000); // needs longer timeout?
              Serial.println();
              Serial.print(">> ");
              Serial.println(aux_str);
               memset(aux_str,0,sizeof(aux_str));
    
              char sendtcp[] = "AT+CIPSEND=0,";   // unknown data string length --> WORKS! COMMA IS NECESSARY
              Serial.println();
              Serial.print(">> ");
              Serial.println(sendtcp);
              answer = sendATcommand(sendtcp,">",5000);
    
              sprintf(aux_str2, "{\"k\":\"%s\",\"d\":\"%s\",\"t\":[\"%lu\",\"NODE_DATA\"]}%s\r\n\r\n",devicekey,dataToSend,serNum,ctrlZ);
              delay(60);
              Serial.println();
              Serial.print(">> ");
              Serial.println(aux_str2);
              answer = sendATcommand(aux_str2,"OK\r\n\r\n+CIPSEND: 0,", 5000);
              Serial.print("answer = ");
              Serial.println(answer);

              for (byte y = 0; y < sizeof(successfulSend); y++){
                if(response[y] == successfulSend[y]){
                  tcpSent = true;
                } 
               }

              Serial.print("tcpSent = ");
              Serial.println(tcpSent);
              
              Serial.println();
              Serial.print(">> ");
              Serial.println(closesocket);
              answer = sendATcommand(closesocket,"OK\r\n\r\n+CIPCLOSE: 0,",5000);
              
            }  // end if tcpSent == false
            
            else { break; }
              
//          } // end for loop that tries to send 3 times

        }   // end if nodeData[index][0] != 0
       
     }  // end loop through nodeData
    
     delay(100);
      } else {
        Serial.println("Failed to connect to Hologram");
      }   
  } // end if gStatus == 1

  delay(3000); 
  fonaOff();

  Serial.println("Done sendData from array");
  
}

//======================================================================================

//--------------- Turn Fona on or off --------------------------------------------------

void toggle() {

  digitalWrite(Fona_Key, LOW);
  delay(3000);
  digitalWrite(Fona_Key, HIGH);

}

//======================================================================================

//--------------- Initialize Fona (SIM5320) ------------------------------------------------------

void fonaOn() {
  Serial.println();
  Serial.println("fonaON(): Turning FONA on...");
  unsigned long pTimeout = millis();// 15-Jan-2020: + 10000;
  
  while((digitalRead(Fona_PS) == LOW) && ((millis() - pTimeout) < 10000)){   // SHOULD BE IF?
  toggle();
  delay(2000);
  }

  if(digitalRead(Fona_PS) == HIGH){
  fonaSerial->begin(4800);

  if (! fona.begin(*fonaSerial)) { // turn on cell module
    if (! fona.begin(*fonaSerial)) {    // try again
    }
    else {
      fonaON = true;
    }
  }   
  else { // all is good
   fonaON = true;
  }

//  // With John's autobaud:
//
//  if (CellModem->begin(57600)){
//    Serial.println("MCU - modem link successful!");
//    fonaON = true;  
//    delay(3000);
//    GPRS_on();
//  } else {
//    Serial.println("ERROR: Cell module failed to initialize");
//  }
  
  delay(3000);

  if(init1){
    changeTimeout();
  //  checkTimeout();
  }
  
  GPRS_on();
  }
  else {
    fonaON = false;
    Serial.println("ERROR: FONA failed to turn on");
  }

 
  
}

//======================================================================================

//--------------- Turn Fona off --------------------------------------------------------

void fonaOff() {
//  uint8_t n = fona.getNetworkStatus();

//  if (n == 5){

  GPRS_off();
  delay(500);
 Serial.println();
Serial.println("fonaOff(): Turning FONA off..."); 
  fonaSerial ->end();

// with John's autobaud:
//  CellModem->end();
  delay(1000);

//  toggle();
//  } else{}

}

//======================================================================================

//--------------- Establish GPRS connection --------------------------------------------

boolean startGPRS(boolean onoff){
      
// uint8_t n = fona.getNetworkStatus();
//  getNetworkStatFull();  // 23Feb21
  
  n = fona.getNetworkStatus(); // 22Feb21: make n a global variable instead of local
  delay(1000);
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
        
  memset(aux_str,0,sizeof(aux_str));
  
  if (onoff) {

    char AT1[] = "AT+CGATT=1";
    Serial.println();
    Serial.print(">> ");
    Serial.println(AT1);
    answer = sendATcommand(AT1,"OK",30000);    // was 30000
    if (answer == 0) {
      return false;
    }

    char AT2[] = "AT+CGSOCKCONT=1";
    sprintf(aux_str, "%s,\"IP\",\"%s\"",AT2,APN);
    delay(50);
    Serial.println();
    Serial.print(">> ");
    Serial.println(aux_str);
    answer = sendATcommand(aux_str,"OK",20000); // was 30000

    if (answer == 0) {
      return false;
    }
    char start2TCP[] = "AT+CSOCKSETPN=1";
    Serial.println();
    Serial.print(">> ");
    Serial.println(start2TCP);
    answer= sendATcommand(start2TCP,"OK",20000);

    if (answer == 0){
      return false;
    }

    char AT3[] = "AT+CIPMODE=0";
    Serial.println();
    Serial.print(">> ");
    Serial.println(AT3);
    answer = sendATcommand(AT3,"OK",20000); // was 30000

    if (answer == 0){
      return false;
    }

   char AT4[] = "AT+NETOPEN";
   Serial.println();
   Serial.print(">> ");
   Serial.println(AT4);
   answer = sendATcommand(AT4,"OK\r\n+NETOPEN:0",20000);  // was 180000
    
  return true;
  delay(50);
 
  } else {

    char AT5[] = "AT+NETCLOSE";
    Serial.println();
    Serial.print(">> ");
    Serial.println(AT5);
    answer = sendATcommand(AT5, "Network closed", 20000);
    return false;
   
  }
}

//======================================================================================

//--------------- Establish GPRS connection --------------------------------------------

void GPRS_on() {
Serial.println();
Serial.println("GPRS_on(): Starting GPRS...");
  
if (!startGPRS(true)) {
   Serial.println(F("ERROR: GPRS failed to turn on"));
    gStatus = 0;
  }
  else {
    Serial.println(F("GPRS on"));
    gStatus = 1;
//    getNetworkStatFull();  // 23Feb21
  }
  delay(3000);
}

//======================================================================================

//--------------- Turn off GPRS connection ---------------------------------------------

void GPRS_off() {
Serial.println();
Serial.println("GPRS_off(): Turning off GPRS...");

  char AT5[] = "AT+NETCLOSE";
  Serial.println(AT5);
    Serial.println();
    Serial.print(">> ");
    answer = sendATcommand(AT5, "Network closed", 20000);
  delay(100);
}

//======================================================================================

//--------------- Send AT Command ------------------------------------------------------

int8_t sendATcommand(char* ATcommand, char* expected_answer1, unsigned int timeout)
{

  uint8_t x = 0,  answer = 0;
  unsigned long previous;

  memset(response, '\0', 100);    // Initialize the string

  delay(100);

  while ( Serial1.available() > 0) Serial1.read();   // Clean the input buffer

  Serial1.println(ATcommand);    // Send the AT command

  x = 0;
  previous = millis();

  // this loop waits for the answer
  do {

    if (Serial1.available() != 0) {
      if (x == 100)
      {
        strncpy(response, response + 1, 99);
        response[99] = Serial1.read();
      }
      else
      {
        response[x] = Serial1.read();
//        Serial.print(response[x]);
        x++;
      }
      // check if the desired answer is in the response of the module
      if (strstr(response, expected_answer1) != NULL || isSubstring(expected_answer1, response))
      //if (isSubstring(expected_answer1, response))
      {
        answer = 1;
      }
//      Serial.println(answer);
    }
    // Waits for the asnwer with time out
  }
   
  while ((answer == 0) && ((millis() - previous) < timeout));

//  Serial.println(answer);
  delay(50);
//  Serial.println("\n\n----------------Response Start-----------------");
//  delay(50);
  Serial.println();
  Serial.print("  << ");
  Serial.println(response);
  delay(50);
//  Serial.println("---------------Response End------------------\n\n");
//  delay(50);
  return answer;
}

int8_t sendATcommand(char* ATcommand, char* expected_answer1, unsigned int timeout, char result[100])
{

  uint8_t x = 0,  answer = 0;
  char response[100];
  unsigned long previous;

  memset(response, '\0', 100);    // Initialize the string

  delay(100);

  while ( Serial1.available() > 0) Serial1.read();   // Clean the input buffer

  Serial1.println(ATcommand);    // Send the AT command

  x = 0;
  previous = millis();

  // this loop waits for the answer
  do {

    if (Serial1.available() != 0) {
      if (x == 100)
      {
        strncpy(response, response + 1, 99);
        response[99] = Serial1.read();
      }
      else
      {
        response[x] = Serial1.read();
//        Serial.print(response[x]);
        x++;
      }
      // check if the desired answer is in the response of the module
      
      if (strstr(response, expected_answer1) != NULL)
      {
        answer = 1;
      }
    }
    // Waits for the asnwer with time out
  }
  
  while ((answer == 0) && ((millis() - previous) < timeout));
//  Serial.println("-----------Response Start-------------");
//  delay(100);
  Serial.println();
  Serial.print("  << ");
  Serial.println(response);
  delay(50);
//  Serial.println("-----------Response End -------------");
  for (int i = 0; i < 100; i++)
    result[i] = response[i];
//  Serial.println(*result);
  return answer;
//  Serial.println(answer);
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

/*// float getMOSFETresistance(float temp, float voltSD) {
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
*/

//Returns voltage of solar cell in centi-volts (battery load may affect voltage)
float GetVSolar() {
//unsigned int GetVSolar() {
  pinMode(pin_solarVoltage, INPUT);  
  float result;
  uint16_t rawSolarV = analogRead(pin_solarVoltage);
  uint16_t calc = (uint16_t)(((uint32_t)rawSolarV * 2 * ADC_REF_VOLTAGE * 100 + (ADC_MAXVALUE / 2)) / ADC_MAXVALUE);
  result = float(calc/100) + float((calc % 100))/100;
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

  uint16_t dec = (uint16_t)(vSolar*100) % 100;
  uint16_t vSolar100 = 100*((uint16_t)vSolar) + dec;
  
  return (uint16_t)(((uint32_t)vSolar100 * 1000) / iSolarRes100);
  // Explanation:
  // V = I * R  -->  I = V / R
  // NOTE: Both vSolar100 and iSolarRes100 are multiplied by 100 for greater precision but they are divided so that cancels out.
  // NOTE: As needed, terms have been typecasted to uint32_t to avoid overflow for 2 byte integer.
  // Multiplied by 1000 for conversion from amperes to "milliamps".
}
/*
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
*/

//======================================================================================
//======================================================================================
//======================================================================================

//--------------- Main Menu ------------------------------------------------------------

void MainMenu()
{
  if (Serial.available() > 0){
    Serial.read();       // clear serial input buffer
  }
  datafile[1] = (GatewayID / 10) + 48;
  datafile[2] = (GatewayID % 10) + 48; 
 
  delay(50);
  
  numNodes = EEPROM.read(EEPROM_NODE_COUNT);                        // read variables stored in memory

  if(!radioSwitch){
    GatewayID = EEPROM.read(EEPROM_DEFAULT_RADIO);
  } else {
    GatewayID = EEPROM.read(EEPROM_ACTIVE_RADIO);
  }

  uploadInt = EEPROM.read(EEPROM_ALRM2_INT);
  EEPROM.get(EEPROM_DEVKEY, devicekey);
   
  EEPROM.get(EEPROM_PROJECTID, projectID);
  
  EEPROM.get(EEPROM_NODEIDS, NodeIDs);
  interval = EEPROM.read(EEPROM_ALRM1_INT);

  delay(30);

  battV = calcbattV();

//  Serial.println();
  Serial.println(F("Soil Water Data Network - Cellular Gateway"));
  Serial.print(F("Version "));
  Serial.println(VERSION);  
  Serial.println();
  
  Serial.print(F("Serial Number: "));
  Serial.println(serNum);
  Serial.print(F("LoRa Radio Freq (MHz): "));
  Serial.println(LoRaFREQ);
  Serial.print(F("Project ID: "));
  Serial.println(projectID);
  Serial.print(F("Gateway Radio ID: "));
  Serial.println(GatewayID);
  Serial.print(F("Device key: "));
  Serial.println(devicekey);
  Serial.print(F("# of nodes: "));
  Serial.println(numNodes);
  Serial.print(F("Networked Node IDs: "));
    for (byte i=0; i< numNodes; i++){
      if(numNodes > 1 && i < (numNodes - 1)){
      Serial.print(NodeIDs[i]);
      Serial.print(", ");
      }
      else if (numNodes > 1 && i == (numNodes - 1)){
        Serial.print(NodeIDs[i]);
      } else {
        Serial.print(NodeIDs[i]);
      }
    }
  Serial.println();
  Serial.print(F("Data file: "));
  Serial.println(datafile);
  Serial.println();

  interval = EEPROM.read(EEPROM_ALRM1_INT);
  Serial.print(F("Measurement Interval: "));
  Serial.println(String(interval) + " mins"); 
  Serial.print(F("Upload interval (hrs): "));
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
  Serial.flush();

  if (battV <= lowBatt){
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
  Serial.println(F("Menu options "));
  Serial.println(F("   1  <--  Cellular Signal Scouting Mode")); // 06Aug20
  Serial.println(F("   0  <--  Enter configuration string"));    // 25-Feb-2020: enter config info all at once
  Serial.println(F("   c  <--  Set clock to NIST time"));    // set clock to NIST time  
  Serial.println(F("   i  <--  Enter project ID"));                      // set siteID
  Serial.println(F("   g  <--  Change Gateway radio ID"));
  Serial.println(F("   d  <--  Enter Hologram Device Key"));
  Serial.println(F("   n  <--  Enter Node radio IDs")); 
  Serial.println(F("   u  <--  Set data upload to cloud interval"));
  Serial.println(F("   m  <--  Set measurement interval"));
//  Serial.println(F("   S  <--  Synchronize Gateway & Node clocks"));
  Serial.println(F("   f  <--  See list of saved files"));
//  Serial.println(F("   s  <--  See sensor list"));
  Serial.println(F("   p  <--  Print node data to screen"));
  Serial.println(F("   e  <--  Erase microSD card"));
  Serial.println(F("   x  <--  Exit menu"));             // exit
  Serial.print(F("Enter choice: "));
  Serial.flush();

  timeout = millis() + 30000;                         // wait 30 secs for input
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

  switch (menuinput)                               // take action based on user input
  {
    case 49:                  // ------ 1 - Enter scouting mode ---------------------------------------------
      Serial.println();
      Serial.println("Entering signal scouting mode...");
      scoutMode();
      MainMenu();
      break;
    case 48:                   // ------ 0 - Enter config string ---------------------------------------------
      Serial.println();
      Serial.print(F("Enter configuration string: "));
      charinput();
      Serial.println();
      decodeConfig(charInput);
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

      MainMenu();
      break;

     case 100: case 68:         // ---------- d - Enter Hologram Device Key ---------------------------------------------------
      Serial.println();
      devKey();

      MainMenu();
      break;
        
     case 110: case 78:         // ---------- n - Enter Node radio IDs ---------------------------------------------------       
      Serial.println();
      nodeIDs();

      MainMenu();                                      // go back to menu
      break;    

    case 117:   // ------ u - Set upload interval ---------------------------------------------
      Serial.println();
      uploadInterval();
      MainMenu();
      break;
      
    case 109: case 77:        //--------- m - Set measurement interval----------------------------
      Serial.println();
      if(!measureInt()){
        Serial.println(F("ERROR: upload interval must be entered first. Returning to menu..."));
      }
      MainMenu();
      break;

    /*case 83:    // ------ S -Synchronize G & N clocks ---------------------------------------------
      Serial.println();
      Serial.println(F("Make sure same Node menu option open in another Serial Monitor window"));
      Serial.println(F("Sending time to Node..."));
      syncTime();
      delay(500);
      MainMenu();
      break;
      */
    
    case 102: case 70:          // ------ f - See list of saved files ---------------------------------------------
      Serial.println();
      Serial.println(F("Saved files: "));     // list all files on SD card
      delay(10);
    
      if (!SD.begin(SD_CS)) {Serial.println("ERROR: SD fail!");}

      root = SD.open("/");
      printDirectory(root, 0);
      Serial.println();

      MainMenu();
      break;

    case 112: case 80:          // ------ p - Print node data to screen ---------------------------------------------
      Serial.println();
      Serial.println(F("Print data files: "));

      delay(20);
      if (!SD.begin(SD_CS)) {Serial.println("ERROR: SD fail!");}
      
      myfile = SD.open(datafile);
      if(myfile){
        while(myfile.available()){
          Serial.write(myfile.read());
        }
      }
      myfile.close();
      Serial.println();
      
      MainMenu();
      break;


    case 101: case 69:         // ------ e - Erase SD card ---------------------------------------------
      Serial.println();
      Serial.println(F("Erase SD card: "));

//      SPI.begin();
      delay(20);
      if (!SD.begin(SD_CS)) {Serial.println("ERROR: SD fail!");}

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

      MainMenu();
      break;
  }
   
  Serial.println(F("Exit"));                       // exit
  Serial.println();
  delay(100);
  if( userinput == true){
    digitalWrite(LED,LOW);
  }
}

//======================================================================================
//======================================================================================
//======================================================================================

//-------------- Decode config string --------------------------------

void decodeConfig(char config_string[55]){
  
  bool configOK = true;
  byte configSize;

  char buf[55];

  for(byte i = 0; i < 55; i++){  
    buf[i] = config_string[i];   // put config string into a buffer
  }

  for(byte i = 0; i < 55; i++){
    if(buf[i] == 0){                // find the length of the config string
      configSize = i + 1;
      break;
    }
  }
  
  char configG[configSize];        // create new array of correct size
  for(byte i = 0; i < configSize; i++){
    configG[i] = buf[i];
  }
  Serial.println(configG);


  //--- Find comma positions
  
  uint8_t commaPos[15]; 
  byte commaCount = 0;
  
  for (uint8_t i = 0; i < configSize ; i++){
    if (configG[i] == ','){
      commaPos[commaCount] = i;
      commaCount++;
//      Serial.println(commaCount); 
    }
  }
  
//  Serial.print("Commas at positions ");
//  for(byte q = 0; q < commaCount; q++){
//    Serial.print(commaPos[q]);
//    Serial.print(" ");
//  }
//  Serial.println();

 //--- Get Project ID
  
  byte firstComma = 1;

  if(commaPos[0] > 5){
    Serial.println("ERROR: Project ID missing!");
    configOK = false;
  }

  if(configOK){
    byte i = 0;
    for (i = 0; i < commaPos[0]; i++) {
      projectID[i] = configG[i];
    }
    projectID[i] = 0;
    Serial.print(F("projectID: "));
    Serial.println(projectID); 
     
 //--- Match serial num

  uint8_t SerNum[8];

//  if(!use_siteID){
//    for (byte i = 0; i < 8; i++){
//      SerNum[i] = configG[i] - 48;
//    }
//  } else {
    for (byte j = 0; j < 8; j++){
      SerNum[j] = configG[j + commaPos[0] + 1] - 48;
//      Serial.print(SerNum[j]);
    }
//  }

  uint32_t serNumCheck = 0;
   
  for (byte i = 0; i < 8; i++){
    long baseTen = 1;
    for(byte j = 0; j < (7 - i); j++){
      baseTen = baseTen * 10;
    }
    serNumCheck += (SerNum[i] * baseTen);
  }

  Serial.println(serNumCheck);

  if (serNumCheck != serNum){
    Serial.println("ERROR: Serial numbers do not match!");  
    configOK = false;  
  }

  Serial.println(commaPos[firstComma + 1] - commaPos[firstComma] + 1);
  if((commaPos[firstComma + 1] - commaPos[firstComma] + 1) != 10){
    configOK = false;
    Serial.println("ERROR: Invalid device key!");
  } else {   
    byte y = 0; 
    for(byte i = commaPos[firstComma] + 1; i < commaPos[firstComma + 1]; i++){
      devicekey[y] = configG[i];  
      y++;
    }
    devicekey[8] = 0;
    Serial.print("device key: ");
    Serial.println(devicekey);    
  }

  //--- get Gateway ID

  if ((commaPos[firstComma + 2] - commaPos[firstComma + 1]) == 2){   // radio ID is one digit
    GatewayID = configG[commaPos[firstComma + 1] + 1] - 48;
  } else {
    GatewayID = 10 * (configG[commaPos[firstComma + 1] + 1] - 48) + (configG[commaPos[firstComma + 2] - 1] - 48);
  }

  Serial.print("Gateway ID: ");
  Serial.println(GatewayID);

  //--- get # Nodes and IDs

  byte pos = commaPos[firstComma + 2] + 1;
  numNodes = configG[pos] - 48;

  Serial.print("numNodes: ");
  Serial.println(numNodes);

  byte t = 0;
  for (byte i = 0; i < numNodes; i++){
   if(((3 + t) != (commaCount - 1)) && ((4 + t) != (commaCount - 1))){
    if((commaPos[firstComma + 4 + t] - commaPos[firstComma + 3 + t]) == 2){
      NodeIDs[i] = configG[commaPos[firstComma + 3 + t] + 1] - 48;
    } else {
      NodeIDs[i] = 10 *(configG[commaPos[firstComma + 3 + t] + 1] - 48) + configG[commaPos[firstComma + 3 + t] + 2] - 48;
    }
    t++;
    }
    Serial.print("NodeIDs[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(NodeIDs[i]);
  }

//  if(!use_siteID){
//    if((commaCount - 5) != numNodes){
//      Serial.println("ERROR: Incorrect number of radio IDs"); // too many or too few radio IDs listed
//      configOK = false;
//    }
//  } else {
     if((commaCount - 6) != numNodes){
      Serial.println("ERROR: Incorrect number of radio IDs"); // too many or too few radio IDs listed
      configOK = false;
    }
//  }

  //--- get measurement interval
  
  byte lastComma = commaCount - 1;
  interval = 10 * (configG[commaPos[lastComma - 1] + 1] - 48) + (configG[commaPos[lastComma - 1] + 2] - 48);
  
  Serial.print("interval: ");
  Serial.println(interval);

    if (interval != 10 && interval != 15 && interval != 20 && interval != 30 && interval != 60) {   
      Serial.println(F("ERROR: Invalid interval (every 10, 15, 20, 30, or 60 mins)"));
      configOK = false;
    }  

  //--- get upload interval

  uploadInt = configG[commaPos[lastComma] + 1] - 48;
  Serial.print("uploadInt: ");
  Serial.println(uploadInt);

    if (uploadInt != 1 && uploadInt != 4){
      Serial.println(F("ERROR: Invalid upload interval (every 1 or 4 hours)"));
      configOK = false;
    }
  }
  
 if(!configOK){
  Serial.println("!!! FAIL: Invalid configuration !!!"); 
 } else {
  Serial.println("Configuration complete");
//    EEPROM.update(EEPROM_SITEID_FLAG, use_siteID);
    EEPROM.put(EEPROM_PROJECTID, projectID);
  if(GatewayID != default_radioID){
    radioSwitch = 1;
    EEPROM.update(EEPROM_ACTIVE_RADIO,GatewayID);
  } else {
    radioSwitch = 0;
  }  
  EEPROM.update(EEPROM_FLAG_RADIO, radioSwitch);
  LoRa.setThisAddress(GatewayID);
  EEPROM.put(EEPROM_DEVKEY, devicekey);
  EEPROM.update(EEPROM_NODE_COUNT, numNodes);                           // save number of reps to EEPROM
  EEPROM.put(EEPROM_NODEIDS, NodeIDs);
  EEPROM.update(EEPROM_ALRM1_INT, interval);
  EEPROM.update(EEPROM_ALRM2_INT, uploadInt);
  
 }

}

void changeDefault(){
   Serial.println(F("Would you like to change the radio ID from the default value?"));
  Serial.print(F(" Enter 'y' for yes, 'n' for no: "));
  charinput();
  radioSwitch = (charInput[0] == 'y' || charInput[0] == 'Y'? 1:0);
  EEPROM.update(EEPROM_FLAG_RADIO, radioSwitch);
  if(radioSwitch){
    Serial.println();
    Serial.print(F("Enter new radio ID: "));
    getinput();
    GatewayID = indata;
    EEPROM.update(EEPROM_ACTIVE_RADIO,GatewayID);
    Serial.println(F("Radio ID updated"));
  } else {
    Serial.println(F("Default radio ID accepted"));
    GatewayID = default_radioID;
  }
  LoRa.setThisAddress(GatewayID);
}
 
void devKey(){
  Serial.print(F("Enter the Hologram device key for this Gateway: "));
  bool devkeyOK = true;
  charinput();
  char devkeyTemp[9];
  
  for (byte i = 0; i < 8; i++){
    if(charInput[i] != 0){
      devkeyTemp[i] = charInput[i];
    } else {
      devkeyOK = false;
    }
  }
  devkeyTemp[8] = 0;
  
  if(devkeyOK){ 
    for(byte i = 0; i < 9; i++){
      devicekey[i] = devkeyTemp[i];
    }
    EEPROM.put(EEPROM_DEVKEY, devicekey);
    delay(10);
  } else {
    Serial.println("ERROR: Invalid device key!");
  }
  delay(500);  
}

void nodeIDs(){
   Serial.print(F("How many nodes are at this site (max. "));
      Serial.print(maxNodes);
      Serial.print(")? ");
      getinput();
      numNodes = indata;
      long utimeout = millis() + 10000;
      while (numNodes > maxNodes || numNodes <= 0 && millis() < utimeout){
        Serial.println(F("Invalid number of nodes.  How many nodes are at this site (max.")); 
        Serial.print(maxNodes);  
        Serial.println(F(")?"));
        getinput();
        numNodes = indata;
      }

      EEPROM.update(EEPROM_NODE_COUNT, numNodes);                           // save number of reps to EEPROM
      delay(20);

//      Serial.println();

      for (byte n = 1; n <= numNodes; n++){
        Serial.print(F("Enter the radio ID for Node "));
        Serial.print(n);
        Serial.print(": ");
        byteInput();
        NodeIDs[n-1] = u_indata;
      }
      delay(500);

      EEPROM.put(EEPROM_NODEIDS, NodeIDs);
}

void uploadInterval(){
  Serial.println(F("Set upload interval to 1 or 4 hours."));
  Serial.print(F("Enter \"1\" or \"4\": "));
  byteInput();
    uploadInt = u_indata;
//    Serial.println(uploadInt,DEC);
    long utimeout = millis() + 10000;
  
  while((uploadInt != 1 && uploadInt != 4) && millis() < utimeout){
    Serial.println(F("Invalid interval. Please enter 1 or 4."));
    byteInput();
  }

  EEPROM.update(EEPROM_ALRM2_INT, uploadInt);
  delay(20);
}

boolean measureInt(){

  if(uploadInt == 4){
   Serial.print(F("Enter measurement interval (10, 15, 20, 30, or 60 mins): "));
      Serial.flush();
      boolean intSet = false;
      long utimeout = millis() + 10000;
      while(intSet == false && millis() < utimeout){        
        getinput();
        if (indata != 10 && indata != 15 && indata != 20 && indata != 30 && indata != 60) {
          Serial.print(F("Invalid interval. Enter measurement interval (10, 15, 20, 30, or 60): "));
          Serial.flush();
        }

        // CODE BELOW NEEDS TESTING  
        /* else if(indata == 15 && numNodes > 2){   // 08Jan21: numNodes limit for 10 minutes?
          Serial.println(F("ERROR: maximum 2 Nodes at 15 minute interval"));
          Serial.println(F("Please enter another measurement interval: "));
        } else if (indata == 20 && numNodes > 4){
          Serial.println(F("ERROR: maximum 4 Nodes at 20 minute interval"));
          Serial.println(F("Please enter another measurement interval: "));
        } */

        else {
          intSet = true;
          interval = indata;
          EEPROM.update(EEPROM_ALRM1_INT, interval);
          delay(20);
          return true;
        }   
     }
  } else if (uploadInt == 1){
    Serial.print(F("Enter measurement interval (10, 15, 20, 30, or 60 mins): "));
      Serial.flush();
      boolean intSet = false;
      while(intSet == false){        
        getinput();
        if (indata != 10 && indata != 15 && indata != 20 && indata != 30 && indata != 60) {
          Serial.print(F("Invalid interval. Enter measurement interval (10, 15, 20, 30, or 60): "));
          Serial.flush();
        }
        // CODE BELOW NEEDS TESTING
        /*
        else if(indata == 10 && numNodes > 3){
          Serial.println(F("ERROR: maximum 3 Nodes at 10 minute interval"));
          Serial.println(F("Please enter another measurement interval: "));
        } else if(indata == 15 && numNodes > 8){
          Serial.println(F("ERROR: maximum 8 Nodes at 15 minute interval"));
          Serial.println(F("Please enter another measurement interval: "));
        } */
        else {
          intSet = true;
          interval = indata;
          EEPROM.update(EEPROM_ALRM1_INT, interval);
          return true;
        }   
     }
  } 
  else {  // uploadInt != 1 or 4 
    return false;
  }
}

void NISTsync(){
  Serial.println(F("Synchronizing clock to NIST server ... "));
      fonaOn();
      delay(3000);
      timeSuccess = NISTtime();
      if (timeSuccess == false){
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
        fonaOff();
        delay(1000);
      }
}

//======================================================================================
//======================================================================================
//======================================================================================

//-------------- Get User Input for numerical variables --------------------------------

void getinput()
{
  timeout = millis() + 15000;        // allow user time to input

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

void byteinput()
{
  timeout = millis() + 15000;        // allow user time to input

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
  byte numincoming;
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

  byte numincoming;
  
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

//--------------- Sync G & N clocks [NOT USED] ----------------------------------------------------

/*void syncTime(){
//   if (!manager.init()) {                      // initialize radio
//    Serial.println("Radio failed");
//  }
//    driver.setFrequency(LoRaFREQ);
//  driver.setTxPower(TxPower);   
// manager.setRetries(retryNum);

  if (!LoRa.init(LoRaFREQ, timeoutACK, TxPower, retryNum, GatewayID)) {                      // initialize radio
    Serial.println("Radio failed");
    return;
  }

  uint8_t buf[4];     // array to receive time request from Node
  uint8_t len = sizeof(buf);
  uint8_t from;
  boolean timeSynced = false;
//  delay(5000);

  long timeout = millis();// + 45000;

  while(((millis() - timeout) < 45000) && (timeSynced == false)){
//    Serial.println(timeSynced);
/*   if(manager.recvfromAckTimeout(buf, &len, radioTimeout, &from)){  // if time request received
    if (len > 0) {
      for (byte j=0; j < sizeof(buf); j++){     // store tranmission into buffer
//      Serial.print(char(buf[j]));
      }
      readClock();
      String Tupdate = String(yrs) + "/" + String(mnths) + "/" + String(days) + "|" + String(hrs) + ":" + String(mins) + ":" + String(secs); 
      uint8_t len_resp = Tupdate.length()+1;
          char resp1[len_resp];
          uint8_t resp[len_resp];
          Tupdate.toCharArray(resp1, len_resp);
//          Serial.println(resp1);
          for (byte y = 0; y <= len_resp; y++) {           // convert timestamp to uint8_t for sending
            resp[y] = resp1[y];
          }
      manager.sendtoWait(resp,len_resp,from);    
      timeSynced = true;
    }
   }
   if (LoRa.recvfromAckTimeout(buf, &len, timeoutSmallPkt, &from)) { // if time request received
      if (len > 0) {
        for (byte j = 0; j < sizeof(buf); j++) {  // store tranmission into buffer
          //      Serial.print(char(buf[j]));
        }
        readClock();
        String Tupdate = String(yrs) + "/" + String(mnths) + "/" + String(days) + "|" + String(hrs) + ":" + String(mins) + ":" + String(secs);
        uint8_t len_resp = Tupdate.length() + 1;
        char resp1[len_resp];
        uint8_t resp[len_resp];
        Tupdate.toCharArray(resp1, len_resp);
        //          Serial.println(resp1);
        for (byte y = 0; y <= len_resp; y++) {           // convert timestamp to uint8_t for sending
          resp[y] = resp1[y];
        }
        LoRa.sendtoWait(resp, len_resp, from);
        timeSynced = true;
      }
    }
  }
  if (timeSynced ==  true){
    Serial.print(F("Node "));
    Serial.print(from);
    Serial.println(F(" succesfully synced to Gateway"));
  } else {
    Serial.println("Synchronization failed. Return to menu to try again.");
  }
      
}
*/
//======================================================================================

//--------------- Cellular Signal Strength Scout Mode ----------------------------------

void scoutMode(){   // 06Aug20
  Serial.println(F("Turning cell module on..."));
  fonaOn();

  if(fonaON){
    if(gStatus == 1){     
      Serial.println(F("Enter 'x' at any time to stop"));
      while(Serial.read() != 'x'){
    
        uint8_t rssi = fona.getRSSI(); 
    //    Serial.println(response);
        
        // getRSSI() sends AT+CSQ Query Signal quality command:
        // returns <rssi>,<ber> 
        // <rssi> = received signal strength indication
        // 0      --> -113 dBm or less
        // 1      --> -111 dBm 
        // 2...30 --> -109...-53 dBm
        // 31     --> -51 dBm or greater
        // 99     --> not known or not detectable
        // <ber> = channel bit error rate
        
//        String rssi = response.substring(response.length()-2);
//        int rssiNum = rssi.toInt();
        uint8_t dBm = 113 - 2*rssi;
        Serial.print(F("RSSI (dBm): -"));
        Serial.print(dBm);
    
        if(dBm < 70){
          Serial.println("  EXCELLENT");  
        } else if(dBm > 70 && dBm <= 85){
          Serial.println("  GOOD");
        } else if (dBm > 85 && dBm <= 100){
          Serial.println("  FAIR"); 
        } else if(dBm == 99){
          Serial.println("  UNKNOWN");
        } else {
          Serial.println("  POOR");
        }       
        delay(1000);
      }
      Serial.println(F("Turning cell module off..."));
      fonaOff();
    } else {
      Serial.println("ERROR: could not connect to network");
    }
  } else {
    Serial.println("ERROR: could not turn on cellular module");
  }
}

void getFreeRAM() {
  extern uint16_t __heap_start, *__brkval;
  uint16_t v;
  uint16_t freeRAM;
  freeRAM = (uint16_t)&v - (__brkval == 0 ? (uint16_t) &__heap_start : (uint16_t) __brkval);
  Serial.print("Free RAM: ");
  Serial.print(freeRAM);
  Serial.print(',');
}

void checkTimeout(){
  Serial.println();
  Serial.println("Checking timeouts: ");

  char check[] = "AT+CIPTIMEOUT?";
  answer = sendATcommand(check, "+CIPTIMEOUT: ", 20000);
  delay(100);
}

void changeTimeout(){
//  Serial.println();
//  Serial.println("Changing timeouts: ");

  char change[] = "AT+CIPTIMEOUT=21000,10000,10000";
  answer = sendATcommand(change, "OK", 20000);
  delay(100);
}

void getNetworkStatFull(){
  
  Serial.println("get full Network status");
  
  char nstat[] = "AT+CPSI?";
  unsigned long timeout = 10000;
  uint8_t x = 0;
  unsigned long previous;

  memset(response, '\0', 100);    // Initialize the string

  delay(100);

  while ( Serial1.available() > 0) Serial1.read();   // Clean the input buffer
  Serial1.println(nstat);    // Send the AT command

  x = 0;
  previous = millis();

  // this loop waits for the answer
  do {
    if (Serial1.available() != 0) {
      if (x == 100)
      {
        strncpy(response, response + 1, 99);
        response[99] = Serial1.read();
      }
      else
      {
        response[x] = Serial1.read();
        Serial.print(response[x]);
        x++;
      }     
    }
    // Waits for the asnwer with time out
  }
   
  while (((millis() - previous) < timeout));

  delay(50);
  Serial.println(response);
  delay(50);

}
