/* Version 102
  Differences from version 11:
  - Fixes bugs in fieldSync()
  
  Acclima Cooperative - Cellular Gateway
    Components:
      - ATMega1284P with MoteinoMEGA core
      - DS3231 Precision RTC
      - LoRa radio transceiver
      - SimCom 5320A cellular module
      - microSD slot

    Main Functions: 
      - Wake at user-specified intervals to receive incoming data arrays 
        from networked nodes via LoRa radio transceivers, respond with 
        time update
      - Store received data arrays onto microSD card, send arrays to
        Hologram Cloud web host via TCP using cellular module

    Supplemental Functions:
      - Synchronize time with NIST time server
      - Receive sensor information from Nodes, send to Hologram
      - UI via Main Menu and Quick Setup over Serial Monitor
          
    Program Summary: dictated by two alarms
      - Alarm 1: wakes up 1 minute after Nodes take measurements
        - Query Nodes sequentially with timestamp, receive data response (over LoRa)
        - Save received data to microSD card
      - Alarm 2: wake up at 4 minutes past upload interval
        - if time to get sensor lists: get lists from Nodes over LoRa, send to Hologram  (cellular + TCP)   
        - if time for time update: get time from NIST server
        - if time to send data to Hologram: read in data from microSD card, send to Hologram

    Info stored on EEPROM:
       0:
       1:  GatewayID
       2:  siteID (3 bytes, locations 2-4)
       3:  
       4:  
       5:  numNodes
       6:  interval 
       7:  NodeIDs (10 (maxNodes) bytes, locations 7-16
       8:  
       9:  
       10: 
       20: device key (8 bytes, locations 20-27)
       30: alarm2int
        

   Written by:
   Alondra Thompson, USDA-ARS Sustainable Agricultural Systems Lab
   Justin Ayres, Univeristy of Maryland Computer Science Department
   John Anderson, Acclima Inc. 
   
   Last edited: May 16, 2019

*/

//===================================================================================================

// ------- Libraries ----------------------------------------------------

#include <SD.h>                                    // SD card library
#include <SPI.h>                                   // SPI library for microSD breakout
#include <Wire.h>                                  // standard I2C library
#include <EEPROM.h>                                // ATmega1284P internal non-volatile memory
#include "avr/sleep.h"                             // sleep functions
#include "DS3232RTC.h"                             // RTC library 
#include <RH_RF95.h>                               // Moteino LoRa library
#include <RHReliableDatagram.h>                    // allows acknowledgements and retries
#include "Adafruit_FONA.h"                         // for Adafruit FONA 3G cellular breakout
#include "avr/io.h"
#include "avr/interrupt.h"
#include "avr/wdt.h"                               // controls watchdog timer (we want to turn it off)
#include "SetSpeed.h"                              //John, 27-Mar-2019: Required to slow MCU speed to 8 MHz using a prescaler


// ------- Assign Pins ----------------------------------------------------

#define LED        15       // MoteinoMEGA LED
#define SD_CS      3        // D3 for SD card ChipSelect 
#define hardSS     4
#define Fona_Key   0        // turn FONA on/off
#define Fona_RST   13       // FONA hard reset - toggle low for 100ms to reset
#define FONA_RX    11       // communication w/ FONA if using software serial, hardware Serial1 is the same
#define FONA_TX    10
#define Fona_PS    25       // FONA Power Status - LOW when off, HIGH when on
#define BattV      24       // A0, for calculating Vin from battery
#define Mbatt      14       // Switches voltage divider on/off
#define baudRate   57600    //John, 27-Mar-2019: Changed from 115200 to 57600 because when MCU at 8 MHz the sampling resolution is reduced
#define maxNodes   10
#define FONA_MSG_SIZE 255   // max length for sending to Cloud
#define pin_SD_OFF 1        //physical pin 41  D1
#define pin_MOSI 5          //physical pin 1  D5
#define pin_solarVoltage    A5
#define ADC_REF_VOLTAGE     3.3
#define ADC_RESOLUTION      1024
#define pin_solarShort      A4
#define SOLAR_CALIB      1.0          //This will become a EEPROM constant that is set during factory config – for now just use 1.0
//#define LoRaFREQ      915           //For US-based LoRa
//#define LoRaFREQ      433             //For Region 1 LoRa (Jordan)

// ------- Declare Variables -----------------------------------------------

  SetSpeed ss;  //John, 27-Mar-2019: Special creation of a dummy SetSpeed object to invoke prescaler on MCU as soon as possible

//-----*** Site/Gateway Identifier ***-----

  char  siteID[4];                              // char array for 3-digit site ID


//-----*** for EEPROM ***-----
#define EEPROMSHIFT                 2800
#define EEPROM_GATEWAYID            (1 + EEPROMSHIFT)         // storage location for gatewayID                     (was gatewayMem)
#define EEPROM_SITEID               (2 + EEPROMSHIFT)         // first storage location for siteID array            (was siteIDMem)
#define EEPROM_NODE_COUNT           (5 + EEPROMSHIFT)         // storage location for numNodes                      (was nodesMem)
#define EEPROM_ALRM1_INT            (6 + EEPROMSHIFT)         // storage location for Alarm 1 interval              (was intervalMem)
#define EEPROM_NODEIDS              (7 + EEPROMSHIFT)         // first storage location for nodeIDs array           (was nodeIDsMem)
#define EEPROM_DEVKEY               (20 + EEPROMSHIFT)        // first storage location for device key array        (was devkeyMem)
#define EEPROM_ALRM2_INT            (30 + EEPROMSHIFT)        // storage location for alarm2int                     (was alarm2Mem)


#define EEPROM_OPTSRADIO            17



// byte  gatewayMem = 1;                         // storage location for GatewayID
// byte  siteIDMem = 2;                          // first storage location for siteID array
// byte  nodesMem = 5;                           // storage location for numNodes
// byte  intervalMem = 6;                        // storage location for Alarm 1 interval (interval)
// byte  nodeIDsMem = 7;                         // first storage location for nodeIDs array
// byte  devkeyMem = 20;                         // first storage location for device key array
// byte  alarm2Mem = 30;                         // storage loation for alarm2int


uint16_t LoRaFREQ;                            // This value is determined by reading EEPROM constant (see method getLoRaFreq())




//-----*** for Main Menu ***-----

  int    menuinput;                             // user input in menu
  int    numincoming;                           // number of bytes coming from user input
  long   timeout;                               // msec to wait for user input
  int    indata;                                // integer values entered as user unput
  int    input;                                 // holder for converting ASCII value to decimal
  int    incoming[7];                           // holder for integer user input (getinput())
  char   charInput[20];                         // holder for character user input (charinput())
  char   incomingChar[20];                      // holder for incoming char or byte values
  uint8_t  u_indata;                            // holder for incoming uint8_t value (byteInput())
  boolean firsttime = true;                     // tracks if first time menu appears (give quick start option only once) 


//-----*** for battV (12V) calculation ***-----

  float battV;                                      // battery voltage (V) --> max 12V with voltage regulator
  float resist = 1.333;                             // for voltage divider: R1=10k, R2=30k, resist =  (R1+R2)/R2
  float multiplier = 0.00322;                       // analog input resolution (3.3V / 1024 units)
  float lowBatt = 3.4;                              // low battery limit

//---*** Counters ***---

  byte    i;                                    // counter for user input functions
  boolean init1 = true;

//-----*** for LoRa Radio ***-----

  //-- Identifiers, Metadata
  
  uint8_t GatewayID;                            // Gateway radio ID
  uint8_t NodeIDs[maxNodes];                    // networked Node IDs
  byte  numNodes;                               // number of nodes connected to Gateway

  //-- Settings
  
  uint16_t radioTimeout = RH_DEFAULT_TIMEOUT;   // 200 ms timeout
  uint8_t  retryNum = 10; //RH_DEFAULT_RETRIES;        // 3 retries
  byte TxPower = 20;                            // in dBm, default 13 dBm, range: +5 to +23
  
  //-- for Data Storage
  
  char nodeData[maxNodes][FONA_MSG_SIZE];       // 2-D array for storing all node data
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  
  String  allIDs = "";                           // String for holding received sensor lists
  uint8_t IDfails[maxNodes];                    // for tracking radio IDs of Nodes that failed to send IDs (starts same as NodeIDs, successes set to 0)
  boolean IDsuccess = false;                    // for tracking state of sensor ID receipts (false ==> not all Nodes have sent ID


//-----*** for RTC ***-----

  //-- Time and Date values
  
  byte    secs;                                 // time and date values
  byte    mins;
  byte    hrs;
  byte    days;
  byte    mnths;
  int     yrs;
  byte    startHr;                              // holder for hours value on start up
  
  tmElements_t tm;
  
  //-- for Alarms

  byte  interval;                               // user-selected loop interval (minutes), interval for Alarm 1
  byte  alarmMins;                              // minutes value at which Alarm 1 goes off
  byte  alarm2int;                              // # hours between sending data to cloud (Alarm 2)
  byte  IDmins = 36;                            // wake-up to receive sensor IDs @ 12:36am Central time (for Alarm 2)
  byte  IDhrs = 18;
  byte  interrupted;                            // hours value of last wake-up before setting alarm to IDhrs and IDmins (to maintain alarm2int)
  byte  NISThr;                                // hours value to sync to NIST time (Alarm 2)  
  byte  NISTmin;
  boolean timeSuccess = false;                  // tracks success of syncing to NIST server                    
  
  //-- Time Storage
  
  String Timestamp="";                          // for compiling timestamp into YYYY_MM_DD_hh:mm:ss_UTC format


//-----*** for microSD ***-----

  //-- File names
  
  char  datafile[] = "000.txt";                   // microSD text data file name
  char  dumpfile[] = "dump.txt";                  // dump file name (contains only unsent data)
  char  sensorIDfile[] = "sensors.txt";           // file for sensor lists
  
  //-- Boolean
  
  boolean dataSaved = false;                      // flag for confirming successful save
  
  //-- for erasing SD card
  
  byte  DeletedCount = 0;                         // number of files deleted  
  byte  FolderDeleteCount = 0;                    // number of deleted folders
  byte  FailCount = 0;                            // delete fails
  String rootpath = "/";                          // indicates path 


//-----*** for SIM5320 ***-----

  //-- Comm with module
  
  char response[100];                               // holder for responses from cellular module and network
  uint8_t answer;                                   // flag for matching expected response to actual response (sendATcommand())
  char aux_str[FONA_MSG_SIZE];                      // buffer for compiling AT commands
  boolean fonaON = false;                           // flag for cellular module power status
  
  //-- Network settings
  char  APN[] = "hologram";                         // cellular provider APN
  uint8_t gStatus;                                  // FONA GPRS status

  //-- for fieldSync
  boolean userinput = false;
  char toSend[FONA_MSG_SIZE];
  
  //-- for posting to Hologram Cloud ---
  
  char  devicekey[9];
  boolean missed = false;                           // tracks if data fails to send to Hologram; set to true if gStatus = 0 in sendData(), resets in sendMissed()
  byte  numMissed = 0;                              // tracks how many unsent data strings; iterates in sendData if gStatus = 0; resets to zero in sendMissed()
  boolean missedSent = false;                       // tracks if missed data sent to Hologram


// ------- Initialize ----------------------------------------------------

  RH_RF95 driver;                                     // declare radio type 
  RHReliableDatagram manager(driver, GatewayID);      // initialize radio mode
  
  HardwareSerial *fonaSerial = &Serial1;              // initialize serial port for comm to SIM5320    
  
  Adafruit_FONA_3G fona = Adafruit_FONA_3G(Fona_RST); // initialize cellular module object
  
  File myfile;    // initialize object for sensor data file 
  File dump;      // initialize object for dump file
  File IDfile;    // initialize object for sensor list file
  File root;      // initialize dummy root file

//  Sd2Card card;

//===========================================================================================

//-------------- Setup ----------------------------------------------------------------------
void setup()
{
   Serial.begin(baudRate);
    delay(100);
    Serial.println("Hello");

      //--- Pin settings

    pinMode(LED, OUTPUT);                           
      digitalWrite(LED, HIGH);
    pinMode(SD_CS, OUTPUT);                         // set ChipSelect pin as output
    pinMode(hardSS, OUTPUT);
      digitalWrite(hardSS, HIGH);
    pinMode(Fona_Key, OUTPUT);                      
    pinMode(Fona_PS, INPUT);                        // input; reads HIGH or LOW
    
    pinMode(BattV, INPUT);                          // reads A0 to calculate SLA battery V
    pinMode(pin_solarVoltage, INPUT);
    pinMode(pin_solarShort, OUTPUT);
    pinMode(pin_SD_OFF, OUTPUT); 
    
    digitalWrite(Fona_Key, HIGH);                   // set HIGH so toggle() works
    randomSeed(analogRead(4));                      // to randomize IP choices in NISTtime 
    
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
//    SD_off();
  

  //--- Power saving
  
    wdt_disable();                                 // turn off watchdog timer 
    RTC_osc_off();                                 // Turn off 32kHz output

  //--- Initialize radio settings

    LoRaFREQ = getLoRaFreq();

    if (!manager.init()) {                      // initialize radio
      Serial.println("ERROR: radio init failed");
      return;
    }
     driver.setFrequency(LoRaFREQ);
  driver.setTxPower(TxPower);   
 
    GatewayID = EEPROM.read(EEPROM_GATEWAYID);
    manager.setThisAddress(GatewayID);
    manager.setTimeout(radioTimeout);
    manager.setRetries(retryNum);

  

  //--- Read variables from memory ---
  
    EEPROM.get(EEPROM_SITEID, siteID);          // get siteID and store into array
     delay(20);
     for (byte s = 0; s < 3; s++){
        datafile[s] = siteID[s];
        }
        siteID[3] = 0;
      delay(50);
      
    interval = EEPROM.read(EEPROM_ALRM1_INT); 
    numNodes = EEPROM.read(EEPROM_NODE_COUNT);
    alarm2int = EEPROM.read(EEPROM_ALRM2_INT);
    EEPROM.get(EEPROM_DEVKEY, devicekey);
    EEPROM.get(EEPROM_NODEIDS, NodeIDs);
  
  //--- set IDfails array

    for(byte u = 0; u < numNodes; u++){    // store all radio IDs into IDfails
      IDfails[u] = NodeIDs[u];
    }
  
  //--- Go to main menu for setup
//    Serial.begin(baudRate);
//    delay(100);
    
    MainMenu();

  //--- Time sync in field 

    if(userinput == false){    // if no user input during menu
//      Serial.end();
      delay(50);
//      digitalWrite(LED,HIGH);
      fieldSync();
      digitalWrite(LED,LOW);
    } 

  //--- Set alarms

    readClock();
    startHr = hrs;                        // save hour to set time for getting NIST time update
    
    if(alarm2int == 4){
      NISThr = startHr + 2;
      NISTmin = 8;
    } else {
      NISThr = startHr;
      NISTmin = 8; 
    }
    
    // clear alarm registers
    RTC.setAlarm(ALM1_MATCH_MINUTES,0,0,0,0);   // set alarm values to 0
    RTC.setAlarm(ALM2_MATCH_MINUTES,0,0,0,0);
    RTC.alarm(ALARM_1);                         // turn off alarm 1
    RTC.alarm(ALARM_2);                         // turn off alarm 2
    RTC.alarmInterrupt(ALARM_1, true);          // turn on alarm 1 interrupt 
    RTC.alarmInterrupt(ALARM_2, true);          // turn on alarm 2 interrupt
    RTC.squareWave(SQWAVE_NONE);                // Use SQW pin as interrupt generator
    
    readClock();
   
    setAlarm1();            // set time for Alarm 1
    setAlarm2();            // set time for Alarm 2
    RTC.alarm(ALARM_1);     // turn on Alarm 1
    RTC.alarm(ALARM_2);     // turn on Alarm 2
    delay(50);

}



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
//======================================================================================
//======================================================================================

//-----------------*** Main Loop ***----------------------------------------------------

void loop()
{
    sleepNow();

    if (RTC.alarm(ALARM_1)){
      calcbattV();  
      if(battV <= lowBatt){
        setAlarm1();    // set Alarm 1 to one minute after the "measurement" interval   
        sleepNow();
      } else {
        digitalWrite(LED, HIGH);
        getData();
        saveData();     // save received data to SD card
        resetArrays();
        digitalWrite(LED,LOW);
        setAlarm1();    // set Alarm 1 to one minute after the "measurement" interval    
      } 
    }   // end if alarm 1
    else if (RTC.alarm(ALARM_2)){
      calcbattV(); 
      if(battV <= lowBatt){
        setAlarm2();      
        sleepNow();
      } else { 
        readClock();
        if (hrs == IDhrs && mins == IDmins){
          if (IDsuccess == false){
            receiveIDs();
          } else {}
         }
          else if (hrs == NISThr && mins == NISTmin){ 
           digitalWrite(LED,HIGH); 
           fonaOn();
           delay(3000);
           timeSuccess = NISTtime();  // change to NISTtime when function ready
           fonaOff();
           digitalWrite(LED,LOW);
           delay(1000);
          } else {
            digitalWrite(LED,HIGH);
            sendDataSD();
            digitalWrite(LED,LOW);
          }
        setAlarm2();
      }    
    }   // end if Alarm 2*/
}
//======================================================================================
//======================================================================================
//======================================================================================

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
  Serial.end();     // turn off Serial comm
  driver.sleep();  // put radio to sleep
  analogComp_off(); // turn off analog comparator
  ADC_off();        // turn off ADC
  JTAG_off();       // disable On-Chip Debug system
  SD_off();
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);            //Power down completely on sleep
  sleep_enable();
  setRTCInterrupt();
  sleep_mode();                                  // puts MEGA to sleep

  // after wake-up interrupt
//  digitalWrite(LED,HIGH);
  ADC_on();
  SD_on();
  Serial.begin(baudRate);
//  SPI.begin();
  delay(1000);
//  digitalWrite(LED,LOW);
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
//  pinMode(pin_SD_OFF, OUTPUT);
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

void fieldSync(){
  Serial.println();
  Serial.println("fieldSync(): Syncing time with Nodes...");
 
//  delay(3000);
  
  fonaOn();
  delay(3000);
 
  if (digitalRead(Fona_PS) == HIGH && gStatus == 1){
      timeSuccess = NISTtime();
//      GPRS_off();
      fonaOff();
    } else {
      Serial.println("Could not connect to network");
    }

    if (!manager.init()){                     // turn on radio  
      return;
    }   

  driver.setFrequency(LoRaFREQ);
  driver.setTxPower(TxPower);   
 manager.setRetries(retryNum);
   
    syncNodes();
    delay(5000);
    
    fonaOn();
    delay(3000);

//      GPRS_on();
      if(digitalRead(Fona_PS) == HIGH && gStatus == 1) {
        if(!sendInit()){
//          SPI.begin();
          delay(20);
          dump = SD.open(dumpfile, FILE_WRITE);
          delay(50);
          if(dump){
            dump.println(toSend);
          }
          delay(50);
          dump.close();
        }
        delay(1000);
        } 
//        GPRS_off(); 
      

  uint8_t ok[2];
  uint8_t okLen = 2;
  uint8_t node;
  uint8_t stopNow[1];
      
  stopNow[0] = GatewayID; 
  
  for (byte y = 0; y < numNodes; y++){       
    uint8_t to = NodeIDs[y];
    Serial.println(stopNow[0]);
    manager.sendtoWait(stopNow,1,to);   // send message to Nodes to stop listening and send IDs
  
    if(manager.available()){
      manager.recvfromAckTimeout(ok,&okLen,radioTimeout,&node);   // receive ack from Node
   
    }
  }  
   receiveIDs();
  Serial.println("Done");
  
//  GPRS_on();   

  

  delay(1000);
  fonaOff();
  init1 = false;
 
}

//---------- Sync time with Nodes ----------------------------------

void syncNodes(){
  Serial.println();
  Serial.println("syncNodes(): Querying Nodes with timestamp...");
  String synced="";
  String unsynced="";
  
   uint8_t unsyncedNodes[numNodes];    // keep track of unsynced nodes, synced ones get set to 0

   for(byte f = 0; f < numNodes; f++){
     unsyncedNodes[f] = NodeIDs[f];  
   }
      
  unsigned long twoHours = 7200000;   // 2 hrs in milliseconds
  unsigned long timeToWait = millis() + twoHours;
  unsigned long perNode = timeToWait/numNodes;    // max time to spend per Node
  boolean nodeSynced = false;
  boolean done = false;
  boolean missedSyncs = false;
    
  while (millis() < timeToWait && done == false){
    for (byte y = 0; y < numNodes; y++){    // iterate through all networked Nodes     
      uint8_t to = unsyncedNodes[y];  
      nodeSynced = false;
      Serial.print("Pinging Node ");
      Serial.println(to);
      
      while (millis() < perNode && nodeSynced == false){
   
      readClock();
      String Tupdate = String(yrs) + "/" + String(mnths) + "/" + String(days) + "|" + String(hrs) + ":" + String(mins) + ":" + String(secs); 
      uint8_t len_resp = Tupdate.length()+1;
          char resp1[len_resp];
          uint8_t resp[len_resp];
          Tupdate.toCharArray(resp1, len_resp);
          for (byte y = 0; y <= len_resp; y++) {    // convert timestamp to uint8_t for sending
            resp[y] = resp1[y];
          }
          Serial.println(resp1);
      manager.sendtoWait(resp, len_resp, to);    // send timestamp
        delay(radioTimeout);
        
      if (manager.available()){   // if response received
        uint8_t len = sizeof(buf);
        uint8_t from;
        nodeSynced = true;   
        if (manager.recvfromAckTimeout(buf, &len, radioTimeout, &from)){    // if response received from node (will be Node's radio address)
               for(byte u = 0; u<len;u++){
                    Serial.print(buf[u]);
                    }
                    Serial.println();
    
            Serial.print("Received response from ");
            Serial.println(to);
            unsyncedNodes[y] = 0;
            if (y == numNodes - 1){
              done = true;
            }
          }    // end receive message
       }    // end if manager.available
       else {    // if Node doesn't acknowledge time receipt
             missedSyncs = true;
             nodeSynced = false;
            }
        
      }    // end while millis < perNode  
    }    // end for loop 
  }    // end while millis < timeToWait    

  for(byte i = 0; i< numNodes; i++){    // list out synced and unsynced radio IDs
    if(missedSyncs == true){
      if(unsyncedNodes[i]== 0){  
        synced += NodeIDs[i];
        synced += ' ';
      } else { 
        unsynced += unsyncedNodes[i];
        unsynced += ' ';
      }
    } else {
      synced += NodeIDs[i];
      synced += ' ';  
    }  
  }
    
  byte syncLen = synced.length();
  char syncSend[syncLen];
  synced.toCharArray(syncSend,syncLen);   

  readClock();
  if(missedSyncs == true){    // prepare Hologram message          
    byte unsyncLen = unsynced.length()+1;
    char unsyncSend[unsyncLen];
    unsynced.toCharArray(unsyncSend,unsyncLen);
    delay(50);
    sprintf(toSend,"%s(%d) Gateway awake at %d/%d/%d %d:%d. Synced to Nodes: %s Missed Nodes: %s.",siteID,GatewayID,mnths,days,yrs,hrs,secs,syncSend,unsyncSend);
  } else{
    sprintf(toSend,"%s(%d) Gateway awake at %d/%d/%d %d:%d. Synced to Nodes: %s Missed Nodes: none.",siteID,GatewayID,mnths,days,yrs,hrs,secs,syncSend);
  }
  Serial.println(toSend);

  boolean synced1 = false;
    
  for(byte j = 0; j < numNodes; j++){   // did G sync with at least one N?
    if(unsyncedNodes[j] == 0){
      synced1 = true;
    }
  }

  if(synced1 == true){
    return true;
  } else { 
    return false;
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
    
    char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"cloudsocket.hologram.io\",9999";
    Serial.println(tcpinit); 
    answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",10000); 
//    answer = sendATcommand(tcpinit,"OK",30000,response); 
//    Serial.print("answer = ");
//    Serial.println(answer);
    
//    if (answer == 1){

    char sendtcp[] = "AT+CIPSEND=0,";   
         Serial.println();
     Serial.print(">> ");
    Serial.println(sendtcp);
    answer = sendATcommand(sendtcp,">",10000); 
    
    sprintf(aux_str, "{\"k\":\"%s\",\"d\":\"%s\",\"t\":[\"%s\",\"CROWN\",\"init\"]}%s\r\n\r\n",devicekey,toSend,siteID,ctrlZ);   
    delay(60);
         Serial.println();
     Serial.print(">> ");
    Serial.println(aux_str);
    answer = sendATcommand(aux_str,"OK\r\n\r\n+CIPSEND:0,",10000);

    char closesocket[] = "AT+CIPCLOSE=0";
         Serial.println();
     Serial.print(">> ");
    Serial.println(closesocket);
    answer = sendATcommand(closesocket,"OK/r/n+CIPCLOSE:0,0",5000);
    return true;
  } 
  else {
    Serial.println("ERROR: Failed to connect to Hologram");
    return false;
  }
//  }
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
  yrs = tm.Year + 1970;  //tm.Year --> years since 1970

  delay(50);

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

  if (alarm2int == 4){                          // if upload interval = 4
    if (hrs + 2 == NISThr && mins > IDmins){    // if next alarm should be time to get NIST time
      alarm2Hrs = NISThr%24;
      alarm2Mins = NISTmin;    
    }
    else if (hrs == (NISThr%24)){               // alarm after getting NIST time
      alarm2Hrs = (hrs + 2)%24;
      alarm2Mins = 4;
    }
    else if (hrs <= IDhrs && mins < IDmins && (hrs + alarm2int) > IDhrs){    // next alarm should be at 18:37 to receive and send sensor list
      interrupted = hrs;
      alarm2Hrs = IDhrs;
      alarm2Mins = IDmins;
    }
    else{                                       // alarm to get data from Nodes
      alarm2Hrs = (hrs + alarm2int) % 24;
      alarm2Mins = 4;  
    }
  } else if(alarm2int == 1){                    // if upload interval is every hour
    if(hrs == NISThr && mins < NISTmin){        // if next alarm should be time to get NIST time
      alarm2Hrs = NISThr;
      alarm2Mins = NISTmin;
    } else if (hrs == NISThr && mins >= NISTmin){ // alarm after getting NIST time
      alarm2Hrs = (NISThr%24)+1;
      alarm2Mins = 4;
    }
    else if (hrs <= IDhrs && mins < IDmins && (hrs + alarm2int) > IDhrs){    // next alarm should be at 18:37 to receive and send sensor list
      interrupted = hrs;
      alarm2Hrs = IDhrs;
      alarm2Mins = IDmins;
    }
    else{                                       // alarm to get data from Nodes
    alarm2Hrs = (hrs + alarm2int) % 24;
    alarm2Mins = 4;  
    }
  }
  
  RTC.setAlarm(ALM2_MATCH_HOURS, 0, alarm2Mins, alarm2Hrs, 0); 

  Serial.print("Alarm 2 set for: ");
  Serial.print(alarm2Hrs);
  Serial.print(":");
  Serial.println(alarm2Mins);
  delay(50);

}

//=======================================================================================

//--------------- Calculate Battery Voltage ---------------------------------------------

float calcbattV() {
  analogComp_on();
  digitalWrite(Mbatt, HIGH);  // turn on voltage divider
  delay(50);
  
  int Aout;
  Aout = analogRead(BattV); // analogRead returns an integer between 0 and 1023
  battV = Aout * multiplier * resist;

  digitalWrite(Mbatt, LOW);   // turn off voltage divider
  delay(50);
}




//======================================================================================

//---------- Get data from Nodes -------------------------------------------------------

void getData(){
  Serial.println();
  Serial.println("getData(): Querying Nodes for data...");
  dataSaved = false;
  boolean gotData = false;

  // Step 1 - turn on radio
  
  if (!manager.init()){                     // turn on radio
    Serial.println("radio failed");
  }
  driver.setFrequency(LoRaFREQ);
  driver.setTxPower(TxPower);   
 manager.setRetries(retryNum);
  delay(2000);

  // Step 2 - Send timestamp to Ns one by one     

  numNodes = EEPROM.read(EEPROM_NODE_COUNT);   
  EEPROM.get(EEPROM_NODEIDS, NodeIDs);
  uint8_t to;
  
  for (byte y = 0; y < numNodes; y++){    // iterate through all networked Nodes
    gotData = false;
    to = NodeIDs[y];  
//    Serial.print("Querying Node ");
//    Serial.print(to);
//    Serial.println("...");
    
    readClock();
    String Tupdate = String(yrs) + "/" + String(mnths) + "/" + String(days) + "|" + String(hrs) + ":" + String(mins) + ":" + String(secs); 
    uint8_t len_resp = Tupdate.length()+1;
        char resp1[len_resp];
        uint8_t resp[len_resp];
        Tupdate.toCharArray(resp1, len_resp);
//            Serial.println(resp1);
        for (byte y = 0; y <= len_resp; y++) {           // convert timestamp to uint8_t for sending
          resp[y] = resp1[y];
        }
            
    manager.sendtoWait(resp, len_resp, to);   // send timestamp
    delay(radioTimeout);
    unsigned long startTime = millis();
  
    while(gotData == false && millis() <= startTime + radioTimeout * retryNum) { 
      if (manager.available()){         // if response received
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (manager.recvfromAckTimeout(buf, &len, radioTimeout, &from)){   // if data received from node
//          digitalWrite(LED,HIGH);
            for( byte a = 0; a < len; a++){
              nodeData[y][a] = buf[a];     // store data into 2D char array   
              Serial.print(nodeData[y][a]);
            }
          gotData = true;          
        }
//        digitalWrite(LED,LOW);
      }   // end if manager.available
    }   // end while loop
    
      Serial.println();
      int wait = retryNum*radioTimeout;
      delay(wait);
  }   // end for loop
  
}


//======================================================================================

//---------- Get sensor lists from Nodes -----------------------------------------------

void receiveIDs(){
  Serial.println();
  Serial.println("receiveIDs(): Waiting for IDs...");
//  SPI.begin();
  delay(20);
  IDfile = SD.open(sensorIDfile, FILE_WRITE);   // write received transmissions to SD
  delay(200);

  String sensorIDs = "";
  char aux_str2[FONA_MSG_SIZE];
    
/*  char IPs[2][14] ={"18.206.138.90","34.206.148.15"};    // static IP addresses for Hologram cloud
  char ipaddr[14];                              // holder for one of the two IP addresses
   
  long y = random(0,2);  

  for(byte u = 0; u < 14; u++){
    ipaddr[u]=IPs[y][u];
//    Serial.print(IPs[y][u]);
  }*/
     
  char ctrlZ[2];
       ctrlZ[0] = 0x1A;
       ctrlZ[1] = 0x00;

  memset(aux_str,0,sizeof(aux_str));
  delay(50);

    if (!manager.init()){                     // turn on radio
    Serial.println("radio failed");
    return;
  }
  driver.setFrequency(LoRaFREQ); //John: 07-May-2019, after call to manager.init() the radio frequency and power must be reset to desired value (otherwise defaults to 915 MHz)
  driver.setTxPower(TxPower);   
     manager.setRetries(retryNum);
       
  for (byte y = 0; y < numNodes; y++){   // iterates through networked Nodes

    sensorIDs = "";
    boolean response = false;
    uint8_t buf1[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t IDplease[4] = {1,1,1,1};  
    uint8_t to;
      
    if (IDfails[y] != 0){       // if Node has not sent sensor IDs     
      long idtimeout = millis() + 60000;
      Serial.print("Querying Node "); 
        
      to = IDfails[y];  
      Serial.print(to);
      Serial.println("...");
        
//      manager.sendtoWait(IDplease, 4, to);  // ask for IDs     
//        delay(radioTimeout);
      
      while(millis() < idtimeout && response == false){  
            manager.sendtoWait(IDplease, 4, to);  // ask for IDs     
        delay(radioTimeout);
        
        if (manager.available()){
        uint8_t len = sizeof(buf1);
        uint8_t from;
          response = true;
        if (manager.recvfromAckTimeout(buf1, &len,radioTimeout, &from)){   // if IDs received from node
            for (byte j = 0; j < len; j++){
              if(buf1[j]!=0 && buf1[j] != 13 && buf1[j] != 10){
                sensorIDs += String(char(buf1[j]));
                delay(8);
                Serial.print(buf[j]);
              } 
              IDsuccess = true; 
              
              IDfails[y] = 0;    // remove radio ID from fail list
            } 
           
            timestamp();       // get timestamp and save sensor list
            if(IDfile){
               IDfile.println();
               IDfile.print("Node "); IDfile.print(to); IDfile.print(',');
               IDfile.print(Timestamp); IDfile.println(": ");
               IDfile.print(sensorIDs);
               IDfile.println();
              }
            Serial.println(sensorIDs);

            uint8_t len = sensorIDs.length();
            char sensorIDsArr[len];
            sensorIDs.toCharArray(sensorIDsArr, len);
   
             if (gStatus == 1) {   
//              char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",";   
//              sprintf(aux_str2,"%s\"%s\",9999",tcpinit,ipaddr);  
      
//              Serial.println(aux_str2);
              char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"cloudsocket.hologram.io\",9999";
               Serial.println();
                Serial.print(">> ");  
                Serial.println(tcpinit);
              answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",10000); 
        
              char sendtcp[] = "AT+CIPSEND=0,";   // unknown data string length 
                  Serial.println();
     Serial.print(">> ");
              Serial.println(sendtcp);
              answer = sendATcommand(sendtcp,">",10000);
          
              sprintf(aux_str2, "{\"k\":\"%s\",\"d\":\"Node %d: %s\",\"t\":[\"%s\",\"SENSORS\"]}%s\r\n\r\n",devicekey,to,sensorIDsArr,siteID,ctrlZ);
              delay(60);
                   Serial.println();
     Serial.print(">> ");
              Serial.println(aux_str2);
              answer = sendATcommand(aux_str2,"OK\r\n\r\n+CIPSEND: 0,",10000);
        
              char closesocket[] = "AT+CIPCLOSE=0";
                   Serial.println();
     Serial.print(">> ");
              Serial.println(closesocket);
              answer = sendATcommand(closesocket,"OK\r\n\r\n+CIPCLOSE: 0,",5000);
         
              delay(100);
            }
         
        } 
//        else {    // if no response from Node
//          IDsuccess  = false;  // there's at least one Node that hasn't sent IDs
//          Serial.println("ERROR: missing sensor IDs from at least one Node");
//        }
        }   // end if mananger.available
      }   // end while loop
    }   // end if IDfails 
    if (response == true){
      IDsuccess = true;}
      else { IDsuccess = false;}
    
  } // end for loop

  IDfile.close(); delay(200);

  // set alarm 2 for next send data interval
  Serial.println("Done receiving IDs");
  if (init1 == false){
    RTC.setAlarm(ALM2_MATCH_HOURS, 0, 4, (interrupted + 4)%24, 0); 
  }
   delay(1000);
}

//======================================================================================

//---------- Get time from NIST server, update RTC -------------------------------------

boolean NISTtime(){
  Serial.println();
  Serial.println("NISTtime(): Getting time from NIST...");
  timeSuccess = false;

  char IPs[4][12] ={"129.6.15.28","129.6.15.29","129.6.15.30","129.6.15.27"};
  char ipaddr[12];
   
  long y = random(0,4);  

  for(byte u = 0; u < 12; u++){
    ipaddr[u]=IPs[y][u];
//    Serial.print(IPs[y][u]);
  }

  if(gStatus == 1){

    char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",";
    sprintf(aux_str,"%s\"%s\",13",tcpinit,ipaddr);
//      char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"time.nist.gov\",13";
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
    sendATcommand(gettime,commandTime,10000, response);  // --> WORKS!
    parseResponse(response);

/*    char closetcp[] = "AT+CIPCLOSE=0";
    Serial.println();
     Serial.print(">> ");
    Serial.println(closetcp);
    answer = sendATcommand(closetcp,"OK\r\n\r\n+CIPCLOSE: 0,0",10000);*/
   
  } else { 
    delay(500);
    Serial.println("ERROR: No connection to network. Network time update unsuccessful.");
    timeSuccess = false;
  }
  return timeSuccess;
}

void parseResponse(char response[100]) {
  int i = 0;
  while (response[i] != '-') i++;
  
  if (i >= 100) {
    Serial.println("ERROR: Could not parse response");
    return;
  }
  
  int yr = (10 * (response[i-2] - 48) + (response[i-1] - 48));
  byte mnth = 10 * (response[i+1] - 48) + (response[i+2] - 48);
  byte dy = 10 * (response[i+4] - 48) + (response[i+5] - 48);
  i += 7;
  byte hr = 10 * (response[i] - 48) + (response[i+1] - 48);
  byte mins = 10 * (response[i+3] - 48) + (response[i+4] - 48);
  byte sec = 10 * (response[i+6] - 48) + (response[i+7] - 48);
 
  Serial.print("Timestamp: ");
  Serial.println(String(dy)+"-"+String(mnth)+"-"+String(yr)+" "+String(hr)+":"+String(mins)+":"+String(sec));

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


//======================================================================================

//--------------- Save Data to SD ------------------------------------------------------

void saveData() {
//  SPI.begin();
  Serial.println();
  Serial.println("saveData(): Saving data to SD card...");
  dataSaved = false;
  delay(50);

   myfile = SD.open(datafile, FILE_WRITE);   // write received transmissions to SD
   delay(100);
   dump = SD.open(dumpfile, FILE_WRITE);
   delay(100);

   if (dump){
    for (byte j = 0; j < numNodes; j++){       // write data into SD text file
      myfile.println(nodeData[j]);
      delay(100);
      dump.println(nodeData[j]);
      delay(100);
      Serial.println("----Data----");
      Serial.println(nodeData[j]);
      delay(100);
      Serial.println("----Data----");
    }  
      myfile.close();
      delay(200);
      dump.close();
      delay(200);
      
      dataSaved = true;   
   } else {
      Serial.println("ERROR: Failed to Open SD. Sending Data Now...");
      sendDataArray();  
   }

}

//======================================================================================

//--------------- Set data arrays to zero ----------------------------------------------

void resetArrays(){
  for (byte index = 0; index < numNodes; index++){
    memset(nodeData[index], 0, sizeof(nodeData[index]));
  }    
}

//======================================================================================

//--------------- Send data to Hologram Cloud ----------------------------------------------

//--------------- from SD card

void sendDataSD() {

//  SPI.begin();
  
  unsigned int pvCurrent = getSolarCurrent();
  float pvVoltage = getSolarVoltage();
  unsigned int pvVoltageSend = pvVoltage * 1000; // convert float (V) to int (mV)
  
  char aux_str2[FONA_MSG_SIZE];

 /* char IPs[2][14] ={"18.206.138.90","34.206.148.15"};
  char ipaddr[14];
   
  long y = random(0,2);  

  for(byte u = 0; u < 14; u++){
    ipaddr[u]=IPs[y][u];
//    Serial.print(IPs[y][u]);
  }*/
        
  fonaOn();
  delay(3000);
    
  calcbattV();                            // get Li-ion battery voltage
  int battmV = battV*1000;                // convert V to mV  
    
  char ctrlZ[2];
       ctrlZ[0] = 0x1A;
       ctrlZ[1] = 0x00;
      
  memset(aux_str,0,sizeof(aux_str));
  delay(50);

  if (gStatus == 1) {
    Serial.println("sendDataSD(): Sending data from SD...");
    // Send node data      
      char c[1];
      int pos = 0;
      c[0] = 0;
      int eof = 0;

//      char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",";   
//      sprintf(aux_str,"%s\"%s\",9999",tcpinit,ipaddr);   
//           Serial.println();
//     Serial.print(">> ");  
//      Serial.println(aux_str);
      char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"cloudsocket.hologram.io\",9999";
      Serial.println(tcpinit); 
      answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",10000); 

      if (answer == 1){
      
      char sendtcp[] = "AT+CIPSEND=0,";   // unknown data string length
           Serial.println();
     Serial.print(">> ");
      Serial.println(sendtcp);
      answer = sendATcommand(sendtcp,">",10000); 
      
      timestamp();
      byte len = Timestamp.length()+1;
      char timeSend[len]; 
      Timestamp.toCharArray(timeSend,len); delay(20);
      
      sprintf(aux_str, "{\"k\":\"%s\",\"d\":\"%s~%s~%d~%d~%d\",\"t\":[\"%s\",\"battmV\",\"G PV\"]}%s\r\n\r\n",devicekey,siteID,timeSend,battmV,pvCurrent,pvVoltageSend,siteID,ctrlZ);   // send battery voltage
      delay(60);
           Serial.println();
     Serial.print(">> ");
      Serial.println(aux_str);
      answer = sendATcommand(aux_str,"OK\r\n\r\n+CIPSEND: 0,",5000);
         memset(aux_str,0,sizeof(aux_str));

      char closesocket[] = "AT+CIPCLOSE=0";
           Serial.println();
     Serial.print(">> ");
      Serial.println(closesocket);
      answer = sendATcommand(closesocket,"OK\r\n\r\n+CIPCLOSE: 0,",5000);
    
//    SPI.begin();
    delay(20);
    while (eof != -1) {  
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
      
      while (eof != -1 && c[0] != '\n' && c[0] != 0) {
        dataToSend[ind++] = c[0];
        if ((eof = dump.read(c, 1)) == -1)
          break;
      }
      
      dataToSend[ind-1] = '\0'; // REMOVE CARRAIGE RETURN AT END OF ARRAY
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

        for (byte i = 1; i <= 3; i++){    // try to send data max 3 times
          if (tcpSent == false){  
        
//            char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",";   
//
//            sprintf(aux_str,"%s\"%s\",9999",tcpinit,ipaddr);
//            Serial.println();
//            Serial.print(">> ");
//            Serial.println(aux_str);
            char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"cloudsocket.hologram.io\",9999";
            answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",10000); // needs longer timeout?
                 memset(aux_str,0,sizeof(aux_str));

            if (answer == 1){
            char sendtcp[] = "AT+CIPSEND=0,";   // unknown data string length --> WORKS! COMMA IS NECESSARY
                 Serial.println();
     Serial.print(">> ");
            Serial.println(sendtcp);
            answer = sendATcommand(sendtcp,">",5000);
  
            sprintf(aux_str2, "{\"k\":\"%s\",\"d\":\"%s\",\"t\":[\"%s\",\"DATA\"]}%s\r\n\r\n",devicekey,dataToSend,siteID,ctrlZ);
            delay(60);
                 Serial.println();
     Serial.print(">> ");
            Serial.println(aux_str2);
            answer = sendATcommand(aux_str2,"OK\r\n\r\n+CIPSEND: 0,", 5000);
               memset(aux_str2,0,sizeof(aux_str));
            Serial.print("answer = ");
            Serial.println(answer);

            for (byte y = 0; y < sizeof(successfulSend); y++){
              if(response[y] == successfulSend[y]){
                tcpSent = true;
              } 
            }
            
            Serial.print("tcpSent = ");
            Serial.println(tcpSent);
            
            char closesocket[] = "AT+CIPCLOSE=0"; 
            Serial.println();
            Serial.print(">> ");
            Serial.println(closesocket);
            answer = sendATcommand(closesocket,"OK\r\n\r\n+CIPCLOSE: 0,",5000);
            } else {
              Serial.println("ERROR: Failed to connect to Hologram");
            }
          }  // end if tcpSent == false
          
          else { break; }
            
        } // end for loop that tries to send 3 times
      }   // end if dataToSend[0] != 0
            
      c[0] = 0;
      dump.close();
      delay(100);
    }  // end while !eof loop
      
      delay(100);
      if (SD.remove(dumpfile))
        Serial.println("File Cleared");
      else
        Serial.println("File Clear Failed");
      delay(50);
   } else {
        Serial.println("Failed to connect to Hologram");
      }
  } // end if gStatus == 1
   
  delay(3000); 
  fonaOff();

  Serial.println("Done sendData");
  
}

//--------------- from array if SD fails

void sendDataArray() {

  unsigned int pvCurrent = getSolarCurrent();
  float pvVoltage = getSolarVoltage();
  unsigned int pvVoltageSend = pvVoltage * 1000; // convert float (V) to int (mV)
  
  char aux_str2[FONA_MSG_SIZE];
  
 /* char IPs[2][14] ={"18.206.138.90","34.206.148.15"};
  char ipaddr[14];
   
  long y = random(0,4);  

  for(byte u = 0; u < 14; u++){
    ipaddr[u]=IPs[y][u];
//    Serial.print(IPs[y][u]);
  }*/
        
  fonaOn();
  delay(3000);
        
  calcbattV();                            // get Li-ion battery voltage
  int battmV = battV*1000;                // convert V to mV  
    
  char ctrlZ[2];
       ctrlZ[0] = 0x1A;
       ctrlZ[1] = 0x00;
      
  memset(aux_str,0,sizeof(aux_str));
  delay(50);

  if (gStatus == 1) {
    Serial.println("sendDataArray(): Sending data from array...");
    // Send node data          
//     char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",";     
//     sprintf(aux_str,"%s\"%s\",9999",tcpinit,ipaddr);
//     Serial.println();
//     Serial.print(">> ");
//     Serial.println(aux_str);

    char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"cloudsocket.hologram.io\",9999";
     answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",20000); // needs longer timeout?
        memset(aux_str,0,sizeof(aux_str));

          if (answer == 1){

     char sendtcp[] = "AT+CIPSEND=0,";   // unknown data string length --> WORKS! COMMA IS 
          Serial.println();
     Serial.print(">> ");
     Serial.println(sendtcp);
     answer = sendATcommand(sendtcp,">",20000); 
      
      timestamp();
      byte len = Timestamp.length()+1;
      char timeSend[len]; 
      Timestamp.toCharArray(timeSend,len); delay(20);
      
      sprintf(aux_str, "{\"k\":\"%s\",\"d\":\"%s~%s~%d~%d~%d\",\"t\":[\"%s\",\"battmV\",\"G PV\"]}%s\r\n\r\n",devicekey,siteID,timeSend,battmV,pvCurrent,pvVoltageSend,siteID,ctrlZ);   // send battery voltage
      delay(60);
          Serial.println();
     Serial.print(">> ");
     Serial.println(aux_str);
     answer = sendATcommand(aux_str,"OK\r\n\r\n+CIPSEND: 0,",20000);

     char closesocket[] = "AT+CIPCLOSE=0";
          Serial.println();
     Serial.print(">> ");
     Serial.println(closesocket);
     answer = sendATcommand(closesocket,"OK\r\n\r\n+CIPCLOSE: 0,",10000);

     for (byte index = 0; index < numNodes; index++){  
               
      // Do not send anything if blank
        boolean tcpSent = false;
        char successfulSend[] = "\r\nOK\r\n\r\n+CIPSEND: 0,";
        
        if (nodeData[index][0] != 0) {
  
          for (byte i = 1; i <= 3; i++){    // try to send data max 3 times
            if (tcpSent == false){  
              Serial.print("Sending to Hologram attempt ");
              Serial.println(i);
              
//              char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",";   
//              sprintf(aux_str,"%s\"%s\",9999",tcpinit,ipaddr);
              
              char tcpinit[] = "AT+CIPOPEN=0,\"TCP\",\"cloudsocket.hologram.io\",9999";
              answer = sendATcommand(tcpinit,"OK\r\n\r\n+CIPOPEN: 0,0",30000); // needs longer timeout?
              Serial.println();
              Serial.print(">> ");
              Serial.println(aux_str);
               memset(aux_str,0,sizeof(aux_str));
    
              char sendtcp[] = "AT+CIPSEND=0,";   // unknown data string length --> WORKS! COMMA IS NECESSARY
              Serial.println();
              Serial.print(">> ");
              Serial.println(sendtcp);
              answer = sendATcommand(sendtcp,">",20000);
    
              sprintf(aux_str2, "{\"k\":\"%s\",\"d\":\"%s\",\"t\":[\"%s\",\"DATA\"]}%s\r\n\r\n",devicekey,nodeData[index],siteID,ctrlZ);
              delay(60);
              Serial.println();
              Serial.print(">> ");
              Serial.println(aux_str2);
              answer = sendATcommand(aux_str2,"OK\r\n\r\n+CIPSEND: 0,", 20000);
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
              answer = sendATcommand(closesocket,"OK\r\n\r\n+CIPCLOSE: 0,",10000);
              
            }  // end if tcpSent == false
            
            else { break; }
              
          } // end for loop that tries to send 3 times

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
  unsigned long pTimeout = millis() + 10000;
  
  while(digitalRead(Fona_PS) == LOW && millis() < pTimeout){   // SHOULD BE IF?
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

  delay(3000);
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

  toggle();
//  } else{}

}

//======================================================================================

//--------------- Establish GPRS connection --------------------------------------------

boolean startGPRS(boolean onoff){
      
 uint8_t n = fona.getNetworkStatus();
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
//======================================================================================
//======================================================================================

//--------------- Main Menu ------------------------------------------------------------

void MainMenu()
{
  if (Serial.available() > 0){
    Serial.read();       // clear serial input buffer
  }
  
  numNodes = EEPROM.read(EEPROM_NODE_COUNT);                        // read variables stored in memory
  GatewayID = EEPROM.read(EEPROM_GATEWAYID);
  alarm2int = EEPROM.read(EEPROM_ALRM2_INT);
  EEPROM.get(EEPROM_DEVKEY, devicekey);
  EEPROM.get(EEPROM_SITEID, siteID);
  siteID[3] = 0;
  
  EEPROM.get(EEPROM_NODEIDS, NodeIDs);
  interval = EEPROM.read(EEPROM_ALRM1_INT);

  delay(30);

  calcbattV();

  Serial.println();
  Serial.println(F("Soil Water Data Network - Cellular Gateway"));
  Serial.println();
  
  Serial.print(F("Site ID: "));
  Serial.println(siteID);
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
  Serial.println(alarm2int);

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
    Serial.println("WARNING: BATTERY VOLTAGE TOO LOW!! PLEASE CHARGE!!");
  }
    
  //delay(10);
  Serial.println();

  if (firsttime == true){
    char quickinput;
    Serial.println(F("Would you like to enter Quick Setup Mode?"));
    Serial.print(F("Enter \"y\" for yes or \"n\" for no: "));
 
  timeout = millis() + 30000;                         // wait 30 secs for input
  
    while (millis() < timeout)
    {
      if (Serial.available() > 0)                        // if something typed, go to menu
      {
        userinput = true;
        quickinput = Serial.read();               // get user input
        Serial.println(quickinput);           
        if (quickinput == 'y'){
                 quickStart();
            }
        else {
        firsttime = false; 
        Serial.println();
        break;}
      }
    }
  } 
  
  Serial.println();
  Serial.println(F("Menu options "));
  
  Serial.println(F("   c  <--  Set clock to NIST time"));    // set clock to NIST time  
  Serial.println(F("   g  <--  Enter Gateway site and radio IDs"));
  Serial.println(F("   n  <--  Enter Node radio IDs")); 
  Serial.println(F("   u  <--  Set data upload to cloud interval"));
  Serial.println(F("   m  <--  Set measurement interval"));
  Serial.println(F("   S  <--  Synchronize Gateway & Node clocks"));
  Serial.println(F("   f  <--  See list of saved files"));
  Serial.println(F("   s  <--  See sensor list"));
  Serial.println(F("   p  <--  Print node data to screen"));
  Serial.println(F("   e  <--  Erase microSD card"));
  Serial.println(F("   x  <--  Exit menu"));             // exit
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

    case 99: case 67:          // ------ c - Set clock ---------------------------------------------

      NISTsync();
   
      MainMenu();
      break;

     case 103: case 71:         // ---------- g - Enter Gateway site and radio IDs ---------------------------------------------------

      gatewayIDs();

      MainMenu();
      break;
        
     case 110: case 78:         // ---------- n - Enter Node radio IDs ---------------------------------------------------       
      
      nodeIDs();

      MainMenu();                                      // go back to menu
      break;    

    case 117:   // ------ u - Set upload interval ---------------------------------------------
    
      uploadInt();
      MainMenu();
      break;
      
    case 109: case 77:        //--------- m - Set measurement interval----------------------------

      if(!measureInt()){
        Serial.println(F("ERROR: upload interval must be entered first. Returning to menu..."));
      }
      MainMenu();
      break;

    case 83:    // ------ S -Synchronize G & N clocks ---------------------------------------------
      Serial.println(F("Make sure same Node menu option open in another Serial Monitor window"));
      Serial.println(F("Sending time to Node..."));
      syncTime();
      delay(500);
      MainMenu();
      break;
      
    
    case 102: case 70:          // ------ f - See list of saved files ---------------------------------------------
      Serial.println(F("Saved files: "));     // list all files on SD card
      delay(10);
    
      if (!SD.begin(SD_CS)) {}

      root = SD.open("/");

      printDirectory(root, 0);
      Serial.println();

      MainMenu();
      break;

    case 115:         // ------ s - See list of sensors ---------------------------------------------

      Serial.println(F("Sensors: "));
//      SPI.begin();
      delay(20);
      
      if (!SD.begin(SD_CS)){}
      IDfile = SD.open(sensorIDfile);
      
  //    printAll(IDfile, 0);
      delay(50);
        if(IDfile){
          while(IDfile.available()){
            Serial.write(IDfile.read());
          }
        }
        IDfile.close();
      Serial.println();
      MainMenu();
      break;

    case 112: case 80:          // ------ p - Print node data to screen ---------------------------------------------

      Serial.println(F("Print data files: "));

//      SPI.begin();
      delay(20);
      if (!SD.begin(SD_CS)) {}
//      root = SD.open("/");
//
//      printAll(root, 0);
//      delay(50);
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

      Serial.println(F("Erase SD card: "));

//      SPI.begin();
      delay(20);
      if (!SD.begin(SD_CS)) {}

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
      break;                                       // exit switch(case) routine

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

//-------------- Quick Start Mode --------------------------------

void quickStart(){
  Serial.println();

    //--- Step 1: Enter Gateway IDs ---
      
      gatewayIDs();
      Serial.println();

    //--- Step 2: Enter Node info ---

      nodeIDs();
      Serial.println();

    //--- Step 3: Set upload interval ---
  
      uploadInt();
      Serial.println();
      
    //--- Step 4: Set measurement interval ---
       
      measureInt();
      Serial.println();
      
   
   //--- Step 5: set clock to NIST time --- 
   
      NISTsync();
      Serial.println();
      
  firsttime = false;
  Serial.println(F("Quick setup complete, going to Main Menu..."));
  Serial.println();
  Serial.println();
  delay(200);
    
}

void gatewayIDs(){
   Serial.print(F("Enter 3-digit site ID (ex. ABC): "));    // set siteID
      charinput();
      
        for (byte i = 0; i < 4; i++){
          siteID[i] = charInput[i];
        }
      
      datafile[0] = siteID[0];
      datafile[1] = siteID[1];
      datafile[2] = siteID[2];
      
       EEPROM.put(EEPROM_SITEID, siteID);  
        delay(10);
     Serial.print(F("Enter Gateway radio ID: "));
       getinput();
       GatewayID = indata;
       manager.setThisAddress(GatewayID);
        delay(10);
       EEPROM.update(EEPROM_GATEWAYID, GatewayID);
        delay(10);

        Serial.print(F("Enter the Hologram device key for this Gateway: "));
        charinput();

        for (byte i = 0; i < 8; i++){
          devicekey[i] = charInput[i];
        }
        devicekey[8] = 0;
        EEPROM.put(EEPROM_DEVKEY, devicekey);
          delay(10);
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

void uploadInt(){
  Serial.println(F("Set upload interval to 1 or 4 hours."));
  Serial.print(F("Enter \"1\" or \"4\": "));
  byteInput();
    alarm2int = u_indata;
//    Serial.println(alarm2int,DEC);
    long utimeout = millis() + 10000;
  
  while((alarm2int != 1 && alarm2int != 4) && millis() < utimeout){
    Serial.println(F("Invalid interval. Please enter 1 or 4."));
    byteInput();
  }

  EEPROM.update(EEPROM_ALRM2_INT, alarm2int);
  delay(20);
}

boolean measureInt(){

  if(alarm2int == 4){
   Serial.print(F("Enter measurement interval (15, 20, 30, or 60 mins): "));
      Serial.flush();
      boolean intSet = false;
      long utimeout = millis() + 10000;
      while(intSet == false && millis() < utimeout){        
        getinput();
        if (indata != 15 && indata != 20 && indata != 30 && indata != 60) {
          Serial.print(F("Invalid interval. Enter measurement interval (15, 20, 30, or 60): "));
          Serial.flush();
        }
  
        else if(indata == 15 && numNodes > 2){
          Serial.println(F("ERROR: maximum 2 Nodes at 15 minute interval"));
          Serial.println(F("Please enter another measurement interval: "));
        } else if (indata == 20 && numNodes > 4){
          Serial.println(F("ERROR: maximum 4 Nodes at 20 minute interval"));
          Serial.println(F("Please enter another measurement interval: "));
        } 

        else {
          intSet = true;
          interval = indata;
          EEPROM.update(EEPROM_ALRM1_INT, interval);
          delay(20);
          return true;
        }   
     }
  } else if (alarm2int == 1){
    Serial.print(F("Enter measurement interval (15, 20, 30, or 60 mins): "));
      Serial.flush();
      boolean intSet = false;
      while(intSet == false){        
        getinput();
        if (indata != 15 && indata != 20 && indata != 30 && indata != 60) {
          Serial.print(F("Invalid interval. Enter measurement interval (15, 20, 30, or 60): "));
          Serial.flush();
        }
        else if(indata == 10 && numNodes > 3){
          Serial.println(F("ERROR: maximum 3 Nodes at 10 minute interval"));
          Serial.println(F("Please enter another measurement interval: "));
        } else if(indata == 15 && numNodes > 8){
          Serial.println(F("ERROR: maximum 8 Nodes at 15 minute interval"));
          Serial.println(F("Please enter another measurement interval: "));
        } 
        else {
          intSet = true;
          interval = indata;
          EEPROM.update(EEPROM_ALRM1_INT, interval);
          return true;
        }   
     }
  } 
  else {  // alarm2int != 1 or 4 
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
      } else {
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

//--------------- Sync G & N clocks ----------------------------------------------------

void syncTime(){
   if (!manager.init()) {                      // initialize radio
    Serial.println("Radio failed");
  }
    driver.setFrequency(LoRaFREQ);
  driver.setTxPower(TxPower);   
 manager.setRetries(retryNum);
 
  uint8_t buf[4];     // array to receive time request from Node
  uint8_t len = sizeof(buf);
  uint8_t from;
  boolean timeSynced = false;
//  delay(5000);

  long timeout = millis() + 45000;

  while(millis() < timeout && timeSynced == false){
//    Serial.println(timeSynced);
   if(manager.recvfromAckTimeout(buf, &len, radioTimeout, &from)){  // if time request received
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
  }
  if (timeSynced ==  true){
    Serial.print(F("Node "));
    Serial.print(from);
    Serial.println(F(" succesfully synced to Gateway"));
  } else {
    Serial.println("Synchronization failed. Return to menu to try again.");
  }
  
      
}
