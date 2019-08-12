/* Version 15
   Differences from v14:
    - Fixes bugs in fieldSync()
    
    
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

    Info stored on EEPROM:
    NOTE: ALL ADDRESSES BELOW SHIFTED BY DEFINED "EEPROMSHIFT"
     0:
     1:  radioID
     2:  GatewayID
     3:  siteID (six bytes, locations 3-7)
     4:  
     5:  
     6: 
     7:
     8:  interval
     9:  
     10: 
     11: 
     12: 
     13: 
     14: gatewayPresent    
     15: 
     16: RHPresent    (may not need)
     17:      

   Written by:
   Alondra Thompson, USDA-ARS Sustainable Agricultural Systems Lab
   Justin Ayres, Univeristy of Maryland Computer Science Department
   
   Last edited: May 16, 2019
 
*/

//===================================================================================================

//------------ Libraries --------------------------------------

#include "AcclimaSDI12.h"
#include <SPI.h>                                      // SPI functions for Flash chip
#include <Wire.h>                                     // I2C functions for RTC
#include <EEPROM.h>                                   // built-in EEPROM routines
#include <avr/sleep.h>                                // sleep functions
#include <DS3232RTC.h>    
#include <SPIFlash_MX25R6435F.h>                      // edited SPIFlash library to work with 64Mbit chip
//#include "Adafruit_SHT31.h"                           // for SHT31-D relative humidity sensor
#include <RH_RF95.h>                                  // Moteino LoRa library
#include <RHReliableDatagram.h>                       // allows acknowledgements and retries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>                                  // controls watchdog timer (we want to turn it off)
#include <stdio.h>
#include "SetSpeed.h"

//------------- Assign Pins --------------------------------------

#define SDI12Data 10                                 // Pin for SDI-12 sensors; handles interrupts
#define SDI12Pwr 0                                   // Switches power to SDI-12 sensors on/off
#define LED 15                                       // LED on D15
#define BattV A0                                     // A0 for reading Vout (to calculate battV)
#define Mbatt 14                                    // Switches voltage divider on/off
#define Flash_SS 23
#define baudRate 57600                               //John, 27-Mar-2019: Changed from 115200 to 57600 because when MCU at 8 MHz the sampling resolution is reduced
#define pin_SDIv12 1                                 //pin 41 --> D1  -- switch SDI-12 power supply to 12 volts (vs. 7.5)
#define pin_solarVoltage    A5
#define ADC_REF_VOLTAGE     3.3
#define ADC_RESOLUTION      1024
#define pin_solarShort      A4     
#define SOLAR_CALIB      1.0          //This will become a EEPROM constant that is set during factory config – for now just use 1.0
//#define LoRaFREQ      915           //For US-based LoRa
//#define LoRaFREQ      433             //For Region 1 LoRa (Jordan)

//------------- Declare Variables ---------------------------------


//-----*** Identifiers ***-----
 
  char  siteID[6];                                     // char array for five-digit site ID
  uint8_t  radioID;                                    // node radio ID
  uint8_t  GatewayID;                                  // Gateway radio ID


//-----*** for EEPROM ***-----
#define EEPROMSHIFT                 2800
#define EEPROM_RADIOID              (1 + EEPROMSHIFT)         // node radio id                                      (was radioMem)
#define EEPROM_GATEWAYID            (2 + EEPROMSHIFT)         // storage location for gatewayID                     (was gatewayMem)
#define EEPROM_SITEID               (3 + EEPROMSHIFT)         // first storage location for siteID array            (was siteIDMem)
#define EEPROM_ALRM1_INT            (9 + EEPROMSHIFT)         // storage location for Alarm 1 interval              (was intervalMem)
#define EEPROM_GW_PRESENT           (14 + EEPROMSHIFT)        // store "1" for yes, "0" for node                    (was gatewayPresMem)
#define EEPROM_RH_PRESENT           (16 + EEPROMSHIFT)        //                                                    (was RHPresMem)

#define EEPROM_SERIALNUM            10
#define EEPROM_OPTSRADIO            17

// byte  radioMem = 1;
// byte  gatewayMem = 2;
// byte  siteIDMem = 3;
// byte  intervalMem = 9;
// byte  gatewayPresMem = 14;     // store "1" for yes, "0" for no
// byte  RHPresMem = 16;

//The following is a field/variable, NOT a EEPROM location
byte  gatewayPresent = 0;

//The following value is determined by reading EEPROM constant (see method getLoRaFreq())
uint16_t LoRaFREQ;                            

  

//-----*** for Flash ***-----

  uint32_t addr;                                                  // address counter for writing to Flash  
  uint32_t startAddr = 0x003000;                                  // address to put first byte of data string
  uint32_t *ptr_startAddr = &startAddr;                           // pointer to location of startAddr  
  uint32_t firstAddrInNextSector = 0x004000;                      // variable to increment sector boundary
  uint32_t *ptr_firstAddrInNextSector = &firstAddrInNextSector;   // pointer to lastAddrInSector location
  
  const uint32_t sectorLength = 4096;
  const uint32_t maxAddr = 0x7FFFFF;
  const uint32_t firstDataAddr = 0x003000;
  
  const uint32_t FlashHeader_Sector0 = 0x000000;
  const uint32_t FlashHeader_Sector1 = 0x001000;
  const uint32_t FlashHeader_Sector2 = 0x002000;
  
  const uint32_t startAddrLoc = FlashHeader_Sector2;
  const uint32_t firstAddrLoc = FlashHeader_Sector2 + 4;
  
  uint32_t IDaddr = 0x001000; 
  uint32_t lastIDaddr;        
    

//-----*** for Main Menu ***-----

int   indata;                                       // user input data
int   incoming[7];
char  incomingChar[20];
char  charInput[20];
boolean firsttime = true;


//-----*** Data Variables ***-----

  //-- SDI-12 addresses and metadata

  char oldAddress = '!';
  char Tsensor;
  char RHsensor;
  char CS655addr;
  char activeSDI12[9];                                // array holding active sensor addresses (up to 8 sensors)
  byte registerSDI12[8] = {                          // holds addresses of active SDI12 addresses
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

  //-- Sensor data
  
  String SDI12data = "";                              // String for compiling data from SDI12 sensors
  String allData = "";  
  int measDelay;                                      // delay for SDI-12 measurement 
  boolean SDI12response = false;
  byte RHPresent = 0;
  byte TPresent = 0;
  char sep = '~';                                     // data delimiter
  
  //-- Battery voltage measurement and calc
  
  float multiplier = 0.00322;                         // scaling factor for analogRead
  float resist = 1.333;                               // for voltage divider: R1=10k, R2=30k, resist =  (R1+R2)/R2
  float battV;                                        // battery voltage
  float lowBatt = 3.4;                                // low battery limit
  

//-----*** for Loop ***-----

  boolean dataSent = false;
  boolean updated = false;
  boolean userinput = false;
  boolean IDsuccess = false;  // for sending sensor IDs


//-----*** for RTC ***-----

  //-- Time and Date values
  
  byte   secs;                  // time and date values
  byte   mins;
  byte   hrs;
  byte   days;
  byte   mnths;
  int    yrs;
  char   TMZ[]="UTC";           // Time Zone
  
  String timestamp = "";
  tmElements_t tm;
    
  //-- Alarms
    
  byte  interval;              // user-defined measurement interval 
  byte  alarmMins;
  byte  IDmins = 36;           // wake up at 12:36 CST (18:36 UTC) to send sensor IDs
  byte  IDhrs = 18;    
  boolean timeUpdated = false; 

  //-- Temperature sensor
  
  float  boxTemp;


//-----*** LoRa Radio Settings ***-----

  byte     TxPower = 20;
  unsigned int  radioTimeout = RH_DEFAULT_TIMEOUT;   // changed from 300 ms to 200ms(default)
  uint8_t  retryNum = 10; //RH_DEFAULT_RETRIES;            // changed from 20 to 3 (default) 


// ------- Initialize ----------------------------------------------------

//Adafruit_SHT31 sht31 = Adafruit_SHT31();           // Humidity sensor

RH_RF95 driver;                                    // Initiate LoRa transceiver
RHReliableDatagram manager(driver, radioID);

SDI12 SDI12port(SDI12Data);  
SPIFlash flash(Flash_SS, 0xC228);                  // C228 for Macronix 64 Mbit MX25R6435F Flash chip
  SetSpeed ss; 
//===================================================================================================

//------------- Set Up ------------------------------------------------------------------------------

void setup() {
   Serial.begin(baudRate);
    delay(100);
    Serial.println("Hello");
  //--- Initialize

   // I2C
    SPI.begin();
    Wire.begin();                                   // begin I2C
    delay(300);

    // Flash
    
    if(!flash.initialize()){
//    Serial.println(F("Flash failed to initialize"));
    } else {
      flash.sleep();
    } 

    // RTC
    
    RTC.begin();
   
  //--- Pin Settings
  
//    pinMode(hardSS,OUTPUT);
    pinMode(BattV, INPUT);
    pinMode(Mbatt, OUTPUT);
      digitalWrite(Mbatt, LOW);  
    pinMode(SDI12Pwr, OUTPUT);
      digitalWrite(SDI12Pwr,HIGH);
    pinMode(LED, OUTPUT);      
      digitalWrite(LED, HIGH);
    pinMode(pin_SDIv12, OUTPUT);
      digitalWrite(pin_SDIv12, HIGH);  // set SDI-12 power supply to 12V (when to set to 7.5v?)
    pinMode(pin_solarVoltage, INPUT);
    pinMode(pin_solarShort, OUTPUT);
    

  //--- Power Saving

   wdt_disable();                        // turn off watchdog timer 
    RTC_osc_off();                         // Turn off 32kHz output   

    
  //--- Read variables from EEPROM

    radioID = EEPROM.read(EEPROM_RADIOID);                        // read board ID number from EEPROM
    manager.setThisAddress(radioID);
   
    EEPROM.get(EEPROM_SITEID, siteID);
      delay(20);
    interval = EEPROM.read(EEPROM_ALRM1_INT);
    GatewayID = EEPROM.read(EEPROM_GATEWAYID);
      delay(20);  
    gatewayPresent = EEPROM.read(EEPROM_GW_PRESENT);
      delay(20);
//    RHPresent = EEPROM.read(EEPROM_RH_PRESENT);
//    delay(20);  


    LoRaFREQ = getLoRaFreq();   //John


  //--- read in address counters from Flash    

    recallAddrPtrs();  
//    delay(100);

  //--- Radio settings
    manager.init();
//    delay(100);
//    Serial.println("radio initialized");
//    delay(100);
    manager.setRetries(retryNum);
    driver.setFrequency(LoRaFREQ);
    driver.setTxPower(TxPower);                      // transmission power (dB)

  //--- Scan SDI-12 Addresses 
  
//    Serial.begin(baudRate);
//    delay(100);
  
//    digitalWrite(SDI12Pwr,HIGH);                    // turn on SDI12 sensors
    SDI12port.begin();
    delay(100);
  
    
    Serial.println(F("Scanning for SDI-12 sensors...")); //digitalWrite(LED, HIGH); 
    Serial.println();
  
    for (int index = 0; index < 9; index++) activeSDI12[index] = '\0'; delay(50);
  
    for (byte i = '0'; i <= '9'; i++) {
//      Serial.print("Scanning address ");
//      Serial.println((char) i); 
      if (checkActiveSDI12(i)) setActiveSDI12(i); // scan address space 0-9
    }
    for (byte i = 'a'; i <= 'z'; i++) if (checkActiveSDI12(i)) setActiveSDI12(i); // scan address space a-z
  
    for (byte i = 'A'; i <= 'Z'; i++) if (checkActiveSDI12(i)) setActiveSDI12(i); // scan address space A-Z
    
    delay(20);
    // scan address space 0-9
    for (char i = '0'; i <= '9'; i++){ if (isActiveSDI12(i)) {
      
        compileInfoSDI12(i);  
      }}
        
    // scan address space a-z
    for (char i = 'a'; i <= 'z'; i++){ if (isActiveSDI12(i)) {
        compileInfoSDI12(i);  
      }}
        
    // scan address space A-Z
    for (char i = 'A'; i <= 'Z'; i++){ if (isActiveSDI12(i)) {
        compileInfoSDI12(i); 
      }}
        
    saveSDI12Info();  

    delay(50);

//    digitalWrite(SDI12Pwr, LOW);  // turn off SDI12 sensors

    Serial.println();
    Serial.println();
    Serial.println(F("Done"));
    delay(50);
    SDI12port.end();

     //--- Start continuous measurements for T and RH

  if(TPresent == 1 || RHPresent == 1){
  
    SDI12port.begin();
    delay(300);
    
    
    String Tstart = "";       // start continuous T measurements on T sensor
    Tstart += Tsensor;
    Tstart += "C1!";
    SDI12port.sendCommand(Tstart);
      delay(300);
    while(SDI12port.available()) SDI12port.read();
    delay(50);

    
    String Rstart = "";       // start continuous RH measurements on T/RH sensor
    Rstart += RHsensor;
    Rstart += "R4!";
    SDI12port.sendCommand(Rstart);
      delay(300);
    while(SDI12port.available()) SDI12port.read();
    delay(100);

   
    String RTstart = "";        // start continuous T measurements on T/RH sensor
    RTstart += RHsensor;
    RTstart += "C1!";
    Serial.println(RTstart);
    SDI12port.sendCommand(RTstart);
      delay(300);
    while(SDI12port.available()) Serial.write(SDI12port.read());
    delay(100);
    SDI12port.clearBuffer();
    
 
    
    SDI12port.end(); 
    delay(50);
  }

  //--- Go to menu 

    menu();                                          // display menu on startup


  //----- set alarms -----  
  
  // clear alarm registers
    RTC.setAlarm(ALM1_MATCH_MINUTES,0,0,0,0);  // set alarm values to 0
    RTC.setAlarm(ALM2_MATCH_MINUTES,0,0,0,0);
    RTC.alarm(ALARM_1);                    // turn off alarm 1
    RTC.alarm(ALARM_2);                    // turn off alarm 2
    RTC.alarmInterrupt(ALARM_1, true);     // turn on alarm 1 interrupt 
    RTC.alarmInterrupt(ALARM_2, true);     // turn of alarm 2 interrupt 
    RTC.squareWave(SQWAVE_NONE);           // Use SQW pin as interrupt generator
    
    

   //--- Time sync in field 

    if(userinput == false && gatewayPresent == 1){
//      Serial.end();
      delay(50);
//      digitalWrite(LED,HIGH);
      fieldSync();
      digitalWrite(LED,LOW);
    } 

   //--- Set alarm times 
    
    readClock();
    RTC.alarm(ALARM_1);   
    resetAlarm(interval); 


  if(gatewayPresent == 1){
    if(IDsuccess == true){
//      Serial.println("Done sending IDs");
      RTC.alarmInterrupt(ALARM_2, false);    // turn off alarm 2 interrupt 
    } else {
      RTC.alarm(ALARM_2); // turn on alarm 2
      setAlarm2(1);  // try again tomorrow
    }
  }
 
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


//===================================================================================================
//===================================================================================================
//===================================================================================================

//-------------*** Main Loop ***---------------------------------------------------------------------

void loop() {

  sleepNow();

  if(RTC.alarm(ALARM_1)){    // if alarm 1 goes off
     readClock();
     calcbattV();
     
     if (battV <= lowBatt){        // if battery low skip measurements, go to sleep, wake up again at next interval
      resetAlarm(interval);
      sleepNow();
     } 
     
     // scenario 1: No RH sensor, no Gateway (Bare Node w/o G)
     
     else if (RHPresent == 0 && gatewayPresent == 0){
      Serial.println("RHPresent == 0 && gatewayPresent == 0");     
       if(mins % interval == 0){
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
     
     else if (RHPresent == 1 && gatewayPresent == 0){     
      Serial.println("RHPresent == 1 && gatewayPresent == 0"); 
      Serial.println("heaterON");
       if((mins + 6) % interval == 0){
          heaterON();
          resetAlarm(3);
       }
       else if((mins + 3) % interval == 0){
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

     else if(RHPresent == 1 && gatewayPresent == 1){ 
      Serial.println("RHPresent == 1 && gatewayPresent == 1");                           
       if(mins % interval == 1 ){
        Serial.println("Sending to Gateway");
          digitalWrite(LED, HIGH);
          listenRespond();
          byte n = interval - 7;
          resetAlarm(n);
          digitalWrite(LED, LOW);   
       }
       if((mins + 6) % interval == 0){
        Serial.println("heaterON");
          heaterON();
          resetAlarm(3);
       }
       else if((mins + 3) % interval == 0){
        Serial.println("heaterOFF");
          heaterOFF();
          resetAlarm(3);
       }
       else if (mins % interval == 0){       // mins % interval == 0
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
      Serial.println("RHPresent == 0 && gatewayPresent == 1"); 
       if(mins % interval == 1){
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

  else if (RTC.alarm(ALARM_2)){ 
    Serial.println("ALARM 2");
    readClock();

    calcbattV();
    if (battV <= lowBatt){        // if battery low skip loop, go to sleep
      setAlarm2(1);
      sleepNow();
    } else {
      if(IDsuccess == false){           
        sendIDs();    // if successful will turn off alarm 2, else will try again same time next day
      } else {}
    }
  }
 
}
//===================================================================================================
//===================================================================================================
//===================================================================================================

//===================================================================================================

//--------- Find Active SDI-12 Sensor Addresses -----------------------------------------------------

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
  char sensorID[50];
   byte x = 0; 
  
  for (byte j = 0; j < buf_length; j++){   // save sensor ID to buffer array and print
    char c = SDI12port.read();
        sensorIDs += c;
        delay(8);
        sensorID[x] = c;
        delay(8);
        x++;
  }

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

  if(findSensorType(sensorID,RHsens)){
//    Serial.print("Thum at address ");
//    Serial.println((char) i);
    RHPresent = 1;
    RHsensor = (char) i;
    String Rcommand = "";
    Rcommand += RHsensor;
    Rcommand += "XHP60!";                 // set polling rate for RH sensor to 60 seconds
//    Serial.println(Rcommand);
    SDI12port.sendCommand(Rcommand);
    delay(300);
    while(SDI12port.available()){ Serial.write(SDI12port.read());}
    
    String Tcommand = "";
    Tcommand += RHsensor;
    Tcommand += "XTP60!";                 // set polling rate for T sensor included w RH sensor to 60 seconds
    SDI12port.sendCommand(Tcommand);
    delay(300);
    while(SDI12port.available()){ Serial.write(SDI12port.read());}
  } 
  else if(findSensorType(sensorID, Tsens)){
//    Serial.print("Temp at address ");
//    Serial.println((char) i);
    Tsensor = (char) i;
    String Tcommand = "";
    Tcommand += Tsensor;
    Tcommand += "XTP60!";
//    Serial.println(Tcommand);
    SDI12port.sendCommand(Tcommand);
    delay(300);
    while(SDI12port.available()){ SDI12port.read();}
  } 
  else if(findSensorType(sensorID, CSid)){
    CS655addr = (char) i;
//    Serial.print("CS655 at address ");
//    Serial.println(CS655addr);
  }
  else {}
  
  Serial.flush();
}

void saveSDI12Info(){   
  flash.wakeup();
  delay(50);

  if(!isBlank(IDaddr)){
        flash.sectorErase4K(IDaddr);
      } delay(50);
      
  int ids_length = sensorIDs.length();
  char ids[ids_length];

  sensorIDs.toCharArray(ids,ids_length);

  flash.writeBytes(FlashHeader_Sector1,ids,ids_length);
  delay(300);

  lastIDaddr = FlashHeader_Sector1 + ids_length; delay(10);
  
  printSDI12Info();
  delay(20);   
  flash.sleep();

}

void printSDI12Info(){    
//  Serial.flush();
  for (uint32_t a= FlashHeader_Sector1; a < lastIDaddr; a++){
    Serial.print(char(flash.readByte(a)));
    delay(8);
  }
}
  

void setActiveSDI12(char c){
  for (int index = 0; index < 9; index++){
    if (activeSDI12[index] == '\0'){
      activeSDI12[index] = c;
      return;
    }
  }
}

boolean isActiveSDI12(char c){
  for (int index = 0; index < 9; index++){
    if (activeSDI12[index] == c)
      return true;
  }
  return false;
}

boolean findSensorType(char fullID[28], char type[5]){
  boolean found = false;
  for(byte i =0; i < 28; i++){
//    Serial.print("fullID[");
//      Serial.print(i);
//      Serial.print("] = ");
//      Serial.println(fullID[i]);
    if(fullID[i] == type[0]){
      byte j = 1;
      byte k = i;
      while(j<4){
        if(fullID[k+j] != type[j]){ found = false; break;}
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
  if (found == true){ return true;}
  else {return false;}
}

//===================================================================================================

//------------- Sleep Functions ---------------------------------------------------------------------

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
//  flash.sleep();    // turn off flash --> DON'T NEED HERE, IS IN saveData()
  
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);            //Power down completely on sleep
  sleep_enable();
  setRTCInterrupt();
  sleep_mode();                                  // puts MEGA to sleep

  // after wake-up interrupt
//  digitalWrite(LED,HIGH);
  ADC_on();
  Serial.begin(baudRate);
  SPI.begin();
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

float getBoxT(){
  boxTemp = (float)RTC.temperature()/4.0;
  return boxTemp;
}

byte resetAlarm(byte n){
//  readClock();
  
  if (n == interval){
  alarmMins = ((((mins + n)%60)/n)*n);
  } 
  else if (mins + n >= 60){
    alarmMins = (mins + n)%60;
  }
  else {
    alarmMins = mins + n;
  }

  RTC.setAlarm(ALM1_MATCH_MINUTES,0,alarmMins,0,0);
  
  Serial.print("Next alarm at: ");
 
  Serial.println(alarmMins);
  delay(50);
}

void setAlarm2(byte p){   
  readClock();
  byte alarm2Days;

   if (mnths % 2 == 0 && mnths != 2 && (days + p > 30)){     // 30 days in month
      alarm2Days = 1;
   } else if (mnths == 2 && (days + p) > 28){       // if February
    alarm2Days = 1;
   } else if (mnths % 2 == 1 && (days + p > 31)){    // 31 days in month
    alarm2Days = 1;
   } else {
    alarm2Days = days + p;
   }
   
  RTC.setAlarm(ALM2_MATCH_MINUTES, 0, IDmins, IDhrs, 0); 
  Serial.print("Alarm 2 set for: ");
  Serial.print(alarm2Days);
  Serial.print(',');
  Serial.print(IDhrs);
  Serial.print(":");
  Serial.println(IDmins);
  
}

//===================================================================================================

//--------- Sync time with Gateway after field install ----------------------------------------------

void fieldSync(){
  if (!manager.init()){                     // turn on radio  
    return;
  }

  driver.setFrequency(LoRaFREQ);
  driver.setTxPower(TxPower);   
  manager.setRetries(retryNum);
  
  Serial.println("Waiting to sync with Gateway...");
    
  uint8_t buf[20];     // array to receive time update from gateway
  uint8_t len = sizeof(buf);
  uint8_t from;
  boolean done = false;
  uint8_t ok[1];
  ok[0] = radioID;
  
  unsigned long twoHours = 7200000; // 2 hrs in milliseconds
  unsigned long timeToWait = millis() + twoHours;

  while(millis() < timeToWait && done == false){  // wait for 1 hour or until receive confirmation from Gateway that sync is done
   
   if(manager.recvfromAckTimeout(buf, &len, radioTimeout, &from)){  // if time update received
    for(byte u = 0; u<len;u++){
    Serial.print(buf[u]);
    }
    Serial.println();
    
    if (len > 1 && buf[0] != GatewayID) {
      Serial.println(ok[0]);
      manager.sendtoWait(ok,sizeof(ok),GatewayID);
      
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
      
      Serial.println("Got timestamp from Gateway");
      
     }  // end if sccanf
    }
     else if (buf[0] == GatewayID){   // Gateway done syncing times
      done = true;    // set flag to break out of while loop
     
       Serial.println("Done");
     }
//    }  // end if(len>0)
   }  // end if transmission received
  } // end while loop
    sendIDs();
 
}

//===================================================================================================

//------------- Send sensor list ---------------------------------------------------------------------

void sendIDs(){  
  delay(1000);
  Serial.println("Sending sensor list...");
  flash.wakeup();

  uint32_t L = lastIDaddr - FlashHeader_Sector1;
  uint8_t IDlist[L];

  
  boolean tooLong = false;
  if (L >= RH_RF95_MAX_MESSAGE_LEN){ 
    tooLong = true;
  }
  byte k=0;
  
  if (!manager.init()){                     // turn on radio
    Serial.println("radio failed");
    return;
  }
  driver.setFrequency(LoRaFREQ); //John: 07-May-2019, after call to manager.init() the radio frequency and power must be reset to desired value (otherwise defaults to 915 MHz)
  driver.setTxPower(TxPower);   
   manager.setRetries(retryNum);
   
    //--- read in sensor list from Flash
    for(uint32_t y= FlashHeader_Sector1; y < lastIDaddr; y++){  // read info from Flash into buffer
      if(flash.busy() == false){ 
        if(flash.readByte(y) != 13 && flash.readByte(y) != 10){ 
        IDlist[k] = flash.readByte(y); delay(8);
        Serial.print(char(IDlist[k]));
        k++;}
        else if (flash.readByte(y) == 10 || flash.readByte(y) == 13){
          IDlist[k] = '+';
            Serial.print(char(IDlist[k]));
            k++;
        } else {}
      } else {delay(10);}
    }
    delay(10);
    flash.sleep();
    Serial.println();

   //--- put list into sendable array

    uint8_t buf[4];     // array to receive time update from gateway
    uint8_t len = sizeof(buf);
    uint8_t from;
   
     
    if (tooLong == false){ 
     uint8_t send1[L];
     for(byte r = 0; r < L; r++){
        send1[r] = IDlist[r];
      }
      delay(5000);

      long idtimeout = millis() + 60000;  
      while(IDsuccess == false && millis() <= idtimeout){  // wait for max 1 minute    
        Serial.println(millis());
        if(manager.recvfromAckTimeout(buf, &len, radioTimeout, &from)){  // if request received
//          if (len > 0) {
            if (buf[0] == 1 && buf[3] == 1){
              for(byte i = 0; i < len; i++){
                  Serial.println(char(buf[i]));}
              Serial.println("Sending send1...");   // send sensor IDs part 1
              delay(radioTimeout);
              if(manager.sendtoWait(send1,sizeof(send1),GatewayID)){
                IDsuccess = true;
              }
            }
//          }   // end if len > 0
          IDsuccess = true;
        } 
//        Serial.println(IDsuccess);
      }   // end while loop
    }   // end if tooLong = false
    else {
       uint8_t send2a[RH_RF95_MAX_MESSAGE_LEN];
       uint8_t send2b[L - RH_RF95_MAX_MESSAGE_LEN +1];
       send2b[L - RH_RF95_MAX_MESSAGE_LEN] = 0;
       
       for(byte r = 0; r < RH_RF95_MAX_MESSAGE_LEN; r++){
        send2a[r] = IDlist[r];
       }
       for(byte r = 0; r < (L - RH_RF95_MAX_MESSAGE_LEN); r++){
        send2b[r] = IDlist[r+RH_RF95_MAX_MESSAGE_LEN];  
       }
       while(IDsuccess == false && mins <= IDmins){  // wait for max 1 minute
        readClock();
        Serial.println(secs);
      
        if(manager.recvfromAckTimeout(buf, &len, radioTimeout, &from)){  // if request received
        if (len > 0) {
          if (buf[0] == 1 && buf[3] == 1){
            Serial.println("Sending send2a...");   // send sensor IDs part 1
            manager.sendtoWait(send2a,sizeof(send2a),GatewayID);
            delay(radioTimeout/2);
            Serial.println("Sending send2b...");   // send sensor IDs part 2
            
            if(manager.sendtoWait(send2b,sizeof(send2b),GatewayID)){
              IDsuccess = true;
            }
          }
         }    // end if len > 0
        }
       }    // end while loop
    }   // end else      


             
}

//===================================================================================================

//------------- Listen for timestamp & send data ---------------------------------------------------------------------

void listenRespond(){

  SPI.begin();
  delay(200);

// Step 1 - turn on radio
  
  if (!manager.init()){                     // turn on radio
    Serial.println("radio failed");
  }
  driver.setFrequency(LoRaFREQ);  //John: 07-May-2019, after call to manager.init() the radio frequency and power must be reset to desired value (otherwise defaults to 915 MHz)
  driver.setTxPower(TxPower);   
  manager.setRetries(retryNum);
  
// Step 2 - get data ready to send

  byte Len;
  Len = allData.length() + 1;                  // size array 
  char Package[Len];
  allData.toCharArray(Package, Len);           // convert String data to char array
  uint8_t Data[Len];

  for (byte x = 0; x < Len; x++){
    Data[x] = Package[x];                   // convert char array to sendable uint8_t array
//    Serial.print(char(Data[x]));
  }

// Step 3 - wait to receive time update from Gateway, if received respond with data and update clock
  
  uint8_t buf[20];     // array to receive time update from gateway
  uint8_t len = sizeof(buf);
  uint8_t from;
  timeUpdated = false;
  Serial.println("Waiting for Gateway...");

  while(timeUpdated == false && mins <=alarmMins){  // wait for max 1 minute
    readClock();
    
    if(manager.recvfromAckTimeout(buf, &len, radioTimeout, &from)){  // if time update received
    if (len > 0) {
      
//      for (byte j=0; j < sizeof(buf); j++){     // store tranmission into buffer
////      Serial.print(char(buf[j]));
//      }
      
      manager.sendtoWait(Data, Len, GatewayID); // send data to Gateway

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
      timeUpdated = true;
      Serial.println("Successful time update");
    }
   }  // end if(len>0)
  }  // end if transmission received
  else { Serial.println(secs);}
  }  // end while loop

//  readClock();
//  resetAlarm(interval);
//  digitalWrite(LED,LOW);
}


//===================================================================================================

//------------- Initiate Measurements from Active Sensors -------------------------------------------

void readSensors() {

//  digitalWrite(SDI12Pwr, HIGH);
  SDI12port.begin();
  delay(500);  
  allData = "";

 //  get measurements from address space 0-9
  
  for (char j1 = '0'; j1 <= '9'; j1++) if (isActiveSDI12(j1)) {
      measureSDI12(j1);       
  }
  
  //  get measurements from address space a-z
  
  for (char j1 = 'a'; j1 <= 'z'; j1++) if (isActiveSDI12(j1)) {
      measureSDI12(j1);
//      allData += SDI12data + sep;
//      delay(100);
  }

  //  get measurements from address space A-Z
  for (char j1 = 'A'; j1 <= 'Z'; j1++) if (isActiveSDI12(j1)) {
      measureSDI12(j1);
//      allData += SDI12data + sep;
//      delay(100);
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

void measureSDI12(char index){
//  Serial.println(index);
  if (index != Tsensor && index != RHsensor){
//    Serial.println("Sensor NOT a T or RH sensor");
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
     
     else if (index == Tsensor){
        String stopMeas = "";
        stopMeas += Tsensor;
        stopMeas += "C3!";
//        Serial.println(stopMeas);
//        Serial.println();
        SDI12port.sendCommand(stopMeas);
        delay(500);
        if (SDI12port.available() > 0) SDI12response = true;
        while(SDI12port.available()) SDI12port.read();
        
        SDI12port.clearBuffer();
        delay(50);
        
        getResponseSDI12(index);
        allData += SDI12data + sep;
        delay(100);
     }
     else {
      
        String stopTMeas = "";
        stopTMeas += RHsensor;
        stopTMeas += "C3!";
//        Serial.println();
//        Serial.println(stopTMeas);
        SDI12port.sendCommand(stopTMeas);
        delay(500);
        if (SDI12port.available() > 0) SDI12response = true;
        while(SDI12port.available()) SDI12port.read();
        
        SDI12port.clearBuffer();
        delay(50);
        
        getResponseSDI12(index);
        allData += SDI12data + sep;
        delay(100);
             
        String stopRHMeas = "";
        stopRHMeas += RHsensor;
        stopRHMeas += "R6!";
//        Serial.println();
//        Serial.println(stopRHMeas);
        SDI12port.sendCommand(stopRHMeas);
        delay(500);
        if (SDI12port.available() > 0) SDI12response = true;
//        while(SDI12port.available()) Serial.write(SDI12port.read());

//        String RHdata = "";
//        RHdata += RHsensor;
//        RHdata += "M!";                 // get instantaneous T and RH
//        SDI12port.sendCommand(RHdata);
//        delay(300);
//        if (SDI12port.available() > 0) SDI12response = true;
//        while(SDI12port.available()) SDI12port.read();
//        
//        SDI12port.clearBuffer();
//        delay(50);
        
        getRHresponse(index);
        allData += SDI12data + sep;
        delay(100);
     }

     
}

//------------- Parse response from M! command for delay value --------------------------------------

int parseResp(){  
  delay(100); 

  byte incomingBytes;
 
  if (SDI12port.available()>0){
    SDI12response = true;
    incomingBytes = SDI12port.available();
    
    char response[incomingBytes];
    
    for ( byte i = 0; i < incomingBytes; i++){
      response[i] = SDI12port.read();
//      Serial.print(response[i],DEC);
    }

  measDelay = (response[3]-48);  // get decimal value of char, convert to milliseconds
  measDelay = measDelay*1000; 
    
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

  if(SDI12response == true){         // if a response to M! was received, send D0!

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

//        while (SDI12port.available()){
//          Serial.print(SDI12port.read());
//        }
      
   // Step 3 -- store bytes into array
   
    if (incomingbytes > 5){
      for (byte i = 0; i < (incomingbytes); i++){
        char c = SDI12port.read(); 
             response[i] = c;         // store c in array if not at last byte
//               Serial.print(c);
               delay(15);
      }    // end for loop

   // Steps 3 & 4 -- 3) take out "address CR LF" response from TDRs; 4) convert array to String, replacing "+" and "-" with delimiter, add in "-"
     
      SDI12data = "";         // clear String if not already
      delay(20);

//        char RHTresponse[incomingbytes-2];  
        char TDRresponse[incomingbytes];              // remove first three chars (i<CR><LF>) and last two (<CR><LF>) from array

        if(index == RHsensor || index == Tsensor){
          for (byte i = 0; i < incomingbytes; i++){      
            TDRresponse[i] = response[i];
//            Serial.print(TDRresponse[i]);
            delay(15);
          }
//          Serial.println();
        } else {/*
//        for (byte i = 0; i <= incomingbytes-5; i++){      
//          TDRresponse[i] = response[i+3];
//            Serial.print(TDRresponse[i]);*/
            for (byte i = 0; i < incomingbytes-2; i++){      // was -2
          TDRresponse[i] = response[i];
//            Serial.print(TDRresponse[i],DEC);
            delay(15);
        }
        Serial.println();
          delay(15);
        }
        
        byte j = sizeof(TDRresponse);
          for (byte i = 0; i < j; i++){            // put response from TDR into String
            if (TDRresponse[i] != 10 && TDRresponse[i] != 13 && TDRresponse[i] > 42  && TDRresponse[i] < 123){  // don't want to save LF or CR or out of range chars
              if ((TDRresponse[i] == '+') || (TDRresponse[i] == '-')){   // check for + and -, replace with delimiter 
                SDI12data += sep;
                  delay(20);        // from John's demo: 11 ms ensures that there has been enough time to receive another byte
                  if(TDRresponse[i] == '-'){
                    SDI12data += '-';                  // add '-' back in when needed
                      delay(20);
                  }
              } else {
                SDI12data += TDRresponse[i];         // add char to String
                delay(20);
              }
            }
           else {}
         }  // end for loop  
          
    } else {
      if(index == CS655addr){
        SDI12data = String(index) + "~-999~-999~-999";
      }
      else if (index == Tsensor || index == RHsensor){
        SDI12data = String(index) + "~-999~-999~-999~-999~-999~-999~-999";
      }
      else {
        SDI12data = String(index) + "~-999~-999~-999~-999~-999";
      }   // add error String (incomingbytes <= 3)
    }    
  }  
  else {   // if there was no response to M!
        if(index == CS655addr){
          SDI12data = String(index) + "~-999~-999~-999";
        }
        else if (index == Tsensor || index == RHsensor){
          SDI12data = String(index) + "~-999~-999~-999~-999~-999~-999~-999";
       }
        else {
          SDI12data = String(index) + "~-999~-999~-999~-999~-999";
        }   // add error String (incomingbytes <= 3)
  }
       delay(300);
       
  Serial.println(SDI12data);
  SDI12port.clearBuffer(); 
  delay(50);

 

  //--- Start next continuous measurements for T and RH sensors
    
    if(index == Tsensor){
      String Tstart = "";
      Tstart += Tsensor;
      Tstart += "C1!";
      SDI12port.sendCommand(Tstart);
        delay(500);
      while(SDI12port.available()) SDI12port.read();
      delay(50);
    } 
    else if(index == RHsensor){   
      String RTstart = "";        // start continuous T measurements on T/RH sensor
      RTstart += RHsensor;
      RTstart += "C1!";
      SDI12port.sendCommand(RTstart);
        delay(500);
      while(SDI12port.available()) SDI12port.read();
      delay(50);
    
//      String Rstart = "";
//      Rstart += RHsensor;
//      Rstart += "C4!";
//      SDI12port.sendCommand(Rstart);
//        delay(300);
//      while(SDI12port.available()) SDI12port.read();
//      delay(50);
    } 
    else {}

    SDI12port.clearBuffer(); 
    delay(50);

}

void getRHresponse(char index){

    if(SDI12response == true){
   // Step 1 -- create char array to receive data --
   
      byte incomingbytes = SDI12port.available();
//         Serial.println(incomingbytes);
      char response[incomingbytes];   // replace trailing CR with NULL and remove LF      

//        while (SDI12port.available()){
//          Serial.print(SDI12port.read());
//        }
      
   // Step 3 -- store bytes into array
   
    if (incomingbytes > 5){
      for (byte i = 0; i < (incomingbytes); i++){
        char c = SDI12port.read(); 
             response[i] = c;         // store c in array if not at last byte
//               Serial.print(c);
               delay(15);
      }    // end for loop

   // Steps 3 & 4 -- 3) take out "address CR LF" response from TDRs; 4) convert array to String, replacing "+" and "-" with delimiter, add in "-"
     
      SDI12data = "";         // clear String if not already
      delay(20);

//        char RHTresponse[incomingbytes-2];  
        char TDRresponse[incomingbytes];              // remove first three chars (i<CR><LF>) and last two (<CR><LF>) from array

        if(index == RHsensor || index == Tsensor){
          for (byte i = 0; i < incomingbytes; i++){      
            TDRresponse[i] = response[i];
//            Serial.print(TDRresponse[i]);
            delay(15);
          }
//          Serial.println();
        } else {
//        for (byte i = 0; i <= incomingbytes-5; i++){      
//          TDRresponse[i] = response[i+3];
//            Serial.print(TDRresponse[i]);
            for (byte i = 0; i < incomingbytes-2; i++){      
          TDRresponse[i] = response[i];
//            Serial.print(TDRresponse[i]);
          delay(15);
        }
//        Serial.println();
        }
        
        byte j = sizeof(TDRresponse);
          for (byte i = 0; i < j; i++){            // put response from TDR into String
            if (TDRresponse[i] != 10 && TDRresponse[i] != 13 && TDRresponse[i] > 42  && TDRresponse[i] < 123){  // don't want to save LF or CR or out of range chars
              if ((TDRresponse[i] == '+') || (TDRresponse[i] == '-')){   // check for + and -, replace with delimiter 
                SDI12data += sep;
                  delay(20);        // from John's demo: 11 ms ensures that there has been enough time to receive another byte
                  if(TDRresponse[i] == '-'){
                    SDI12data += '-';                  // add '-' back in when needed
                      delay(20);
                  }
              } else {
                SDI12data += TDRresponse[i];         // add char to String
                delay(20);
              }
            }
           else {}
         }  // end for loop  
          
    } else {
    
        SDI12data = String(index) + "~-999~-999~-999~-999~-999";
   
    }    
  }  
  else {   // if there was no response to R6!
         SDI12data = String(index) + "~-999~-999~-999~-999~-999";
         
  }
       delay(300);

       
      String Rstart = "";
      Rstart += RHsensor;
      Rstart += "R4!";
      SDI12port.sendCommand(Rstart);
        delay(500);
      while(SDI12port.available()) SDI12port.read();
      delay(50);

  SDI12port.clearBuffer(); 
  delay(50);
}

//===================================================================================================

//--------------- Calculate Battery Voltage ---------------------------------------------------------

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

//===================================================================================================

//------------- Compile timestamp  --------------------------------------------------------------

void Timestamp() {      // compile timestamp

  calcbattV();
  getBoxT();
  unsigned int pvCurrent = getSolarCurrent();
  float pvVoltage = getSolarVoltage();
  
  
  timestamp = "";

  delay(20);
  timestamp += siteID;
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
  
  allData = timestamp + allData; // + '\r' + '\n';

  delay(100);
  
  Serial.println(allData);
  delay(50);
}

//===================================================================================================

//------------- Save data to Flash --------------------------------------------------------------

void saveData(){
  
  compileData();

  flash.wakeup();
   delay(50);
  uint32_t Len;
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

    *ptr_startAddr = firstAddrInNextSector;   // update start address to first address in next sector
    Serial.println(*ptr_startAddr);
    *ptr_firstAddrInNextSector = firstAddrInNextSector + sectorLength;    // update for next sector
    Serial.println(*ptr_firstAddrInNextSector);
    delay(50);
  
    writeData();

    *ptr_startAddr += Len;

  } else if (spaceNeeded > maxAddr){   // if last sector is full
//  Serial.println(" > maxAddr");
    zeroOut();   
    *ptr_startAddr = firstDataAddr;    // start at beginning of data storage space
    flash.sectorErase4K(startAddr);    // erase sector
    *ptr_firstAddrInNextSector = firstDataAddr + sectorLength;    //update firstAddrInNextSector
    writeData();
    *ptr_startAddr += Len;
  } else {
//Serial.println("within sector");
    writeData();
    *ptr_startAddr = spaceNeeded;    // update startAddr for next empty byte
    Serial.println(startAddr);
  }

  saveAddrPtrs();
  delay(200);
  
  flash.sleep();
  
//  Serial.print("Updated startAddr: ");
//  Serial.println(startAddr, DEC);

}

bool isBlank(uint32_t addr){
  uint8_t buf[16];
  flash.readBytes(addr, buf, 16);
  bool isEmpty = true;
  for (uint8_t i = 0; i < 16; i++){
    if (buf[i] != 0xFF){
      isEmpty = false;
    }
    return isEmpty;
  }
}
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
  flash.wakeup();
  delay(50);
  
  byte buf[4];
  flash.readBytes(startAddrLoc, buf, 4);
  delay(20);
  uint32_t test = ((buf[3] << 0) & 0xFF) + ((buf[2] << 8) & 0xFFFF) + ((buf[1] << 16) & 0xFFFFFF) + ((buf[0] << 24) & 0xFFFFFFFF);
  
  if (test > firstDataAddr){
    *ptr_startAddr = test;
  }
  delay(20);

//      Serial.println(*ptr_startAddr);
    
  byte buf2[4];
  flash.readBytes(firstAddrLoc, buf2, 4);
    delay(20);  
  uint32_t test2 = ((buf2[3] << 0) & 0xFF) + ((buf2[2] << 8) & 0xFFFF) + ((buf2[1] << 16) & 0xFFFFFF) + ((buf2[0] << 24) & 0xFFFFFFFF);
  if (test2 > 0x004000){
     *ptr_firstAddrInNextSector = test2;
  }
  delay(20);

//      Serial.println(*ptr_firstAddrInNextSector);

  flash.sleep();
  delay(50);
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

//--------------- RH Sensor Heater Functions -------------------------------------------

void heaterON(){
  SDI12port.begin();
  delay(100);

  String heatOn = "";
  heatOn += RHsensor;
  heatOn += "M7!";
  SDI12port.sendCommand(heatOn);
  delay(300);

  while(SDI12port.available()) SDI12port.read();
  delay(20);
  SDI12port.end();
  delay(20);
}

void heaterOFF(){
  SDI12port.begin();
  delay(100);

  String heatOff = "";
  heatOff += RHsensor;
  heatOff += "M8!";
  SDI12port.sendCommand(heatOff);
  delay(300);

  while(SDI12port.available()) SDI12port.read();
  delay(20);
  SDI12port.end();
  delay(20);
}


//===================================================================================================
//===================================================================================================
//===================================================================================================

//------------- Menu Routine ------------------------------------------------------------------------

void menu()
{

  if (Serial.available() > 0)
  {
    Serial.read();       // clear serial input buffer
  }

  EEPROM.get(EEPROM_SITEID,siteID);
  radioID = EEPROM.read(EEPROM_RADIOID);
  GatewayID = EEPROM.read(EEPROM_GATEWAYID);
  interval = EEPROM.read(EEPROM_ALRM1_INT);
  gatewayPresent = EEPROM.read(EEPROM_GW_PRESENT);
  delay(50);

  calcbattV();

  Serial.println();
  Serial.println(F("Wireless SDI-12 Datalogger"));               // print out board info
  Serial.println();
  Serial.print(F("Site ID: "));
  Serial.println(siteID);
  Serial.print(F("Radio ID: "));
  Serial.println(radioID);
  Serial.print(F("Gateway Radio ID: "));
  if (gatewayPresent == 1){
  Serial.println(GatewayID);
  } else {
    Serial.println("-");
  }
  Serial.print(F("Measurement Interval: "));
  Serial.println(String(interval) + " mins"); 
//  Serial.print(F("RH sensor? "));
//  if(RHPresent == 0){
//    Serial.println("No");
//  } else {
//    Serial.println("Yes");
//  }
 
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

  if (battV <= lowBatt){
    Serial.println("WARNING: BATTERY VOLTAGE TOO LOW!! PLEASE CHARGE!!");
  }
    
  Serial.println();

  if (firsttime == true){
  char quickinput;
    Serial.println(F("Would you like to enter Quick Setup Mode?"));
    Serial.print(F("Enter \"y\" for yes or \"n\" for no: "));
 
  long timeout = millis() + 30000;                         // wait 30 secs for input
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
      else {firsttime = false; 
      Serial.println();
      break;}
    }
  }
 } 

  Serial.println();
  Serial.println(F("Menu options: "));

  Serial.println(F("   c  <--  Set clock"));                        // change month, day, year, hour, minute 
  Serial.println(F("   i  <--  Set IDs"));                          // set siteID, radioID, GatewayID
  Serial.println(F("   m  <--  Set measurement interval"));         // choose how often to take measurements from sensors
//  Serial.println(F("   h  <--  Add SHT31-D RH sensor"));            // choose to log data from extra sensors
  Serial.println(F("   a  <--  Change SDI-12 sensor address"));
  Serial.println(F("   t  <--  Test sensors"));                     // takes three measurements from sensors 
  Serial.println(F("   S  <--  Synchronize Gateway & Node clocks"));  // get time from Gateway, update clock
  Serial.println(F("   p  <--  Print all data to screen"));         // print data to Serial Monitor
  Serial.println(F("   e  <--  Erase all data"));                   // delete all data from Flash
  Serial.println(F("   x  <--  Exit menu"));                        // exit

  byte   menuinput;                                    // user input to menu prompt
  long  timeout;                                      // length of time to wait for user 
  
  timeout = millis() + 30000;                                      // wait 30 secs for input
  while (millis() < timeout)
  {
    menuinput = 120;
    if (Serial.available() > 0)                                    // if something typed, go to menu
    {
      menuinput = Serial.read();               // get user input
      while (Serial.available() > 0)
      {
        Serial.read();
      }
      break;
    }
  }

  switch (menuinput)
  {

    case 99: case 67:           // ------ c - Set clock ---------------------------------------------
    
      setRTCTime();
   
      menu();
      break;


    case 105: case 73:          // ---------- i - Set IDs ---------------------------------------------------

      setIDs();
      menu();                                      // go back to menu
      break;
  
    case 109: case 77:         //--------- m - Set measurement interval ----------------------------

      measureInt();
      menu();
      break;

    case 83:   //--------- S - Synchronize Gateway & Node clocks ----------------------------

      Serial.println();
      Serial.println(F("Waiting for time from Gateway..."));
      syncTime();
      delay(500);
      menu();
      break;
    

   /* case 104: case 72:         //--------- h - Add thermistor & SHT21-D sensor ----------------------------
      
      addRH();

      menu();
      break;*/

    case 97: case 65:           // ------ a - Change sensor address ---------------------------------------------

      sensorAddress();
      delay(100);
      menu();
      break;
        
    case 116: case 84:          // ------ t - Test sensors ---------------------------------------------
      
      Serial.println(F("Test measurements:"));     // take 3 readings to check sensors
      Serial.println();
      delay(10);

      for (byte g = 1; g < 4; g++) {
        readClock();
        Timestamp();         // compile timestamp
        delay(50);
        readSensors();           // read all sensors
        compileData();
      //  Serial.println(allData);
        delay(200);
      }
      menu();
      break;
  
 

     case 112: case 80:          // ------ p - Print node data to screen ---------------------------------------------
      
      Serial.println(F("Print Data: "));     // download all data in Flash
      delay(100);

//      recallAddr();
     
      for(unsigned long i=firstDataAddr; i< *ptr_startAddr; i++){
        if (flash.readByte(i) == 43){
          Serial.println(); delay(5);     
        } else if (flash.readByte(i) == 0){} 
        else {
        delay(5);Serial.print(char(flash.readByte(i)));       
        }
//      Serial.print(flash.readByte(i));
      }
      delay(500);
      
      menu();
      break;

    case 101: case 69:          // ------ e - Erase Flash ---------------------------------------------

      Serial.println(F("Are you sure you want to erase all data stored in memory?"));
      Serial.print(F("Enter 'y' for yes or 'n' for no: "));
      charinput();
      
      if(charInput[0]=='y' || charInput[0]=='Y'){
       Serial.println(F("Erasing data..."));
        
        for (uint32_t sect = firstDataAddr; sect <= maxAddr; sect + sectorLength){
          if (!isBlank(sect)){
            flash.sectorErase4K(sect);
          } else {break;}
        }
       
//       flash.chipErase();               // if saving info to header, should change this to only erase sectors with data
       delay(2000);
       *ptr_startAddr = firstDataAddr;
//       startAddr = firstDataAddr;
       *ptr_firstAddrInNextSector = 0x004000;
       delay(50);
       saveAddrPtrs();
       delay(100);
//       addr = 0;
//       saveAddr();
      } else {}
      delay(500);
      menu();
      break;

    case 120: case 88:          // ------ x ----------------------------------------------
      Serial.println(F("Exit"));                           // exit
      Serial.println();
      delay(10);
      break;

  }
  if(userinput == true){
    digitalWrite(LED,LOW);
  }
}

void setRTCTime(){
  
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
      /*Serial.print(F("  input Time Zone: "));
      charinput();
        for (byte i = 0; i < 4; i++){
          TMZ[i] = charInput[i];
        }
        //TMZ[3]=0;
       EEPROM.put(TMZMem,TMZ);
         delay(10);*/
      RTC.write(tm);
      delay(50);
}

void setIDs(){
  Serial.println(F("Set IDs: "));
  
  Serial.print(F("  Enter 5-digit site ID (ex. ABC1b): "));    // set siteID
  charinput();
  for (byte i = 0; i < 6; i++){
     siteID[i] = charInput[i];
  }
  EEPROM.put(EEPROM_SITEID, siteID);  
  delay(10);
  
  Serial.print(F("  Enter radio ID: "));
  getinput();
  radioID = indata;
  manager.setThisAddress(radioID);
  delay(10);
  EEPROM.update(EEPROM_RADIOID, radioID);
  delay(10);
       
  Serial.println();
      
  Serial.print(F("  Are you using a Gateway? Enter 'y' for yes, 'n' for no: ")); 
  charinput();
  gatewayPresent = (charInput[0] == 'y' || charInput[0] == 'Y'? 1:0);
  EEPROM.update(EEPROM_GW_PRESENT, gatewayPresent);
      
  if (gatewayPresent == 1){
    Serial.print(F("    Enter Gateway radio ID: "));
    getinput();
    GatewayID = indata;
    EEPROM.update(EEPROM_GATEWAYID, GatewayID);
    delay(10);
  } 

  delay(100);
}

void measureInt(){
    Serial.println();
    Serial.print(F("Enter measurement interval (every 15, 20, 30, or 60 mins): "));
    Serial.flush();
    getinput();
    while (indata != 5 && indata != 15 && indata != 20 && indata != 30 && indata != 60) {   
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

//======================================================================================
//======================================================================================
//======================================================================================

//-------------- Quick Start Mode --------------------------------

void quickStart(){
  Serial.println();

 //--- Step 1: Set Clock ---
      
      setRTCTime();
      Serial.println();

    //--- Step 2: Enter Node info ---

      setIDs();
      Serial.println();

      
    //--- Step 3: Set measurement interval ---
       
      measureInt();
      Serial.println();
      
   
   //--- Step 4: add RH or not --- 
   
//      addRH();
//      Serial.println();
      
  firsttime = false;
  Serial.println(F("Quick setup complete, going to Main Menu..."));
  Serial.println();
  Serial.println();
  delay(200);
    
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
     // Serial.println(numincoming);
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

void syncTime(){
   if (!manager.init()) {                      // initialize radio
    Serial.println("Radio failed");
  }
  driver.setFrequency(LoRaFREQ);  //John: 07-May-2019, after call to manager.init() the radio frequency and power must be reset to desired value (otherwise defaults to 915 MHz)
  driver.setTxPower(TxPower);
  manager.setRetries(retryNum);
  
  uint8_t buf[19];     
  uint8_t len = sizeof(buf);
  uint8_t from;
  boolean timeSynced = false;
  uint8_t hello[] = {1,1,1,1};
  
  readClock();    
  long timeout = millis() + 45000;    // 45 seconds

  while ((millis() < timeout)  && (timeSynced == false)){
    manager.sendtoWait(hello, 4, GatewayID);
    
    if(manager.recvfromAckTimeout(buf, &len, radioTimeout, &from)){  // if time update received
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
   }  // end if transmission received
  }

  if (timeSynced == false){
    Serial.println("Synchronization failed. Return to menu to try again.");
  } else {
    Serial.println();
    Serial.println("Gateway & Node successfully synced");
  }
}


//======================================================================================

//-------------- Change sensor address --------------------------------

void sensorAddress(){
  oldAddress = '!';
//    digitalWrite(SDI12Pwr,HIGH);                    // turn on SDI12 sensors
    SDI12port.begin();
    delay(100);
    boolean found = false;
 

    for (char j1 = '0'; j1 <= '9'; j1++) if (isActiveSDI12(j1)) {
      found = true;
       oldAddress = j1;
      changeAddress();    
    }
  
  for (char j1 = 'a'; j1 <= 'z'; j1++) if (isActiveSDI12(j1)) {
       found = true;
        oldAddress = j1;
      changeAddress(); 
  }

  for (char j1 = 'A'; j1 <= 'Z'; j1++) if (isActiveSDI12(j1)) {
      found = true;
       oldAddress = j1;
      changeAddress(); 
  }

 if(!found){
    Serial.println("No sensor detected. Check physical connections."); // couldn't find a sensor. check connections..
  }  
    
  
}

void changeAddress(){
//  char oldAddress = '!';
  String myCommand = "";

  Serial.print(F("Current address: "));
 
  Serial.println(oldAddress);

  Serial.print("Enter new address: ");                             // prompt for a new address
    while(!Serial.available());
    char newAdd= Serial.read();

    // wait for valid response
    while( ((newAdd<'0') || (newAdd>'9')) && ((newAdd<'a') || (newAdd>'z')) && ((newAdd<'A') || (newAdd>'Z'))){
      if(!(newAdd =='\n') || (newAdd =='\r') || (newAdd ==' ')) {
        Serial.println("Not a valid address. Please enter '0'-'9', 'a'-'A', or 'z'-'Z'.");
      }
      while(!Serial.available());
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
    if(SDI12port.available()){
      while(SDI12port.available()){
      Serial.write(SDI12port.read());
      }
      Serial.println("Success.") ;
    }
}
