#ifndef GlobalDefs_H
#define GlobalDefs_H

#include <SPIFlash.h>
//#include <SPIFlash_MX25R6435F.h>                      // edited SPIFlash library to work with 64Mbit chip
#include <SPI.h>
#include <TimeLib.h>  //for time_t type
#include "AcclimaSDI12.h"

#define SPI_DEFAULT_SPEED   8000000

#define FIVESECONDWAIT      5000

#define LoRa_RegOpMode      0x01        //Operating Modes Register (including SLEEP and STDBY)
#define LoRa_MODE_SLEEP     0
#define LoRa_MODE_STDBY     1
#define LoRa_MODE_RXSINGLE  6

#define ADC_REF_VOLTAGE     3.3
#define ADC_RESOLUTION      1024
#define SOLAR_CALIB         1.0        //This number should be close to 1.0 (it is the resistance of the solar shorting resistor)

//#define pin_Flash_CS        23    //Flash CS: physical pin 26 --> Arduino pin D23
#define pin_RTCInt          A2    //RTC Interrupt: physical pin 35 --> Arduino pin A2
#define pin_USBInt          11    //FTDI USBInt: physical pin 12 --> Arduino pin D11
#define pin_RX0             8     //PCINT24 -- can use this as an interrupt to wake MCU from sleep
#define pin_TX0             9
#define pin_SDA             20
#define pin_SCL             19
#define pin_LoRa_CS         4     //LoRa Radio CS: physical pin 44 --> Arduino pin D4
#define pin_mBatt           14    //pin to turn on battery voltage measurement circuit
#define pin_battV           A0    //analog pin to measure battery voltage

#define pin_SDI12_DATA      10    //The pin of the SDI-12 data line
#define pin_SDI12_POWER_EN  0     //The pin to turn on power supply for SDI-12
#define pin_SDI12_TX_EN     3     //Transmit enable pin; when this is high the node can transmit on SDI-12 bus BUT it must be asserted low before any data can be received
#define pin_SDI12_LOW_VOLTS (-1)  //Switch SDI-12 power supply to 7.5 volts (vs. 12 volts)... negative here to signify that the pin is brought LOW to enable the 7.5 volt level

#define pin_LED             15    //Moteino MEGAs have LEDs on D15
//below added 06-Mar-2019 for Node HW rev. D

#define pin_solarShort      A4
#define pin_solarVoltage    A5

#define MAX_SDI12_READINGS  10    //this represents the maximum number of readings (could have several readings per SDI-12 device) supported by the node
#define READ_CMD_DEFAULT    "M!"
#define MAX_SENSORS         16
#define MAX_RESP            125   // max length of sensor output

#define EEPROMSHIFT                 2800  

//#define DEBUG_ON   //for printing debug statements


//extern SPIClass spi;
////extern SPIFlash flash;              //(pin_Flash_CS, 0xC228); //0xC228 is manufacturer ID for Macronix MX25R6435F
//extern uint32_t firstLogAddr;           //RAM pointer to first log entry (oldest)
//extern uint32_t lastLogAddr;            //RAM pointer to last log entry (most recent); Perhaps not necessary as this can be determined using "prevLogAddr(nextLogAddr)"
//extern uint32_t nextLogAddr;            //RAM pointer to next location to write log entry (this is determined every time upon power-up using method "getLogPtrs" in Flash_Func.h)
//extern uint32_t firstBlankSectorAddr;
//extern uint32_t lastLogId;          //NOTE: Max value of lastLogId is 0xFFFFFFFE however the variable in RAM will be set to 0xFFFFFFFF upon reaching that maximum so that the
//                                    //      next flash write operation will overflow and wrap the value to zero again.  (see updateLogPtrs function in Flash_Func.cpp)


typedef uint16_t RAM_TypeSize;      //For ATmega1284P the RAM size is 16,384 bytes so a 16-bit unsigned int (65,535) easily addresses the whole memory space

struct FactoryInfo {
  const char model[10]      = "Node";      //model name
  char verHW                ;              //hardware version
  char optsRadio            ;              //radio options (code character for LoRa radio & cell modem; see documentation)
  const byte verFW_maj      = 0;           //major firmware version
  const byte verFW_min      = 6;           //minor firmware version
  const uint16_t verFW_bld  = 101;         //firmware build
  const char flavor         = 'U';         //A = Acclima; U = USDA
  const char verUI          = '0';         //user interface version
  uint32_t sernum           ;              //serial number
  uint16_t mfgDate          ;              //9857 corresponds to 01-Apr-2019 (April fools!)
};

extern FactoryInfo facInfo;

struct sensor{
  char addr;
  char ID[36];
  char type[5];
  char cmd[5];
  uint8_t numOut;
  char depth[6] = {0,0,0,0,0,0};
  char data[SDI12_BUFFER_SIZE];
} allSensors[MAX_SENSORS];

#define NUM_TYPES           6
const char * const sensTypes[][3] = {{"TR31","M!","06"},      // including pEC and tt
                                    {"CS65","M4!","06"},
                                    {"Temp","M!","01"},
                                    {"THum","M!","02"},
                                    {"CLIM","R7!","14"},
                                    {"SIL4","M1!","02"}};


//WARNING! If the NodeLog struct is changed below be sure to make identical changes to the same struct in the gateway firmware GlobalDefs.h file!
/*struct NodeLog {
  uint32_t logId;
  uint32_t timestamp;
  uint16_t flags;
  uint16_t battV;
  uint16_t boxTemp;
  uint16_t iSolar;
  uint16_t vSolar;
  uint16_t logSize;   //This is the size of the structure WITH the length of the SDI-12 data string added (not including the null-terminator which is not present when saved to flash).
  char* dataSDI12;    //NOTE: When this structure is saved to flash, the pointer here is written as the length of the SDI-12 data string and the characters of the string immediately follow (without null-terminator).
  };*/
//struct NodeLog {
// uint32_t logId;
// uint16_t lenSDI12;
// uint16_t logSize;
// char* dataStr;    //NOTE: When this structure is saved to flash, the pointer here is written as the length of the SDI-12 data string and the characters of the string immediately follow (without null-terminator).
//};


/*
  Status:

  Gets status of device which includes:
    -date/time timestamp
    -most recent logId
    -most recent configId
    -battery voltage
    -box temperature
    -solar current
    -solar voltage
    -available RAM

    To include later:
    -FOR NODE: next log time
    -FOR GATEWAY: next sync time
    -bootCnt
    -initBits               :last boot test results
    -lastLowV               :last low voltage
    -lastLowVTS             :last low voltage timestamp
    -solar current means
    -most recent ErrLogId
    -average RSSI (incoming LoRa radio signal strength)
    -
*/
//struct NodeStatus {
//  public:
//  uint32_t sernum;
//  uint32_t timestamp;
//  uint32_t lastLogId;
//  uint32_t lastConfigId;
//  uint16_t battV;
//  uint16_t boxTemp;
//  uint16_t iSolar;
//  uint16_t vSolar;
//  uint16_t availRAM;
//};


struct GatewayStatus {
  //TODO: Flesh this out and implement
};

//typedef struct LogDecoded {   //size: 28
//  uint32_t logId;
////  time_t timestamp; //time_t is uint32_t eqiuvalent
////  uint16_t flags;
////  float battV;
////  float boxTemp;
////  uint16_t iSolar;
////  float vSolar;
//  uint16_t lenSDI12;
//  uint16_t logSize;   // 12-Feb-2020: was line above
//  char* strSDI12;
//} LogDecoded;
//
//typedef struct NodeLog NodeLog;
//typedef struct NodeStatus NodeStatus;
//typedef struct GatewayStatus GatewayStatus;


extern void printDateTime(time_t t);
extern bool getInput(byte** buf, uint16_t* len, uint16_t minCharsBeforeTerminator, byte terminator, uint16_t timeout);
extern RAM_TypeSize getFreeRAM();
//extern LogDecoded* decodeLogEntry(NodeLog* log);    // To get integer values to floats, etc.


#endif
