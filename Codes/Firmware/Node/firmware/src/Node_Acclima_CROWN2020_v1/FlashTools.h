/*
 *  FlashTools
 *
 *  For use with Arduino SPIFlash library to be used as a wrapper to that class and provide tools to manage log space (for 
 *  simplified writing of logs and to spread out the logs for better wear-leveling of flash memory).
 *  
 *  Authors:        John Anderson
 *                  Acclima, Inc.
 *
 *  Created:        Jul 2019
 *  Last Modified:  Feb 2020
 *
 */


#ifndef FlashTools_H
#define FlashTools_H


#include <SPIFlash.h>
//#define TEXT_OUT
// #define DEBUG_ON

#define delayRes            25          // The delay resolution when looping, waiting on flash to be ready.  DO NOT ADJUST lower than 5.

// NOTE: The following constants are specific to Macronix 64 Mbit MX25R6435F Flash chip:
#define pin_Flash_CS        23          //Flash CS: physical pin 26 --> Arduino pin D23
#define flash_deepPowerDown 0xB9        //Opcode to put flash into low power mode when sleeping
#define flash_releasePowerDown 0xAB

#define flash_totalSize     0x800000    //total bytes available of flash memory
#define flash_sectorSize    0x1000      //4096 (0x1000) Bytes in each sector
#define flash_sectorMask    0xFFF000    //when using bitwise AND, allows upper sector bits but not lower bits between sector lines
#define flash_lastAddress   (flash_totalSize - 1)

#define flash_header_begin  0           //flash header at beginning of memory space
#define flash_header_size   0x2000      //2 sectors reserved for flash header:

#define LOG_beginAddr       (flash_header_begin + flash_header_size)
#define LOG_endAddr         (flash_totalSize)
#define LOG_minSize         32          //NOTE: Log size must be a power of 2 to line up with sector boundaries -- pad the log with unused bytes if needed!  WARNING! Modify LOG_maxTries below if changing this value from 32!
#define LOG_maxSize         flash_sectorSize
#define LOG_maxId           0xFFFFFFFE  // The value 0xFFFFFFFF is used to mark that logID is uninitialized therefore all logIds must be within 0x00 to 0xFFFFFFFE

#define LOG_logSizeOffset   4           // Offset from log beginning to the two bytes that define log size (NOT totalLogSize which must always be a multiple of 32)
#define LOG_headerSize      sizeof(LogHeader)           // The number of bytes used by log header
#define LOG_maxTries        (18 + (LOG_maxSize / LOG_minSize)) 
/*                           18 is the number of tries to divide 0x800000 by two until it reaches 0x20 (32 which is minLogSize) (math goes like this: LOG2(0x800000)-LOG2(0x20))
 *                           ...BUT must allow an additional (maxLogSize / minLogSize) because larger log entries might misdirect search slightly (because when using seekLogId 
 *                           the addr can be shifted by as much as maxLogSize).
 */



// v v v   public non-member function prototypes   v v v
bool getInput(uint8_t** buf, uint16_t len);
bool getInput(uint8_t** buf, uint16_t len, uint16_t minCharsBeforeTerminator);
bool getInput(uint8_t** buf, uint16_t* len, uint16_t minCharsBeforeTerminator, uint8_t terminator);
bool getInput(uint8_t** buf, uint16_t len, uint16_t minCharsBeforeTerminator, uint8_t terminator, uint16_t timeout);
bool getMoreInput(uint8_t** buf, uint16_t* len, uint16_t minCharsBeforeTerminator, uint8_t terminator, uint16_t timeout);
// ^ ^ ^   public non-member function prototypes   ^ ^ ^



/*  Simple String Log Format:
 *
 *  Offset  Bytes   Description  
 *  0x00    4       log ID
 *  0x04    2       log entry total size including string data payload (but not including the memory zeroed out in flash to get the logs to be 32 byte aligned)
 *  0x06    ?       string data payload (can be as large as 4090 characters)
 *  ?       ?       remaining unused bytes are filled with value 0 until log lines up with 32 byte boundary
 *
 *
 *  string data payload format:
 *  TODO: Specify string format
 *
 */
struct LogHeader {
  uint32_t logId;
  uint16_t logSize;
};

typedef struct LogHeader LogHeader;


class FlashTools {
public:  
  FlashTools();

  // Public methods for general flash usage:
  bool init(uint8_t pinSS_flash, uint16_t jedecID=0);
  void flash_sleep();
  void flash_wake();
  bool wait(uint32_t timeout);
  void chipErase();
  void readBytes(uint32_t addr, void* buffer, uint16_t length);
  uint8_t readByte(uint32_t addr);

  // v v v  The enclosed functions can only be used OUTSIDE the log space (for use with flash header, etc.)  v v v
  bool eraseSector(uint32_t addr);
  bool writeBytes(uint32_t addr, const void* buffer, uint16_t length);
  bool writeByte(uint32_t addr, uint8_t value);
  // ^ ^ ^  The enclosed functions can only be used OUTSIDE the log space (for use with flash header, etc.)  ^ ^ ^


  // Public methods for dealing with logs and log space:
  bool logSetup(uint32_t logSpace_begin, uint32_t logSpace_end, uint16_t minLogSize, uint16_t maxLogSize, bool printReport = false);
  // Why avoiding String objects?  Because not sure how to avoid memory fragmentation and don't have time to research it.
  // See the following:  https://www.arduino.cc/reference/tr/language/variables/data-types/stringobject/  
  bool writeLog(char *data, uint16_t length);
  bool getLog(uint32_t address, LogHeader *headerOut, char **dataOut);

  uint32_t logAddrFromId(uint32_t logId);
  uint32_t logIdFromAddr(uint32_t address);
  uint16_t getLogSize(uint32_t logHeaderAddr);
  uint16_t getLogMemorySize(uint32_t logHeaderAddr);

  bool verifyLogHeader(uint32_t address);
  bool printLog(uint32_t addr);
  void printLogs();
  void printData();   // added by AIT 6-Mar-2020
  bool eraseLogs();
  uint32_t firstLogAddress();
  uint32_t lastLogAddress();
  uint32_t nextLogAddress();
  uint32_t blankSectorAddress();
  uint32_t logId();
  uint32_t logSpaceBeginAddress();
  uint32_t logSpaceEndAddress();
  uint16_t minimumLogSize();
  uint16_t maximumLogSize();
  void dmaMenu();

private:
  SPIFlash *flash;
  uint32_t firstLogAddr;          // address of oldest log entry (memory is recycled and overwrites oldest entries first)
  uint32_t lastLogAddr;           // address of newest log entry (the one most recently written)
  uint32_t nextLogAddr;           // address where the next log entry is to be written
  uint32_t blankSectorAddr;       // address of the first blank sector of flash
  uint32_t lastLogId;             // last written logId (always increasing, never wraps in lifetime of the node)
  uint32_t logSpace_begin;
  uint32_t logSpace_end;
  uint16_t minLogSize;
  uint16_t maxLogSize;
  

  bool is_EmptyToData_Boundary(uint32_t addr);

  void logInit();


  void updateLogSpace(uint16_t lastLogSize); //Takes care of updating log memory pointer variables after each log write including when the logs wrap from end of memory space to beginning
  

  bool eraseSector(uint32_t addr, bool allowLogspaceErase);
  
  // bool getLogId(uint32_t *addr, uint32_t *logId);

  bool isBlank(uint32_t addr);
  uint32_t wrapLogAddr(uint32_t addr);
  void normLogAddr(uint32_t *addr, bool wrap);

  void shiftLogAddr(uint32_t *addr, int8_t seekCode, uint16_t shift);
  uint32_t truncLogAddr(uint32_t addr);
  bool seekLogId(uint32_t *addr, int8_t seekCode, uint32_t *logId);
  bool seekLogAddr(uint32_t *addr, int8_t seekCode);
  bool readLogId(uint32_t addr, uint32_t *logId);
  uint32_t getNextLogAddr(uint32_t logHeaderAddr);
  uint32_t getNextLogSectorAddr(uint32_t addr);
  uint32_t initMSSize(uint32_t addr, int8_t sign);
  bool binSearchForBlank(uint32_t *addr);
  bool binSearch(uint32_t *addr, int8_t sign);

  bool getType1Bounds(uint32_t *addrL, uint32_t *addrR);
  bool getType2Bounds(uint32_t *addrL, uint32_t *addrR);
  bool getType3Bounds(uint32_t *addrL, uint32_t *addrR);
  bool getType4Bounds(uint32_t *addrL, uint32_t *addrR);
  bool boundSearch(uint32_t *addrL, uint32_t *addrR);
};

#endif
