#include <Arduino.h>
#include "FlashTools.h"


// v v v   private function prototypes   v v v
uint8_t log2pow2(uint16_t val);
bool inputProcessingLoop(uint8_t** buf, uint16_t* len, uint16_t minCharsBeforeTerminator, uint8_t terminator, uint16_t timeout, uint16_t idx);
// ^ ^ ^   private function prototypes   ^ ^ ^


FlashTools::FlashTools() {
  // Constructor
  // Object is setup using init method
}


bool FlashTools::init(uint8_t pinSS_flash, uint16_t jedecID) {
  flash = new SPIFlash(pinSS_flash, jedecID);
  return flash->initialize();
}


// Returns true if the flash is available, false if it was not available within timeout period specified.
// Maximum wait duration is capped at 15 minutes
bool FlashTools::wait(uint32_t timeout) {  
  // Capping the maximum wait duration to 15 minutes
  if (timeout > 15 * 60 * 1000)
    timeout = 15 * 60 * 1000;

  // Adjusting the delay resolution to ensure that [targetCnt] is not overflowed
  uint16_t resDelay = timeout / 5000;
  if (resDelay > delayRes)
    resDelay = delayRes;

  uint16_t targetCnt = timeout / resDelay;
  uint16_t cnt = 0;
  bool flashDone = false;
  while ((cnt < targetCnt) && (!flashDone)) {
    if (!flash->busy()) {
      flashDone = true;
      break;
    }
    delay(resDelay);
    cnt++;    
  }
  return flashDone;
}


void FlashTools::chipErase() {
  flash->chipErase();
}

void FlashTools::readBytes(uint32_t addr, void* buffer, uint16_t length) {
  flash->readBytes(addr, buffer, length);
}

uint8_t FlashTools::readByte(uint32_t addr) {
  return flash->readByte(addr);
}

bool FlashTools::writeBytes(uint32_t addr, const void* buffer, uint16_t length) {
  if ((addr >= logSpace_begin) && (addr < logSpace_end)) {    
    return false;
  }
  flash->writeBytes(addr, buffer, length);
  return true;
}

bool FlashTools::writeByte(uint32_t addr, uint8_t value) {
  if ((addr >= logSpace_begin) && (addr < logSpace_end)) {    
    return false;
  }
  flash->writeByte(addr, value);
  return true;
}


// This is the exposed method...protects logspace so that log structure is not destroyed which could cause log failure.
// It is unnecessary to erase logs because log memory space is recycled and gaps in logId are disallowed (could cause failure).
// It is possible to erase all logs using [eraseLogs] method.
bool FlashTools::eraseSector(uint32_t addr) {
  return eraseSector(addr, false);
}

bool FlashTools::eraseSector(uint32_t addr, bool allowLogspaceErase) {
  if (!wait(1000)) {
    // Waited one second...flash not responding!
    return false;
  }

  if ((addr >= logSpace_begin) && (addr < logSpace_end)) {
    if (!allowLogspaceErase)
      return false;
  }

  #ifdef TEXT_OUT
  Serial.print("Erasing sector containing address 0x");
  Serial.println(addr, HEX);
  Serial.flush();
  #endif

  flash->blockErase4K(addr);
  if (!wait(1000)) {
    // Waited one second...flash not responding!
    return false;
  }
  return true;
}



void FlashTools::flash_sleep() {
  //Put memory into deep power down mode -- this saves another 7 uA
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(pin_Flash_CS, LOW);  //Assert chip select
  SPI.transfer(flash_deepPowerDown);
  digitalWrite(pin_Flash_CS, HIGH); //Release chip select
  SPI.endTransaction();  
}

void FlashTools::flash_wake() {
  //Must toggle CS low for at least 20ns to release flash from deep sleep mode
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(pin_Flash_CS, LOW);  //Assert chip select
  __asm__ __volatile__("nop");
  digitalWrite(pin_Flash_CS, HIGH); //Release chip select
  SPI.endTransaction();
}

uint32_t FlashTools::getNextLogSectorAddr(uint32_t addr) {
  return wrapLogAddr((addr + flash_sectorSize) & flash_sectorMask);
}

// bool FlashTools::getLogId(uint32_t *addr, uint32_t *logId) {
//   return seekLogId(addr, 1, logId);
// }

uint32_t FlashTools::logAddrFromId(uint32_t logId) {
  const uint32_t errorRes = 0xFFFFFFFF;
  if ((lastLogAddr >= flash_totalSize) || (firstLogAddr >= flash_totalSize)) {    //27-Jul-2019: changed flash_totalSize from 0xFFFFFF    
    return errorRes;
  }
  uint32_t firstId, lastId, testAddr, testId;
  if ((readLogId(firstLogAddr, &firstId)) && (readLogId(lastLogAddr, &lastId))) {  //bool readLogId(uint32_t addr, uint32_t *logId)
    if ((logId >= firstId) && (logId <= lastId)) {
      if (lastLogAddr > firstLogAddr) { //straightforward scenario where logs are not wrapped
        /* If logs are all the same size, the equation below will get there exactly.  
         * If the logs are of varying sizes it will get close, then just seek the rest of the way
         */
        testAddr = truncLogAddr((uint32_t)(firstLogAddr + (float)(logId - firstId) * ((float)(lastLogAddr - firstLogAddr) / (float)(lastId - firstId))));        
      } else {
        //Same algorithm as above applies IF we shift all the wrapped logs higher than logSpace_end (in other words
        //we must straighten the logs back out or unwrap them)... then apply the math and re-wrap the result.
        uint32_t uwLastLog; //unwrapped lastLogAddr
        uwLastLog = (lastLogAddr - logSpace_begin + logSpace_end);
        testAddr = ((uint32_t)(firstLogAddr + (float)(logId - firstId) * ((float)(uwLastLog - firstLogAddr) / (float)(lastId - firstId))));        
        testAddr = wrapLogAddr(testAddr);
      }

      //Fine tuning will need to happen with logSpace that has varying length logs...
      //NOTE: Might need to iterate the algorithm above in certain extreme situations of small and large logs mixed -- do the same math with a more localized sample?
      if (seekLogId(&testAddr, 1, &testId)) {
        uint8_t cnt = 255;  //provide a failsafe just in case it can't find it for some reason
        while ((testId != logId) && (cnt > 0)) {
          if (testId > logId) {
            testAddr -= minLogSize;
            seekLogId(&testAddr, -1, &testId);
          } else {
            testAddr += minLogSize;
            seekLogId(&testAddr, 1, &testId);
          }
          cnt--;
        }
        
        if (cnt != 0)
          return testAddr;
        else 
          return errorRes;
      } else {
        return errorRes;
      }
    }
  }

  return errorRes;
}

bool FlashTools::seekLogAddr(uint32_t *addr, int8_t seekCode) {
  uint32_t logId;
  return seekLogId(addr, seekCode, &logId);
}

void FlashTools::updateLogSpace(uint16_t lastLogSize) {
  uint32_t oldNextLogAddr = nextLogAddr;
  lastLogId++;
  nextLogAddr = wrapLogAddr(nextLogAddr + lastLogSize);

  uint32_t nextSector, oldestSector;
  nextSector = getNextLogSectorAddr(nextLogAddr);   //This will wrap to beginning of log memory space upon reaching logSpace_end


  if (!isBlank(nextSector)) {
    eraseSector(nextSector, true);
    oldestSector = getNextLogSectorAddr(nextSector);

    // The following locates the first log address to the right of the empty sector (because it may not always fall on the transition from empty to data due to logs allowed to overlap sector boundaries)
    if (seekLogAddr(&oldestSector, 1)) {
      firstLogAddr = oldestSector;
    }
  }
  blankSectorAddr = nextSector;
  lastLogAddr = oldNextLogAddr;

  if (firstLogAddr >= flash_totalSize)   //This is for the case where previously empty memory gets its first log entry -- we must set BOTH firstLogAddr and lastLogAddr to the single log entry
    firstLogAddr = lastLogAddr;
}


uint8_t log2pow2(uint16_t val) {
  bool addOne = false;
  if (val % 2 > 0)
    addOne = true;
  uint8_t result = 0;
  do {
    val = val >> 1;
    result++;
    if (val % 2 > 0)
      addOne = true;
  } while (val > 1);
  if (addOne)
    result++;
  return result;
}



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
 *  TODO:
 *
 */
bool FlashTools::writeLog(char *data, uint16_t length) {
  if (length <= (maxLogSize - LOG_headerSize)) {  // We can support log entries as large as one sector which is 4096 bytes (plus 6 header bytes)
    uint32_t logId = lastLogId + 1;
    uint16_t logSize = LOG_headerSize + length;
    uint16_t totalLogSize;
    if ((logSize % minLogSize) != 0)
      totalLogSize = logSize + (minLogSize - (logSize % minLogSize));
    else 
      totalLogSize = logSize;


    // BUFFER SETUP (doesn't matter at this point if wrapping or not):
    // Initialize buffer      
    uint8_t buf[totalLogSize];

    // Add header
    memcpy(buf, &logId, 4);
    memcpy(buf + LOG_logSizeOffset, &logSize, 2);

    // Add data
    memcpy(buf + LOG_headerSize, data, length);

    // Now fill the remainder with zeros
    memset(buf + LOG_headerSize + length, 0, totalLogSize - logSize);

    if ((totalLogSize + nextLogAddr) > logSpace_end) {
      // This scenario requires wrapping the log due to end of logspace boundary
      // NOTE: Because we force the logSize to be a multiple of minLogSize, the header will always be able to fit within remaining space
      uint16_t remainingSpace = logSpace_end - nextLogAddr;      
      uint16_t bytesStillNeeded = totalLogSize - remainingSpace;
      uint16_t dataBytesBeforeBoundary = remainingSpace - LOG_headerSize;

      // Write to log space BEFORE boundary
      flash->writeBytes(nextLogAddr, buf, remainingSpace);

      // // // Set address to logSpace_begin to write the remainder of the log entry
      // // nextLogAddr = ;

      wait(500); // Provide a pause for previous call to writeBytes to finish (is this necessary?)
      
      // Serial.print("Now writing to logSpace_begin: ");
      // Serial.println((char*)(buf + remainingSpace));

      // Write to log space AFTER boundary
      flash->writeBytes(logSpace_begin, buf + remainingSpace, totalLogSize - remainingSpace);
    } else {
      // Straightforward writing of log
      flash->writeBytes(nextLogAddr, buf, totalLogSize);
    }

    // Update log pointers, erase next sector (if needed)
    updateLogSpace(totalLogSize);

    return true;
  } else 
    return false;
}



bool FlashTools::verifyLogHeader(uint32_t address) {
  uint32_t logId; // Don't need this other than that it is required by readLogId method...
  return (readLogId(address, &logId));
}


// It is the responsibility of caller to free the result stored in dataOut!!!
bool FlashTools::getLog(uint32_t address, LogHeader *headerOut, char **dataOut) {
  bool result = false;

  // First verify that the address points to the log header  
  if (verifyLogHeader(address)) {
    // Now extract header information
    flash->readBytes(address, headerOut, sizeof(LogHeader));

    // Determine size of data and init the buffer
    uint16_t dataLen = headerOut->logSize - sizeof(LogHeader);
    *dataOut = (char*)malloc(dataLen + 1);

    // Extract data
    if (address + headerOut->logSize > logSpace_end) {
      uint16_t lenBeforeBoundary = (logSpace_end - address) - sizeof(LogHeader);

      flash->readBytes(address + sizeof(LogHeader), *dataOut, lenBeforeBoundary);
      flash->readBytes(logSpace_begin, *dataOut + lenBeforeBoundary, dataLen - lenBeforeBoundary);
    } else {
      flash->readBytes(address + sizeof(LogHeader), *dataOut, dataLen);
    }

    // Add null terminator
    *(*dataOut + dataLen) = 0;
    
    // Set result and return
    result = true;
  }

  return result;
}



bool FlashTools::printLog(uint32_t addr) {
  char *data;
  LogHeader lh;
  if (getLog(addr, &lh, &data)) {
//    Serial.print("\nLog ID: ");
//    Serial.println(lh.logId);
//    Serial.print("Data: ");
    Serial.println(data);    
    Serial.flush();
    free(data);
    return true;
  } else {
    Serial.println("Could not retrieve log!");
    return false;
  }
}



void FlashTools::printLogs() {

  if ((lastLogAddr >= logSpace_end) && (firstLogAddr >= logSpace_end)) {
    Serial.println("ERROR: There are no logs!");
    return;
  }

  uint16_t lenStr = 10;   //30 byte array should be more than enough
  char* input = (char*)malloc(lenStr);
  bool retryInput = false, haveResp = false;
  char resp;
  Serial.print("\n\nPrint Logs:\n");
  do {
    Serial.println("\nWould you like to print most recent logs? (y/n/c): ");
    lenStr = 10;  //WARNING! This assignment is required before each use because the result is returned using the same variable!
    if (getInput(&input, &lenStr, 0xFFFF, 0, 60000)) {  //no terminator, 60 second timeout
      resp = input[0];
      uint32_t logCnt = 0xFFFFFFFF;
      uint32_t logId = 0xFFFFFFFF;
      
      if ((resp == 'y') || (resp == 'Y')) {
        do {
          Serial.println("How many logs would you like to print?");
          haveResp = false;
          lenStr = 10;  //WARNING! This assignment is required before each use because the result is returned using the same variable!          
          if (getInput(&input, &lenStr, 0xFFFF, 0, 0)) {  //no terminator, no timeout
            logCnt = atol(input);
          }
          if (logCnt != 0xFFFFFFFF)
            haveResp = true;
        } while (!haveResp);

        uint32_t dummy, tmpAddr = lastLogAddr;
        uint32_t printed = 0;
        for (uint32_t i = 0; i < logCnt; i++) {
          if (printLog(tmpAddr))
            printed++;
          tmpAddr = tmpAddr - minLogSize;
          tmpAddr = wrapLogAddr(tmpAddr);
          if (i != (logCnt - 1)) { //don't need to do it the last time...
            if (!seekLogId(&tmpAddr, -1, &dummy)) {
              Serial.print("End of logs! Only ");
              Serial.print(printed);              
              Serial.println(" printed!");
              Serial.flush();
              break;
            }
          }
        }
        free(input);
        return;
      } else if ((resp == 'n') || (resp == 'N')) {
        do {
          Serial.println("What logId would you like to start with?");
          haveResp = false;
          lenStr = 10;  //WARNING! This assignment is required before each use because the result is returned using the same variable!          
          if (getInput(&input, &lenStr, 0xFFFF, 0, 0)) {  //no terminator, no timeout
            logId = atol(input);
          }
          if (logId != 0xFFFFFFFF)
            haveResp = true;
        } while (!haveResp);
        
        do {
          Serial.println("How many logs would you like to print?");
          haveResp = false;
          lenStr = 10;  //WARNING! This assignment is required before each use because the result is returned using the same variable!          
          if (getInput(&input, &lenStr, 0xFFFF, 0, 0)) {  //no terminator, no timeout
            logCnt = atol(input);
          }
          if (logCnt != 0xFFFFFFFF)
            haveResp = true;
        } while (!haveResp);
        
        uint32_t dummy, tmpAddr = logAddrFromId(logId);
        Serial.print("Beginning address: 0x");
        Serial.println(tmpAddr, HEX);
        Serial.flush();
        if (tmpAddr != 0xFFFFFFFF) {
          uint32_t printed = 0;
          for (uint32_t i = 0; i < logCnt; i++) {
            if (printLog(tmpAddr))
              printed++;
            tmpAddr = getNextLogAddr(tmpAddr);
            if (i != (logCnt - 1)) { //don't need to do it the last time...
              if (!readLogId(tmpAddr, &dummy)) {     //bool readLogId(uint32_t addr, uint32_t *logId);
                Serial.print("End of logs! Only ");
                Serial.print(printed);
                Serial.println(" printed!");
                Serial.flush();
                break;
              }
            }
          }
          free(input);
          return;
        } else {
          Serial.print("Error! LogId ");
          Serial.print(logId);
          Serial.println(" not found!");
          Serial.flush();
        }
      } else if ((resp == 'c') || (resp == 'C') || (resp == 'q') || (resp == 'Q') || (resp == 'e') || (resp == 'E')) {    //cancel, quit, or exit
        free(input);
        return;
      } else //some other unrecognized entry -- repeat loop
        retryInput = true;
      
    } else {
      //timeout...return
      free(input);
      return;
    }
  } while (retryInput);

}

// added by AIT 6-Mar-2020

void FlashTools::printData(){
  if ((lastLogAddr >= logSpace_end) && (firstLogAddr >= logSpace_end)) {
    Serial.println("ERROR: There are no logs!");
    return;
  }

  uint32_t logCnt = logIdFromAddr(lastLogAddr - 1);
  uint32_t dummy, tmpAddr = lastLogAddr;
  uint32_t printed = 0;
  for (uint32_t i = 0; i < logCnt; i++) {
    if (printLog(tmpAddr))
      printed++;
    tmpAddr = tmpAddr - minLogSize;
    tmpAddr = wrapLogAddr(tmpAddr);
    if (i != (logCnt - 1)) { //don't need to do it the last time...
      if (!seekLogId(&tmpAddr, -1, &dummy)) {
        Serial.print("End of logs! ");
        Serial.print(printed);              
        Serial.println(" printed!");
        Serial.flush();
        break;
      }
    }
  }
  
}



uint32_t FlashTools::firstLogAddress() {
  return firstLogAddr;
}

uint32_t FlashTools::lastLogAddress() {
  return lastLogAddr;
}

uint32_t FlashTools::nextLogAddress() {
  return nextLogAddr;
}

uint32_t FlashTools::blankSectorAddress() {
  return blankSectorAddr;
}

uint32_t FlashTools::logId() {
  return lastLogId + 1;
}

uint32_t FlashTools::logSpaceBeginAddress() {
  return logSpace_begin;
}

uint32_t FlashTools::logSpaceEndAddress() {
  return logSpace_end;
}

uint16_t FlashTools::minimumLogSize() {
  return minLogSize;
}

uint16_t FlashTools::maximumLogSize() {
  return maxLogSize;
}

// NOTE: DUE TO THE WAY THE FLASH ALGORITHM WORKS, IT IS NOT ALLOWED TO ERASE MIDDLE LOGS -- MUST ERASE STARTING AT OLDEST LOGS OR ERASE ALL LOGS ENTIRELY!
bool FlashTools::eraseLogs() {
  if (firstLogAddr <= flash_lastAddress) {

    // The logic below WAS from [firstLogAddr] to [lastLogAddr] however that DID NOT account for case where [lastLogAddr]
    // spilled over into next sector (because logs are allowed to traverse sector boundaries) -- this is an easy way to fix 
    // it (by using [nextLogAddr] instead) and at worst will result in erasing an already blank sector.
    if (firstLogAddr > nextLogAddr) {
      // WRAPPED LOGS SCENARIO
      // Here start at logSpace_begin and erase all sectors up to and including nextLogAddr
      // Then start at firstLogAddr, erase that sector and all sectors up to logSpace_end
      #ifdef TEXT_OUT
      Serial.println("Erasing sectors with logs (WRAPPED SCENARIO)...");
      #endif
      
      for (uint32_t a = logSpace_begin; a <= nextLogAddr; a += 0x1000) {
        if (!eraseSector(a, true)) {
          return false;
        }
      }
      for (uint32_t a = firstLogAddr; a < logSpace_end; a += 0x1000) {
        if (!eraseSector(a, true)) {
          return false;
        }
      }
    } else {
      // NON-WRAPPED LOGS SCENARIO
      // Here start at firstLogAddr, erase that sector and all sectors up to and including nextLogAddr            
      #ifdef TEXT_OUT
      Serial.println("Erasing sectors with logs (NON-WRAPPED SCENARIO)...");
      #endif
      for (uint32_t a = firstLogAddr; a <= nextLogAddr; a += 0x1000) {
        if (!eraseSector(a, true)) {
          return false;
        }
      }
    }
  } else {
    #ifdef TEXT_OUT
    Serial.println("Flash log space already blank!");
    #endif
  }  
  return true;
}



//Normalize Log Address:  Parameter "wrap" allows address to wrap: maps memory location 0x000000 to logSpace_begin
//                        will correct for misaligned "addr" parameter by adjusting it toward closest direction (tie favors ADDING to "addr")
void FlashTools::normLogAddr(uint32_t *addr, bool wrap) {
  uint8_t remainder;  //NOTE: type of remainder must be large enough to allow for a value of (minLogSize-1)

  //First check for alignment -- it MUST be evenly divisible by minLogSize, if not then correct to closest value that makes it evenly divisible
  remainder = *addr % minLogSize;
  if (remainder != 0) {
    if (remainder >= minLogSize/2) {
      *addr += (minLogSize-remainder);
    } else {
      *addr -= remainder;
    }
  }
  
  //Next check to see it is within log memory space:
  if (wrap) {
    *addr = wrapLogAddr(*addr);
  } else {
    //if wrap = false then truncate any outliers
    if (*addr > logSpace_end) {
      *addr = logSpace_end;
    }
  }

}


/*
 *    if (seekcode == -1):  will shift to a lower address 
 *    if (seekcode == 1):   will shift to a higher address 
 */
void FlashTools::shiftLogAddr(uint32_t *addr, int8_t seekCode, uint16_t shift) {
  if (seekCode <= 0) {
    *addr -= shift;
  } else {
    *addr += shift;
  }
  *addr = wrapLogAddr(*addr);
}


/*
 * Returns logSize from log header.
 * If log entry total size is unreasonable the function will return a size of zero.
 *
 */
uint16_t FlashTools::getLogSize(uint32_t logHeaderAddr) {
  uint16_t logSize;
  flash->readBytes(logHeaderAddr + LOG_logSizeOffset, &logSize, 2);

  if (logSize > maxLogSize)
    logSize = 0;

  return logSize;
}

uint16_t FlashTools::getLogMemorySize(uint32_t logHeaderAddr) {
  uint16_t logSize = getLogSize(logHeaderAddr);
  uint16_t logMemSize = 0;  
  if (logSize > 0) {
    uint16_t remainder = logSize % minLogSize;
    if (remainder != 0)
      logMemSize = (logSize + (minLogSize - remainder));
    else 
      logMemSize = logSize;
  }
  return logMemSize;
}

uint32_t FlashTools::getNextLogAddr(uint32_t logHeaderAddr) {
  uint32_t nextAddr = wrapLogAddr(logHeaderAddr + getLogMemorySize(logHeaderAddr));
  return nextAddr;
}

/*
 * Simply test to see if there are 15 consecutive bytes of 0xFF
 */
bool FlashTools::isBlank(uint32_t addr) {
  uint8_t buf[16];
  flash->readBytes(addr, buf, 16);
  for (uint8_t i = 0; i < 16; i++) {
    if (buf[i] != 0xFF)
      return false;
  }

  return true;
}




uint32_t FlashTools::logIdFromAddr(uint32_t address) {
  uint32_t logId;

  if (readLogId(address, &logId)) {
    return logId;
  } else 
    return 0xFFFFFFFF;
}




// Reads logId at addr parameter and compares it with next logId to ensure consecutive... if there isn't a 
// following log header then the memory should be blank and it checks to ensure that is the case, in this special
// case however it still must be verified and that is done by seeking left from original address to find a log
// header to verify there is a consecutive logId link.  Empty memory to the left also verifies.
bool FlashTools::readLogId(uint32_t addr, uint32_t *logId) {
  *logId = 0; //This must be done if there are any high order bytes because they may not be overwritten by flash read below!?
  flash->readBytes(addr, logId, 4);
  if (*logId > LOG_maxId) {
    //Disallowed values for logId -- return false
    return false;
  }

  uint32_t nxtAddr, nxtLogId=0;
  nxtAddr = getNextLogAddr(addr);

  if (nxtAddr == addr) {
    return false;
  }

  flash->readBytes(nxtAddr, &nxtLogId, 4);
  if (nxtLogId == *logId + 1) {
    return true;
  } else if (nxtLogId == 0xFFFFFFFF) {
    // bool nextIsBlank = verifyBlank(nxtAddr + 4);
    bool nextIsBlank = isBlank(nxtAddr);
    if (nextIsBlank) {
      // Now we need to verify we actually hit a log header via some other method -- seek left for a log header and then ensure consecutive with this one
      uint32_t addrToLeft = addr - minLogSize;
      uint32_t logIdToLeft;

      if (seekLogId(&addrToLeft, -1, &logIdToLeft)) {     // <--- may need to manually do this instead of calling it (because we've already got it called it once and on th stack)
        if (logIdToLeft + 1 == *logId) {
          // log header verified!          
          return true;
        } else {
          // Not log header          
          return false;
        }
      } else {
        // One more possible way of verifying is if the left takes us to blank memory (via wrap around)
        if (isBlank(addrToLeft)) {
          return true;
        } else {
          return false;
        }
      }
    }
    return nextIsBlank;
  } else {
    return false;
  }
}

/*
 * Tries to read log header at addr parameter and will modify logId if successful.
 * 
 * If memory space is blank it will return with false immediately.
 * If memory space is not blank but the address is misaligned with the header it will shift 
 * until it is able to find a valid log header OR until it has exhausted shifting as an option.
 * seekCode deterines which direction it will seek as follows:
 *    if (seekcode == -1):  will seek to a lower address 
 *    if (seekcode == 1):   will seek to a higher address  
 * 
 */
bool FlashTools::seekLogId(uint32_t *addr, int8_t seekCode, uint32_t *logId) {
  if (isBlank(*addr)) {
    *logId = 0xFFFFFFFF;
    return false;
  }  
  uint16_t cnt = 0;
  bool tryNorm = true;
  do {
    if (readLogId(*addr, logId)) {
      return true;
    }
    //Didn't work -- possibly not aligned to log header so shift according to seekCode    
    if (tryNorm) {
      //make one attempt to align by shifting addr so that it is aligned with memory sectors as headers should be
      tryNorm = false;
      normLogAddr(addr, true);
    } else {
      //now try to align by shifting by minLogSize as it may just be in the middle of a larger log header
      shiftLogAddr(addr, seekCode, minLogSize);
      cnt++;
    }

    if (isBlank(*addr)) { //because we've shifted, we have to check this again...      
      *logId = 0xFFFFFFFF;
      return false;
    }

  } while (cnt <= maxLogSize / minLogSize);
  *logId = 0xFFFFFFFF;
  return false;
}

/*
 * Adjusts log address to be within logSpace...instead of wrapping it truncates.
 */
uint32_t FlashTools::truncLogAddr(uint32_t addr) {
  if (addr >= logSpace_end) {
    return (logSpace_end - minLogSize); //Here we back it off from the boundary by minLogSize because the address at logSpace_end is NOT writeable
  }
  if (addr < logSpace_begin)
    return logSpace_begin;
  return addr;  
}

/*
 * Wrap boundaries on left and right extremes.
 */
uint32_t FlashTools::wrapLogAddr(uint32_t addr) {
  uint32_t result = addr;
  bool neededAdj;
  do {

    if (result >= logSpace_end) {
      result = (result - logSpace_end) + logSpace_begin;
      neededAdj = true;
    } else if (result < logSpace_begin) {
      result = logSpace_end - (logSpace_begin - result);
      neededAdj = true;
    } else 
      neededAdj = false;
  } while (neededAdj);

  return result;
}

void FlashTools::logInit() {
  //initialize parameters with 0xFFFFFFFF so that we can measure progress
  firstLogAddr = 0xFFFFFFFF;
  lastLogAddr = 0xFFFFFFFF;
  nextLogAddr = 0xFFFFFFFF;
  blankSectorAddr = 0xFFFFFFFF;
}


/*
 * WARNING! Using msSize that is not a power of 2 creates an issue where msOffset may not be aligned to log header...
 *          THIS MUST BE CORRECTED using "normLogAddr" method to realign to log headers.
 */
uint32_t FlashTools::initMSSize(uint32_t addr, int8_t sign) {
  if (sign > 0) {
    //exclude addresses lower than current location from search
    return logSpace_end - addr;
  } else {
    //exclude addresses higher than current location from search
    return addr - logSpace_begin;
  }
}



/*
 * This function searches for a blank section of memory (tested positive by 15 consecutive bytes of 0xFF).  It does not attempt to seek to 
 * either edge or the center of the blank space, it simply returns true at the moment that blank space was detected.  The parameter "addr"
 * is modified so the result is returned by the pointer parameter.
 */
bool FlashTools::binSearchForBlank(uint32_t *addr) {
  uint32_t msSize, msOffset, idFarL, idFarR, tmpId, lastId;
  uint8_t maxTries = LOG_maxTries;
  int8_t sign;  
  
  *addr = logSpace_end - minLogSize;
  if (!seekLogId(addr, -1, &idFarR)) {
    //Couldn't find logId at very end?  Is the address blank?
    return isBlank(*addr);  //addr is shifted during seekLogId call so if we were right next to blank memory then addr will now be pointing to it
  }

  *addr = logSpace_begin;
  if (!seekLogId(addr, 1, &idFarL)) {
    //Couldn't find logId at very beginning?  Perhaps we were in the middle of a larger log that had wrapped and just to the left of blank memory... 
    return isBlank(*addr);  //addr is shifted during seekLogId call so if we were right next to blank memory then addr will now be pointing to it
  }


  lastId = idFarL;
  msOffset = *addr;
  msSize = initMSSize(*addr, 1);
  sign = 1; //start out seeking in right (increasing) direction
  
  do {
    if (msSize > minLogSize) {
      msSize /= 2;
    }
    if ((sign == -1) && (msSize>msOffset))
      msOffset = (msOffset + logSpace_end - logSpace_begin) - msSize;    //This must be done to bypass negative values (using type uint32_t)
    else 
      msOffset += sign*msSize;
    *addr = msOffset;
    normLogAddr(addr, false);  //to keep it within the correct range (no wrapping) AND to ensure it is aligned with log header
    
    if (!isBlank(*addr)) {
      if (seekLogId(addr, sign, &tmpId)) {  //use "sign" here so that if logSize is greater than msSize that it will seek through the log anyway and not get stuck here
        if (sign > 0) {
          //Searching left to right, for normal consecutive numbering, right logIds should be greater than left logIds
          if (tmpId > lastId) {  //means our target (left_GT_right) has not yet been detected
            //keep searching same direction
          } else {
            //shift in consecutive numbering detected -- switch direction to narrow in on it
            sign = -1;
          }
          lastId = tmpId;  
        } else {
          //Searching right to left, for normal consecutive numbering, right logIds should be greater than left logIds
          if (tmpId < lastId) {  //means our target (left_GT_right) has not yet been detected
            //keep searching same direction
          } else {
            //shift in consecutive numbering detected -- switch direction to narrow in on it
            sign = 1;
          }
          lastId = tmpId;
        }
      } else { //seekLogId returned false
        //There is a chance that the call to seekLogId read through to blank memory so need to check just one more time.
        if (isBlank(*addr))
          return true;
      }
    } else {  
      //"addr" in blank memory area -- return successful      
      return true;
    }
    
    maxTries--;
  } while (maxTries > 0);
  
  return false;
}



bool FlashTools::is_EmptyToData_Boundary(uint32_t addr) {  
  // determine if sector to left is empty
  // verify current address is NOT empty
  bool result = false;
  result = isBlank(wrapLogAddr((flash_sectorMask & addr) - flash_sectorSize));
  if (result) {
    result = !isBlank(addr);
  }
  return result;
}


/*
 * Searches for the transition or boundary from blank memory to data.
 * The result is always reported in terms of the log header's address (which since the logs
 * can be variable length may not always be the same distance from the blank memory).  Also, because 
 * logs can span over sector boundaries it is possible that there is non-blank (but garbage) data
 * at the transition from blank memory to data (seeking higher addresses).  This function will 
 * report the log header address and NOT the precise moment of transition from blank memory to data.
 * 
 * If the addr parameter points to an address that is not blank, the binSearch algorithm will first 
 * locate the blank sector using the logIds.  Once the blank sector is found it will start from 
 * inside to search for the log boundary; which direction is determined by the sign parameter.
 * 
 * This search does not wrap
 * 
 * sign parameter:
 *  -1: seek left (decreasing memory address)
 *  +1: seek right (increasing memory address)
 */
bool FlashTools::binSearch(uint32_t *addr, int8_t sign) {
  uint32_t msSize, msOffset, initAddr, logId;
  int8_t initSign;
  uint8_t maxTries = LOG_maxTries;
  bool foundTarget = false;
  
  initSign = sign;
  if (!isBlank(*addr)) {
    //addr parameter not pointing to blank sector -- must find blank sector first!
    if (binSearchForBlank(addr)) {
      //Now that blank memory space is found, we can search for the specified edge using the code below
      //After completion of finding blank sector "initAddr" must be set to that location (happens below)
    } else {
      //Could not find blank memory!!!
      return false;
    }
  }
    
  initAddr = *addr; //This is true for all cases (addr modified when searching for blank space above included)
  msOffset = *addr;
  msSize = initMSSize(*addr, sign);

  do {
    if (!isBlank(*addr)) {
      if ((is_EmptyToData_Boundary(*addr)) && (initSign == 1)) {
        //Now fine tune if needed by seeking from end of blank sector through left-over data until a log header is found
        // *addr = ((flash_sectorMask & *addr) - flash_sectorSize) + minLogSize;
        *addr = (flash_sectorMask & *addr);
        seekLogAddr(addr, 1);     // Added 27-Feb-2020 to ensure that the log header is reported and not simply the address of transition from empty memory to data
        return true;
      }

      if (seekLogId(addr, sign, &logId)) {  //use "sign" here so that if logSize is greater than msSize that it will seek through the log anyway and not get stuck here
        if (initSign < 0) {
          //SEEKING LEFT BOUNDARY
          if (isBlank(getNextLogAddr(*addr))) {
            //Successfully located a left boundary
            return true;
          } else {
            sign = 1; //ensure direction is going right toward blank sector (because we've gone past the left boundary)
          }
        } else {
          //SEEKING RIGHT BOUNDARY
          if (isBlank(*addr - minLogSize)) {
            //Successfully located a right boundary

            //Now fine tune if needed by seeking through left-over data until a log header is found
            seekLogAddr(addr, 1);     // Added 27-Feb-2020 to ensure that the log header is reported and not simply the address of transition from empty memory to data
            return true;
          } else {
            sign = -1; //ensure direction is going left toward blank sector (because we've gone past the right boundary)
          }
        }
      } else {        
        if (isBlank(*addr)) {  
          //Call to seekLogId read through to blank memory so need to fine-tune boundary seeking the other direction
          sign = -sign;
        }
      }
    } else {
      //In blank memory area seeking logSpace
      if (initSign < 0) {
        //seeking left boundary
        sign = -1;
      } else {
        //seeking right boundary
        sign = 1;
      }
    }
    
    if (msSize > minLogSize) {
      msSize /= 2;
    }
    if ((sign == -1) && (msSize > msOffset))
      msOffset = (msOffset + logSpace_end - logSpace_begin) - msSize;    //This must be done to bypass negative values (using type uint32_t)
    else 
      msOffset += sign * msSize;
    *addr = msOffset;
    normLogAddr(addr, false);  //to keep it within the correct range (no wrapping) AND to ensure it is aligned with log header
    
    maxTries--;
  } while (maxTries > 0);
  return false;
}

/*
 * TYPE 1 algorithm:
 * Right bound of empty memory is logSpace_begin BUT firstLogAddr may not always correspond with empty memory bound!  Must seek right from logSpace_begin to locate firstLogAddr.
 */
bool FlashTools::getType1Bounds(uint32_t *addrL, uint32_t *addrR) {
  bool success;
  *addrR = logSpace_begin;  
  success = seekLogAddr(addrR, 1);      // If addrR is not a log header, seek right to find firstLogAddr
  if (success) {
    *addrL = logSpace_end - minLogSize;  //start out seeking left from end of memory space -- must not start at logSpace_end because that address cannot be read (wraps to logSpace_begin)
    success = binSearch(addrL, -1);
  }
  return success;
}

/*
 * TYPE 2 algorithm:
 * For determining left bound of blank memory, start at logSpace_end and binSearch left for logId; for determining right
 * bound start at logSpace_begin and binSearch right for logId
 */
bool FlashTools::getType2Bounds(uint32_t *addrL, uint32_t *addrR) {
  bool success;
  *addrR = logSpace_begin;
  *addrL = logSpace_end - minLogSize;  //start out seeking left from end of memory space -- must not start at logSpace_end because that address cannot be read (wraps to logSpace_begin)
  success = binSearch(addrR, 1);
  if (success) {
    success = binSearch(addrL, -1);
  }
  return success;
}

/*
 * TYPE 3 algorithm:
 * Left bound of empty memory (lastLogAddr) is known by calling seekLogId going left from (logSpace_end - minLogSize); 
 * binSearch right from logSpace_begin to determine right bound 
 */
bool FlashTools::getType3Bounds(uint32_t *addrL, uint32_t *addrR) {
  bool success;
  *addrL = logSpace_end - minLogSize;
  *addrR = logSpace_begin;
  success = seekLogAddr(addrL, -1);    //bool seekLogId(addrL, -1, &dummy);  27-Feb-2020: Using method [seekLogAddr] as it does the same thing
  if (success) {
    success = binSearch(addrR, 1);
  }
  return success;
}

/*
 * TYPE 4 algorithm:
 * Seek starting from left extreme of memory space and use logId of logs to detemine if blank sector was skipped 
 * (logIds on left of blank memory will be LARGER than logIds on right of memory space ...UNLESS the logId overflowed).
 * 
 * binSearch has been modified to be able to seek for empty memory and then search for the bound specified.  
 * Use the result from the first call to seek for the other bound.
 */
bool FlashTools::getType4Bounds(uint32_t *addrL, uint32_t *addrR) {
  bool success;
  *addrR = logSpace_begin;
  success = binSearch(addrR, 1);
  if (success) {
    *addrL = *addrR - minLogSize;
    success = binSearch(addrL, -1);
  }
  return success;
}

/*
 * Boundary Search:
 * Seeks for the boundaries between empty memory and data.
 * 
 * Three main scenarios of flash memory usage:
 * 1. INITIAL (entire log space blank)
 * 2. FILLING (at least one log entry with data starting at the beginning of log space; more than one blank sector)
 * 3. CIRCULAR BUFFER (the remainder (if any) of the sector being written to is blank as well as one full
 *    empty sector marking the divide between lastLogAddr and firstLogAddr).
 * 
 * See illustrations below:
 * KEY:   F = firstLogAddr;   L = lastLogAddr;   N = nextLogAddr;   B = firstBlankSector
 *        . = empty sector;   # = data sector;   ~ = empty OR partial data;
 * 
 * TYPE 0 algorithm
 * INITIAL: (log memory space unused -- all sectors empty)
 * ----------------------------------------------------------------------------------------------------------
 * ..........................................................................................................
 * ----------------------------------------------------------------------------------------------------------
 * ^^
 * NB                                                                                       (F,L not defined)
 * 
 * 
 * TYPE 1 algorithm
 * FILLING: (at least one log entry with the logs at beginning of memory space; more than one blank sector)
 * ----------------------------------------------------------------------------------------------------------
 * #~........................................................................................................
 * ----------------------------------------------------------------------------------------------------------
 * ^^^
 * FNB
 * L
 * 
 * TYPE 1 algorithm
 * FILLING: (second example; most common scenario with a new logger)
 * ----------------------------------------------------------------------------------------------------------
 * #################################################################################################~········
 * ----------------------------------------------------------------------------------------------------------
 * ^                                                                                               ^^^
 * F                                                                                               LNB
 * 
 * 
 * TYPE 1 algorithm
 * CIRCULAR BUFFER: (the remainder of the writing sector is empty as well as one full empty sector)
 * ----------------------------------------------------------------------------------------------------------
 * ########################################################################################################~·
 * ----------------------------------------------------------------------------------------------------------
 * ^                                                                                                      ^^^
 * F                                                                                                      LNB
 * 
 * 
 * TYPE 2 algorithm
 * CIRCULAR BUFFER: (second example)
 * ----------------------------------------------------------------------------------------------------------
 * .########################################################################################################~
 * ----------------------------------------------------------------------------------------------------------
 * ^^                                                                                                      ^^
 * BF                                                                                                      LN
 * 
 * 
 * TYPE 3 algorithm
 * CIRCULAR BUFFER: (third example)
 * ----------------------------------------------------------------------------------------------------------
 * ~.########################################################################################################
 * ----------------------------------------------------------------------------------------------------------
 * ^^^                                                                                                      ^
 * NBF                                                                                                      L
 * 
 * 
 * TYPE 4 algorithm
 * CIRCULAR BUFFER: (fourth example; most common scenario with an established logger)
 * ----------------------------------------------------------------------------------------------------------
 * #################~.#######################################################################################
 * ---------------------------------------------------------------------------------------------------------- 
 *                 ^^^^
 *                 LNBF
 * 
 * 
 * 
 * Boundary determined by transition from empty memory to data or from data to empty -- this must be 
 * determined down to the smallest log size.
 * 
 * Special consideration:
 * a. If blank memory found at beginning of log space AND found at higher address near halfway point
 *    then the entire log space must be EMPTY (this is the initial state).
 * b. If log(s) found at beginning of log space AND very end of log space is blank AND more than one blank 
 *    sector before the end then the memory is in a FILLING condition.
 * c. If log(s) found at beginning of log space AND at the very end then it must be a CIRCULAR BUFFER condition
 *    with the blank sector somewhere in between.
 * 
 * Procedure:
 * 1. Determine if special consideration (a) above applies and if it does return false with addr parameter set 
 *    to beginning of log space.
 * 2. Test for memory space form (TYPE 1, TYPE 2, TYPE 3, or TYPE 4) by trying to read LogId at left and 
 *    right extremes and at middle of the memory space.
 * 3. Use algorithm that is tuned for the TYPE determined in step 2 above.
 *    
 *    TYPE 0 algorithm:
 *    Log memory space is empty.  Test beginning address and one point halfway through; both must be blank.
 *
 *    TYPE 1 algorithm:
 *    Right bound of empty memory is known immediately; to determine left bound, begin seeking from end of memory space.
 *    
 *    TYPE 2 algorithm:
 *    For determining left bound of blank memory, start at right extreme and seek left for logId; for determining right
 *    bound start at left extreme and seek right for logId
 *    
 *    TYPE 3 algorithm:
 *    Left bound of empty memory is known immediately; seek right to determine right bound
 * 
 *    TYPE 4 algorithm:
 *    Seek starting from left extreme of memory space and use logId of logs to detemine if blank sector was skipped 
 *    over (logIds on left of blank memory will be LARGER than logIds on right of memory space).
 * 
 */
bool FlashTools::boundSearch(uint32_t *addrL, uint32_t *addrR) {
  uint32_t maxId, minId;
  uint8_t formType;
  int8_t sign = 1;  //either +1 or -1 (used to determine direction of binary search)
  bool foundTarget = false, seekBlank;

  //Step 1: Test for Initial/Blank condition (TYPE 0)
  if ((isBlank(logSpace_begin)) && (isBlank(logSpace_begin + ((logSpace_end - logSpace_begin) / 2)))) {
    *addrL = logSpace_begin;
    *addrR = logSpace_begin;
    return true;
  }
  //Step 2: Determine shape of memory space (TYPE 1, TYPE 2, TYPE 3, or TYPE 4 as outlined in comment header above)
  const uint32_t undef = 0xFFFFFFFF;
  *addrL = logSpace_begin;
  if (seekLogId(addrL, -1, &minId)) {
    // found header
  } else {
    // check for condition where logs have wrapped and logId is not available EXACTLY at logSpace_begin address    
    *addrL = logSpace_begin;
    if (seekLogId(addrL, 1, &minId)) {
      // found header to right of addrL
    } else
      minId = undef;
  }

  *addrL = logSpace_end - minLogSize;
  if (seekLogId(addrL, -1, &maxId)) {
    //don't need to do anything?
  } else {
    maxId = undef;
  }
  if ((minId < undef) && (maxId == undef)) {
    //This is TYPE 1
    formType = 1;
  } else if ((minId == undef) && (maxId == undef)) {  //Don't need to sample middle point because TYPE 0 would have been determined in Step 1
    formType = 2;
  } else if ((minId == undef) && (maxId < undef)) {
    formType = 3;
  } else {  //This is the only possibility left so don't need to check for it
    formType = 4;
    //reset logId holders as they will be used to determine where blank sector is for TYPE 4
    maxId = 0x00000000;
    minId = 0xFFFFFFFF;
  }

  //Step 3: Use specialized algorithm for each memory form type
  switch (formType) {
    case 1:
      return getType1Bounds(addrL, addrR);
    case 2:
      return getType2Bounds(addrL, addrR);
    case 3:
      return getType3Bounds(addrL, addrR);
    case 4:
      return getType4Bounds(addrL, addrR);
    default:
      //Error, should not ever arrive here.
      return false;
  }
}


bool FlashTools::logSetup(uint32_t logSpace_begin, uint32_t logSpace_end, uint16_t minLogSize, uint16_t maxLogSize, bool printReport) {
  bool success = true;

  if (!wait(5000)) //wait a maximum of 5 seconds for any pending flash operations to finish
    success = false;

  if (success) {
    // set private values that define logspace
    this->logSpace_begin = logSpace_begin;
    this->logSpace_end = logSpace_end;
    this->minLogSize = minLogSize;
    this->maxLogSize = maxLogSize;

    cli();  //clear interrupts to avoid being interrupted during logSetup
    
    logInit();
    uint32_t addrL, addrR;
    success = boundSearch(&addrL, &addrR);
    if (success) {
      lastLogAddr = addrL;
      firstLogAddr = addrR;
      nextLogAddr = getNextLogAddr(lastLogAddr);
      if ((nextLogAddr & flash_sectorMask) == nextLogAddr)
        blankSectorAddr = nextLogAddr;
      else 
        blankSectorAddr = (nextLogAddr + flash_sectorSize) & flash_sectorMask;

      blankSectorAddr = wrapLogAddr(blankSectorAddr);
      
      if (lastLogAddr >= flash_totalSize) {   //Note: This only happens when flash is empty or cannot be determined    27-Jul-2019: changed from 0xFFFFFF
        lastLogId = 0;
      } else {
        seekLogId(&lastLogAddr, 1, &lastLogId);        
      }
    } else {
      if ((addrL == logSpace_begin) && (addrR == logSpace_begin)) {
        //This means empty memory!
        lastLogAddr = flash_totalSize;
        firstLogAddr = flash_totalSize;
        nextLogAddr = addrL;
        blankSectorAddr = addrL + flash_sectorSize;
        lastLogId = 0xFFFFFFFF;  //This setting alows the next entry to overflow the uint32_t and start at zero for the first log entry                
      }
    }

    sei();  //re-enable interrupts
  }

  if ((printReport) && (success)) {
    uint32_t beginLogs = logSpaceBeginAddress();
    uint32_t endLogs = logSpaceEndAddress();
    uint32_t logSpaceB = endLogs - beginLogs;
    uint16_t sectorsAvail = logSpaceB / 0x1000;
    // uint16_t logSpaceKB = logSpaceB / 1000;

    Serial.println("\nLog setup successful!");
    Serial.println();
//    Serial.print("Log space begin address: 0x");
//    Serial.println(beginLogs, HEX);
//    Serial.print("Log space end address: 0x");
//    Serial.println(endLogs, HEX);
//    Serial.print("Sectors available to log space: ");
//    Serial.print(sectorsAvail);
//    Serial.print(" (");
//    Serial.print(logSpaceB);
//    Serial.println(" bytes)");
    // Serial.print(logSpaceKB);
    // Serial.println(" KB)");

    uint32_t addrFL = firstLogAddress();
    uint32_t logIdFL = logIdFromAddr(addrFL);

    uint32_t addrLL = lastLogAddress();
    uint32_t logIdLL = logIdFromAddr(addrLL);

    uint32_t addrNL = nextLogAddress();
    uint32_t logIdNL = logId();


    /*Serial.print("First log address: 0x");
    Serial.print(addrFL, HEX);
    Serial.print("; log ID: ");
    (logIdFL != 0xFFFFFFFF) ? Serial.println(logIdFL) : Serial.println("--");
    

    Serial.print("Last log address: 0x");
    Serial.print(addrLL, HEX);
    Serial.print("; log ID: ");
    (logIdLL != 0xFFFFFFFF) ? Serial.println(logIdLL) : Serial.println("--");

    Serial.print("Next log address: 0x");
    Serial.print(addrNL, HEX);
    Serial.print("; next log ID: ");
    Serial.println(logIdNL);

    Serial.print("Blank sector address: 0x");
    Serial.println(blankSectorAddress(), HEX);*/
    Serial.flush();
  } else if (printReport) {
    Serial.println("Flash logSetup method failed!");
  }

  return success;
}





// v v v   flash Direct Memory Access (DMA) menu and supporting functions   v v v

void printDMAmenu();
bool seekFlash(SPIFlash *flash, uint32_t* beginAddr, uint8_t targetVal, bool notOp, int16_t stepSize);
bool seekFlash(SPIFlash *flash, uint32_t* beginAddr, uint32_t endAddr, uint8_t targetVal, bool notOp, int16_t stepSize);
uint16_t testAndRemove(char* text, char testChar);
uint16_t reportBytes(SPIFlash *flash, uint32_t addr, uint16_t count, bool addrDisplay);
uint16_t flashWriteBytes(SPIFlash *flash, uint32_t addr, uint8_t value, uint16_t count);
bool getCmdArgs(char* cmd, char* arg1, char* arg2, char* arg3, char* arg4);





void FlashTools::dmaMenu() {  
  while (Serial.available()) {
    Serial.read();                                  //Clear the input buffer
  }

  Serial.flush();                                   //Clear output buffer

  Serial.println("\r\n\r\nDIRECT MEMORY ACCESS MENU");
  Serial.println("1.0");  
  Serial.println("\r\nWARNING!\r\nDo not use the commands below unless you know what you are doing!");  
  Serial.println("\r\n");
  Serial.println("Flash Memory Information:");
  Serial.println("Sector size:        0x1000 (4096) bytes");
  Serial.println("Number of sectors:  2048 (8 MB total memory)");
  Serial.println("Address range:      0x000000 - 0x7FFFFF");
  Serial.println("\r\n");
  printDMAmenu();

  bool tmpFlag1, tmpFlag2;
  uint8_t val8;
  uint16_t val16;
  uint32_t addr1 = 0;
  uint32_t addr2 = 0;
  uint16_t inputLen = 30;   //30 byte array should be more than enough
  char* input = (char*)malloc(inputLen);
  char *arg1, *arg2, *arg3, *arg4;
  char* tmpStr;

  arg1 = (char*)malloc(10);
  arg2 = (char*)malloc(10);
  arg3 = (char*)malloc(10);
  arg4 = (char*)malloc(10);
  tmpStr = (char*)malloc(33);

  bool dmamLoop = true;
  while (dmamLoop) {    //Here we have software send commands to seek for logs

    inputLen = 30;
    if (getInput(&input, &inputLen, 0xFFFF, 0, 0)) {

      Serial.println(input);

      getCmdArgs(input, arg1, arg2, arg3, arg4);

      // First argument is ALWAYS an address so do it here:
      if (strlen(arg1) > 0) {
        addr1 = strtol(arg1, nullptr, 0); // <--- Use 0 as base and it will decode based on string entered (need to precede hex strings with 0x)
      } else {
        addr1 = 0;
      }

      tmpFlag1 = false;
      tmpFlag2 = false;      
      //Now input == cmd so switch and execute
      switch (*(input + 0)) {
        case 'x':
        case 'X':
          dmamLoop = false;            
          break;

        case 's':
          // This version specifies two addresses -- the seek happens between the addresses
          if (strlen(arg2) > 0) {
            addr2 = strtol(arg2, nullptr, 0); // <--- Use 0 as base and it will decode based on string entered (need to precede hex strings with 0x)
          } else {
            addr2 = 0xFFFFFFFF;               // This is the undefined case -- seekFlash function will determine either upper or lower bound automatically (based on seek direction)
          }

          if (strlen(arg3) > 0) {
            tmpFlag1 = (testAndRemove(arg3, '!') > 0);
            val8 = strtol(arg3, nullptr, 0); // <--- Use 0 as base and it will decode based on string entered (need to precede hex strings with 0x)
          } else {
            //default is seek for empty flash:
            val8 = 255;  
            tmpFlag1 = true;
          }

          if (strlen(arg4) > 0) {
            val16 = strtol(arg4, nullptr, 0);   // I am using an unsigned 16-bit integer here to store a signed value but it works out (as long as the step size is smaller than 32,768 which should always be the case)
          } else {
            val16 = 1;
          }

          if (seekFlash(flash, &addr1, addr2, val8, tmpFlag1, val16)) {
            Serial.print("0x");
            Serial.println(ltoa(addr1, tmpStr, 16));
          } else 
            Serial.println("FAILED!");
          break;

        case 'S':
          // This version only has a starting address
          if (strlen(arg2) > 0) {
            tmpFlag1 = (testAndRemove(arg2, '!') > 0);
            val8 = strtol(arg2, nullptr, 0); // <--- Use 0 as base and it will decode based on string entered (need to precede hex strings with 0x)
          } else {
            //default is seek for empty flash:
            val8 = 255;  
            tmpFlag1 = true;
          }

          if (strlen(arg3) > 0) {
            val16 = strtol(arg3, nullptr, 0);   // I am using an unsigned 16-bit integer here to store a signed value but it works out (as long as the step size is smaller than 32,768 which should always be the case)
          } else {
            val16 = 1;
          }

          if (seekFlash(flash, &addr1, val8, tmpFlag1, val16)) {
            Serial.print("0x");
            Serial.println(ltoa(addr1, tmpStr, 16));
          } else 
            Serial.println("FAILED!");
          break;

        case 'r':
          tmpFlag1 = true;
        case 'R':
          if (strlen(arg2) > 0) {
            val16 = strtol(arg2, nullptr, 0);
          } else {
            val16 = 1;
          }

          if (val16 != reportBytes(flash, addr1, val16, !tmpFlag1)) {
            Serial.print("FAILED!");    //because we didn't get as many bytes as requested (ran into memory boundary)
          }
          Serial.print((char)3);        //ETX CR LF to terminate long passages of text
          Serial.print("\r\n");
          break;

        case 'e':
        case 'E':
          eraseSector(addr1, true);
          Serial.println("DONE.");
          break;

        case 'w':
        case 'W':
          if (strlen(arg2) > 0) {            
            val8 = strtol(arg2, nullptr, 0); // <--- Use 0 as base and it will decode based on string entered (need to precede hex strings with 0x)
          } else {            
            val8 = 0;
          }

          if (strlen(arg3) > 0) {
            val16 = strtol(arg3, nullptr, 0);
          } else {
            val16 = 1;
          }

          if (val16 != flashWriteBytes(flash, addr1, val8, val16)) {   //uint16_t flashWriteBytes(uint32_t addr, uint8_t value, uint16_t count) 
            Serial.print("FAILED!");
          } else {
            Serial.print("DONE.");
          }
          break;

        default:
          Serial.println("\r\n\r\n");
          printDMAmenu();
          break;
      }
      
    }
  }
  
  free(arg1);
  free(arg2);
  free(arg3);
  free(arg4);
  free(tmpStr);

  free(input);
  
  Serial.println("DIRECT MEMORY ACCESS MODE TERMINATED");
  delay(1000);  
}


void printDMAmenu() {
  Serial.println("Supported Commands:");  
  Serial.println("WARNING! Seek commands can take as long as 12 minutes to complete!");
  Serial.println("S<n>,[!]<x>,<t>......Seek for value x (or NOT x), begin at n with step");
  Serial.println("                     size t (optional, signed int, sign is direction).");
  Serial.println("s<m>,<n>,[!]<x>,<t>..As above but specify start and end address m & n.");  
  Serial.println("r<n>,<x>.............Report x bytes of memory beginning at n.");
  Serial.println("R<n>,<x>.............As above but include address tag for each value.");
  Serial.println("E<n>.................Erase sector containing address n.");
  Serial.println("\nNOTE: Flash write requires that you first erase the sector!");
  Serial.println("W<n>,<x>.............Write value x to memory at address n.");
  Serial.println("W<n>,<x>,<t>.........As above but repeat t times.");  
  Serial.println("X....................Exit direct memory access mode and continue boot");
  Serial.println("NOTE: Base-16 (hexadecimal) numbers supported; use prefix \"0x\".");
  Serial.print((char)1);  //SOH for notification to UI software
  Serial.print("\r\n\r\nEnter Command: ");
  Serial.flush();
}


// Seeks for targetVal (or seeks for anything but targetVal using notOp param) beginning at beginAddr, stepSize determines resolution
// First positive result is returned via beginAddr
bool seekFlash(SPIFlash *flash, uint32_t* beginAddr, uint8_t targetVal, bool notOp, int16_t stepSize) {
  return seekFlash(flash, beginAddr, 0xFFFFFFFF, targetVal, notOp, stepSize);
}

// Seeks for targetVal (or seeks for anything but targetVal using notOp param) beginning at beginAddr and ending at endAddr, stepSize determines resolution.
// First positive result is returned via beginAddr
bool seekFlash(SPIFlash *flash, uint32_t* beginAddr, uint32_t endAddr, uint8_t targetVal, bool notOp, int16_t stepSize) {
  uint8_t val;
  // vvv INITIALIZE PARAMS vvv
  if (stepSize == 0)
    stepSize = 1;

  if (*beginAddr > flash_lastAddress)
    *beginAddr = 0;

  // Fix direction (in case it was not entered or was entered incorrectly)
  if (endAddr <= flash_lastAddress) {
    if (*beginAddr > endAddr) {
      if (stepSize > 0)
        stepSize = -stepSize;
    } else {
      // This option overrides a user input -- step size WAS SPECIFIED NEGATIVE but the address range specified that step should be positive
      if (stepSize < 0)
        stepSize = -stepSize;
    }
  } else {
    // Set endAddr as it was not specified
    if (stepSize < 0) {
      endAddr = 0;
    } else {
      endAddr = flash_lastAddress;
    }
  }
  // ^^^ INITIALIZE PARAMS ^^^
  
  if (stepSize > 0) {
    // Seeking toward flash end (stop at specified endAddr)
    while (*beginAddr <= endAddr) {
      val = flash->readByte(*beginAddr);
      
      if (notOp) {
        if (val != targetVal)
          return true;
      } else {
        if (val == targetVal)
          return true;
      }
      (*beginAddr) = (*beginAddr) + stepSize;
    }
  } else {
    // Seeking toward flash beginning (stop at specified endAddr)
    while (*beginAddr >= endAddr) {
      val = flash->readByte(*beginAddr);
      
      if (notOp) {
        if (val != targetVal)
          return true;
      } else {
        if (val == targetVal)
          return true;
      }
      if (*beginAddr > endAddr) {
        (*beginAddr) = (*beginAddr) + stepSize;     // Keep in mind that stepSize is a negative number in this case
      } else 
        return false; //This is necessary to catch special case (because unsigned cannot go negative)
    }    
  }

  return false;
}



//This both tests for testChar param as well as removes the char from the string so that it can be passed to atoi(), strtol(), or similar.
uint16_t testAndRemove(char* text, char testChar) {
  uint16_t result = 0;
  uint8_t len;
  len = strlen(text);

  for (int i = 0; i < len; i++) {
    if (*(text + i) == testChar) {
      result++;
      *(text + i) = ' ';  //replace with space (so that number can be extracted)
      // Serial.print("Found, index: ");
      // Serial.println(i);
    }
  }
  // Serial.print("Result: ");
  // Serial.print(result);    
  // Serial.print("; text: ");
  // Serial.println(text);
  return result;
}

uint16_t reportBytes(SPIFlash *flash, uint32_t addr, uint16_t count, bool addrDisplay) {
  uint8_t val;  
  uint32_t lastAddr = addr + count - 1;
  char* tmpStr = (char*)malloc(33);
  if (lastAddr > flash_lastAddress)
    lastAddr = flash_lastAddress;
  count = 0;  //reuse param variable to track actual count and report result
  while (addr <= lastAddr) {
    val = flash->readByte(addr);
    if (addrDisplay) {
      Serial.print("0x");
      Serial.print(ltoa(addr, tmpStr, 16));
      Serial.print(": ");
    }
    Serial.println(val);
    addr++;
    count++;
    Serial.flush();
    //delay(10);    //This is required for slower applications that cannot keep up with the port
  }
  free(tmpStr);
  return count;
}

uint16_t flashWriteBytes(SPIFlash *flash, uint32_t addr, uint8_t value, uint16_t count) {
  uint32_t lastAddr = addr + count - 1;  
  if (lastAddr > flash_lastAddress)
    lastAddr = flash_lastAddress;
  count = 0;  //reuse param variable to track actual count and report result
  while (addr <= lastAddr) {
    // Could add flash busy check here but don't want it to cause an infinite loop if flash has a problem... the following will wait for 1500 milliseconds max if needed
    uint8_t cnt = 0;
    while ((flash->busy()) && (cnt < 30)) {
      cnt++;
      delay(50);
    }
    if (cnt >= 30)
      break;
    
    flash->writeByte(addr, value);
    addr++;
    count++;
  }
  return count;
}


bool getCmdArgs(char* cmd, char* arg1, char* arg2, char* arg3, char* arg4) {
  bool success = false;
  uint8_t len = strlen(cmd);
  
  char cmdResult; //need a temporary spot to store cmd character while we process full cmd input string
  char rChar;     //the read character
  //char cmdResult = *(cmd + 0);   //index zero is command character BUT it isn't always guaranteed -- what if leading space?
  //default all args to empty
  *arg1 = 0;
  *arg2 = 0;
  *arg3 = 0;
  *arg4 = 0;


  uint8_t pIdx = 0;        //parameter index -- which parameter we're gathering currently
  uint8_t tIdx = 0;        //target index to keep track of where we're writing within the arg
  uint8_t idx = 0;         //index for reading the cmd input string
  while (idx < len) {
    rChar = *(cmd + idx);    
    if ((rChar != ' ') && (rChar != ',')) {      
      switch (pIdx) {        
        case 0:          
          cmdResult = rChar;
          *(cmd + idx) = 0; //to empty the remainder of the cmd string
          //Because this is always a single character, we don't set tIdx and we increment pIdx directly
          pIdx++;
          //Some commands don't have any args.  Set success true, if error situation arises set false again later.
          success = true;
          break;
        case 1:          
          *(arg1 + tIdx) = rChar;
          *(arg1 + tIdx + 1) = 0; //pre-emptively null-terminate the string
          *(cmd + idx) = 0; //to empty the remainder of the cmd string
          tIdx++;
          break;
        case 2:          
          *(arg2 + tIdx) = rChar;
          *(arg2 + tIdx + 1) = 0; //pre-emptively null-terminate the string
          *(cmd + idx) = 0; //to empty the remainder of the cmd string
          tIdx++;
          break;
        case 3:          
          *(arg3 + tIdx) = rChar;
          *(arg3 + tIdx + 1) = 0; //pre-emptively null-terminate the string
          *(cmd + idx) = 0; //to empty the remainder of the cmd string
          tIdx++;
          break;
        case 4:          
          *(arg4 + tIdx) = rChar;
          *(arg4 + tIdx + 1) = 0; //pre-emptively null-terminate the string
          *(cmd + idx) = 0; //to empty the remainder of the cmd string
          tIdx++;
          break;
      }
      
    } else {
      if ((tIdx > 0) || (rChar == ',')) {        
        //recording has begun for arg at pIdx OR we've parsed into a comma -- both mean parameter index increases
        tIdx = 0;
        pIdx++;
      }      
      *(cmd + idx) = 0;
    }
    idx++;
  }

  //now we return the emptied cmd with just the single command character (all indexes of cmd have been zeroed)
  *cmd = cmdResult; 

  return success;
}


//WARNING: When using getInput you must always specify buffer size being passed in!
bool getInput(uint8_t** buf, uint16_t len) {
  return getInput(buf, &len, 0xFFFF, (char)0, (uint16_t)0);
}

//WARNING: When using getInput you must always specify buffer size being passed in!
bool getInput(uint8_t** buf, uint16_t len, uint16_t minCharsBeforeTerminator) {
  return getInput(buf, &len, minCharsBeforeTerminator, (char)0, (uint16_t)0);
}

//WARNING: When using getInput you must always specify buffer size being passed in!
bool getInput(uint8_t** buf, uint16_t len, uint16_t minCharsBeforeTerminator, uint8_t terminator) {  
  return getInput(buf, &len, minCharsBeforeTerminator, terminator, (uint16_t)0);
}

/*
 * Pass in: 
 *    a buffer, the buffer's length, terminator settings, and timeout
 *    minCharsBeforeTerminator: if 65535 (0xFFFF) then there is no terminator, else, the port must get at least this value before triggering a terminator
 *    timeout: if value 0 is passed in then there is no timeout
 * 
 * Get out:
 *    the buffer filled with characters and the length of the null terminated string (counting the null character)
 *
 * WARNING: When using getInput you must always specify buffer size being passed in!
 *
 */
bool getInput(uint8_t** buf, uint16_t* len, uint16_t minCharsBeforeTerminator, uint8_t terminator, uint16_t timeout) {
  return inputProcessingLoop(buf, len, minCharsBeforeTerminator, terminator, timeout, 0);
}

#define MAXRXLEN 520
bool getMoreInput(uint8_t** buf, uint16_t* len, uint16_t minCharsBeforeTerminator, uint8_t terminator, uint16_t timeout) {
  uint16_t oldIdx = (*len - 1);
  *len = MAXRXLEN;
  return inputProcessingLoop(buf, len, minCharsBeforeTerminator, terminator, timeout, oldIdx);
}

// Do not call directly!
bool inputProcessingLoop(uint8_t** buf, uint16_t* len, uint16_t minCharsBeforeTerminator, uint8_t terminator, uint16_t timeout, uint16_t idx) {
  bool done = false, expired = false;
  uint8_t nocharLoops = 0;
  uint8_t *tmpBuf = (uint8_t*)malloc(2);
  unsigned long timeExpire = millis() + (unsigned long)timeout;
  do {
    if (Serial.available() > 0) {
      if (idx < *len - 1) {
        Serial.readBytes(tmpBuf, 1);
        *(*buf + idx++) = *(tmpBuf + 0);
        //Serial.print((char) *(*buf + (idx - 1)));  //debug
        nocharLoops = 0;
        if (idx >= minCharsBeforeTerminator) {
          if (*(*buf + (idx - 1)) == terminator) 
            done = true;
        }
      } else
        done = true;  //cannot exceed size of buffer so terminate immediately 
    } else {
      if (nocharLoops > 10) {
        done = true;
      } else if (idx >= 1)  //only begin wait after at least one character is received
        nocharLoops++;
      delay(10);
    }
    if (timeout > 0) {
      if (millis() > timeExpire) {
        expired = true;        
      }
    }
  } while ((!done) && (!expired));

  free(tmpBuf);

  if (idx > 0) {
    *(*buf + idx) = 0;    //add terminator
    *len = idx + 1;       //report length of result  
  }

  if (expired)
    return false;
  
  return done;  
}

// ^ ^ ^   flash Direct Memory Access (DMA) menu and supporting functions   ^ ^ ^
