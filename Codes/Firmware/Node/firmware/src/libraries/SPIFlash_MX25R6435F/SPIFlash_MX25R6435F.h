// Copyright (c) 2013-2015 by Felix Rusu, LowPowerLab.com
// SPI Flash memory library for arduino/moteino.
// This works with 256byte/page SPI flash memory
// For instance a 4MBit (512Kbyte) flash chip will have 2048 pages: 256*2048 = 524288 bytes (512Kbytes)
// Minimal modifications should allow chips that have different page size but modifications
// DEPENDS ON: Arduino SPI library
// > Updated Jan. 5, 2015, TomWS1, modified writeBytes to allow blocks > 256 bytes and handle page misalignment.
// > Updated Feb. 26, 2015 TomWS1, added support for SPI Transactions (Arduino 1.5.8 and above)
// > Selective merge by Felix after testing in IDE 1.0.6, 1.6.4

// Edited by Alondra Thompson for use with Macronix MX25R6435F 64 Mbit Flash chip Aug 2018

// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code

#ifndef _SPIFLASH_MX25R6435F_H_
#define _SPIFLASH_MX25R6435F_H_

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <wiring.h>
#include "pins_arduino.h"
#endif

#include <SPI.h>

/// IMPORTANT: NAND FLASH memory requires erase before write, because
///            it can only transition from 1s to 0s and only the erase command can reset all 0s to 1s
/// See http://en.wikipedia.org/wiki/Flash_memory
/// The smallest range that can be erased is a sector (4K, 32K, 64K); there is also a chip erase command

/// Standard SPI flash commands
/// Assuming the WP pin is pulled up (to disable hardware write protection)
/// To use any write commands the WEL bit in the status register must be set to 1.
/// This is accomplished by sending a 0x06 command before any such write/erase command.
/// The WEL bit in the status register resets to the logical �0� state after a
/// device power-up or reset. In addition, the WEL bit will be reset to the logical �0� state automatically under the following conditions:
/// � Write Disable operation completes successfully
/// � Write Status Register operation completes successfully or aborts
/// � Protect Sector operation completes successfully or aborts
/// � Unprotect Sector operation completes successfully or aborts
/// � Byte/Page Program operation completes successfully or aborts
/// � Sequential Program Mode reaches highest unprotected memory location
/// � Sequential Program Mode reaches the end of the memory array
/// � Sequential Program Mode aborts
/// � Block Erase operation completes successfully or aborts
/// � Chip Erase operation completes successfully or aborts
/// � Hold condition aborts
#define SPIFLASH_WRITEENABLE      0x06        // write enable (WREN pgs. 18, 20)
#define SPIFLASH_WRITEDISABLE     0x04        // write disable (WRDI pg. 18, 21)

#define SPIFLASH_SECTORERASE_4K   0x20        // erase one 4K sector of flash memory (SE pg. 17)
#define SPIFLASH_BLOCKERASE_32K   0x52        // erase one 32K block of flash memory (BE 32k pg. 17)
#define SPIFLASH_BLOCKERASE_64K   0xD8        // erase one 64K block of flash memory (BE pg. 17)
#define SPIFLASH_CHIPERASE        0x60        // chip erase (may take several seconds depending on size) (CE pg. 17, 49)
                                              // but no actual need to wait for completion (instead need to check the status register BUSY bit)
#define SPIFLASH_STATUSREAD       0x05        // read status register (RDSR pgs. 18, 26)
#define SPIFLASH_STATUSWRITE      0x01        // write status register (WRSR pg. 18, 32)
#define SPIFLASH_FAST_READ        0x0B        // read array (fast, need to add 1 dummy byte after 3 address bytes) (FAST READ pg. 17, 37)
#define SPIFLASH_READ             0x03        // read array (low frequency) (READ pg. 17, 36)

#define SPIFLASH_SLEEP            0xB9        // deep power down (DP - deep power down pg. 18)
#define SPIFLASH_WAKE             0xAB        // deep power wake up (AB => RES read electronic ID pg. 19; not recommended for use pg. 23)
#define SPIFLASH_BYTEPAGEPROGRAM  0x02        // write (1 to 256bytes) (PP page program pg. 17, 50)
#define SPIFLASH_IDREAD           0x9F        // read JEDEC manufacturer and device ID (2 bytes, specific bytes for each manufacturer and device) (REMS pg. 19,24-25)
//#define SPIFLASH_MACREAD          0x4B        // read unique ID number (MAC) - NOT USED WITH MX25R6435L

class SPIFlash {
public:
 // static uint8_t UNIQUEID[8];
  SPIFlash(uint8_t slaveSelectPin, uint16_t jedecID=0);
  boolean initialize();
  void command(uint8_t cmd, boolean isWrite=false);
  uint8_t readStatus();
  uint8_t readByte(uint32_t addr);
  void readBytes(uint32_t addr, void* buf, uint16_t len);
  void writeByte(uint32_t addr, uint8_t byt);
  void writeBytes(uint32_t addr, const void* buf, uint16_t len);
  boolean busy();
  void chipErase();
  void sectorErase4K(uint32_t address);
  void blockErase32K(uint32_t address);
  void blockErase64K(uint32_t addr);
 uint16_t readDeviceId();
  //uint8_t* readUniqueId();

  void sleep();
  void wakeup();
  void end();
protected:
  void select();
  void unselect();
  uint8_t _slaveSelectPin;
  uint16_t _jedecID;
  uint8_t _SPCR;
  uint8_t _SPSR;
#ifdef SPI_HAS_TRANSACTION
  SPISettings _settings;
#endif
};

#endif
