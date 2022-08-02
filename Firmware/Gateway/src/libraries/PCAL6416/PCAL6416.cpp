// PCAL6416 I2C IO expander
#include "PCAL6416.h"
#include <Arduino.h>
// define release-independent I2C functions
#if defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#include <TinyWireM.h>
#define i2cBegin TinyWireM.begin
#define i2cBeginTransmission TinyWireM.beginTransmission
#define i2cEndTransmission TinyWireM.endTransmission
#define i2cRequestFrom TinyWireM.requestFrom
#define i2cRead TinyWireM.receive
#define i2cWrite TinyWireM.send
#elif ARDUINO >= 100
#include <Wire.h>
#define i2cBegin Wire.begin
#define i2cBeginTransmission Wire.beginTransmission
#define i2cEndTransmission Wire.endTransmission
#define i2cRequestFrom Wire.requestFrom
#define i2cRead Wire.read
#define i2cWrite Wire.write
#else
#include <Wire.h>
#define i2cBegin Wire.begin
#define i2cBeginTransmission Wire.beginTransmission
#define i2cEndTransmission Wire.endTransmission
#define i2cRequestFrom Wire.requestFrom
#define i2cRead Wire.receive
#define i2cWrite Wire.send
#endif

#define PCAL6416_DEFAULT_ADDR	0x20
#define PCAL6416_ADDR_SEL_BIT	0x01

#define NumCommands	11

#define CmdNum_Input		0
#define CmdNum_Output		1
#define CmdNum_Polarity		2
#define CmdNum_Config		3
#define CmdNum_DriveA		4
#define CmdNum_DriveB		5
#define CmdNum_Latch		6
#define CmdNum_PullEnable	7
#define CmdNum_PullSelect	8
#define CmdNum_IntMask		9
#define CmdNum_IntStatus	10
#define Cmd_OutCfg			0x4F

uint8_t command[NumPorts][NumCommands] = {
	{0, 2, 4, 6, 0x40, 0x41, 0x44, 0x46, 0x48, 0x4A, 0x4C },	// Port 0
	{1, 3, 5, 7, 0x42, 0x43, 0x45, 0x47, 0x49, 0x4B, 0x4D }		// port 1
};

PCAL6416::PCAL6416(bool AddrHigh) {
	if (AddrHigh) I2CAddress = PCAL6416_DEFAULT_ADDR | PCAL6416_ADDR_SEL_BIT;
	else I2CAddress = PCAL6416_DEFAULT_ADDR;
}

bool PCAL6416::begin(bool beginWire) {
    if (beginWire) i2cBegin();
	// read the current output status to prove the system is working.  return false if not working
	if (readRegisters(command[0][CmdNum_Output], portOut, 2)) return false;
	return true;
}

uint8_t PCAL6416::readRegisters(uint8_t RegAddr, uint8_t *values, uint8_t nBytes)
{
    i2cBeginTransmission(I2CAddress);
    i2cWrite(RegAddr);
    if (LastError = i2cEndTransmission()) return LastError;
    i2cRequestFrom(I2CAddress, nBytes);
    for (uint8_t i=0; i<nBytes; i++) values[i] = i2cRead();
    return 0;
}

uint8_t PCAL6416::readRegister(uint8_t RegAddr)
{
    i2cBeginTransmission(I2CAddress);
    i2cWrite(RegAddr);
    if (LastError = i2cEndTransmission()) return 0;
    i2cRequestFrom(I2CAddress, 1);
    return i2cRead();
}

uint8_t PCAL6416::writeRegisters(uint8_t RegAddr, uint8_t *values, uint8_t nBytes)
{
    i2cBeginTransmission(I2CAddress);
    i2cWrite(RegAddr);
    for (uint8_t i=0; i<nBytes; i++) i2cWrite(values[i]);
	LastError = i2cEndTransmission();
	return LastError;
}

uint8_t PCAL6416::writeRegister(uint8_t RegAddr, uint8_t value)
{
    i2cBeginTransmission(I2CAddress);
    i2cWrite(RegAddr);
    i2cWrite(value);
	LastError = i2cEndTransmission();
	return LastError;
}

uint8_t PCAL6416::input(uint8_t PortNum) {
	if (PortNum >= NumPorts) return 0;
	return readRegister(command[PortNum][CmdNum_Input]);
}

bool PCAL6416::output(uint8_t PortNum, uint8_t value) {
	if (PortNum >= NumPorts) return false;
	if (writeRegister(command[PortNum][CmdNum_Input], value)) return false;
	portOut[PortNum] = value;
	return true;
}

bool PCAL6416::bitsOut(uint8_t PortNum, uint8_t BitMask, uint8_t BitValue) {
	if (PortNum >= NumPorts) return false;
	uint8_t newPortOut;
	newPortOut = (portOut[PortNum] & ~BitMask) | BitValue;
	if (newPortOut == portOut[PortNum]) return true;			// no change
	return output(PortNum, newPortOut);
}

triState_t PCAL6416::scanTriState(uint8_t PortNum) {
	triState_t triVal = {0};
	if (PortNum >= NumPorts) return triVal;
	// set all pins as inputs
	if (writeRegister(command[PortNum][CmdNum_Config], 0xFF)) return triVal;
	// Enable pull ups
	writeRegister(command[PortNum][CmdNum_PullSelect], 0xFF);
//	Serial.print("Pull Sel = "); Serial.println(readRegister(command[PortNum][CmdNum_PullSelect]));
	writeRegister(command[PortNum][CmdNum_PullEnable], 0xFF);
//	Serial.print("Pull En = "); Serial.println(readRegister(command[PortNum][CmdNum_PullEnable]));
	// read pins
	triVal.High = readRegister(command[PortNum][CmdNum_Input]);
//	Serial.print("Reading = "); Serial.println(triVal.High, HEX);
	// enable pull downs
	writeRegister(command[PortNum][CmdNum_PullSelect], 0);
//	Serial.print("Pull Sel = "); Serial.println(readRegister(command[PortNum][CmdNum_PullSelect]));
	// read pins
	triVal.Low = readRegister(command[PortNum][CmdNum_Input]);
//	Serial.print("Reading = "); Serial.println(triVal.Low, HEX);
	triVal.Low = ~triVal.Low;
	// Done reading.  turn off pulls
	writeRegister(command[PortNum][CmdNum_PullEnable], 0);
	// assign a value.  Tri-state = 0, low = 1, high = 2
	triVal.Floating = triVal.High & triVal.Low;	// these are the tri-state pins.
	triVal.High ^=  triVal.Floating;			// clear tri-states out of the pull-up value
	triVal.Low ^=  triVal.Floating;				// clear tri-states out of the pull-down value
//	Serial.print("Floating = "); Serial.println(triVal.Floating, HEX);
//	Serial.print("High = "); Serial.println(triVal.High, HEX);
//	Serial.print("Low = "); Serial.println(triVal.Low, HEX);
	return triVal;
}

// decode a tristate reading (base 3 number) into binary int.  Start at LowBitNum and end with HighBitNum (inclusive)
// Allowed bit numbers are 0-7
uint16_t PCAL6416::TriStateToInt(triState_t triVal, uint8_t LowBitNum, uint8_t HighBitNum) {
	if (LowBitNum > 7) return 0;
	if (HighBitNum > 7) HighBitNum = 7;
	if (HighBitNum < LowBitNum) return 0;
	LowBitNum = 1 << LowBitNum;
	HighBitNum = 1 << HighBitNum;
	uint16_t val = 0;
	for (uint8_t bit = HighBitNum; bit >= LowBitNum; bit = bit >> 1) {
		val *= 3;
		if (triVal.Low & bit) val += 1;
		else if (triVal.High & bit) val += 2;
	}
	return val;
}

RadioCardID_t PCAL6416::readID() {
	RadioCardID_t RadioID = { 255, 255, 255 };
	triState_t tval = scanTriState(0);
	if ((tval.Floating == 0) && (tval.Low == 0) && (tval.High == 0)) return RadioID;
	RadioID.LoRa = TriStateToInt(tval, 0, 1);
	RadioID.Cell = TriStateToInt(tval, 2, 3);
	RadioID.Ver = TriStateToInt(tval, 4, 7);
	return RadioID;
}


// uint16_t PCAL6416::ExtractBitsBase3(uint16_t base2EncodedVal, uint8_t LowBitNum, uint8_t HighBitNum) {
	// if (LowBitNum > 7) return 0;
	// if (HighBitNum > 7) HighBitNum = 7;
	// if (HighBitNum < LowBitNum) return 0;
	// uint16_t base3[] = {1, 3, 9, 27, 81, 243, 729, 2187, 6561};
	// base2EncodedVal = base2EncodedVal /  base3[LowBitNum];
	// base2EncodedVal = base2EncodedVal % base3[HighBitNum + 1];
// }
