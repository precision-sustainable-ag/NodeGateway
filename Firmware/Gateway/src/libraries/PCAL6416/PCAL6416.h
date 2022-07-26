// PCAL6416 I2C IO expander

#ifndef _PCAL6416_H
#define _PCAL6416_H

#include <stdint.h>

#define NumPorts	2

typedef struct {
	uint8_t Floating;
	uint8_t Low;
	uint8_t High;
} triState_t;

typedef struct {
	uint8_t LoRa;	// 0=none, 1=915MHz, 2=868MHz, 3=433MHz, 4=470MHz, 5-9=unknown but may be valid (future), 255=unable to read
	uint8_t Cell;	// 0=none, 1=SIM5320A, 2=SIM5320E, 3=SIM5320J, 4-9=unknown (future), 255=unable to read
	uint8_t Ver;	// Card compatibility version.  0,1 in use.  2-89 are future revisions.  255=unable to read
						// 0 = original release.  SIM Detect pin pulled low when inserted.
						// 1 = Push-Push SIM card.  SIM detect pin floats when card inserted.
} RadioCardID_t;


class PCAL6416 {
public:
	PCAL6416(bool AddrHigh);
	bool begin(bool beginWire);
	uint8_t readRegisters(uint8_t RegAddr, uint8_t *values, uint8_t nBytes);
	uint8_t writeRegisters(uint8_t RegAddr, uint8_t *values, uint8_t nBytes);
	uint8_t readRegister(uint8_t RegAddr);
	uint8_t writeRegister(uint8_t RegAddr, uint8_t values);
	uint8_t input(uint8_t PortNum);
	bool output(uint8_t PortNum, uint8_t value);
	bool bitsOut(uint8_t PortNum, uint8_t BitMask, uint8_t BitValue);
	triState_t scanTriState(uint8_t PortNum);
	uint16_t TriStateToInt(triState_t triVal, uint8_t LowBitNum, uint8_t HighBitNum);
	RadioCardID_t readID();
	uint8_t LastError;
private:
	uint8_t portOut[NumPorts];
	uint8_t I2CAddress;
};






#endif // _PCAL6416_H