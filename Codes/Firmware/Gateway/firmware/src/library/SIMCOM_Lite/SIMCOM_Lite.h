#ifndef SIMCOM_LITE_H
#define SIMCOM_LITE_H


#include <Arduino.h>
#include <stdint.h>


typedef uint8_t byte;

#define MAX_RESPONSE_SEARCH_TERMS   5
#define SIMCOM_SHORT_BUF_LEN  255
#define SIMCOM_LONG_BUF_LEN   1499  // Length of the larger general purpose buffer.  Note:  MTU=1500 (maximum transmission unit, ethernet)
#define RESPONSE_FAIL         -1
#define pin_SIMCOM_PWR_KEY      0
#define pin_SIMCOM_PWR_STATUS   25
#define pin_SIMCOM_RESET        13
#define SIMCOM_BOOT_DELAY       5100  // boot delay to <UART ready> in milliseconds after powering SIMCOM radio module
#define SIMCOM_PWRDWN_DELAY     7600  // maximum power down delay in millseconds


#define _CR           13
#define _LF           10

#define SIMCOM_STATUS_CONSTR  0x01  // The SimCom5320 object has been constructed with a valid Serial Port object
#define SIMCOM_STATUS_UART    0x02  // The UART in the MCU is configured
#define SIMCOM_STATUS_STARTED 0x04  // The SimCom5320 modem has been powered on
#define SIMCOM_STATUS_GPRS    0x08  // the GPRS connection is open
#define SIMCOM_STATUS_HTTPS   0x10  // the HTTPS services are running and connected
#define SIMCOM_STATUS_POSTED  0x20  // when set, the last post attempt was successful. Cleared on connection close.
#define SIMCOM_STATUS_UDPSOC  0x40  // when set, a UDP Socket is open



class SimCom {
  public:
    SimCom(HardwareSerial& myPort);
    uint8_t GetStatus();    // read the status byte, which is private and read-only    
    bool begin(uint32_t Baudrate = 57600);
    bool end();
    
// THIS SECTION COULD BE PRIVATE - I will expose these, but use carefully
  public:
    // void ClearRX(void);
    // void SendBuf(byte *buf, uint16_t len);  // send a binary data buffer
    // bool SendBufGetOK(byte *buf, uint16_t len, uint32_t timeout); // send a binary data buffer, look for OK response
    // uint16_t RecBuf(uint8_t *buf, uint16_t Num, uint32_t timeout);  // receive Num bytes from the serial stream
    void SendAT(char *buf);         // send a null-terminated AT command string.  Opening AT and closing \n should not be included.   
    // bool WaitForCarat(uint32_t timeout);
    bool GetOK(uint32_t timeout);
    bool SendATGetOK(char *buf, uint32_t timeout);  // send a common AT command that expects a simple OK response
    uint16_t RecvAT(char *buf, uint16_t maxlen, uint32_t timeout, uint8_t NumLF); // receive the response to an AT command
    uint32_t AutoBaud();          // Find the Baud Rate of the SimCom5320.  My Baud rate will be set to this on exit.
    bool powerOff(bool waitPwrDown);
    bool powerOn(bool waitPwrUp);
    void hardReset();

    uint8_t getSignalQuality();

    bool ChangeBaud(uint32_t BaudRate);
    
    
    char      shortbuf[SIMCOM_SHORT_BUF_LEN];
    char      longbuf[SIMCOM_LONG_BUF_LEN + 1];
  private:
    uint8_t     status = 0;         // see flags:  SIMCOM_STATUS_XXXX.  DONT MESS WITH THIS DIRECTLY!
    HardwareSerial  *hwport = 0;    // reference to my serial port
//    SoftwareSerial  *swport = 0;
    Stream      *port = 0;
};



#endif  // SIMCOM_LITE_H