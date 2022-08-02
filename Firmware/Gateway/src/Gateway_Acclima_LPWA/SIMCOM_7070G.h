#ifndef SIMCOM_7070G_H
#define SIMCOM_7070G_H

#include <arduino.h>
/*
 * 
 * SIMCOM_7070G.h 
 * 
 * Provides tools for communicating with the SIMCOM SIM7070G radio modules.
 * 
 * The module communicates using AT commands through TX and RX UART pins (at minimum) so it must be made accessible to the MCU in this manner.  
 * Pass in the handle to the UART that is connected to to the radio module to begin using.
 * 
 */

#define pin_SIMCOM_PWR_KEY      0
#define pin_SIMCOM_PWR_STATUS   25
#define pin_SIMCOM_RESET        13
#define SIMCOM_BOOT_DELAY       1850  //boot delay to <UART ready> in milliseconds after powering SIMCOM radio module
#define SIMCOM_PWRDWN_DELAY     1850  //maximum power down delay in millseconds
#define SIMCOM_RX_BUFF_LEN      1000  //size of circular buffer -- when reaches this number it will overwrite older data
#define FONA_MSG_SIZE           1000

#define RADIO_BOARD_REV_C



class SIMCOM_7070G {
  public:    
    SIMCOM_7070G();
    SIMCOM_7070G(Stream *simcom_UART, uint32_t baudrate);
    ~SIMCOM_7070G();
    
    void setPort(Stream *simcom_UART);
    void setBaudrate(uint32_t baudrate);
    bool begin();
    bool end();
    bool available();
    bool powerOff(bool waitPwrDown);
    bool powerOn(bool waitPwrUp);
    void hardReset();
    void checkInput();
    bool commInit();
    void setRXEventHandler(void (*evtHandler)(uint8_t rxCnt, uint16_t unreadCnt));
    uint16_t getRX(uint16_t cnt, char** rxOut);
    uint16_t peekRX(uint16_t offset, uint16_t cnt, char** rxOut);
    uint16_t getUnreadCnt();

    // Adapted from Adafruit_FONA.h
    bool detectSIM();   
    uint8_t getNetworkStatus();
    int8_t getRSSIdBm();
    char replybuffer[255];
    bool sendMsg(char* msg, uint16_t msgLen, uint16_t timeout);

    bool echo(bool onoff);
    bool startIP();
    bool closeIP();
    bool openTCP(char* IPaddr, uint16_t IPlen, char* portNum, uint16_t portLen);
    bool closeTCP();
    uint16_t sendAT(char* ATcmd, uint16_t wait4resp, unsigned int timeout);
  
  private:
    Stream *port;
    uint32_t baud;
    bool portSet;
    void (*evtHandler)(uint8_t rxCnt, uint16_t unreadCnt);
    void writeRXBuf(char rx);
    void init_SIMCOM();
    char* rxBuf;          //circular buffer
    uint16_t rxReadIdx;   //the beginning index of unread buffer data
    uint16_t rxWriteIdx;  //the rx buffer write index, the read index reads to here
    
};

boolean checkOK(char *cellResp, uint16_t respLen);
uint16_t sendATcommand(char* ATcommand, uint16_t wait4resp, unsigned int timeout);
bool isSubstring(char* substr, char* str, uint16_t subLen, uint16_t strLen);
void printResp(uint16_t respLen);
bool printCheckOK(uint16_t respLen);

 
#endif  //!SIMCOM_7070G_H
