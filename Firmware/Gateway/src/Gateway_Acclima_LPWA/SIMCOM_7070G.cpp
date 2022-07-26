#include "SIMCOM_7070G.h"

#define debug_on

void sendStr(char* data, Stream *port);    //Temporary prototype

SIMCOM_7070G::SIMCOM_7070G() {
  init_SIMCOM();
  this->port = nullptr;
  this->baud = 0;
}

SIMCOM_7070G::SIMCOM_7070G(Stream *simcom_UART, uint32_t baudrate) {
  init_SIMCOM();
  this->port = simcom_UART;
  this->baud = baudrate;  
  this->commInit();
}

void SIMCOM_7070G::init_SIMCOM() {
  //Set up pins immediately so that reset is not being held
  digitalWrite(pin_SIMCOM_PWR_KEY, LOW);
  pinMode(pin_SIMCOM_PWR_KEY, OUTPUT);
  digitalWrite(pin_SIMCOM_RESET, HIGH);   //Set high so radio is not held in reset -- use reset sparingly (perhaps as last resort)
  pinMode(pin_SIMCOM_RESET, OUTPUT);
  pinMode(pin_SIMCOM_PWR_STATUS, INPUT);
  
  rxBuf = (char*)malloc(SIMCOM_RX_BUFF_LEN);
  rxReadIdx = 0;  
  rxWriteIdx = 0;
}

bool SIMCOM_7070G::commInit() {
  if ((this->port != nullptr) && (this->baud > 0)) {
    ((HardwareSerial*)port)->begin(baud);
    this->portSet = true;
    return true;
  } else 
    return false;
}

void SIMCOM_7070G::writeRXBuf(char rx) {
  this->rxBuf[rxWriteIdx++] = rx;
  if (rxWriteIdx == rxReadIdx)
    rxReadIdx++;    //rxReadIdx must always keep ahead which means unread characters will be lost / recycled
  if (rxWriteIdx >= SIMCOM_RX_BUFF_LEN)
    rxWriteIdx = 0;
  if (rxReadIdx >= SIMCOM_RX_BUFF_LEN)
    rxWriteIdx = 0;
}



/*
 * Returns number of characters successfully loaded into rxOut... if zero then rxOut is nullptr
 * 
 * when rxReadIdx == rxWriteIdx that means we have nothing to return (no new data)
 * 
 */
uint16_t SIMCOM_7070G::getRX(uint16_t cnt, char** rxOut) {
  if (rxReadIdx == rxWriteIdx) {
    *rxOut = nullptr;
    return 0;
  }

  uint16_t result;
  if (rxWriteIdx > rxReadIdx) {
    //straighforward approach (nothing wrapped)
    result = rxWriteIdx - rxReadIdx;
    if (result > cnt)                           //normalize if not the whole space wanted
      result = cnt;

    //debug vv 
//    Serial.print("NOT WRAPPED... size: ");
//    Serial.print(result);
//    Serial.print("; rxReadIdx: ");
//    Serial.print(rxReadIdx);
//    Serial.print("; rxWriteIdx: ");
//    Serial.print(rxWriteIdx);
//    Serial.println();
//    Serial.flush();
    //debug ^^
    
    *rxOut = (char*)malloc(result);
    memcpy(*rxOut, rxBuf + rxReadIdx, result);
    //update rxReadIdx
    rxReadIdx += result;
    if (rxReadIdx >= SIMCOM_RX_BUFF_LEN)
      rxReadIdx = SIMCOM_RX_BUFF_LEN - rxReadIdx;
  } else {
    //rxWriteIdx has wrapped but rxReadIdx has not yet wrapped
    result = SIMCOM_RX_BUFF_LEN - rxReadIdx;    //up to the buffer overflow boundary
    result += rxWriteIdx;                       //include the remaining count up to the write index
    if (result > cnt)                           //normalize if not the whole space wanted
      result = cnt;

    //debug vv 
//    Serial.print("WRAPPED! size: ");
//    Serial.print(result);
//    Serial.print("; rxReadIdx: ");
//    Serial.print(rxReadIdx);
//    Serial.print("; rxWriteIdx: ");
//    Serial.print(rxWriteIdx);
//    Serial.println();
//    Serial.flush();
    //debug ^^
      
    *rxOut = (char*)malloc(result);
    if (rxReadIdx + result > SIMCOM_RX_BUFF_LEN) {
      uint16_t boundCnt = SIMCOM_RX_BUFF_LEN - rxReadIdx;
      memcpy(*rxOut, rxBuf + rxReadIdx, boundCnt);
      memcpy(*(rxOut + boundCnt), rxBuf, result - boundCnt);
    } else {
      memcpy(*rxOut, rxBuf + rxReadIdx, result);
    }
    //update rxReadIdx
    rxReadIdx += result;
    if (rxReadIdx >= SIMCOM_RX_BUFF_LEN)
      rxReadIdx = SIMCOM_RX_BUFF_LEN - rxReadIdx;
  }  
  return result;
}

/*
 * Returns number of characters successfully loaded into rxOut... if zero then rxOut is nullptr
 * 
 * when rxReadIdx == rxWriteIdx that means we have nothing to return (no new data)
 * 
 */
uint16_t SIMCOM_7070G::peekRX(uint16_t offset, uint16_t cnt, char** rxOut) {
  if (rxReadIdx == rxWriteIdx) {
    *rxOut = nullptr;
    return 0;
  }
  return 0;
}

uint16_t SIMCOM_7070G::getUnreadCnt() {
  uint16_t result;
  if (rxWriteIdx > rxReadIdx) {
    //straighforward approach (nothing wrapped)
    result = rxWriteIdx - rxReadIdx;
  } else {
    //rxWriteIdx has wrapped but rxReadIdx has not yet wrapped
    result = SIMCOM_RX_BUFF_LEN - rxReadIdx;    //up to the buffer overflow boundary
    result += rxWriteIdx;
  }
  return result;
}

bool SIMCOM_7070G::available() {
  return port->available();
}

void SIMCOM_7070G::setPort(Stream *simcom_UART){
  this->port = simcom_UART;
}

void SIMCOM_7070G::setBaudrate(uint32_t baudrate) {
  this->baud = baudrate;    //TODO: keep out disallowed baud rates
}

bool SIMCOM_7070G::powerOff(bool waitPwrDown) {
  //first ensure power is on so that we don't turn it on accidentally by trying to turn it off
  #ifdef RADIO_BOARD_REV_C
  if (digitalRead(pin_SIMCOM_PWR_STATUS))
  #else  
  if (!digitalRead(pin_SIMCOM_PWR_STATUS))
  #endif
  // if (analogRead(pin_SIMCOM_PWR_STATUS) < 300)
    return true;   //already off -- exit function without doing anything  
  
  digitalWrite(pin_SIMCOM_PWR_KEY, LOW);
  pinMode(pin_SIMCOM_PWR_KEY, OUTPUT);
  delay(50);
  digitalWrite(pin_SIMCOM_PWR_KEY, HIGH);
  delay(1250);                              // Requires minimum of 1.2 seconds for power off
  digitalWrite(pin_SIMCOM_PWR_KEY, LOW);

  //now wait for power down... maximum time specified by SIMCOM_PWRDWN_DELAY but check periodically because it may be faster
  if (waitPwrDown) {
    int8_t cnt = 20;
    uint16_t divDelay = SIMCOM_PWRDWN_DELAY / cnt;
    
    do {
      #ifdef RADIO_BOARD_REV_C
      if (digitalRead(pin_SIMCOM_PWR_STATUS))
      #else
      if (!digitalRead(pin_SIMCOM_PWR_STATUS))
      #endif
      // if (analogRead(pin_SIMCOM_PWR_STATUS) < 300)
        return true;
      delay(divDelay);
      Serial.println("Checking for power down now...");   //TODO: remove after debug  
      cnt--;
    } while (cnt>=0);
  }
  return false;
}

bool SIMCOM_7070G::powerOn(bool waitPwrUp) {
  //First check to see if radio is already on
  #ifdef RADIO_BOARD_REV_C
  if (!digitalRead(pin_SIMCOM_PWR_STATUS))
  #else
  if (digitalRead(pin_SIMCOM_PWR_STATUS))
  #endif
  
  // if (analogRead(pin_SIMCOM_PWR_STATUS) > 800)
    return true; //already running, no need to toggle power key pin to turn on

  digitalWrite(pin_SIMCOM_PWR_KEY, LOW);
  pinMode(pin_SIMCOM_PWR_KEY, OUTPUT);
  delay(50);
  digitalWrite(pin_SIMCOM_PWR_KEY, HIGH);    
  delay(1050);                               // Requires minimum of 1 second for power on
  digitalWrite(pin_SIMCOM_PWR_KEY, LOW);

  //now wait for boot sequence... maximum time specified by SIMCOM_BOOT_DELAY but check periodically because it may boot faster
  if (waitPwrUp) {
    int8_t cnt = 20;
    uint16_t divDelay = SIMCOM_BOOT_DELAY / cnt;
    
    do {
      #ifdef RADIO_BOARD_REV_C
      if (!digitalRead(pin_SIMCOM_PWR_STATUS))
      #else
      if (digitalRead(pin_SIMCOM_PWR_STATUS))
      #endif
      // if (analogRead(pin_SIMCOM_PWR_STATUS) > 800)
        return true;
      delay(divDelay);
      cnt--;
    } while (cnt>=0);
  }
  return false;
}

SIMCOM_7070G::~SIMCOM_7070G() {
  powerOff(true);
  free(rxBuf);
}


bool SIMCOM_7070G::begin() {
  uint8_t cnt;  
  bool success = false;
  if (!portSet) {
    this->commInit();
  }
  
  if (portSet) {
    uint8_t attempts = 0;   //on the second attempt, try a hard reset using pin_SIMCOM_RESET
    
    do {      
      if (powerOn(true)) {
        port->write("AT\r");

        char buf[100];
        bool cr = false;        
        uint8_t idx = 0;
        cnt = 0;
        while (cnt<20) {
          while (port->available()) {
            buf[idx] = (char)port->read();
            if (buf[idx] == '\r') {
              cr = true;
              buf[idx+1] = 0;
              break;
            }
            idx++;
          }
          
          if (cr) {            
            if (strstr(buf, "OK")) {
              //This means SIMCOM radio module UART is now operational
              success = true;
              break;
            }
            idx = 0;
          }
          delay(100);
          cnt++;
        }        
      } else 
        hardReset();

      if (success)
        return success;
      
      attempts++;
    } while (attempts < 3);
  } else {
    Serial.println("port was nullptr!");    //TODO: remove debug statement later
  }
  return success;
}


bool SIMCOM_7070G::end() {
  //TODO: Need to determine how to terminate radio data links, etc. and shutdown nicely
  return false;
}


void SIMCOM_7070G::hardReset() {
  Serial.println(" *** Hard reset!!!");    //TODO: remove debug statement later
  digitalWrite(pin_SIMCOM_RESET, HIGH);
  pinMode(pin_SIMCOM_RESET, OUTPUT);
  delay(100);
  digitalWrite(pin_SIMCOM_RESET, LOW);
  delay(100);
  digitalWrite(pin_SIMCOM_RESET, HIGH);  
}


void SIMCOM_7070G::setRXEventHandler(void (*evtHandler)(uint8_t rxCnt, uint16_t unreadCnt)) {
  this->evtHandler = evtHandler;
}



void SIMCOM_7070G::checkInput() {
  //This relies on polling to discover if there is any new data available coming from the modem

  //TODO: disable logging of port data when radio is shutting off until PSTAT is high again (get garbage data)
  
  uint8_t rxCnt = 0;  
  while (port->available()) {
    writeRXBuf(port->read());
    rxCnt++;
  }
  if (rxCnt > 0) {
    (*evtHandler)(rxCnt, this->getUnreadCnt());
  }
}



void sendStr(char* data, Stream *port) {
  uint16_t idx = 0;
  while (data[idx] != 0) {
    Serial.print(data[idx]);
    port->write(data[idx]);
    idx++;
  }
  Serial.println();
}

/********* Additions based on Adafruit_FONA.h *************/

bool SIMCOM_7070G::detectSIM() {
  char cmd[] = "AT+CCID\r"; 
  uint16_t numOut = sendATcommand(cmd,200,300);
//  Serial.println(numOut);
//  char resp[numOut];
//  for(byte i = 0; i < numOut; i++){
//    resp[i]=Serial1.read();
//    Serial.print(resp[i]);
//  }
  
  if(numOut > 20) return true; else return false;
}
  
bool SIMCOM_7070G::echo(bool onoff) {
  char cmd[] = "ATE";
  uint8_t setStat = onoff;
  char fullcmd[5];
  sprintf(fullcmd, "%s%d\r",cmd,setStat);
  uint16_t numOut = sendATcommand(fullcmd,20,30);
//  Serial.println(numOut);
//  char resp[numOut];
//  for(byte i = 0; i < numOut; i++){
//    resp[i]=Serial1.read();
////    Serial.print(resp[i]);
//  }
  bool ok = printCheckOK(numOut); 
  return ok;
}

uint8_t SIMCOM_7070G::getNetworkStatus(){
  char getStat[] = "AT+CREG?\r";
  uint16_t numOut = sendATcommand(getStat,20,30);
//  Serial.println(numOut);
  char resp[numOut];
  for(byte i = 0; i < numOut; i++){
    resp[i]=Serial1.read();
//    Serial.print(resp[i]);
  }

  char * comma;
  comma = strchr(resp,',');
  char nS_char;
  nS_char = resp[comma-resp+1];
  uint8_t netStat = nS_char - 48;
  #ifdef debug_on 
    Serial.println(netStat); 
  #endif

  return netStat; 
  
}

bool SIMCOM_7070G::startIP(){
  char cmd[] = "AT+CNCFG=0,1,\"hologram\"\r";
  char cmd2[] = "AT+CNACT=0,1\r";
//  Serial.println(cmd);
  uint16_t numOut = sendATcommand(cmd,100,1000);
//  Serial.println(numOut);
//  char resp[numOut];
//  for(byte i = 0; i < numOut; i++){
//    resp[i]=Serial1.read();
////    Serial.print(resp[i]);
//  }
  #ifdef debug_on 
    printResp(numOut);
  #endif
  delay(500);
//  Serial.println(cmd2);
  uint16_t numOut2 = sendATcommand(cmd2,100,1000);
//  printResp(numOut2);
  bool ok = printCheckOK(numOut2);
  return ok;  
}

bool SIMCOM_7070G::closeIP(){
  char cmd[] = "AT+CNACT=0,0\r";
  uint16_t numOut = sendATcommand(cmd,100,500);
////  Serial.println(numOut);
//  char resp[numOut];
//  for(byte i = 0; i < numOut; i++){
//    resp[i]=Serial1.read();
////    Serial.print(resp[i]);
//  }
  bool ok = printCheckOK(numOut);
  return ok;
}

int8_t SIMCOM_7070G::getRSSIdBm(){
  char cmd[] = "AT+CSQ\r";
  uint16_t numOut = sendATcommand(cmd,10,20);
//  Serial.println(numOut);
  char resp[numOut];
  for(byte i = 0; i < numOut; i++){
    resp[i]=Serial1.read();
//    Serial.print(resp[i]);
  }

  char * colon;
  colon = strchr(resp,':');
  char rssi[3];
  rssi[0] = resp[colon-resp+2];
  rssi[1] = resp[colon-resp+3];
  int8_t dBm;
  uint8_t rs;
  if (rssi[1] != ','){
    rs = atoi(rssi);
  } else {
    rs = atoi(rssi[0]);
  }
  dBm = 0 - (113 - 2 * rs);
//  Serial.println(dBm);
  return dBm;
}

bool SIMCOM_7070G::openTCP(char* IPaddr, uint16_t IPlen, char* portNum, uint16_t portLen){
  char cmd1[] = "AT+CAOPEN=0,0,\"TCP\",\"";
  uint8_t cmdLen = sizeof(cmd1) + IPlen + portLen + 1;
  char fullcmd[cmdLen];  
  sprintf(fullcmd,"%s%s\",%s\r",cmd1,IPaddr,portNum);
  delay(80);
  uint16_t numOut = sendATcommand(fullcmd,2000,10000);
//  Serial.println(numOut);
//  printResp(numOut);
  bool ok = printCheckOK(numOut);

  return ok;
  
}

bool SIMCOM_7070G::closeTCP(){
  char cmd[] = "AT+CACLOSE=0\r";
  uint16_t numOut = sendATcommand(cmd,100,2000);
//  printResp(numOut);
////  Serial.println();
  bool ok = printCheckOK(numOut);
  return ok;
}

bool SIMCOM_7070G::sendMsg(char* msg, uint16_t msgLen, uint16_t timeout){
//  Serial.println(msg);
//  Serial.println(msgLen);
  char s1p1[] = "AT+CARECV=0";
      uint8_t digitNum;
    if (msgLen > 10){
      digitNum = 2;
    } else if (msgLen > 1000){
      digitNum = 3;
    } else {
      digitNum = 1;
    }
//    Serial.println(digitNum);
  char s1cmd[sizeof(s1p1) + digitNum + 2];
  sprintf(s1cmd,"%s,%d\r",s1p1,msgLen); delay(50);
//  Serial.println(s1cmd);   
  uint16_t step1 = sendATcommand(s1cmd,1000,3000);   // max msg length = 1000 (arbitrary?)
  #ifdef debug_on 
    printResp(step1); 
  #endif
  
  bool msgSent = false;
  if (step1 > 0){
    char cmd[] = "AT+CASEND=0,";   
    char fullcmd[sizeof(cmd) + digitNum + 2];
    sprintf(fullcmd,"%s%d,%d\r",cmd,msgLen,timeout);
//    Serial.println(fullcmd);
    uint16_t step2 = sendATcommand(fullcmd,1000,5000);
    #ifdef debug_on 
      printResp(step2); 
    #endif
    
    if (step2 > 0){
//    Serial.println(msg);
//    Serial.println(msg[msgLen - 2],DEC);
      Serial1.write(msg);
      delay(100);
      while (Serial1.available() == 0); delay(1000);
      uint16_t numOut = Serial1.available();
      bool ok = printCheckOK(numOut);
    }
  }
}

uint16_t SIMCOM_7070G::sendAT(char* ATcmd, uint16_t wait4resp, unsigned int timeout){
  uint8_t x = 0;
  unsigned long previous;
  uint16_t respLen = 0;
  while ( Serial1.available() > 0) Serial1.read();   // Clean the input buffer

  Serial1.write(ATcmd);    // Send the AT command

  x = 0;
  delay(50);
  previous = millis();

  while (((millis() - previous) < timeout)){
    if (Serial1.available() != 0) { 
      delay(wait4resp);  
      respLen = Serial1.available();
      break;
    }
  }
 
  return respLen;
}

/****** Helpers *******/

bool checkOK(char *cellResp, uint16_t respLen){ 
  char ok[] = "OK";
  if (isSubstring(ok,cellResp,3,respLen)){
    #ifdef debug_on 
//      Serial.println("OK"); 
    #endif
    return true;
  } else {
    #ifdef debug_on 
//      Serial.println("Not OK"); 
    #endif
    return false;
  }
}

void printResp(uint16_t respLen){
  char resp[respLen];
  for(byte i = 0; i < respLen; i++){
    resp[i]=Serial1.read();
    Serial.print(resp[i]);
  }
}

bool printCheckOK(uint16_t respLen){
  char resp[respLen];
  for(byte i = 0; i < respLen; i++){
    resp[i]=Serial1.read();
    #ifdef debug_on 
      Serial.print(resp[i]); 
    #endif
  }
    
  char ok[] = "OK";
  if (isSubstring(ok,resp,3,respLen)){
    #ifdef debug_on 
      Serial.println("OK"); 
    #endif
    return true;
  } else {
    #ifdef debug_on 
      Serial.println("Not OK"); 
    #endif
    return false;
  }
}

bool isSubstring(char* substr, char* str, uint16_t subLen, uint16_t strLen) {
  bool isSub = false;
  for (uint16_t i = 0; i < strLen; i++){
//    Serial.print(str[i]);
    if (str[i] == substr[0]){
      for (uint16_t x = 0; x < subLen - 1; x++){
         if (str[i] == substr[x]){
//          Serial.print("*");
//          Serial.print(substr[x]);
//          Serial.print("*");
          isSub = true;
          i++;
         } else {
          isSub = false;
         }
      }
    }
  }

  return isSub;

}

uint16_t sendATcommand(char* ATcommand, uint16_t wait4resp, unsigned int timeout){
  uint8_t x = 0;
  unsigned long previous;
  uint16_t respLen = 0;
  while ( Serial1.available() > 0) Serial1.read();   // Clean the input buffer

  Serial1.write(ATcommand);    // Send the AT command

  x = 0;
  delay(50);
  previous = millis();

  while (((millis() - previous) < timeout)){
    if (Serial1.available() != 0) { 
      delay(wait4resp);  
      respLen = Serial1.available();
      break;
    }
  }
 
  return respLen;
}
