/* SDI-12 library for Acclima Node based off Stroud Water Research Center Arduino-SDI-12 library
 *  
 *  .cpp file
 *  
 *  Alondra Thompson
 *  December 2018
 */

 /*=========== 0. Includes, Defines, & Variable Declarations =============
*/

#include "AcclimaSDI12.h"                   // Header file for this library
//#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "SDI12_boards1.h"

SDI12 *SDI12::_activeObject = NULL;  // Pointer to active SDI12 object

static const uint16_t bitWidth_micros = (uint16_t) 833;  // The size of a bit in microseconds  - (originally 833)
    // 1200 baud = 1200 bits/second ~ 833.333 µs/bit
static const uint16_t lineBreak_micros = (uint16_t) 13000;  // The required "break" before sending commands - was 12100 (John: 15000)
    // break >= 12ms
static const uint16_t marking_micros = (uint16_t) 9000;  // The required mark before a command or response - was 8330 (John: 10000)
    // marking >= 8.33ms

static const uint8_t txBitWidth = TICKS_PER_BIT;
static const uint8_t rxWindowWidth = RX_WINDOW_FUDGE;  // A fudge factor to make things work
static const uint8_t bitsPerTick_Q10 = BITS_PER_TICK_Q10;
static const uint8_t WAITING_FOR_START_BIT = 0xFF;  // 0b11111111

static uint16_t prevBitTCNT;     // previous RX transition in micros
static uint8_t rxState;          // 0: got start bit; >0: bits rcvd
static uint8_t rxMask;           // bit mask for building received character
static uint8_t rxValue;          // character being built

// static method for getting a 16-bit value from 2 8-bit values
static uint16_t mul8x8to16(uint8_t x, uint8_t y)
{return x*y;}

// static method for calculating the number of bit-times that have elapsed
static uint16_t bitTimes( uint8_t dt )
{
  return mul8x8to16( dt + rxWindowWidth, bitsPerTick_Q10 ) >> 10;
} // bitTimes




/* =========== 1. Buffer Setup ============================================
 */  

uint8_t SDI12::_rxBuffer[SDI12_BUFFER_SIZE];  // 1.1 - buff for incoming
volatile uint8_t SDI12::_rxBufferTail = 0;    // 1.2 - index of buff head
volatile uint8_t SDI12::_rxBufferHead = 0;    // 1.3 - index of buff tail

/* =========== 2. Reading from the SDI-12 buffer  ==========================
 */
// 2.1 - reveals the number of characters available in the buffer
int SDI12::available()
{
//  pinMode(Tx_En,OUTPUT);
//  digitalWrite(Tx_En,HIGH);
  
  if(_bufferOverflow) return -1;
  return (_rxBufferTail + SDI12_BUFFER_SIZE - _rxBufferHead) % SDI12_BUFFER_SIZE;
}

// 2.2 - reveals the next character in the buffer without consuming
int SDI12::peek()
{
//  pinMode(Tx_En,OUTPUT);
//  digitalWrite(Tx_En,LOW);
  if (_rxBufferHead == _rxBufferTail) return -1;  // Empty buffer? If yes, -1
  return _rxBuffer[_rxBufferHead];                // Otherwise, read from "head"
}

// 2.3 - a public function that clears the buffer contents and
// resets the status of the buffer overflow.
void SDI12::clearBuffer()
{
//  pinMode(Tx_En,OUTPUT);
//  digitalWrite(Tx_En,LOW);
  _rxBufferHead = _rxBufferTail = 0;
  _bufferOverflow = false;
}

// 2.4 - reads in the next character from the buffer (and moves the index ahead)
int SDI12::read()
{
 // pinMode(Tx_En,OUTPUT);
//  digitalWrite(Tx_En,LOW);
//  delay(2);
  
  _bufferOverflow = false;                             // Reading makes room in the buffer
  if (_rxBufferHead == _rxBufferTail) return -1;       // Empty buffer? If yes, -1
  uint8_t nextChar = _rxBuffer[_rxBufferHead];         // Otherwise, grab char at head
  _rxBufferHead = (_rxBufferHead + 1) % SDI12_BUFFER_SIZE;  // increment head
  return nextChar;                                     // return the char
}

// 2.5 - these functions hide the stream equivalents to return a custom timeout value
int SDI12::peekNextDigit(LookaheadMode lookahead, bool detectDecimal)
{
  int c;
  while (1) {
    c = timedPeek();

    if( c < 0 ||
        c == '-' ||
        (c >= '0' && c <= '9') ||
        (detectDecimal && c == '.')) return c;

    switch( lookahead ){
        case SKIP_NONE: return -1; // Fail code.
        case SKIP_WHITESPACE:
            switch( c ){
                case ' ':
                case '\t':
                case '\r':
                case '\n': break;
                default: return -1; // Fail code.
            }
        case SKIP_ALL:
            break;
    }
    read();  // discard non-numeric
  }
}

long SDI12::parseInt(LookaheadMode lookahead, char ignore)
{
  bool isNegative = false;
  long value = 0;
  int c;

  c = peekNextDigit(lookahead, false);
  // ignore non numeric leading characters
  if(c < 0)
    return TIMEOUT; // TIMEOUT returned if timeout
    //  THIS IS THE ONLY DIFFERENCE BETWEEN THIS FUNCTION AND THE STREAM DEFAULT!

  do{
    if(c == ignore)
      ; // ignore this character
    else if(c == '-')
      isNegative = true;
    else if(c >= '0' && c <= '9')        // is c a digit?
      value = value * 10 + c - '0';
    read();  // consume the character we got with peek
    c = timedPeek();
  }
  while( (c >= '0' && c <= '9') || c == ignore );

  if(isNegative)
    value = -value;
  return value;
}

// the same as parseInt but returns a floating point value
float SDI12::parseFloat(LookaheadMode lookahead, char ignore)
{
  bool isNegative = false;
  bool isFraction = false;
  long value = 0;
  int c;
  float fraction = 1.0;

  c = peekNextDigit(lookahead, true);
    // ignore non numeric leading characters
  if(c < 0)
    return TIMEOUT; // TIMEOUT returned if timeout
    //  THIS IS THE ONLY DIFFERENCE BETWEEN THIS FUNCTION AND THE STREAM DEFAULT!

  do{
    if(c == ignore)
      ; // ignore
    else if(c == '-')
      isNegative = true;
    else if (c == '.')
      isFraction = true;
    else if(c >= '0' && c <= '9')  {      // is c a digit?
      value = value * 10 + c - '0';
      if(isFraction)
         fraction *= 0.1;
    }
    read();  // consume the character we got with peek
    c = timedPeek();
  }
  while( (c >= '0' && c <= '9')  || (c == '.' && !isFraction) || c == ignore );

  if(isNegative)
    value = -value;
  if(isFraction)
    return value * fraction;
  else
    return value;
}

 /* ======= 3. Constructor, Destructor, SDI12.begin(), and SDI12.end()  =======
  */
  
//  3.1 Constructor
//SDI12::SDI12(bool initSDI12){
//  if (initSDI12){
//  _bufferOverflow = false;
//
//}
//}

SDI12::SDI12(){
  _bufferOverflow = false;
 }

SDI12::SDI12(uint8_t dataPin){
  _bufferOverflow = false;
  _dataPin = dataPin;
}

//  3.2 Destructor
SDI12::~SDI12(){
  setState(DISABLED);
  _activeObject = NULL;
  // Set the timer prescalers back to original values
  // NOTE:  This does NOT reset SAMD board pre-scalers!
  resetSDI12TimerPrescale();
}

//  3.3 Begin
void SDI12::begin(){
  // setState(HOLDING);
    //_dataPin = dataPin;
  setActive();
  // SDI-12 protocol says sensors must respond within 15 milliseconds
  // We'll bump that up to 150, just for good measure, but we don't want to
  // wait the whole stream default of 1s for a response.
  setTimeout(150);
  // Because SDI-12 is mostly used for environmental sensors, we want to be able
  // to distinguish between the '0' that parseInt and parseFloat usually return
  // on timeouts and a real measured 0 value.  So we force the timeout response
  // to be -9999, which is not a common value for most variables measured by
  // in-site environmental sensors.
  setTimeoutValue(-9999);
  // Set up the prescaler as needed for timers
  // This function is defined in SDI12_boards.h
  configSDI12TimerPrescale();
}
void SDI12::begin(uint8_t dataPin){
  _dataPin = dataPin;
  begin();
}

//  3.4 End
void SDI12::end()
{
  setState(DISABLED);
  _activeObject = NULL;
  // Set the timer prescalers back to original values
  // NOTE:  This does NOT reset SAMD board pre-scalers!
  resetSDI12TimerPrescale();
}

//  3.5 Set the timeout return
void SDI12::setTimeoutValue(int value) { TIMEOUT = value; }

//  3.6 Return the data pin for the SDI-12 instance
uint8_t SDI12::getDataPin() { return _dataPin;}

/* =========== 4. Using more than one SDI-12 object  ==========================
 */

// 4.1 - a method for setting the current object as the active object
bool SDI12::setActive()
{
  if (_activeObject != this)
  {
    setState(HOLDING);
    _activeObject = this;
    return true;
  }
  return false;
}

// 4.2 - a method for checking if this object is the active object
bool SDI12::isActive() { return this == _activeObject; }

/* =========== 5. Data Line States ===============================
 */

// 5.1 - Processor specific parity and interrupts

#if defined _AVR_
  #include <avr/interrupt.h>      // interrupt handling
  #include <util/parity.h>        // optimized parity bit handling
#else
uint8_t SDI12::parity_even_bit(uint8_t v)
{
 uint8_t parity = 0;
  while (v)
  {
    parity = !parity;
    v = v & (v - 1);
  }
  return parity;
} 
#endif

// 5.2 - a helper function to switch pin interrupts on or off
void SDI12::setPinInterrupts(bool enable)
{
//  #ifndef SDI12_EXTERNAL_PCINT
      if (enable)
    {
      //#if defined _AVR_
//        *digitalPinToPCICR(_dataPin) |= (1<<digitalPinToPCICRbit(_dataPin));
//        *digitalPinToPCMSK(_dataPin) |= (1<<digitalPinToPCMSKbit(_dataPin));  
      
         PCICR |= (1 << PCIE3); 
           PCMSK3 |= (1 << PCINT26);
       
    }
    else
    {
      *digitalPinToPCMSK(_dataPin) &= ~(1<<digitalPinToPCMSKbit(_dataPin));  
      if(!*digitalPinToPCMSK(_dataPin)){
        *digitalPinToPCICR(_dataPin) &= ~(1<<digitalPinToPCICRbit(_dataPin));
      }
 
      // PCICR &= (1 << PCIE3);
    }
}

// 5.3 - sets the state of the SDI-12 object.
void SDI12::setState(SDI12_STATES state){
  switch (state)
  {
    case HOLDING:
    {
      pinMode(Tx_En, INPUT);
      pinMode(Tx_En, OUTPUT);  
      digitalWrite(Tx_En, HIGH);      // set uC to Tx since no comm expected from sensor
      
      pinMode(_dataPin,INPUT);      // added to make output work after pinMode to OUTPUT (don't know why, but works)
      pinMode(_dataPin,OUTPUT);     // Pin mode = output
      digitalWrite(_dataPin,LOW);   // Pin state = low
      setPinInterrupts(false);      // Interrupts disabled on data pin
      break;
    }
    case TRANSMITTING:
    {
      pinMode(Tx_En, INPUT);
      pinMode(Tx_En, OUTPUT);  
      digitalWrite(Tx_En, HIGH);  // uC transmits to sensor
      delay(2);
      
      pinMode(_dataPin,INPUT);   // added to make output work after pinMode to OUTPUT (don't know why, but works)
      pinMode(_dataPin,OUTPUT);  // Pin mode = output
      setPinInterrupts(false);   // Interrupts disabled on data pin
      break;
    }
    case LISTENING:
    {
      pinMode(Tx_En, OUTPUT);
      digitalWrite(Tx_En, LOW);
      delay(2);
      
      digitalWrite(_dataPin,LOW);   // Pin state = low
      pinMode(_dataPin,INPUT);      // Pin mode = input
      interrupts();                 // Enable general interrupts
      setPinInterrupts(true);       // Enable Rx interrupts on data pin
      rxState = WAITING_FOR_START_BIT;
      break;
    }
    default:  // DISABLED or ENABLED
    {
      pinMode(Tx_En, OUTPUT);
      digitalWrite(Tx_En, HIGH);
      
      digitalWrite(_dataPin,LOW);   // Pin state = low
      pinMode(_dataPin,INPUT);      // Pin mode = input
      setPinInterrupts(false);      // Interrupts disabled on data pin
      break;
    }
  }
}

// 5.4 - forces a HOLDING state.
void SDI12::forceHold(){
    setState(HOLDING);
}

// 5.5 - forces a LISTENING state.
void SDI12::forceListen(){
    setState(LISTENING);
}

/* ============= 6. Waking up, and talking to, the sensors. ===================
 */

// 6.1 - this function wakes up the entire sensor bus
void SDI12::wakeSensors() {
  setState(TRANSMITTING);
  // Universal interrupts can be on while the break and marking happen because
  // timings for break and from the recorder are not critical.
  // Interrupts on the pin are disabled for the entire transmitting state
  digitalWrite(_dataPin, HIGH);
  delayMicroseconds(lineBreak_micros);  // Required break of 12 milliseconds
  digitalWrite(_dataPin, LOW);
  delayMicroseconds(marking_micros);  // Required marking of 8.33 milliseconds
}

// 6.2 - this function writes a character out on the data line
void SDI12::writeChar(uint8_t outChar) {
  uint8_t currentTxBitNum = 0; // first bit is start bit
  uint8_t bitValue = 1; // start bit is HIGH (inverse parity...)

  noInterrupts();  // _ALL_ interrupts disabled so timing can't be shifted

  uint8_t t0 = TCNTX; // start time
  digitalWrite(_dataPin, HIGH);  // immediately get going on the start bit
  // this gives us 833µs to calculate parity and position of last high bit
  currentTxBitNum++;

  uint8_t parityBit = parity_even_bit(outChar);  // Calculate the parity bit
  outChar |= (parityBit<<7);  // Add parity bit to the outgoing character

  // Calculate the position of the last bit that is a 0/HIGH (ie, HIGH, not marking)
  // That bit will be the last time-critical bit.  All bits after that can be
  // sent with interrupts enabled.

  uint8_t lastHighBit = 9;  // The position of the last bit that is a 0 (ie, HIGH, not marking)
  uint8_t msbMask = 0x80;  // A mask with all bits at 1
  while (msbMask & outChar) {
    lastHighBit--;
    msbMask >>= 1;
  }

  // Hold the line for the rest of the start bit duration
  while ((uint8_t)(TCNTX - t0) < txBitWidth) {}
  t0 = TCNTX; // advance start time

  // repeat for all data bits until the last bit different from marking
  while (currentTxBitNum++ < lastHighBit) {
    bitValue = outChar & 0x01;  // get next bit in the character to send
    if (bitValue){
      digitalWrite(_dataPin, LOW);  // set the pin state to LOW for 1's
    }
    else{
      digitalWrite(_dataPin, HIGH);  // set the pin state to HIGH for 0's
    }
    // Hold the line for this bit duration
    while ((uint8_t)(TCNTX - t0) < txBitWidth) {}
    t0 = TCNTX; // advance start time
    outChar = outChar >> 1;  // shift character to expose the following bit
  }

  // Set the line low for the all remaining 1's and the stop bit
  digitalWrite(_dataPin, LOW);

  interrupts(); // Re-enable universal interrupts as soon as critical timing is past

  // Hold the line low until the end of the 10th bit
  uint8_t bitTimeRemaining = txBitWidth*(10-lastHighBit);
  while ((uint8_t)(TCNTX - t0) < bitTimeRemaining) {}

}

// The typical write functionality for a stream object
// This allows you to use the stream print functions to send commands out on
// the SDI-12, line, but it will not wake the sensors in advance of the command.
size_t SDI12::write(uint8_t byte) {
  setState(TRANSMITTING);
  writeChar(byte);         // write the character/byte
  setState(LISTENING);       // listen for reply
  return 1;                  // 1 character sent
}

//    6.3    - this function sends out the characters of the String cmd, one by one
void SDI12::sendCommand(String &cmd) {
//  pinMode(Tx_En, OUTPUT);
//  digitalWrite(Tx_En, HIGH); // datalogger in Tx mode
  
  wakeSensors();             // set state to transmitting and send break/marking
  for (int unsigned i = 0; i < cmd.length(); i++){
    writeChar(cmd[i]);       // write each character
  }
//  digitalWrite(Tx_En, LOW);  // datalogger in Rx mode
  setState(LISTENING);       // listen for reply
}

void SDI12::sendCommand(const char *cmd) {
//  pinMode(Tx_En, OUTPUT);
//  digitalWrite(Tx_En, HIGH); // datalogger in Tx mode
  
  wakeSensors();             // wake up sensors
  for (int unsigned i = 0; i < strlen(cmd); i++){
    writeChar(cmd[i]);      // write each character
  }
//  digitalWrite(Tx_En, LOW);  // datalogger in Rx mode
  setState(LISTENING);      // listen for reply
}

void SDI12::sendCommand(FlashString cmd) {
//  pinMode(Tx_En, OUTPUT);
//  digitalWrite(Tx_En, HIGH); // datalogger in Tx mode
  
  wakeSensors();            // wake up sensors
  for (int unsigned i = 0; i < strlen_P((PGM_P)cmd); i++){
    writeChar((char)pgm_read_byte((const char *)cmd + i));  // write each character
  }
//  digitalWrite(Tx_En, LOW);  // datalogger in Rx mode
  setState(LISTENING);      // listen for reply
}



/* ============== 7. Interrupt Service Routine  ===================
 */

// 7.1 - Passes off responsibility for the interrupt to the active object.
void SDI12::handleInterrupt(){
  if (_activeObject) _activeObject->receiveISR();
}

// 7.2 - Creates a blank slate of bits for an incoming character
void SDI12::startChar()
{
  rxState = 0;           // got a start bit
  rxMask  = 0x01;  // 0b00000001, bit mask, lsb first
  rxValue = 0x00;  // 0b00000000, RX character to be, a blank slate
} // startChar

// 7.3 - The actual interrupt service routine
void SDI12::receiveISR()
{
  uint8_t thisBitTCNT = TCNTX;               // time of this data transition (plus ISR latency)
  uint8_t pinLevel = digitalRead(_dataPin);  // current RX data level

  // Check if we're ready for a start bit, and if this could possibly be it
  // Otherwise, just ignore the interrupt and exit
  if (rxState == WAITING_FOR_START_BIT) {
     // If it is low it's not a start bit, exit
     // Inverse logic start bit = HIGH
    if (pinLevel == LOW) {
      return;
    }
    // If it is HIGH, this should be a start bit
    // Thus set the rxStat to 0, create an empty character, and a new mask with a 1 in the lowest place
    startChar();
  }

  // if the character is incomplete, and this is not a start bit,
  // then this change is from a data, parity, or stop bit
  else {

    // check how many bit times have passed since the last change
    // the rxWindowWidth is just a fudge factor
    uint16_t rxBits = bitTimes(thisBitTCNT - prevBitTCNT);
    // Serial.println(rxBits);
    // calculate how many *data+parity* bits should be left
    // We know the start bit is past and are ignoring the stop bit (which will be LOW/1)
    // We have to treat the parity bit as a data bit because we don't know its state
    uint8_t bitsLeft = 9 - rxState;
    // note that a new character *may* have started if more bits have been
    // received than should be left.
    // This will also happen if the parity bit is 1 or the last bit(s) of the
    // character and the parity bit are all 1's.
    bool nextCharStarted = (rxBits > bitsLeft);

    // check how many data+parity bits have been sent in this frame
    // If the total number of bits in this frame is more than the number of data+parity
    // bits remaining in the character, then the number of data+parity bits is equal
    // to the number of bits remaining for the character and partiy.  If the total
    // number of bits in this frame is less than the number of data bits left
    // for the character and parity, then the number of data+parity bits received
    // in this frame is equal to the total number of bits received in this frame.
    // translation:
    //    if nextCharStarted then bitsThisFrame = bitsLeft
    //                       else bitsThisFrame = rxBits
    uint8_t bitsThisFrame = nextCharStarted ? bitsLeft : rxBits;
    // Tick up the rxState by that many bits
    rxState += bitsThisFrame;

    // Set all the bits received between the last change and this change
    // If the current state is HIGH (and it just became so), then all bits between
    // the last change and now must have been LOW.
    if (pinLevel == HIGH) {
      // back fill previous bits with 1's (inverse logic - LOW = 1)
      while (bitsThisFrame-- > 0) {
        rxValue |= rxMask;  // Add a 1 to the LSB/right-most place
        rxMask   = rxMask << 1;  // Shift the 1 in the mask up by one position
      }
      rxMask = rxMask << 1;  // Shift the 1 in the mask up by one more position
    }
    // If the current state is LOW (and it just became so), then this bit is LOW
    // but all bits between the last change and now must have been HIGH
    else { // pinLevel==LOW
      // previous bits were 0's so only this bit is a 1 (inverse logic - LOW = 1)
      rxMask   = rxMask << (bitsThisFrame-1);  // Shift the 1 in the mask up by the number of bits past
      rxValue |= rxMask;  //  Add that shifted one to the character being created
    }

    // If this was the 8th or more bit then the character and parity are complete.
    if (rxState > 7) {
      rxValue &= 0x7F;  // 0b01111111, Throw away the parity bit
      charToBuffer(rxValue);  // Put the finished character into the buffer


      // if this is LOW, or we haven't exceeded the number of bits in a
      // character (but have gotten all the data bits) then this should be a
      // stop bit and we can start looking for a new start bit.
      if ((pinLevel == LOW) || !nextCharStarted) {
        rxState = WAITING_FOR_START_BIT;  // DISABLE STOP BIT TIMER
      } else {
        // If we just switched to HIGH, or we've exceeded the total number of
        // bits in a character, then the character must have ended with 1's/LOW,
        // and this new 0/HIGH is actually the start bit of the next character.
        startChar();
      }
    }

  }
  prevBitTCNT = thisBitTCNT;  // remember time stamp of this change!
}

// 7.4 - Put a new character in the buffer
void SDI12::charToBuffer( uint8_t c )
{
  // Check for a buffer overflow. If not, proceed.
  if ((_rxBufferTail + 1) % SDI12_BUFFER_SIZE == _rxBufferHead)
    { _bufferOverflow = true; }
  // Save the character, advance buffer tail.
  else
  {
     _rxBuffer[_rxBufferTail] = c;
     _rxBufferTail = (_rxBufferTail + 1) % SDI12_BUFFER_SIZE;
  }
}

// 7.5 - Define AVR Interrupts

//#if defined _AVR_

  //  #if defined(PCINT3_vect)
    ISR(PCINT3_vect){SDI12::handleInterrupt();}
  //  #endif

 //#endif

//SDI12 SDI12port;
 
