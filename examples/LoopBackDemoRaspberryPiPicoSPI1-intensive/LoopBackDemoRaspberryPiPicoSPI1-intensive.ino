//——————————————————————————————————————————————————————————————————————————————
//  ACAN2515 Demo in loopback mode, for the Raspberry Pi Pico
//  Demo sketch that uses SPI1
//——————————————————————————————————————————————————————————————————————————————

#ifndef ARDUINO_ARCH_RP2040
#error "Select a Raspberry Pi Pico board"
#endif

//——————————————————————————————————————————————————————————————————————————————

#include <hardware/pwm.h>
#include <ACAN2515.h>

//——————————————————————————————————————————————————————————————————————————————
// The Pico has two SPI peripherals, SPI and SPI1. Either (or both) can be used.
// The are no default pin assignments so they must be set explicitly.
// Testing was done with Earle Philhower's arduino-pico core:
// https://github.com/earlephilhower/arduino-pico
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2515_SCK  = 14 ; // SCK input of MCP2515 (adapt to your design)
static const byte MCP2515_MOSI = 15 ; // SDI input of MCP2515 (adapt to your design)
static const byte MCP2515_MISO = 12 ; // SDO output of MCP2515 (adapt to your design)

static const byte MCP2515_CS   = 13 ;  // CS input of MCP2515 (adapt to your design)
static const byte MCP2515_INT  = 11 ;  // INT output of MCP2515 (adapt to your design)

static const uint8_t MCP2515_RESET   =  8 ; // MCP2515 reset pin

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can (MCP2515_CS, SPI1, MCP2515_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 20UL * 1000UL * 1000UL ; // 20 MHz

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
//--- Faire un RESET
  digitalWrite (MCP2515_RESET, LOW) ;
  pinMode (MCP2515_RESET, OUTPUT) ;
  delay (10) ;
  digitalWrite (MCP2515_RESET, HIGH) ;
 //--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
  //--- Start serial
  Serial.begin (115200) ;
  //--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  //--- There are no default SPI1 pins so they must be explicitly assigned
  SPI1.setSCK (MCP2515_SCK);
  SPI1.setTX (MCP2515_MOSI);
  SPI1.setRX (MCP2515_MISO);
  SPI1.setCS (MCP2515_CS);
  //--- Begin SPI1
  SPI1.begin () ;
  //--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 125UL * 1000UL) ; // CAN bit rate 125 kb/s
  settings.mRequestedMode = ACAN2515Settings::LoopBackMode ; // Select loopback mode
  const uint16_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Propagation Segment: ") ;
    Serial.println (settings.mPropagationSegment) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW: ") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Triple Sampling: ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
  delay (1000) ;
}

//----------------------------------------------------------------------------------------------------------------------

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;

//——————————————————————————————————————————————————————————————————————————————

static const size_t BUFFER_SIZE = 40 ; 
static uint32_t gBuffer [BUFFER_SIZE] ;
static size_t gCount = 0 ;
static size_t gReadIndex = 0 ;
static uint32_t gIdentifier = 0 ;

//——————————————————————————————————————————————————————————————————————————————

void loop () {
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 1000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedFrameCount) ;
  }
  if (gCount < BUFFER_SIZE) {
    CANMessage frame ;
    frame.id = gIdentifier ;
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      const size_t writeIndex = (gReadIndex + gCount) % BUFFER_SIZE ;
      gBuffer [writeIndex] = gIdentifier ;
      gCount += 1 ;
      gIdentifier += 1 ;
      gIdentifier &= 0x7FF ;
   }
  }
  CANMessage frame ;
  while (can.receive (frame)) {
    gReceivedFrameCount += 1 ;
    if (gCount == 0) {
      Serial.println ("gBuffer underflow") ;
    }else{
      const uint32_t identifier = gBuffer [gReadIndex] ;
      gReadIndex += 1 ;
      gReadIndex %= BUFFER_SIZE ;
      gCount -= 1 ;
      if (identifier != frame.id) {
        Serial.println ("identifier error") ;
      }
    }
  }
}

//——————————————————————————————————————————————————————————————————————————————
