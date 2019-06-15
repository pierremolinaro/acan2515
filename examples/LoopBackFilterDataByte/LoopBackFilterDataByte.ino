//——————————————————————————————————————————————————————————————————————————————
//  ACAN2515 Demo in loopback mode, using hardware SPI, with an external interrupt
//  This code shows how to define and use receive filters
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2515.h>

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 connections: adapt theses settings to your design
//  As hardware SPI is used, you should select pins that support SPI functions.
//  This sketch is designed for a Teensy 3.5, using SPI0 (named SPI)
//  But standard Teensy 3.5 SPI0 pins are not used
//    SCK input of MCP2515 is connected to pin #27
//    SI input of MCP2515 is connected to pin #28
//    SO output of MCP2515 is connected to pin #39
//  User code should configure MCP2515_IRQ pin as external interrupt
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2515_SCK = 27 ; // SCK input of MCP2515
static const byte MCP2515_SI  = 28 ; // SI input of MCP2515
static const byte MCP2515_SO  = 39 ; // SO output of MCP2515

static const byte MCP2515_CS  = 20 ; // CS input of MCP2515
static const byte MCP2515_INT = 37 ; // INT output of MCP2515

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 16 * 1000 * 1000 ; // 16 MHz

//——————————————————————————————————————————————————————————————————————————————
//   RECEIVE FILTERS
//——————————————————————————————————————————————————————————————————————————————
// Extended frames are received by RXB0 --> mask RXM0, acceptance RXF0 and RXF1
// Filters check all 29 bits of identifier
//   acceptance filter #0 --> extended frame, identifier 0x12345678
//   acceptance filter #1 --> extended frame, identifier 0x18765432

// Standard frames are received by RXB1 --> mask RXM1, acceptance RXF2 to RXF5
// Filter check identifier bits from 10 to 4, first byte, second byte is not tested
//   acceptance filter #2 --> standard frame, identifier 0x560 to 0x56F, first byte 0x55
//——————————————————————————————————————————————————————————————————————————————

static void receive0 (const CANMessage & inMessage) {
  Serial.println ("Receive 0") ;
}

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
//--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
//--- Start serial
  Serial.begin (38400) ;
//--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
//--- Define alternate pins for SPI0 (see https://www.pjrc.com/teensy/td_libs_SPI.html)
//    This is only for Teensy
  SPI.setMOSI (MCP2515_SI) ;
  SPI.setMISO (MCP2515_SO) ;
  SPI.setSCK (MCP2515_SCK) ;
//--- Configure SPI
  SPI.begin () ;
//--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 125 * 1000) ; // CAN bit rate 125 kb/s
  settings.mRequestedMode = ACAN2515Settings::LoopBackMode ; // Select loopback mode
  const ACAN2515Mask rxm0 = standard2515Mask (0x7FF, 0xFF, 0) ; // For filter #0 and #1
  const ACAN2515AcceptanceFilter filters [] = {
    {standard2515Filter (0x560, 0x55, 0), receive0}
  } ;
  const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }, rxm0, filters, 1) ;
  if (errorCode != 0) {
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//----------------------------------------------------------------------------------------------------------------------

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gSentFrameCount = 0 ;

//——————————————————————————————————————————————————————————————————————————————

void loop() {
  can.dispatchReceivedMessage () ;
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    switch (gSentFrameCount % 3) {
    case 0 : // Matches filter #0
      frame.id = 0x560 ;
      frame.data [0] = 0x55 ;
      frame.len = 1 ;
      break ;
    case 1 : // Does not match filter #0
      frame.id = 0x560 ;
      frame.data [0] = 0x54 ;
      frame.len = 1 ;
      break ;
    case 2 :  // No data byte, still received
      frame.id = 0x560 ;
      break ;
    }
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentFrameCount) ;
    }else{
      Serial.println ("Send failure") ;
    }
  }
}

//——————————————————————————————————————————————————————————————————————————————
