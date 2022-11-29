//——————————————————————————————————————————————————————————————————————————————
//  ACAN2515 Demo in loopback mode, for ESP32
//——————————————————————————————————————————————————————————————————————————————

#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board"
#endif

//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2515.h>

//——————————————————————————————————————————————————————————————————————————————
//  For using SPI on ESP32, see demo sketch SPI_Multiple_Buses
//  Two SPI busses are available in Arduino, HSPI and VSPI.
//  By default, Arduino SPI uses VSPI, leaving HSPI unused.
//  Default VSPI pins are: SCK=18, MISO=19, MOSI=23.
//  You can change the default pin with additional begin arguments
//    SPI.begin (MCP2515_SCK, MCP2515_MISO, MCP2515_MOSI)
//  CS input of MCP2515 should be connected to a digital output port
//  INT output of MCP2515 should be connected to a digital input port, with interrupt capability
//  Notes:
//    - GPIOs 34 to 39 are GPIs – input only pins. These pins don’t have internal pull-ups or
//      pull-down resistors. They can’t be used as outputs.
//    - some pins do not support INPUT_PULLUP (see https://www.esp32.com/viewtopic.php?t=439)
//    - All GPIOs can be configured as interrupts
// See https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2515_SCK  = 26 ; // SCK input of MCP2515
static const byte MCP2515_MOSI = 19 ; // SDI input of MCP2515
static const byte MCP2515_MISO = 18 ; // SDO output of MCP2515

static const byte MCP2515_CS  = 17 ; // CS input of MCP2515 (adapt to your design)
static const byte MCP2515_INT = 23 ; // INT output of MCP2515 (adapt to your design)
static const byte MCP2515_RESET = 27 ; // RESET input of MCP2515 (adapt to your design)

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 20UL * 1000UL * 1000UL ; // 20 MHz

//-----------------------------------------------------------------

static ACAN2515_Buffer16 gBuffer ;

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
  gBuffer.initWithSize (25) ;
//--- RESET MCP2515
  pinMode (MCP2515_RESET, OUTPUT) ;
  digitalWrite (MCP2515_RESET, LOW) ;
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
//--- Begin SPI
  SPI.begin (MCP2515_SCK, MCP2515_MISO, MCP2515_MOSI) ;
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
}

//——————————————————————————————————————————————————————————————————————————————
//   pseudoRandomValue
//——————————————————————————————————————————————————————————————————————————————

static uint32_t pseudoRandomValue (void) {
  static uint32_t gSeed = 0 ;
  gSeed = 8253729U * gSeed + 2396403U ;
  return gSeed ;
}

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

static uint32_t gBlinkLedDate = 2000 ;
static uint32_t gExtendedFrameCount = 0 ;
static uint32_t gStandardFrameCount = 0 ;
static uint32_t gRemoteFrameCount = 0 ;
static uint32_t gCanDataFrameCount = 0 ;
static uint32_t gReceiveCount = 0 ;

static bool gOk = true ;

//——————————————————————————————————————————————————————————————————————————————

void loop () {
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print ("Counters: ") ;
    Serial.print (gStandardFrameCount) ;
    Serial.print (", ") ;
    Serial.print (gExtendedFrameCount) ;
    Serial.print (", ") ;
    Serial.print (gCanDataFrameCount) ;
    Serial.print (", ") ;
    Serial.print (gRemoteFrameCount) ;
    Serial.print (", ") ;
    Serial.println (gReceiveCount) ;
  }
//--- Send Message
  const uint8_t sendBufferIndex = 0 ;
  if (gOk && (!gBuffer.isFull ()) && can.sendBufferNotFullForIndex (sendBufferIndex)) {
    CANMessage frame ;
    frame.idx = sendBufferIndex ;
    const uint32_t r = pseudoRandomValue () ;
    frame.ext = (r & (1 << 29)) != 0 ;
    frame.rtr = (r & (1 << 30)) != 0 ;
    frame.id = r & 0x1FFFFFFFU ;
    if (frame.ext) {
      gExtendedFrameCount += 1 ;
    }else{
      gStandardFrameCount += 1 ;
      frame.id &= 0x7FFU ;
    }
    frame.len = pseudoRandomValue () % 9 ;
    if (frame.rtr) {
      gRemoteFrameCount += 1 ;
    }else{
      gCanDataFrameCount += 1 ;
      for (uint32_t i=0 ; i<frame.len ; i++) {
        frame.data [i] = uint8_t (pseudoRandomValue ()) ;
      }
    }    
    gBuffer.append (frame) ;
    const bool sendStatusIsOk = can.tryToSend (frame) ;
    if (!sendStatusIsOk) {
      gOk = false ;
      Serial.println ("Send status error") ;
    }
  }
//--- Receive message
  CANMessage receivedFrame ;
  if (gOk && can.receive (receivedFrame)) {
    CANMessage storedFrame ;
    gOk = gBuffer.remove (storedFrame) ;
    if (!gOk) {
      Serial.println ("gBuffer is empty") ;
    }else{    
      gReceiveCount += 1 ;
      bool sameFrames = storedFrame.id == receivedFrame.id ;
      if (sameFrames) {
        sameFrames = storedFrame.ext == receivedFrame.ext ;
      }
      if (sameFrames) {
        sameFrames = storedFrame.rtr == receivedFrame.rtr ;
      }
      if (sameFrames) {
        sameFrames = storedFrame.len == receivedFrame.len ;
      }
      if (!storedFrame.rtr) {
        for (uint32_t i=0 ; (i<receivedFrame.len) && sameFrames ; i++) {
          sameFrames = storedFrame.data [i] == receivedFrame.data [i] ;
        }
      }
      if (!sameFrames) {
        gOk = false ;
        Serial.println ("Receive error") ;
        Serial.print ("  IDF: 0x") ;
        Serial.print (storedFrame.id, HEX) ;
        Serial.print (" :: 0x") ;
        Serial.println (receivedFrame.id, HEX) ;
        Serial.print ("  EXT: ") ;
        Serial.print (storedFrame.ext) ;
        Serial.print (" :: ") ;
        Serial.println (receivedFrame.ext) ;
        Serial.print ("  RTR: ") ;
        Serial.print (storedFrame.rtr) ;
        Serial.print (" :: ") ;
        Serial.println (receivedFrame.rtr) ;
        Serial.print ("  LENGTH: ") ;
        Serial.print (storedFrame.len) ;
        Serial.print (" :: ") ;
        Serial.println (receivedFrame.len) ;     
      }
    }
  }
}

//——————————————————————————————————————————————————————————————————————————————
