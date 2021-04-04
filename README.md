## MCP2515 CAN Controller Library for Arduino

### Compatibility with the ACAN library

This library is fully compatible with the Teensy 3.x ACAN library https://github.com/pierremolinaro/acan, it uses a very similar API and the same `CANMessage` class for handling messages.

### ACAN2515 library description
ACAN2515 is a driver for the MCP2515 CAN Controller. It runs on any Arduino compatible board.

You can choose any frequency for your MCP2515, the actual frequency is a parameter of the library.

The driver supports many bit rates: for a 16 MHz quartz, the CAN bit timing calculator finds settings for standard 62.5 kbit/s, 125 kbit/s, 250 kbit/s, 500 kbit/s, 1 Mbit/s, but also for an exotic bit rate as 727 kbit/s. If the desired bit rate cannot be achieved, the `begin` method does not configure the hardware and returns an error code.

> Driver API is fully described by the PDF file in the `extras` directory.

### Demo Sketch

> The demo sketch is in the `examples/LoopBackDemo` directory.

Configuration is a four-step operation.

1. Instanciation of the `settings` object : the constructor has one parameter: the wished CAN bit rate. The `settings` is fully initialized.
2. You can override default settings. Here, we set the `mRequestedMode` property to true, enabling to run demo code without any additional hardware (no CAN transceiver needed). We can also for example change the receive buffer size by setting the `mReceiveBufferSize` property.
3. Calling the `begin` method configures the driver and starts CAN bus participation. Any message can be sent, any frame on the bus is received. No default filter to provide.
4. You check the `errorCode` value to detect configuration error(s).

```cpp
static const byte MCP2515_CS  = 20 ; // CS input of MCP2515, adapt to your design
static const byte MCP2515_INT = 37 ; // INT output of MCP2515, adapt to your design

ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ; // You can use SPI2, SPI3, if provided by your microcontroller

const uint32_t QUARTZ_FREQUENCY = 16 * 1000 * 1000 ; // 16 MHz

void setup () {
  Serial.begin (9600) ;
  while (!Serial) {}
  Serial.println ("Hello") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 125 * 1000) ; // 125 kbit/s
  settings.mRequestedMode = ACAN2515Settings::LoopBackMode ; // Select loopback mode
  const uint16_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (0 == errorCode) {
    Serial.println ("Can ok") ;
  }else{
    Serial.print ("Error Can: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}
```

Now, an example of the `loop` function. As we have selected loop back mode, every sent frame is received.

```cpp
static uint32_t gSendDate = 0 ;
static uint32_t gSentCount = 0 ;
static uint32_t gReceivedCount = 0 ;

void loop () {
  CANMessage message ;
  if (gSendDate < millis ()) {
    message.id = 0x542 ;
    const bool ok = can.tryToSend (message) ;
    if (ok) {
      gSendDate += 2000 ;
      gSentCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentCount) ;
    }
  }
  if (can.receive (message)) {
    gReceivedCount += 1 ;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedCount) ;
  }
}
```
`CANMessage` is the class that defines a CAN message. The `message` object is fully initialized by the default constructor. Here, we set the `id` to `0x542` for sending a standard data frame, without data, with this identifier.

The `can.tryToSend` tries to send the message. It returns `true` if the message has been sucessfully added to the driver transmit buffer.

The `gSendDate` variable handles sending a CAN message every 2000 ms.

`can.receive` returns `true` if a message has been received, and assigned to the `message`argument.

### Use of Optional Reception Filtering

The MCP2515 CAN Controller implements two acceptance masks and six acceptance filters. The driver API enables you to fully manage these registers.

For example (`loopbackUsingFilters` sketch):

```cpp
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 125 * 1000) ;
  settings.mRequestedMode = ACAN2515Settings::LoopBackMode ; // Select loopback mode
  const ACAN2515Mask rxm0 = extended2515Mask (0x1FFFFFFF) ; // For filter #0 and #1
  const ACAN2515Mask rxm1 = standard2515Mask (0x7F0, 0xFF, 0) ; // For filter #2 to #5
  const ACAN2515AcceptanceFilter filters [] = {
    {extended2515Filter (0x12345678), receive0},
    {extended2515Filter (0x18765432), receive1},
    {standard2515Filter (0x560, 0x55, 0), receive2}
  } ;
  const uint16_t errorCode = can.begin (settings, [] { can.isr () ; }, rxm0, rxm1, filters, 3) ;
```

These settings enable the acceptance of extended frames whose identifier is 0x12345678 or 0x18765432, and data frames whose identifier is 0x560 and first data byte, if any, is 0x55.

The `receive0`, `receive1`, `receive2` functions are call back functions, handled by the `can.dispatchReceivedMessage` function:


```cpp
void loop () {
  can.dispatchReceivedMessage () ; // Do not use can.receive any more
  ...
}
```
