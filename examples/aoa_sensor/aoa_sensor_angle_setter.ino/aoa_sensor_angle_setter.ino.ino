/******************************************************************************
This sketch is used to set various angles on your AoA Sensor Arduino package.
You will have three buttons, one for setting the level flight angle (0g), 
another to set the Warning angle (Warn), and one to set the Stall angle (Stall).

The idea is that you would have an open CAN port somewhere in your airplane
and connect the MCP2515 to the bus. You will then fly on a smooth day, level at 
cruise and:
1. press the 0g button, which will send a CAN-FIX message to the AOA
sensor Arduino, which is listening for that message. Once the message is 
received, the AoA Arduino will take whatever the current reading on its sensor 
and write it to a specific location in EEPROM (bytes 0-3 - Arduinos require 4 
bytes to store a float value). 
2. you will then slow the airplane down and pull the nose up to a point where 
you feel a little uncomfortable, but still a bit before a stall. Press the 
Warn button and another CAN-FIX message will be sent to the AoA 

Then
*******************************************************************************/
// #include <EEPROM.h>

#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

#include <Wire.h>
#include <SPI.h>

#define META_MIN      0b00010000
#define META_MAX      0b00100000
#define META_ZERO_G   0b01110000
#define META_WARN     0b10000000
#define META_STALL    0b10010000
#define CAN0_INT      2

#define ZERO_G_PIN  6
#define WARN_PIN    7
#define STALL_PIN   8

MCP_CAN CAN0(10);
CanFix cf(0x7A);

unsigned long now;
unsigned long lastMessageTimerMillis;
unsigned long messageSendInterval = 100;

void setup() {
  Serial.println("Begin Setup");
  Serial.begin(115200);
  
  cf.setDeviceId(0x7A);
  cf.setModel(0x54321);
  cf.setFwVersion(2);
 
  cf.set_write_callback(can_write_callback);

  // pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ZERO_G_PIN, INPUT);
  pinMode(WARN_PIN, INPUT);
  pinMode(STALL_PIN, INPUT);

  delay(250);

  while(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Unable to begin CAN0");
    delay(1000);
  }

  Serial.println("Setup complete");

  lastMessageTimerMillis = now = millis();
  
}

void loop() {


  unsigned long now = millis();
  if (now - lastMessageTimerMillis > messageSendInterval) {
    if (digitalRead(ZERO_G_PIN) == HIGH) {
      int data = 0x00;
      // SetAngle(META_ZERO_G, data);
      SetAngle(data);
    }

    if (digitalRead(WARN_PIN) == HIGH) {
      int data = 0x01;
      // SetAngle(META_WARN, data);
      SetAngle(data);
    }

    if (digitalRead(STALL_PIN) == HIGH) {
      int data = 0x02;
      // SetAngle(STALL_PIN, data);
      SetAngle(data);
    }
    lastMessageTimerMillis = now;
  }
  
  

}

/*********************************************************************
  This function is a callback function that is used by the CanFix object
  for sending CAN frames on the network.  The CanFix class is agnostic
  toward the actual CAN communication mechanism.  This function should
  translate from the common CanFixFrame structure and send it on the Bus.
**********************************************************************/
void can_write_callback(CanFixFrame frame) {
  CAN0.sendMsgBuf(frame.id, 0, frame.length, frame.data);
}


/*********************************************************************
  This function receives an meta type to change and an angle to set
  for that meta type. Then it pushes the update over the CAN bus for 
  use on the EFIS screen
*********************************************************************/
// void SetAngle(byte metaType, int data) {
void SetAngle(int buttonId) {

  // signed int x = angle * 100;
  CFParameter p;

  p.type = 0x30F; // Custom message type
  p.index = 0x00;
  // p.fcb = metaType;
  p.fcb = 0x00;
  // x = data*100;
  p.data[0] = buttonId;
  p.data[1] = buttonId>>8;
  p.data[2] = buttonId>>16;
  p.data[3] = buttonId>>24;
  p.length = 7;
  cf.sendParam(p);

  // Serial.print("metaType: ");
  // Serial.print(metaType);
  Serial.print("\tbuttonId: ");
  Serial.println(buttonId);

}

