
#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <REG.h>
#include <wit_c_sdk.h>

#include <Wire.h>


// #define CAN0_INT 2
// #define MESSAGE_SEND_INTERVAL 200
// MCP_CAN CAN0(10);

// CanFix cf(0x80);

unsigned long now;
unsigned long lastParameterTimer[5];
float fAcc[3], fGyro[3], fAngle[3];

//signed int pitchAngle = 0;
// bool countUp = true;

void setup() {


  
  Serial.begin(115200);

  //WitInit(NORMAL, 0x50);
  WitInit(WIT_PROTOCOL_I2C, 0x50);

  // cf.setDeviceId(0x80);
  // cf.setModel(0x12346);
  // cf.setFwVersion(3);
  // cf.set_write_callback(can_write_callback);

  delay(250);
  // Wire.begin();

  // while(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
  //   Serial.println("Unable to begin CAN0");
  //   delay(1000);
  // }

  // CAN0.setMode(MCP_NORMAL);
  // pinMode(CAN0_INT, INPUT);

  // connect to the WT901B IMU
  
}

void loop() {

  // unsigned long rxId;
  // unsigned char len = 0;
  // char msgString[128];
  // unsigned char rxBuf[8];

  now = millis();

  if (now - lastParameterTimer[0] > 1000) {
    Serial.println("Doing something...");
    lastParameterTimer[0] = now;
  }

  // attempt to read the data on the Rx/Tx pins

  // if a message is found, put the meesage into a buffer

  // parse the message

  // send the message to the USB device (Android)
  




  
  //Check to see if the CAN0 interrupt pin is low (meaning there's a message waiting)
  // if (!digitalRead(CAN0_INT)) {
    
  //   // read the buffered message and assign it to a couple of variables
  //   CAN0.readMsgBuf(&rxId, &len, rxBuf);

  //   // verify that the message is not an extended type
  //   if ((rxId & 0x80000000) != 0x80000000) {

  //     // Turn the message into a CanFixFrame
  //     CanFixFrame frame;
  //     frame.id = rxId;
  //     frame.length = len;
  //     memcpy(frame.data, rxBuf, 7);
  //     cf.exec(frame);

  //     // Serial.println(frame.id, HEX);
  //     // Serial.println(frame.length, DEC);
  //     // Serial.print(frame.data[0], HEX);
  //     // Serial.print(frame.data[1], HEX);
  //     // Serial.print(frame.data[2], HEX);
  //     // Serial.print(frame.data[3], HEX);
  //     // Serial.print(frame.data[4], HEX);
  //     // Serial.println(frame.data[5], HEX);
  //     // Serial.println("\n");

  //     send_canfix_frame_to_serial(frame);

  //   }
      

  //   //   //send_canfix_frame_to_serial(frame);

  //   // }

    
    
  // }

  
  

  //delay(10);
}



/*********************************************************************
  This function converts CanFix messages to Serial and sends them over
  to the Android device connected on the USB port.
**********************************************************************/
void send_canfix_frame_to_serial(CanFixFrame frame) {
  
  byte message [] = {
    frame.id,
    frame.id >> 8,
    frame.id >> 16,
    frame.data[0],
    frame.data[1],
    frame.data[2],
    frame.data[3],
    frame.data[4],
    frame.data[5],
    frame.data[6],
    frame.data[7]

  };
  
  // byte message [] = { 
  //   0x180,              // Type (PITCH[0])
  //   0x180 >> 8,         // Type (PITCH[1])
  //   0x180 >> 16,        // Type (PITCH[2])     
  //   0X78,               // Node
  //   0x00,               // Index
  //   0x00,               // Function Code
  //   pitchAngle,         // Data LSB
  //   pitchAngle >> 8,    // Data
  //   pitchAngle >> 16,   // Data
  //   pitchAngle >> 24    // Data
  //   //pitchAngle >> 32    // Data MSB
  // };

  // Serial.write(&message);
  Serial.write(message, 11);
}

/*********************************************************************
  This function is a callback function that is used by the CanFix object
  for sending CAN frames on the network.  The CanFix class is agnostic
  toward the actual CAN communication mechanism.  This function should
  translate from the common CanFixFrame structure and send it on the Bus.
**********************************************************************/
// void can_write_callback(CanFixFrame frame) {
//   CAN0.sendMsgBuf(frame.id, 0, frame.length, frame.data);
// }


