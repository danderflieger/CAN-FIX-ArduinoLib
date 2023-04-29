#include <canfix.h>

#include <mcp_can.h>
#include <mcp_can_dfs.h>

// Setup the mcp_can object
#define CAN0_INT 2           // Set INT to pin 2
MCP_CAN CAN0(10);            // Set CS to pin 10

// CANFiX object
CanFix cf(0x77);

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

// This function is a callback function that is used by the CanFix object
// for sending CAN frames on the network.  The CanFix class is agnostic
// toward the actual CAN communication mechanism.  This function should
// translate from the common CanFixFrame structure and send it on the Bus.
void can_write_callback(CanFixFrame frame) {
  CAN0.sendMsgBuf(frame.id, 0, frame.length, frame.data);
}

// Simple report callback.  This callback is called from the CanFix object
// when a Node Report request is received for our node.  This function should
// cause our node to send all of our parameters and the meta data associated 
// with those parameters.
void report_callback(void) {
  Serial.println("Report");
}

void setup() {
  Serial.begin(115200);
  cf.setDeviceId(0x77);  // This sets the device Id for our node
  cf.setModel(0x12345);  // This sets the model number for our node
  cf.setFwVersion(2);    // This sets the firmware revision for our node
  // This sets the write_callback function that the CanFix object will use to send CAN frames
  cf.set_write_callback(can_write_callback);
  // These functions set the callbacks that the CanFix object uses when it recieves these
  // specific frames
  cf.set_report_callback(report_callback);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  
  Serial.println("MCP2515 Library Receive Example...");

}

void loop() {
  CanFixFrame frame; // CANFiX Frame     

  if(!digitalRead(CAN0_INT)) {                  // If CAN0_INT pin is low, read receive buffer
      CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
      
      // Determine if ID is standard (11 bits) or extended (29 bits)
      // We ignore extended id frames because they are not used in CANFiX
      if((rxId & 0x80000000) != 0x80000000) {
        frame.id = rxId;
        frame.length = len;
        memcpy(frame.data, rxBuf, 8);
        cf.exec(frame);

        // Debug messages.  Not necessary for an actual nodes.
        sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
        Serial.print(msgString);
        for(byte i = 0; i<len; i++){
          sprintf(msgString, " 0x%.2X", rxBuf[i]);
          Serial.print(msgString);
        }
        Serial.println();
      }
  }
}
