#include <can.h>

#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

#define CAN0_INT 2
MCP_CAN CAN0(10);

CanFix cf(0x77);


void setup() {
  Serial.begin(115200);
  cf.setDeviceId(0x77);
  cf.setModel(0x12345);
  cf.setFwVersion(2);

  cf.set_write_callback(can_write_callback);
  
  while(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Unable to begin CAN0");
    delay(1000);
  }

  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN_INT, INPUT);

  Serial.println("MCP2515 Initialized Successfully!");

}

void loop() {
  // put your main code here, to run repeatedly:
  CanFixFrame frame;


}

// This function is a callback function that is used by the CanFix object
// for sending CAN frames on the network.  The CanFix class is agnostic
// toward the actual CAN communication mechanism.  This function should
// translate from the common CanFixFrame structure and send it on the Bus.
void can_write_callback(CanFixFrame frame) {
  CAN0.sendMsgBuf(frame.id, 0, frame.length, frame.data);
}


