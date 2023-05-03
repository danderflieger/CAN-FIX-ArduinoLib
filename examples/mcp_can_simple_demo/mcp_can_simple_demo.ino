#include <can.h>

#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <Wire.h>
#include <SPI.h>

#define CAN0_INT 2
MCP_CAN CAN0(10);

CanFix cf(0x77);

unsigned long now;
unsigned long lasttime;
unsigned int airspeed = 1300;
unsigned int verticalspeed;
bool countup[10];


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

  pinMode(CAN0_INT, INPUT);

  Serial.println("MCP2515 Initialized Successfully!");
  now = millis();
  lasttime = now;
  airspeed = 30;

}

void loop() {

  now = millis();

  if (now - lasttime > 150) {
    
    if (airspeed <= 60) {
      countup[0] = true;
    } else if (airspeed >= 1600) {
      countup[0] = false;
    }

    if (countup[0]) airspeed += 5;
    else airspeed -= 5;
    
    CanFixFrame frame;

    CFParameter pIndicatedAirspeed;
    pIndicatedAirspeed.type = 0x183;
    pIndicatedAirspeed.index = 0x00;
    pIndicatedAirspeed.fcb = 0x00;
    pIndicatedAirspeed.data[0] = airspeed;
    pIndicatedAirspeed.data[1] = airspeed>>8;
    pIndicatedAirspeed.length = 5;
    cf.sendParam(pIndicatedAirspeed);

    // update IAS for TAS
    pIndicatedAirspeed.type = 0x18D;
    cf.sendParam(pIndicatedAirspeed);


    if (verticalspeed <= 0) {
      countup[1] = true;
    } else if (verticalspeed >= 1000) {
      countup[1] = false;
    }

    if (countup[1]) verticalspeed += 10;
    else verticalspeed -= 10;

    CFParameter pVerticalSpeed;
    pVerticalSpeed.type = 0x186;
    pVerticalSpeed.index = 0x00;
    pVerticalSpeed.fcb = 0x00;
    pVerticalSpeed.data[0] = verticalspeed;
    pVerticalSpeed.data[1] = verticalspeed>>8;
    pVerticalSpeed.data[2] = verticalspeed>>16;
    pVerticalSpeed.length = 6;
    cf.sendParam(pVerticalSpeed);

    CFParameter pTurnRate;
    pTurnRate.type = 0x403;
    pTurnRate.index = 0x00;
    pTurnRate.fcb = 0x00;
    pTurnRate.data[0] = 0x08;
    pTurnRate.data[1] = 0x00;
    pTurnRate.length = 5;
    cf.sendParam(pTurnRate);

    CFParameter pCylinderHeadTemperature;
    pCylinderHeadTemperature.type = 0x500;
    pCylinderHeadTemperature.index = 0x00;
    pCylinderHeadTemperature.fcb = 0x00;
    pCylinderHeadTemperature.data[0] = 0xAA;
    pCylinderHeadTemperature.data[1] = 0x06;
    pCylinderHeadTemperature.length = 2;
    cf.sendParam(pCylinderHeadTemperature);

    pCylinderHeadTemperature.index = 0x01;
    pCylinderHeadTemperature.data[0] = 0xCC;
    pCylinderHeadTemperature.data[1] = 0x06;
    cf.sendParam(pCylinderHeadTemperature);

    pCylinderHeadTemperature.index = 0x02;
    pCylinderHeadTemperature.data[0] = 0xAC;
    pCylinderHeadTemperature.data[1] = 0x05;
    cf.sendParam(pCylinderHeadTemperature);

    pCylinderHeadTemperature.index = 0x03;
    pCylinderHeadTemperature.data[0] = 0xCD;
    pCylinderHeadTemperature.data[1] = 0x07;
    cf.sendParam(pCylinderHeadTemperature);


    // float temperature = bmp.readTemperature();
    // unsigned int oiltemp = temperature * 10;
    // CFParameter pOilTemp;
    // pOilTemp.type = 0x222;
    // pOilTemp.index = 0x00;
    // pOilTemp.fcb = 0x00;
    // pOilTemp.data[0] = oiltemp;
    // pOilTemp.data[1] = oiltemp>>8;
    // pOilTemp.length = 5;
    // cf.sendParam(pOilTemp);

    lasttime = now;
  }
  
  

}

// This function is a callback function that is used by the CanFix object
// for sending CAN frames on the network.  The CanFix class is agnostic
// toward the actual CAN communication mechanism.  This function should
// translate from the common CanFixFrame structure and send it on the Bus.
void can_write_callback(CanFixFrame frame) {
  CAN0.sendMsgBuf(frame.id, 0, frame.length, frame.data);
}

// CanFixFrame create_can_frame(int id, int index, byte data[]) {
//   Serial.println(id);
//   Serial.println(lsb);
//   for (int i = 0; i < sizeof(data); i++) {
//     Serial.println(data[i]);
//   }
// }


