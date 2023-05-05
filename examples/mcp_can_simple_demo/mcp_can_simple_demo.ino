
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include <can.h>
#include <canfix.h>

#include <mcp_can.h>
#include <mcp_can_dfs.h>


// 

#define BMP280_ADDRESS 0x76
Adafruit_BMP280 bmp;

#define CAN0_INT 2
MCP_CAN CAN0(10);

CanFix cf(0x77);

unsigned long now;
unsigned long lasttime;
unsigned int airspeed = 1300;
signed int verticalspeed;
signed int lateralacceleration;
bool countup[10];
volatile unsigned int counter = 0;
volatile float currentinHg = 30.01;



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
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  // attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);

  Serial.println("MCP2515 Initialized Successfully!");
  now = millis();
  lasttime = now;
  airspeed = 30;

  unsigned status;
  status = bmp.begin(0x76);


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


    if (verticalspeed <= -1000) {
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
    pVerticalSpeed.data[3] = verticalspeed>>24;
    pVerticalSpeed.length = 7;
    cf.sendParam(pVerticalSpeed);

    CFParameter pTurnRate;
    pTurnRate.type = 0x403;
    pTurnRate.index = 0x00;
    pTurnRate.fcb = 0x00;
    pTurnRate.data[0] = 0x08;
    pTurnRate.data[1] = 0x00;
    pTurnRate.length = 5;
    cf.sendParam(pTurnRate);


    if (lateralacceleration < -250.0) countup[4] = true;
    if (lateralacceleration > 250.0) countup[4] = false;
    if (countup[4]) lateralacceleration += 10;
    else lateralacceleration -= 10;

    CFParameter pLateralAcceleration;
    pLateralAcceleration.index = 0x00;
    pLateralAcceleration.fcb = 0x00;
    pLateralAcceleration.type = 0x18B;
    pLateralAcceleration.data[0] = lateralacceleration;
    pLateralAcceleration.data[1] = lateralacceleration>>8;
    pLateralAcceleration.length = 5;
    cf.sendParam(pLateralAcceleration);



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


    CFParameter pRPM;
    pRPM.type = 0x200;
    pRPM.index = 0x00;
    pRPM.fcb = 0x00;
    pRPM.data[0] = 0b11111111;
    pRPM.data[1] = 0b00000100;
    pRPM.length = 2;
    cf.sendParam(pRPM);

    CFParameter pMAP;
    pRPM.type = 0x21E;
    pRPM.index = 0x00;
    pRPM.fcb = 0x00;
    pRPM.data[0] = 2987;
    pRPM.data[1] = 2987>>8;
    pRPM.length = 2;
    cf.sendParam(pRPM);

    float temperature = bmp.readTemperature();
    unsigned int oiltemp = temperature * 10;
    CFParameter pOilTemp;
    pOilTemp.type = 0x222;
    pOilTemp.index = 0x00;
    pOilTemp.fcb = 0x00;
    pOilTemp.data[0] = oiltemp;
    pOilTemp.data[1] = oiltemp>>8;
    pOilTemp.length = 5;
    cf.sendParam(pOilTemp);


    // float currentinHg = 30.01;
    float currentMillibars = currentinHg * 33.864;

    signed int altimeterSetting = currentinHg * 1000;

    CFParameter pAltimeterSetting;
    pAltimeterSetting.type = 0x190;
    pAltimeterSetting.index = 0x00;
    pAltimeterSetting.fcb = 0x00;
    pAltimeterSetting.data[0] = altimeterSetting;
    pAltimeterSetting.data[1] = altimeterSetting>>8;
    pAltimeterSetting.length = 5;
    cf.sendParam(pAltimeterSetting);

    float meters = bmp.readAltitude(currentMillibars);

    signed long indicatedAltitude = meters * 3.2804; //convert to feet

    // Serial.print("currentkPa: ");
    // Serial.print(currentkPa);
    // Serial.print("\tmeters: ");
    // Serial.print(meters);
    // Serial.print("\tindicatedaltitude: ");
    // Serial.println(indicatedaltitude);


    CFParameter pIndicatedAltitude;
    pIndicatedAltitude.type = 0x184;
    pIndicatedAltitude.index = 0x00;
    pIndicatedAltitude.fcb = 0x00;
    pIndicatedAltitude.data[0] = indicatedAltitude;
    pIndicatedAltitude.data[1] = indicatedAltitude>>8;
    pIndicatedAltitude.data[2] = indicatedAltitude>>16;
    pIndicatedAltitude.data[3] = indicatedAltitude>>24;
    pIndicatedAltitude.length = 7;
    cf.sendParam(pIndicatedAltitude);

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

void ai0() {
  if (digitalRead(3) == LOW) {
    // counter++;
    currentinHg -= 0.01;
  } else {
    // counter--;
    currentinHg += 0.01;
  }
}

void ai1() {
  if (digitalRead(4) == LOW) {
    // counter++;
    currentinHg -= 0.01;
  } else {
    // counter--;
    currentinHg += 0.01;
  }
}
