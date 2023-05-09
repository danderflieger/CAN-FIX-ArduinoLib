#include <EEPROM.h>

#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

#include <Wire.h>
#include <SPI.h>
#include <AS5600.h>

// define where in EEPROM the saved values are located (NOT the angles, just where they are 
// located in the EEPROM chip - each reading is a Float value, which is 4 bytes long)
#define EEPROM_ZERO_G_POSITION  0x00
#define EEPROM_WARN_POSITION    0x05
#define EEPROM_STALL_POSITION   0x09

#define META_MIN      0b00010000
#define META_MAX      0b00100000
#define META_ZERO_G   0b01110000
#define META_WARN     0b10000000
#define META_STALL    0b10010000
#define CAN0_INT      2
MCP_CAN CAN0(10);

CanFix cf(0x77);

AMS_5600 ams5600;


float lastAnglePitch = 0.0;
unsigned long now;
unsigned long lastMessageTimerMillis;
unsigned long messageSendInterval = 100;
float smoothReadings = 0.0;
int smoothReadingsCount = 0;



void setup() {
  Serial.println("Begin Setup");
  Serial.begin(115200);
  
  cf.setDeviceId(0x77);
  cf.setModel(0x12345);
  cf.setFwVersion(2);
 
  cf.set_write_callback(can_write_callback);

  // pinMode(LED_BUILTIN, OUTPUT);
  pinMode(6, INPUT);
  delay(250);
  Wire.begin();
  
  if (ams5600.detectMagnet() == 0) {
    while(1) {
      if (ams5600.detectMagnet() == 1) {
        Serial.print("Beginning Current Pitch Magnitude: ");
        Serial.println(ams5600.getMagnitude());
        break;
      } else {
        Serial.println("Can't detect magnet");
      }
      delay(1500);
    }
  }

  while(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Unable to begin CAN0");
    delay(1000);
  }

  Serial.println("Setup complete");
}

void loop() {

  float zeroGValue;
  EEPROM.get(EEPROM_ZERO_G_POSITION, zeroGValue);
  
  float rawAnglePitch = convertRawAngleToDegrees(ams5600.getRawAngle()) - 180;



  unsigned long now = millis();
  float pitchAngle = rawAnglePitch;


  
  if(now - lastMessageTimerMillis < messageSendInterval) {



    smoothReadings += pitchAngle;
    smoothReadingsCount++;

  } else {

    Serial.print("rawAnglePitch: ");
    Serial.print(rawAnglePitch);
    Serial.print("\tzeroGValue: ");
    Serial.println(zeroGValue);

    // Serial.print("\tnew rawAnglePitch: ");
    // Serial.println(rawAnglePitch);


    pitchAngle = smoothReadings / smoothReadingsCount;

    if (zeroGValue != NAN) {
      pitchAngle -= zeroGValue;
    }    

    if (digitalRead(6) == HIGH){
      
      if (rawAnglePitch == zeroGValue) {
        Serial.println("Values already match");
      } else {
        Serial.print("Updating Zero G Value to: ");
        EEPROM.put(EEPROM_ZERO_G_POSITION, rawAnglePitch);
        Serial.println(rawAnglePitch);
      }    
      
    } else {
      
      // Serial.print("EEPROM Zero G Reading: ");
      // Serial.println(zeroGValue);
    }

    sendAoAPitchAngle(pitchAngle);

    lastAnglePitch = pitchAngle;
    lastMessageTimerMillis = now;
    smoothReadings = 0.0;
    smoothReadingsCount = 0;

  }

}

/********************************************************************
  Function: convertRawAngleToDegrees
  In: angle data from AMS_5600 - an integer between 0 and 4095
  Out: human readable degrees as float
  Description: takes the raw angle from the sensor and
  calculates a float value in degrees.
*********************************************************************/
float convertRawAngleToDegrees(word newAngle) {
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087;
  return retVal;
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
  This function receives an angle from the sensor and pushes it over 
  the CAN bus for use on the EFIS screen
*********************************************************************/
void sendAoAPitchAngle(float angle) {

    signed int x = angle * 100;

    CFParameter pAngleOfAttack;
    pAngleOfAttack.type = 0x182; // CAN-FIX Angle of Attack Parameter
    // pAngleOfAttack.type = 0x180; // CAN-FIX Pitch Angle Parameter (testing)
    pAngleOfAttack.index = 0x00;
    pAngleOfAttack.fcb = 0x00;
    pAngleOfAttack.data[0] = x;
    pAngleOfAttack.data[1] = x>>8;
    pAngleOfAttack.data[2] = x>>16;
    pAngleOfAttack.data[3] = x>>24;
    pAngleOfAttack.length = 7;
    cf.sendParam(pAngleOfAttack);

    Serial.print("angle: ");
    Serial.print(angle);
    Serial.print("\t\tx: ");
    Serial.println(x);

}

/*********************************************************************
  This function receives an meta type to change and an angle to set
  for that meta type. Then it pushes the update over the CAN bus for 
  use on the EFIS screen
*********************************************************************/
void setMeta(byte metaType, float data) {

  CFParameter p;
  int x;

  p.type = 0x182; // CAN-FIX Angle of Attack Parameter
  p.index = 0x00;
  p.fcb = metaType;
  x = data*100;
  p.data[0] = x;
  p.data[1] = x>>8;
  p.length = 2;

  cf.sendParam(p);

}

