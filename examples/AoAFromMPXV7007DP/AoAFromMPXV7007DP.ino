#include <EEPROM.h>

#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

#include <Wire.h>
#include <SPI.h>
#include <AS5600.h>

// // define where in EEPROM the saved values are located (NOT the angles, just where they are 
// // located in the EEPROM chip - each reading is a Float value, which is 4 bytes long)
#define EEPROM_ZERO_G_POSITION  0x00
#define EEPROM_WARN_POSITION    0x05
#define EEPROM_STALL_POSITION   0x10

// #define META_MIN      0b00010000
// #define META_MAX      0b00100000
#define META_ZERO_G  0x70 //0b01110000
#define META_WARN    0x80  //0b10000000
#define META_STALL   0x90 //0b10010000
#define CAN0_INT      2
MCP_CAN CAN0(10);

CanFix cf(0x78);

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
  
  cf.setDeviceId(0x78);
  cf.setModel(0x12345);
  cf.setFwVersion(2);
 
  cf.set_write_callback(can_write_callback);
  // cf.set_report_callback(report_callback);

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

  long unsigned int rxId;
  unsigned char len = 0;
  char msgString[128];
  unsigned char rxBuf[8];
  
  // First, get the reading from the encoder. This is exactly what the
  // encoder reads, with no clean-up. If the diametric magnet is 3.2 degrees
  // from 0 on the encoder, that's what we read here. We subtract 180 degrees 
  // from that so we have a range of -180 to 180, rather than 0-360
  float rawAnglePitch = convertRawAngleToDegrees(ams5600.getRawAngle()) - 180;

  float zeroGValue;
  EEPROM.get(EEPROM_ZERO_G_POSITION, zeroGValue);

  unsigned long now = millis();
  float pitchAngle = rawAnglePitch;

  if (zeroGValue != NAN) {
    pitchAngle -= zeroGValue;
  }

  // Next, check to see if there's a message waiting in the CAN buffer. We 
  // specifically want to find one with the 0x30F identifier, which we will
  // use as a signal to update important AoA angles (level flight, warning, and stall)
  if (!digitalRead(CAN0_INT)) {
    // if there is a CAN message waiting, declare a couple of variables that we 
    // can use to investigate the message
    

    // read the message and assign parts to the variables above
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if ((rxId & 0x80000000) != 0x80000000) {
      // Serial.println(rxId, HEX);
      
      // Check to see if the message id is 0x30F, which is undefined in the
      // CANFIX spec - e.g., we can use it for other things like, say, 
      // telling our AoA sensor to set some data fields and write the
      // values to EEPROM so the settings persist after reboot
      if (rxId == 0x4000030F) {
        // Create a CanFixFrame object that we can parse from the variables above
        CanFixFrame frame;
        frame.id = rxId;
        frame.length = len;
        memcpy(frame.data, rxBuf, 8);
        cf.exec(frame);

        // updateAngleValues(rxBuf[3], rawAnglePitch);

        // Depening on the button pressed on the setter device,
        // set some values on this device
        
        int buttonId = rxBuf[3]; // frame.data[3];

        Serial.print("Button pressed: ");
        Serial.print(buttonId);

        if (buttonId == 0x00) {  
          writeEEPROM(EEPROM_ZERO_G_POSITION , rawAnglePitch);
          setMeta(META_ZERO_G, pitchAngle);
          Serial.println("0g Meta message sent ...");
        } else if (buttonId == 0x01) {
          writeEEPROM(EEPROM_WARN_POSITION, pitchAngle);
          setMeta(META_WARN, pitchAngle);
          Serial.println("Warn Meta message sent ...");
        } else if (buttonId == 0x02) {
          writeEEPROM(EEPROM_STALL_POSITION, pitchAngle);
          setMeta(META_STALL, pitchAngle);
          Serial.println("Stall Meta message sent ...");
        } else {
          // Do nothing, not expecting whatever data we received
          Serial.println("Trying to send a Meta value change, but buttonId given ...");
        }
        
        // if (frame.id == 0x30F) {
        //   writeEEPROM(rxBuf[3], rawAnglePitch)
        // }
      }
    
    } 

    
  }


  
  if(now - lastMessageTimerMillis < messageSendInterval) {

    smoothReadings += pitchAngle;
    smoothReadingsCount++;

  } else {


    pitchAngle = smoothReadings / smoothReadingsCount;


    sendAoAPitchAngle(pitchAngle);

    // // Check to see if this is an extended CAN Identifier
    // // (CANFIX does not use extended, so we'll ignore those)
    // if(!digitalRead(CAN0_INT)) {
      
    // }

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
void setMeta(byte metaType, float angle) {

  CFParameter pAoAMeta;
  signed int x = angle * 100;

  pAoAMeta.type = 0x182; // CAN-FIX Angle of Attack Parameter
  pAoAMeta.index = 0x00;
  pAoAMeta.fcb = metaType;
  pAoAMeta.data[0] = x;
  pAoAMeta.data[1] = x>>8;
  pAoAMeta.data[3] = x>>16;
  pAoAMeta.data[4] = x>>24;
  pAoAMeta.length = 7;
  cf.sendParam(pAoAMeta);
  

}

void updateAngleValues(int buttonId, float angle) {
  
  // Depening on the button pressed on the setter device,
  // set some values on this device
  Serial.print("Button pressed: ");
  Serial.print(buttonId);

  if (buttonId == 0x00) {  
    writeEEPROM(EEPROM_ZERO_G_POSITION , angle);
    setMeta(META_ZERO_G, angle);
  } else if (buttonId == 0x01) {
    writeEEPROM(EEPROM_WARN_POSITION, angle);
  } else if (buttonId == 0x02) {
    writeEEPROM(EEPROM_STALL_POSITION, angle);
  } else {
    // Do nothing, not expecting whatever data we received
  }
}

void writeEEPROM(int eepromPosition, float angle) {

  float currentAngle;
  EEPROM.get(eepromPosition, currentAngle);

  Serial.print("\ncurrentAngle: ");
  Serial.print(currentAngle);
  Serial.print("\tangle: ");
  Serial.println(angle);

  if (currentAngle == angle) {
    Serial.println("Values already match");
  } else {
    EEPROM.put(eepromPosition, angle);
    Serial.print("Updating angle: ");
    Serial.print(eepromPosition);
    Serial.print(" to: ");
    Serial.println(angle);
  }

}

