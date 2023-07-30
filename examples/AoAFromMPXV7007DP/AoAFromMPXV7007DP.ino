#include <EEPROM.h>

#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

#include <Wire.h>

// // define where in EEPROM the saved values are located (NOT the angles, just where they are 
// // located in the EEPROM chip - each reading is a Float value, which is 4 bytes long)
#define EEPROM_ZERO_G_POSITION  0x00
#define EEPROM_WARN_POSITION    0x05
#define EEPROM_STALL_POSITION   0x10

#define META_ZERO_G  0b01110000 //0x70 
#define META_WARN    0b10000000 //0x80  
#define META_STALL   0b10010000 //0x90 

// set the interrupt pin to 2
#define CAN0_INT      2

// Set the CAN module's SDA pin on the Arduino (depending on which module you're using)
MCP_CAN CAN0(10); // MCP2515 module (8Mhz)
// MCP_CAN CAN0(17); // All-in-one (16Mhz)

CanFix cf(0x78);

// AMS_5600 ams5600;

// float zeroGValue;
// float warnValue;
// float stallValue;

long zeroGValue;
long warnValue;
long stallValue;

// float lastAnglePitch = 0.0;
unsigned long now;
unsigned long lastPerameterSet;
unsigned long lastTime;
unsigned long messageSendInterval = 100;

long smoothReadings;
int smoothCount;

void setup() {

  Serial.println("Begin Setup");
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  
  // set up CAN-FiX device data
  cf.setDeviceId(0x78);
  cf.setModel(0x12345);
  cf.setFwVersion(2);
 
  cf.set_write_callback(can_write_callback);

  delay(250);
  Wire.begin();

  while(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Unable to begin CAN0");
    delay(1000);
  }

  now = millis();
  

  Serial.println("Setup complete");
  smoothReadings = 0.0;
  smoothCount = 0;
  EEPROM.get(EEPROM_ZERO_G_POSITION, zeroGValue);
  EEPROM.get(EEPROM_WARN_POSITION, warnValue);
  EEPROM.get(EEPROM_STALL_POSITION, stallValue);


  // for (int i = 0; i < EEPROM.length(); i++) {
  //   EEPROM.write(i, 0);
  // }

  // Serial.println("EEPROM Cleared!");

}

void loop() {

  long unsigned int rxId;
  unsigned char len = 0;
  char msgString[128];
  unsigned char rxBuf[8];

  now = millis();

  // First, get the reading from the pressure differential sensor.
  long rawAnglePitch = (float)analogRead(A0) * 100.0; // / 10.0;
  //Serial.println(rawAnglePitch);

  EEPROM.get(EEPROM_ZERO_G_POSITION, zeroGValue);
  // Serial.println(zeroGValue);

  // Next, check to see if there's a message waiting in the CAN buffer. We 
  // specifically want to find one with the 0x30F identifier, which we will
  // use as a signal to update important AoA angles (level flight, warning, and stall)
  // If this message is received, this code block will decipher it and use the current
  // pitchAngle value to set the values permanently in the EEPROM
  if (!digitalRead(CAN0_INT)) {
    
    // read the message and assign parts to the variables above
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if ((rxId & 0x80000000) != 0x80000000) {
      //Serial.println("Packet Recieved ... ");
      
      // Check to see if the message id is 0x30F, which is undefined in the
      // CANFIX spec - e.g., we can use it for other things like, say, 
      // telling our AoA sensor to set some data fields and write the
      // values to EEPROM so the settings persist after reboot
      
      // Serial.println(rxId);

      if (rxId == 0x4000030F) {

        Serial.println("Packet Recieved ... ");

        // Create a CanFixFrame object that we can parse from the variables above
        CanFixFrame frame;
        frame.id = rxId;
        frame.length = len;
        memcpy(frame.data, rxBuf, 8);
        cf.exec(frame);

        // Depening on the button pressed on the setter device,
        // set some values on this device
        
        int buttonId = rxBuf[3]; // frame.data[3];

        Serial.print("Button pressed: ");
        Serial.print(buttonId);



        if (buttonId == 0x00) {  
          
          writeEEPROM(EEPROM_ZERO_G_POSITION , rawAnglePitch);
          setParameter(META_ZERO_G, rawAnglePitch);
          Serial.println("0g Meta message sent ...");

        } else if (buttonId == 0x01) {

          writeEEPROM(EEPROM_WARN_POSITION, rawAnglePitch - zeroGValue);
          setParameter(META_WARN, rawAnglePitch);
          Serial.println("Warn Meta message sent ...");
          
        } else if (buttonId == 0x02) {
          
          writeEEPROM(EEPROM_STALL_POSITION, rawAnglePitch - zeroGValue);
          setParameter(META_STALL, rawAnglePitch);
          Serial.println("Stall Meta message sent ...");

        } else {

          // Do nothing, not expecting whatever data we received
          Serial.println("Trying to send a Meta value change, but invalid buttonId given ...");

        }

      }
    
    } 
    
  }
  
  

  // Read the 0g, Warn, and Stall parameters from the EEPROM (possibly just written a moment ago)
  // long zeroGValue = 517;
  // float zeroGValue;
  // long zeroGValue = EEPROM.get(EEPROM_ZERO_G_POSITION, zeroGValue);

  // long warnValue = 30;
  // float warnValue;
  // long warnValue = EEPROM.get(EEPROM_WARN_POSITION, warnValue);

  // long stallValue = 42;
  // float stallValue;
  // long stallValue = EEPROM.get(EEPROM_STALL_POSITION, stallValue);

  // use the data to send the CAN-FiX parameters out to the FiX Gateway every second
  if (now - lastPerameterSet > 1000) {
    Serial.println("Setting parameters ...");
    
    EEPROM.get(EEPROM_ZERO_G_POSITION, zeroGValue);
    EEPROM.get(EEPROM_WARN_POSITION, warnValue);
    EEPROM.get(EEPROM_STALL_POSITION, stallValue);

    

    Serial.print("zeroGValue from EEPROM: ");
    Serial.println(zeroGValue);

    // Send those values out to the CAN
    setParameter(META_ZERO_G, zeroGValue);
    setParameter(META_WARN, warnValue);
    setParameter(META_STALL, stallValue);

    lastPerameterSet = now;

  }

  // Check to see if the zeroGValue was read correctly (e.g. is it set?)
  // If so, subtract that value from the pitchAngle. Otherwise, leave it alone.
  if (zeroGValue != NAN) {
    rawAnglePitch -= zeroGValue;
  }

  
  if(now - lastTime < messageSendInterval) {

    smoothReadings += rawAnglePitch;
    smoothCount++;

  } else {

    long pitchAngle = smoothReadings / smoothCount;
    //rawAnglePitch = smoothReadings / smoothCount;

    // Serial.print("rawAnglePitch: ");
    // Serial.println(rawAnglePitch);
    Serial.print("pitchAngle: ");
    Serial.println(pitchAngle);
    //sendAoAPitchAngle(rawAnglePitch);
    sendAoAPitchAngle(pitchAngle);

    // lastAnglePitch = pitchAngle;
    // lastAnglePitch = rawAnglePitch;
    lastTime = now;
    smoothReadings = 0;
    smoothCount = 0;

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
  This function receives an angle from the sensor and pushes it over 
  the CAN bus for use on the EFIS screen
*********************************************************************/
void sendAoAPitchAngle(long angle) {

    signed long x = angle; //* 100;

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

    // Serial.print("angle: ");
    // Serial.print(angle);
    // Serial.print("\t\tx: ");
    // Serial.println(x);

}

/*********************************************************************
  This function receives a meta type to change and an angle to set
  for that meta type. Then it pushes the update over the CAN bus for 
  use on the EFIS screen
*********************************************************************/
void setParameter(byte metaType, long angle) {

  CFParameter pAoAMeta;
  signed long x = angle;

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

// void updateAngleValues(int buttonId, float angle) {
  
//   // Depening on the button pressed on the setter device,
//   // set some values on this device
//   Serial.print("Button pressed: ");
//   Serial.print(buttonId);

//   if (buttonId == 0x00) {  
//     writeEEPROM(EEPROM_ZERO_G_POSITION , angle);
//     setMeta(META_ZERO_G, angle);
//   } else if (buttonId == 0x01) {
//     writeEEPROM(EEPROM_WARN_POSITION, angle);
//   } else if (buttonId == 0x02) {
//     writeEEPROM(EEPROM_STALL_POSITION, angle);
//   } else {
//     // Do nothing, not expecting whatever data we received
//   }
// }

void writeEEPROM(int eepromPosition, long angle) {

  long currentAngle;
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



