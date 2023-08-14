#include <EEPROM.h>

#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SPI.h>
// #include <AS5600.h>

// // define where in EEPROM the saved values are located (NOT the angles, just where they are 
// // located in the EEPROM chip - each reading is a Float value, which is 4 bytes long)
// #define EEPROM_ZERO_G_POSITION  0x00
// #define EEPROM_WARN_POSITION    0x05
// #define EEPROM_STALL_POSITION   0x10

#define BNO055_SAMPLE_INTERVAL 100

#define PITCH  0x180 
#define ROLL    0x181
#define HEADING   0x185


#define CAN0_INT      2
MCP_CAN CAN0(10);
CanFix cf(0x78);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float HEADING_OFFSET = -90;

long lastMessageTimerMillis = 0;

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

  while(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Unable to begin CAN0");
    delay(1000);
  }
  

  while (!bno.begin()) {
    Serial.println("No BNO055 detected.");
    delay(1000);
  }
 
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P2);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P2);
  bno.setExtCrystalUse(true);
  

  displaySensorDetails();

  Serial.println("Setup complete");
}

void loop() {

  sensors_event_t event;
  bno.getEvent(&event);

  // long unsigned int rxId;
  // unsigned char len = 0;
  // char msgString[128];
  // unsigned char rxBuf[8];
  



  unsigned long now = millis();
  
  if (now - lastMessageTimerMillis > BNO055_SAMPLE_INTERVAL) {
    
    float heading = event.orientation.x;//event.orientation.heading;
    float pitch = event.orientation.y * -1; // event.orientation.pitch; 
    float roll =   event.orientation.z; //event.orientation.roll; 

    if (heading + HEADING_OFFSET >= 360) heading = heading + HEADING_OFFSET - 360;
    else if (heading + HEADING_OFFSET < 0) heading = heading + HEADING_OFFSET + 360; 
    else heading = heading + HEADING_OFFSET;
    
    sendHeading(heading);
    sendPitch(pitch);
    sendRoll(roll);

    /* Also send calibration data for each sensor. */
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print(F("\tCalibration:\n\t sys: "));
    Serial.print(sys, DEC);
    Serial.print(F("\t gyro: "));
    Serial.print(gyro, DEC);
    Serial.print(F("\t accel: "));
    Serial.print(accel, DEC);
    Serial.print(F("\t mag: "));
    Serial.print(mag, DEC);
    Serial.println(F(""));

    lastMessageTimerMillis = millis();

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
void sendHeading(float heading) {

    unsigned int x = heading * 10;

    CFParameter pHeading;
    pHeading.type = HEADING; //0x182; // CAN-FIX Angle of Attack Parameter
    pHeading.index = 0x00;
    pHeading.fcb = 0x00;
    pHeading.data[0] = x;
    pHeading.data[1] = x>>8;
    pHeading.data[2] = x>>16;
    pHeading.data[3] = x>>24;
    pHeading.length = 7;
    cf.sendParam(pHeading);

    Serial.print("Heading Sent: ");
    Serial.println(heading);

}

void sendPitch(float pitch) {

    unsigned int x = pitch * 100;

    CFParameter pPitch;
    pPitch.type = PITCH; //0x182; // CAN-FIX Angle of Attack Parameter
    pPitch.index = 0x00;
    pPitch.fcb = 0x00;
    pPitch.data[0] = x;
    pPitch.data[1] = x>>8;
    pPitch.data[2] = x>>16;
    pPitch.data[3] = x>>24;
    pPitch.length = 7;
    cf.sendParam(pPitch);

    Serial.print("Pitch Sent: ");
    Serial.println(pitch);

}

void sendRoll(float roll) {

    unsigned int x = roll * 100;

    CFParameter pRoll;
    pRoll.type = ROLL; //0x182; // CAN-FIX Angle of Attack Parameter
    pRoll.index = 0x00;
    pRoll.fcb = 0x00;
    pRoll.data[0] = x;
    pRoll.data[1] = x>>8;
    pRoll.data[2] = x>>16;
    pRoll.data[3] = x>>24;
    pRoll.length = 7;
    cf.sendParam(pRoll);

    Serial.print("Roll Sent: ");
    Serial.println(roll);

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

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");




  delay(500);
}

