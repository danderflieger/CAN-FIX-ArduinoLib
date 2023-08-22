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




  
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;
  EEPROM.get(eeAddress, bnoID);
  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
    Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
    delay(500);
  }
  else
  {
    Serial.println("\nFound Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    displaySensorOffsets(calibrationData);

    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
  }

  displaySensorDetails();
  displaySensorStatus();

  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P2);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P2);
  // bno.setExtCrystalUse(true);
  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  /* always recal the mag as It goes out of calibration very often */
  if (foundCalib){
      Serial.println("Move sensor slightly to calibrate magnetometers");
      while (!bno.isFullyCalibrated())
      {
          bno.getEvent(&event);
          displayCalStatus();
          delay(BNO055_SAMPLE_INTERVAL);
      }
  }
  else
  {
    Serial.println("Please Calibrate Sensor: ");
    uint8_t system_status, self_test_result, system_error;
    system_status = self_test_result = system_error = 0;
    // while (!bno.isFullyCalibrated())
    while (&system_status < 2)
    {
        bno.getEvent(&event);
        
        bno.getSystemStatus(&system_status, &self_test_result, &system_error);

        Serial.print("X: ");
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.print(event.orientation.z, 4);

        /* Optional: Display calibration status */
        displayCalStatus();

        /* New line for the next sample */
        Serial.println("");

        /* Wait the specified delay before requesting new data */
        delay(BNO055_SAMPLE_INTERVAL);
    }
  }

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;

  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  // Check to see if the stored calibration is the same as the one we just figured out. And if not,
  // write the new calibration to EEPROM instead.
  if (
    calibrationData.accel_offset_x != newCalib.accel_offset_x ||
    calibrationData.accel_offset_y != newCalib.accel_offset_y ||
    calibrationData.accel_offset_z != newCalib.accel_offset_z ||
    calibrationData.gyro_offset_x != newCalib.gyro_offset_x ||
    calibrationData.gyro_offset_y != newCalib.gyro_offset_y ||
    calibrationData.gyro_offset_z != newCalib.gyro_offset_z ||
    calibrationData.mag_offset_x != newCalib.mag_offset_x ||
    calibrationData.mag_offset_y != newCalib.mag_offset_y ||
    calibrationData.mag_offset_z != newCalib.mag_offset_z ||
    calibrationData.accel_radius != newCalib.accel_radius ||
    calibrationData.mag_radius != newCalib.mag_radius
  ) {
    Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
  } else {
    Serial.println("New calibration data matches what was found in EEPROM ... NOT updating.");
    Serial.println("\n--------------------------------\n");
  }
  
  delay(500);

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

    displayCalStatus();

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

void displaySensorStatus(void) {
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  // Display the results in the Serial Monitor
  Serial.println();
  Serial.print("System Status: 0x");
  Serial.print(system_status, HEX);
  Serial.print("\tSelf Test: 0x");
  Serial.print(self_test_results, HEX);
  Serial.print("System Error: 0x");
  Serial.print(system_error, HEX);
  Serial.println();
  delay(500);
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

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
      Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}


/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
}

