/****************************************************************
** This code is not working yet. It's designed to collect AHRS data
** from the Adafruit BNO08x IMU and push it to an Android app that
** has code for an artificial horizon/Attitude Indicator (AI).
** It's not working currently.
** Most of the this is from the example code provided by Adafruit
** for use with the IMU chip. But there's also some code added 
** to interface with the MakerPlane CAN-FiX open source CAN standard:
** https://makerplane.org/
** 
** It requires quite a bit of RAM, so it will only compile on 
** Arduino devices that have enough. I'm testing it with the 
** Arduino Nano ESP32
****************************************************************/

#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <Wire.h>

#include <Arduino.h>

// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)

#include <Adafruit_BNO08x.h>
#include <SPI.h>


// For SPI mode, we need a CS pin
// #define BNO08X_CS 10
// #define BNO08X_INT 9


// #define FAST_MODE

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1


struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

MCP_CAN CAN0(10);
CanFix cf(0x78);

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif

#define PITCH   0x180
#define ROLL    0x181
#define HEADING 0x185
#define SAMPLE_INTERVAL 100
unsigned long lastMessageTimerMillis = 0;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup(void) {

  Serial.begin(115200);
  // Serial.begin(19200);
  //while (!Serial) delay(100);     // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("line 71");
  cf.setDeviceId(0x78);
  cf.setModel(0x12345);
  cf.setFwVersion(2);
  // cf.set_write_callback(can_write_callback);
  // digitalWrite(16, HIGH); // Blue
  // digitalWrite(15, LOW);  // Green
  // digitalWrite(14, LOW);  //Red

  delay(250);
  Wire.begin();

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  // if (!bno08x.begin_I2C()) {
  while (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    delay(1000);
    //while (1) { delay(10); }
  }

  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {



    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  unsigned long now = millis();
  if (now - lastMessageTimerMillis > SAMPLE_INTERVAL) {
    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }
      // static long last = 0;
      // long now = micros();
      // Serial.print(now - last);             Serial.print("\t");
      // last = now;
      // Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
      // Serial.print(ypr.yaw);                Serial.print("\t");
      // Serial.print(ypr.pitch);              Serial.print("\t");
      // Serial.println(ypr.roll);

      // send yaw
      // int yawInteger = ypr.yaw * -10;
      //if (yawInteger < 0) yawInteger += 3600;
      // Serial.println(yawInteger);

      float heading = -(ypr.yaw);
      if (heading >= 360) heading = heading -360;
      else if (heading < 0) heading = heading + 360;
      else {}
      
      int yawInteger = (heading * 10);
      //Serial.println(yawInteger);

      char yawData[5];
      sprintf(yawData, "%d", yawInteger);
      
      CanFixFrame yawFrame;

      yawFrame.id = HEADING;
      yawFrame.length = 7;
      // memcpy(yawFrame.data, yawData, 7);
      // yawFrame.data = yawInteger;
      yawFrame.data[0] = 0x78;
      yawFrame.data[1] = 0x00;
      yawFrame.data[2] = 0x00;
      yawFrame.data[3] = yawInteger;
      yawFrame.data[4] = yawInteger >> 8;
      yawFrame.data[5] = yawInteger >> 16;
      yawFrame.data[6] = yawInteger >> 24;
      yawFrame.data[7] = yawInteger >> 32;


      // CFParameter yawFrame;      
      // yawFrame.type = HEADING;
      // yawFrame.index = 0x00;
      // yawFrame.fcb = 0x00;
      // yawFrame.data[0] = yawInteger;
      // yawFrame.data[1] = yawInteger>>8;
      // yawFrame.data[2] = yawInteger>>16;
      // yawFrame.data[3] = yawInteger>>24;
      // yawFrame.length = 7;

      // byte yawData[] = {
      //   0x00,
      //   0x00,
      //   yawInteger,
      //   yawInteger >> 8,
      //   yawInteger >> 16,
      //   yawInteger >> 24,
      //   yawInteger >> 32
      // };
      // memcpy(yawFrame.data, yawData, 7);
      // Serial.println(yawInteger);
      send_canfix_frame_to_serial(yawFrame);
      
      // send_can_frame_to_serial(yawFrame);

      // byte yawMessage[11] = {
      //   HEADING,
      //   HEADING >> 8,
      //   HEADING >> 16,
      //   0x00,
      //   0x00,
      //   0x00,
      //   yawInteger,
      //   yawInteger >> 8,
      //   yawInteger >> 16,
      //   yawInteger >> 24,
      //   yawInteger >> 32
      // };
      // send_can_frame_to_serial(yawMessage);

      // send pitch
      // int pitchInteger = ypr.pitch * 100;
      // byte pitchMessage[11] = {
      //   PITCH,
      //   PITCH >> 8,
      //   PITCH >> 16,
      //   0x00,
      //   0x00,
      //   0x00,
      //   pitchInteger,
      //   pitchInteger >> 8,
      //   pitchInteger >> 16,
      //   pitchInteger >> 24,
      //   pitchInteger >> 32
      // };
      // send_can_frame_to_serial(pitchMessage);

      // send roll
      // int rollInteger = ypr.roll * 100;
      // byte rollMessage[11] = {
      //   ROLL,
      //   ROLL >> 8,
      //   ROLL >> 16,
      //   0x00,
      //   0x00,
      //   0x00,
      //   rollInteger,
      //   rollInteger >> 8,
      //   rollInteger >> 16,
      //   rollInteger >> 24,
      //   rollInteger >> 32
      // };
      // send_can_frame_to_serial(rollMessage);
        

      
    }
    lastMessageTimerMillis = now;
  }
  
}

/*********************************************************************
  This function converts CanFix messages to Serial and sends them over
  to the Android device connected on the USB port.
**********************************************************************/
// void send_can_frame_to_serial(byte message[]) {
void send_canfix_frame_to_serial(CanFixFrame frame) {
// void send_canfix_frame_to_serial(CFParameter frame) {
  
    byte message [] = {
      frame.id >> 16,
      frame.id >> 8,
      frame.id,
      frame.data[0],
      frame.data[1],
      frame.data[2],
      frame.data[3],
      frame.data[4],
      frame.data[5],
      frame.data[6],
      frame.data[7]

    };
    if (Serial.availableForWrite() > 0) {
      Serial.write(message, 11);
    }
    
    // Serial.print(message[0], HEX);
    // Serial.print(message[1], HEX);
    // Serial.print(message[2], HEX);
    // Serial.print(message[3], HEX);
    // Serial.print(message[4], HEX);
    // Serial.print(message[5], HEX);
    // Serial.print(message[6], HEX);
    // Serial.print(message[7], HEX);
    // Serial.print(message[8], HEX);
    // Serial.print(message[9], HEX);
    // Serial.println(message[10], HEX);


  // byte message [] = {
  //   frame.type,
  //   frame.type >> 8,
  //   frame.type >> 16,
  //   frame.index,
  //   frame.fcb,
  //   frame.data[0],
  //   frame.data[1],
  //   frame.data[2],
  //   frame.data[3],
  //   frame.data[4],
  //   frame.data[5]

  // };
  // //Serial.println(message);
  // Serial.write(message, 11);
  
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
  // Serial.write(message, 11);
  
  // Serial.write(message, 11);
  // Serial.print(frame.data[0], HEX);
  // Serial.print(frame.data[1], HEX);
  // Serial.print(frame.data[2], HEX);
  // Serial.print(frame.data[3], HEX);
  // Serial.print(frame.data[4], HEX);
  // Serial.print(frame.data[5], HEX);
  // Serial.print(frame.data[6], HEX);
  // Serial.println(frame.data[7], HEX);

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