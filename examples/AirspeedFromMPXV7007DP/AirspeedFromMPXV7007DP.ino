#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

#include <Wire.h>
#include <SPI.h>

#define CAN0_INT 2


MCP_CAN CAN0(10); // MCP2515 module (8Mhz)
// MCP_CAN CAN0(17); // All-in-one (16Mhz)

CanFix cf(0x82);

int startPressure = 517;
unsigned long now;
unsigned long lastParameterSet;
unsigned long lastTime;
long smoothReading = 0;
int smoothReadingCount = 12;
int smoothReadings[12];
unsigned long smoothCount = 0;



// Define several V speed parameters that will be pushed to the FIX Gateway. 
// These values will eventually set the colored rings/lines on the ASI for
// each of the following speeds. Note: These meta headers are passed in 
// Least Significant Bit format (0001 will be sent as 1000)
#define META_MIN  0b00010000
#define META_MAX  0b00100000
#define VNE       0b01010000
#define VFE       0b01100000
#define VS1       0b10100000
#define VS0       0b10110000
#define VNO       0b10010000


// Instantiate the various V speeds that will appear on the Airspeed Indicator (ASI)
// these values, according to the CAN-FiX spec, are in 1/10 of a knot. So 1000 here
// denotes 100.0 knots on the ASI.
int asiMinValue = 0;    // lowest value on the tape/dial
int asiMaxValue = 2000; // highest value on the tape/dial 
int asiVneValue = 1710; // Never Exceed Speed
int asiVfeValue = 860;  // max flap extension speed
int asiVs1Value = 400;  // Clean stall speed
int asiVs0Value = 200; //350;  // Full flap stall speed
int asiVnoValue = 1250; // max maneuvering speed


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  cf.setDeviceId(0x82);
  cf.setModel(0x12345);
  cf.setFwVersion(2);

  cf.set_write_callback(can_write_callback);
  
  while(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Unable to begin CAN0");
    delay(1000);
  }

  // for (int i=0; i<smoothCount; i++) {
  //   smoothReadings[i] = 0;
  // }

  now = millis();
  lastTime = now;
  lastParameterSet = now;

}

// the loop routine runs over and over again forever:
void loop() {
  
  now = millis();

  // Every second, set the V-Speeds on the Airspeed Indicator (ASI)
  if (now - lastParameterSet > 1000) {
    
    Serial.println("Setting parameters ...");

    SetParameter(META_MIN, asiMinValue);
    SetParameter(META_MAX, asiMaxValue);
    SetParameter(VNE, asiVneValue);
    SetParameter(VFE, asiVfeValue);
    SetParameter(VS1, asiVs1Value);
    SetParameter(VS0, asiVs0Value);
    SetParameter(VNO, asiVnoValue);
    lastParameterSet = now;
    delay(10);

  }


  // read the pressure differential sensor and subtract whatever value is zero when 
  // there is no difference in pressure (called startPressure above)
  smoothReading = analogRead(A0) - startPressure;
  // smoothReading += analogRead(A0);

  // increment a counter so you know how many times the value has been read,
  // which is used to calculate the average of the readings
  smoothCount++;

  if (now - lastTime > 25) {
    
    // int averagedValue = smoothReading / smoothCount;
    int sensorValue = smoothReading;
    int smoothValue = getSmoothValue(sensorValue); //getSmoothValue(smoothReading / smoothCount);

    // Read the voltage value of the pressure differential sensor - 5V = 7000kPa
    double pressure = map(smoothValue, 0, 500, 0, 7000);
    
    if (pressure < 2) {
      pressure = 0;
    }
    
    double metersPerSecond = sqrt((2 * pressure)/1.225);
    unsigned int knots = (metersPerSecond * 1.943) * 10;
    
    // double airspeed = sqrt(2*4)
    

    Serial.print("sensorValue: ");
    Serial.print(sensorValue);
    Serial.print(" | pressure (kPa): ");
    Serial.print(pressure);
    Serial.print("\tSpeed:\t");
    Serial.print(metersPerSecond);
    Serial.print(" meters/second");
    Serial.print("\t");
    Serial.print(float(knots)/10);
    Serial.print(" knots");
    Serial.println("");

    CFParameter pIndicatedAirspeed;
    pIndicatedAirspeed.type = 0x183;
    pIndicatedAirspeed.index = 0x00;
    pIndicatedAirspeed.fcb = 0x00;
    pIndicatedAirspeed.data[0] = knots;
    pIndicatedAirspeed.data[1] = knots>>8;
    pIndicatedAirspeed.data[2] = knots>>16;
    pIndicatedAirspeed.data[3] = knots>>24;
    pIndicatedAirspeed.length = 7;
    cf.sendParam(pIndicatedAirspeed);

    smoothReading = 0;
    smoothCount = 0;
    lastTime = now;

  }


}

// This function is a callback function that is used by the CanFix object
// for sending CAN frames on the network.  The CanFix class is agnostic
// toward the actual CAN communication mechanism.  This function should
// translate from the common CanFixFrame structure and send it on the Bus.
void can_write_callback(CanFixFrame frame) {
  CAN0.sendMsgBuf(frame.id, 0, frame.length, frame.data);
}

int getSmoothValue(int newValue) {

  // Serial.print("newValue: ");
  // Serial.println(newValue);

  for (int i = smoothReadingCount; i>=0; i--) {
    if (i == 0) {
      smoothReadings[i] = newValue;
      // Serial.print("smoothReadings[0]: "); Serial.println(smoothReadings[0]);
    } else {
      smoothReadings[i] = smoothReadings[i-1];
      // Serial.print(smoothReadings[i]);
      // Serial.print(", ");
    }
  }
  

  int smoothedReturnValue = 0;
  for (int i = 0; i < smoothReadingCount; i++) {
    smoothedReturnValue += smoothReadings[i];
  }

  smoothedReturnValue = smoothedReturnValue / smoothReadingCount;
  
  return smoothedReturnValue;
}

void SetParameter(byte metaType, int data) {
    
    CFParameter pAirspeedParameter;
    pAirspeedParameter.type = 0x183;
    pAirspeedParameter.index = 0x00;
    pAirspeedParameter.fcb = metaType;
    pAirspeedParameter.data[0] = data;
    pAirspeedParameter.data[1] = data>>8;
    pAirspeedParameter.data[2] = data>>16;
    pAirspeedParameter.data[3] = data>>24;
    pAirspeedParameter.length = 7;
    cf.sendParam(pAirspeedParameter);

}