#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_BMP280.h>
#define BMP280_ADDRESS 0x76
Adafruit_BMP280 bmp;

#define CAN0_INT 2
// MCP_CAN CAN0(10); // MCP2515 module (8Mhz)
MCP_CAN CAN0(17); // All-in-one (16Mhz)

CanFix cf(0x82);

unsigned long now;
unsigned long lasttime;
unsigned int airspeed = 1300;
signed int verticalspeed;
signed int turnrate;
signed int lateralacceleration;
unsigned int cylinderheadtemperature[4] = {0,0,0,0};
unsigned int exhaustgastemperature[4] = {0,0,0,0};
unsigned int rpm;
unsigned int fuelquantity = 0;
unsigned int fuelflow = 0;
unsigned int fuelpressure = 4000;
unsigned int oilpressure = 3000;
unsigned int voltage = 0;
unsigned int amps = 0;
signed int roll = 0;
signed int heading = 0;
signed int pitch = 0;
bool countup[15];
volatile unsigned int counter = 0;
volatile float currentinHg = 30.01;


void setup() {
  Serial.begin(115200);
  cf.setDeviceId(0x82);
  cf.setModel(0x12345);
  cf.setFwVersion(2);

  cf.set_write_callback(can_write_callback);
  
  while(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("Unable to begin CAN0");
    delay(1000);
  }

  // if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK)
  //   Serial.println("MCP2515 Initialized Successfully!");
  // else
  //   Serial.println("Error Initializing MCP2515...");

  // CAN0.setMode(MCP_NORMAL);

  // pinMode(CAN0_INT, INPUT);
  // pinMode(3, INPUT_PULLUP);
  // pinMode(4, INPUT_PULLUP);

  // attachInterrupt(0, ai0, RISING);
  //attachInterrupt(1, ai1, RISING);

  // Serial.println("MCP2515 Initialized Successfully!");
  now = millis();
  lasttime = now;
  airspeed = 30;

  unsigned status;
  status = bmp.begin(0x76);
}

void loop() {
  // Set the "now" variable to the number of seconds since power 
  // was applied to the Arduino device
  now = millis();

  // check the "now" variable against another one called "lasttime" which
  // is set the "last time" this code block was run. If it's been more 
  // than 150ms, run this block again. If not, don't do anything.
  if (now - lasttime > 200) {

    Serial.println("Doing something ...");

    // First, let's look at the countup[0] variable. The [0] shows that this
    // is an "array" of countup values (e.g. there is more than one that use 
    // the same name). Airspeed is countup[0], Vertical speed will be countup[1], etc.
    // Each of these items in the array is a simple boolean (either a yes or a no)
    // that denotes whether we are counting up (incrementing the airspeed, vertical
    // speed, etc.) or not (decrementing the airspeed, vertical speed, etc.)
    
    // 60 (translated as 6.0 knots in CAN-FiX) is our low speed. If the airspeed gets 
    // lower than that, set the countup[0] value to true and start counting up instead.

    if (airspeed <= 60) {
      countup[0] = true;
    } 
    
    // if the airspeed value is higher than 1600 (160.0 knots), set countup[0] to false
    // which will tell our sendor to start counting down instead
    
    else if (airspeed >= 1600) {
      countup[0] = false;
    }

    // Now that we know whether the airspeed should be incrementing or decrementing (depending
    // on the value of countup[0]), either add 5 (+= 5) or subtract 5 (-= 5) from the current
    // airspeed value

    if (countup[0]) airspeed += 5;
    else airspeed -= 5;
    

    // Now we will create a CAN-FiX Parameter object (CFParameter) named "pIndicatedAirspeed" 
    // a CFParameter object holds details about the message you want to send to the CAN bus and it
    // will contain several properties/values, each detailed below.
    
    CFParameter pIndicatedAirspeed;   

    // (.type) - 0x183 is a hexadecimal representation of the number 387. In the CAN-FiX standard, 0x183/387 
    // is the message type for Indicated Airspeed (IAS). Depending on the type of message you want to
    // send, this value will change. Indicated Altitude (ALT) is 0x184 or 388. There are hundreds of types. This 
    // example program will only demonstrate a few of them.
    
    pIndicatedAirspeed.type = 0x183; 

    // (.index) - Next is the index property, which denotes which sensor of xxx type are we talking about. In the case
    // of IAS, you will only have a single value for the speed of the aircraft, so this will alwasy be 0x00 (or just 0).
    // for IAS. But if we were sending a Cylinder Head Temperature (CHT) reading, for example, we likely have more
    // than one cylinder in our engine. So we might have an index of 0x01 or 0x02 which would tell the FiX Gateway 
    // the CHT that we're sending is with regard to the 2nd or 3rd cylinder, respectively.
    
    pIndicatedAirspeed.index = 0x00;

    // (.fcb) - the "Funcion Code Byte" is a pretty loaded property. Using what we discussed above, this is a single byte 
    // of data, but each bit (remember a byte is 8 bits). You'll probably remember (from a basic computer course that you've
    // taken at some point in your life) that computers communicate using binary. So, using 0s and 1s. But we 
    // can combine those "bits" into larger pieces of data. For example, we can represent the number 37 in three ways:
    // - 37 (decimal - humans have 10 fingers and increment the "10s" place after we reach 9
    // - 0x25 (hexadecimal - where we increment the "16s" once after we reach 0x0F - (0-9 then continue A-F))
    // - 0b00100101 - (binary - where we increment the next field each time we reach 1 - 000, 001, 010, 011, 100, 110, 111, etc)
    // all three of the above example mean the same thing to a computer; that is: 37.
    // The Functional Code Byte is used to determine things about this message, and each bit (0 or 1) means something
    // to the CAN-FiX standard:
    //
    // | Bit 7  | Bit 6   | Bit 5   | Bit 4   | Bit 3   | Bit 2     | Bit 1   | Bit 0   |
    // | Meta3  | Meta2   | Meta1   | Meta0   | Future  | Failure   | Quality | Annuc   |
    //
    // The first 4 bits are used to send updates ABOUT the data (e.g. meta data). Maybe you want to tell the FiX Gateway
    // that your stall speed should be set to 50 knots. You would set the IAS message's Meta value for Vs, which would be
    // 1010, so your fcb value would be 0b10100000 or 0xA0 (same value in binary and hex, respectively).
    // Then you would give the message a "data" value for the new stall speed (maybe 0x50 0x00) to denote the new stall speed
    // The other bits (Future, Failure, Quality, Annuc) are used to give information about the data you're sending. For example,
    // if your sensor has a reason to believe that the value it's about to send might be in question, perhaps you would 
    // set the Quality bit so FiX Gateway knows to warn the pilot about questionable data. You would do this with an fcb value of
    // 0b00000010 (the 1 in there meaning the Quality flag is set). See the Frame Definitions section of the CAN-FiX spec for more info
    
    // If you're just sending a normal reading and there's nothing wrong with the message, .fcb should be 0, 0x00, or 0b00000000 (all 
    // the same value).
    
    pIndicatedAirspeed.fcb = 0x00;

    // Depending on the data being sent, the CAN-FiX standard may require more or fewer Bytes for a specific message type. In the case of 
    // IAS, the range you can send is 0.0 to 999.0. However, the units are 0.1 knots. So an airspeed of 123.4 knots would be communicated
    // as 1234 and the FiX Gateway will divide by 10.
    // If you've been following along with bits and bytes, you'll realize that a single Byte can only contain 256 values (e.g. 0-255). 
    // However, we can combine Bytes of data into other, larger data types. In the case of IAS, we need to be able to contain 0-9999 
    // (ten thousand values). Combining two Bytes of data will allow us plenty of overhead for our range. 
    // For IAS, this value is always positive, so we will use an "unsigned" integer. Unsigned means there are no bits wasted to 
    // specify a negative value (see the Vertical Speed info below to learn about "signed" integers).
    // For now, we will use two combined Bytes for an IAS value. Our range will be from 0000000000000000 to 10011100010000 (0-9999). But
    // an unsigned int has an upper limit of 1111111111111111 or 65,535! To make this easier to read and, more specifically, to adhere
    // to the CAN standard we need to split the Integer into two Bytes: 11111111 and 11111111. 
    // Look at the two data[] fields below (data[0] and data[1]). You will see that we use a variable we set previously (airspeed)
    // for both Bytes. data[0] uses the first 8 bits, but data[1] uses bit shifting for the seconds set of bits: airspeed>>8. 
    // This tells the Arduino to use the last 8 bits of the airspeed integer as a second Byte value. 
    // Bit shifting is beyond the scope of this demo. Watch a YouTube video about it if you want to learn more. 
    // Serial.println(airspeed);
    pIndicatedAirspeed.data[0] = airspeed;
    pIndicatedAirspeed.data[1] = airspeed>>8;
    pIndicatedAirspeed.data[2] = airspeed>>16;
    pIndicatedAirspeed.data[3] = airspeed>>24;

    // Now we indicate how many total Bytes of data we plan to send to the FiX Gateway. 1 for message type (IAS), 1 for index, 1 for FCB,
    // and 2 for the actual data. That's a total of 5 Bytes
    
    pIndicatedAirspeed.length = 7;

    // Now that our CFParameter named "pIndicatedAirspeed" is complete, we'll send it out to the CAN bus where the FiX Gateway will
    // injest it and the pyEfis screen will display it.
    
    // cf.sendParam(pIndicatedAirspeed);

    // Now we're just using the exact same data, but changing the type from IAS to True Airspeed (TAS) and resending it.
    // Note, IAS and TAS are likely not the same, depending on your altitude and air density. This is only a demonstration of
    // how to update a single property in CFParameter object and resend it.
    
    // pIndicatedAirspeed.type = 0x18D;
    // cf.sendParam(pIndicatedAirspeed);

    // setting max and min values for the demo just like before. In real life you would likely read this value 
    // from an Air Data encoder or do some math using previous and current altitude values divided by the time between
    // readings (adjusted to feet/minutes).
    
    if (verticalspeed <= -1000) {
      countup[1] = true;
    } else if (verticalspeed >= 1000) {
      countup[1] = false;
    }

    // Again, increment or decrement the value by 10 depending on whether the demo is counting up or down
    
    if (countup[1]) verticalspeed += 10;
    else verticalspeed -= 10;

    
    // Create a new CFParameter object for Vertical Speed.
    
    CFParameter pVerticalSpeed;

    // Set the message type. 0x186 or 390 denotes Vertical Speed (VERTSP)
    
    pVerticalSpeed.type = 0x186;

    // Like IAS you will only have 1 value for VS. 0x00 means this is the first value
    
    pVerticalSpeed.index = 0x00;

    // Sending a normal data update
    
    pVerticalSpeed.fcb = 0x00;

    // This is where it gets a little tricky. Because Vertical Speed can be positive or negative, we have to specify that. 
    // Unfortunately, to denote a negative, we have to set one of our bits to 1 or 0 to denote a positive or negative number.
    // However, using that bit effectively halves the number of values we can hold in a single integer. So Arduino requires us
    // to use a "signed" Integer for negative values. To overcome the lost values, a Signed Integer is four Bytes long (rather than
    // the two we used for IAS). Here we are cutting a Signed Integer ("verticalspeed") into four Bytes and using bit shifting
    // to use portions of that Signed Integer for each of the Bytes.

    pVerticalSpeed.data[0] = verticalspeed;
    pVerticalSpeed.data[1] = verticalspeed>>8;
    pVerticalSpeed.data[2] = verticalspeed>>16;
    pVerticalSpeed.data[3] = verticalspeed>>24;

    // Because this message contains two additional Bytes compared to IAS, we need to tell specify two more than the
    // 5 we used for IAS. Therefore, our entire message "length" will be 7 Bytes this time.
    
    pVerticalSpeed.length = 7;

    // Send the message to the CAN bus
    
    cf.sendParam(pVerticalSpeed);

    
    if (turnrate <= -10) {
      countup[2] = true;
    } else if (turnrate >= 10) {
      countup[2] = false;
    }

    // Again, increment or decrement the value by 10 depending on whether the demo is counting up or down
    
    if (countup[2]) turnrate += 1;
    else turnrate -= 1;   

    CFParameter pTurnRate;
    pTurnRate.type = 0x403;
    pTurnRate.index = 0x00;
    pTurnRate.fcb = 0x00;
    pTurnRate.data[0] = turnrate;
    pTurnRate.data[1] = turnrate>>8;
    pTurnRate.data[2] = turnrate>>16;
    pTurnRate.data[3] = turnrate>>24;
    pTurnRate.length = 7;
    cf.sendParam(pTurnRate);


    if (lateralacceleration < -250.0) countup[3] = true;
    if (lateralacceleration > 250.0) countup[3] = false;
    if (countup[3]) lateralacceleration += 10;
    else lateralacceleration -= 10;

    CFParameter pLateralAcceleration;
    pLateralAcceleration.index = 0x00;
    pLateralAcceleration.fcb = 0x00;
    pLateralAcceleration.type = 0x18B;
    pLateralAcceleration.data[0] = lateralacceleration;
    pLateralAcceleration.data[1] = lateralacceleration>>8;
    pLateralAcceleration.data[2] = lateralacceleration>>16;
    pLateralAcceleration.data[3] = lateralacceleration>>24;
    pLateralAcceleration.length = 7;
    cf.sendParam(pLateralAcceleration);



    if (cylinderheadtemperature[0] <= 1) countup[4] = true;
    if (cylinderheadtemperature[0] > 2800) countup[4] = false;
    
    if (countup[4]) {
      cylinderheadtemperature[0] += 100;
      cylinderheadtemperature[1] += 100;
      cylinderheadtemperature[2] += 100;
      cylinderheadtemperature[3] += 100;
    } else {
      cylinderheadtemperature[0] -= 100;
      cylinderheadtemperature[1] -= 100;
      cylinderheadtemperature[2] -= 100;
      cylinderheadtemperature[3] -= 100;
    } 

    ConvertCelsiusToFahrenheit(cylinderheadtemperature[0]);
    ConvertCelsiusToFahrenheit(cylinderheadtemperature[1]);
    ConvertCelsiusToFahrenheit(cylinderheadtemperature[2]);
    ConvertCelsiusToFahrenheit(cylinderheadtemperature[3]);

    CFParameter pCylinderHeadTemperature;
    pCylinderHeadTemperature.type = 0x500;
    pCylinderHeadTemperature.index = 0x00;
    pCylinderHeadTemperature.fcb = 0x00;
    pCylinderHeadTemperature.data[0] = cylinderheadtemperature[0];
    pCylinderHeadTemperature.data[1] = cylinderheadtemperature[0]>>8;
    pCylinderHeadTemperature.data[2] = cylinderheadtemperature[0]>>16;
    pCylinderHeadTemperature.data[3] = cylinderheadtemperature[0]>>24;
    pCylinderHeadTemperature.length = 7;
    cf.sendParam(pCylinderHeadTemperature);

    pCylinderHeadTemperature.index = 0x01;
    pCylinderHeadTemperature.data[0] = cylinderheadtemperature[1];
    pCylinderHeadTemperature.data[1] = cylinderheadtemperature[1]>>8;
    pCylinderHeadTemperature.data[2] = cylinderheadtemperature[1]>>16;
    pCylinderHeadTemperature.data[3] = cylinderheadtemperature[1]>>24;
    cf.sendParam(pCylinderHeadTemperature);

    pCylinderHeadTemperature.index = 0x02;
    pCylinderHeadTemperature.data[0] = cylinderheadtemperature[2];
    pCylinderHeadTemperature.data[1] = cylinderheadtemperature[2]>>8;
    pCylinderHeadTemperature.data[2] = cylinderheadtemperature[2]>>16;
    pCylinderHeadTemperature.data[3] = cylinderheadtemperature[2]>>24;
    cf.sendParam(pCylinderHeadTemperature);

    pCylinderHeadTemperature.index = 0x03;
    pCylinderHeadTemperature.data[0] = cylinderheadtemperature[3];
    pCylinderHeadTemperature.data[1] = cylinderheadtemperature[3]>>8;
    pCylinderHeadTemperature.data[2] = cylinderheadtemperature[3]>>16;
    pCylinderHeadTemperature.data[3] = cylinderheadtemperature[3]>>24;
    cf.sendParam(pCylinderHeadTemperature);




    if (exhaustgastemperature[0] <= 1) countup[5] = true;
    if (exhaustgastemperature[0] > 5000) countup[5] = false;
    
    if (countup[5]) {
      exhaustgastemperature[0] += 150;
      exhaustgastemperature[1] += 150;
      exhaustgastemperature[2] += 150;
      exhaustgastemperature[3] += 150;
    } else {
      exhaustgastemperature[0] -= 150;
      exhaustgastemperature[1] -= 150;
      exhaustgastemperature[2] -= 150;
      exhaustgastemperature[3] -= 150;
    } 

    ConvertCelsiusToFahrenheit(exhaustgastemperature[0]);
    ConvertCelsiusToFahrenheit(exhaustgastemperature[1]);
    ConvertCelsiusToFahrenheit(exhaustgastemperature[2]);
    ConvertCelsiusToFahrenheit(exhaustgastemperature[3]);

    CFParameter pExhaustGasTemperature;
    pExhaustGasTemperature.type = 0x502;
    pExhaustGasTemperature.index = 0x00;
    pExhaustGasTemperature.fcb = 0x00;
    pExhaustGasTemperature.data[0] = exhaustgastemperature[0];
    pExhaustGasTemperature.data[1] = exhaustgastemperature[0]>>8;
    pExhaustGasTemperature.data[2] = exhaustgastemperature[0]>>16;
    pExhaustGasTemperature.data[3] = exhaustgastemperature[0]>>24;
    pExhaustGasTemperature.length = 7;
    cf.sendParam(pExhaustGasTemperature);

    pExhaustGasTemperature.index = 0x01;
    pExhaustGasTemperature.data[0] = exhaustgastemperature[1];
    pExhaustGasTemperature.data[1] = exhaustgastemperature[1]>>8;
    pExhaustGasTemperature.data[2] = exhaustgastemperature[1]>>16;
    pExhaustGasTemperature.data[3] = exhaustgastemperature[1]>>24;
    cf.sendParam(pExhaustGasTemperature);

    pExhaustGasTemperature.index = 0x02;
    pExhaustGasTemperature.data[0] = exhaustgastemperature[2];
    pExhaustGasTemperature.data[1] = exhaustgastemperature[2]>>8;
    pExhaustGasTemperature.data[2] = exhaustgastemperature[2]>>16;
    pExhaustGasTemperature.data[3] = exhaustgastemperature[2]>>24;
    cf.sendParam(pExhaustGasTemperature);

    pExhaustGasTemperature.index = 0x03;
    pExhaustGasTemperature.data[0] = exhaustgastemperature[3];
    pExhaustGasTemperature.data[1] = exhaustgastemperature[3]>>8;
    pExhaustGasTemperature.data[2] = exhaustgastemperature[3]>>16;
    pExhaustGasTemperature.data[3] = exhaustgastemperature[3]>>24;
    cf.sendParam(pExhaustGasTemperature);




    if (rpm < 1) countup[6] = true;
    if (rpm > 3010) countup[6] = false;

    if (countup[6]) rpm += 32;
    else rpm -= 32;

    CFParameter pRPM;
    pRPM.type = 0x200;
    pRPM.index = 0x00;
    pRPM.fcb = 0x00;
    pRPM.data[0] = rpm;
    pRPM.data[1] = rpm>>8;
    pRPM.data[2] = rpm>>16;
    pRPM.data[3] = rpm>>24;
    pRPM.length = 7;
    cf.sendParam(pRPM);

    CFParameter pMAP;
    pRPM.type = 0x21E;
    pRPM.index = 0x00;
    pRPM.fcb = 0x00;
    pRPM.data[0] = 2987;
    pRPM.data[1] = 2987>>8;
    pRPM.length = 5;
    cf.sendParam(pRPM);

    float oiltemperature = bmp.readTemperature();
    unsigned int oiltemp = oiltemperature * 10;
    CFParameter pOilTemp;
    pOilTemp.type = 0x222;
    pOilTemp.index = 0x00;
    pOilTemp.fcb = 0x00;
    pOilTemp.data[0] = oiltemp;
    pOilTemp.data[1] = oiltemp>>8;
    pOilTemp.length = 5;
    cf.sendParam(pOilTemp);


    if (oilpressure < 3000) countup[7] = true;
    if (oilpressure > 7500) countup[7] = false;

    if (countup[7]) oilpressure += 15;
    else oilpressure -= 15;

    
    CFParameter pOilPressure;
    pOilPressure.type = 0x220;
    pOilPressure.index = 0x00;
    pOilPressure.fcb = 0x00;
    pOilPressure.data[0] = oilpressure;
    pOilPressure.data[1] = oilpressure>>8;
    pOilPressure.length = 5;
    cf.sendParam(pOilPressure);


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


    if (fuelquantity < 1) countup[8] = true;
    if (fuelquantity > 2000) countup[8] = false;

    if (countup[8]) fuelquantity += 15;
    else fuelquantity -= 15;

    CFParameter pFuelQuantity;
    pFuelQuantity.type = 0x226; // Left Fuel Tank
    pFuelQuantity.index = 0x00;
    pFuelQuantity.fcb = 0x00;
    pFuelQuantity.data[0] = fuelquantity;
    pFuelQuantity.data[1] = fuelquantity>>8;
    pFuelQuantity.data[2] = fuelquantity>>16;
    pFuelQuantity.data[3] = fuelquantity>>24;
    pFuelQuantity.length = 7;
    cf.sendParam(pFuelQuantity);

    pFuelQuantity.type = 0x227; // Right Fuel Tank
    cf.sendParam(pFuelQuantity);



    if (fuelflow < 400) countup[9] = true;
    if (fuelflow > 1000) countup[9] = false;

    if (countup[9]) fuelflow += 20;
    else fuelflow -= 20;

    CFParameter pFuelFlow;
    pFuelFlow.type = 0x21A; // Fuel Flow
    pFuelFlow.index = 0x00;
    pFuelFlow.fcb = 0x00;
    pFuelFlow.data[0] = fuelflow;
    pFuelFlow.data[1] = fuelflow>>8;
    pFuelFlow.data[2] = fuelflow>>16;
    pFuelFlow.data[3] = fuelflow>>24;
    pFuelFlow.length = 7;
    cf.sendParam(pFuelFlow);

    

    if (fuelpressure < 4000) countup[10] = true;
    if (fuelpressure > 5000) countup[10] = false;

    if (countup[10]) fuelpressure += 20;
    else fuelpressure -= 20;

    CFParameter pFuelPressure;
    pFuelPressure.type = 0x21C; // Fuel Pressure
    pFuelPressure.index = 0x00;
    pFuelPressure.fcb = 0x00;
    pFuelPressure.data[0] = fuelpressure;
    pFuelPressure.data[1] = fuelpressure>>8;
    pFuelPressure.data[2] = fuelpressure>>16;
    pFuelPressure.data[3] = fuelpressure>>24;
    pFuelPressure.length = 7;
    cf.sendParam(pFuelPressure);


    if (voltage < 10) countup[11] = true;
    if (voltage > 150) countup[11] = false;

    if (countup[11]) voltage += 2;
    else voltage -= 2;

    CFParameter pVoltage;
    pVoltage.type = 0x50E; // Voltage
    pVoltage.index = 0x00;
    pVoltage.fcb = 0x00;
    pVoltage.data[0] = voltage;
    pVoltage.data[1] = voltage>>8;
    pVoltage.data[2] = voltage>>16;
    pVoltage.data[3] = voltage>>24;
    pVoltage.length = 7;
    cf.sendParam(pVoltage);


    if (amps < 150) countup[12] = true;
    if (amps > 300) countup[12] = false;

    if (countup[12]) amps += 2;
    else amps -= 2;

    CFParameter pAmps;
    pAmps.type = 0x512; // Amps
    pAmps.index = 0x00;
    pAmps.fcb = 0x00;
    pAmps.data[0] = amps;
    pAmps.data[1] = amps>>8;
    pAmps.data[2] = amps>>16;
    pAmps.data[3] = amps>>24;
    pAmps.length = 7;
    cf.sendParam(pAmps);


    if (roll < -3000) countup[13] = true;
    if (roll > 3000) countup[13] = false;

    if (countup[13]) roll += 250;
    else roll -= 300;

    CFParameter pRoll;
    pRoll.type = 0x181; // Roll
    pRoll.index = 0x00;
    pRoll.fcb = 0x00;
    pRoll.data[0] = roll;
    pRoll.data[1] = roll>>8;
    pRoll.data[2] = roll>>16;
    pRoll.data[3] = roll>>24;
    pRoll.length = 7;
    cf.sendParam(pRoll);

    
    //if (heading <= 0) heading += 360;

    signed int magneticHeading = heading - 1800;
    
    if (magneticHeading < -900) countup[14] = true;
    if (magneticHeading > 900) countup[14] = false;

    if (countup[14]) heading += 15;
    else heading -= 15;

    CFParameter pHeading;
    pHeading.type = 0x185; // Magnetic Heading
    pHeading.index = 0x00;
    pHeading.fcb = 0x00;
    pHeading.data[0] = heading;
    pHeading.data[1] = heading>>8;
    pHeading.data[2] = heading>>16;
    pHeading.data[3] = heading>>24;
    pHeading.length = 7;
    cf.sendParam(pHeading);

    Serial.print ("heading: ");
    Serial.print (heading);
    Serial.print ("\tmagneticHeading: ");
    Serial.print (magneticHeading);

    
    if (pitch < -9100) countup[15] = true;
    if (pitch > 9200) countup[15] = false;

    if (countup[15]) pitch += 100;
    else pitch -= 100;

    CFParameter pPitch;
    pPitch.type = 0x180; // Pitch
    pPitch.index = 0x00;
    pPitch.fcb = 0x00;
    pPitch.data[0] = pitch;
    pPitch.data[1] = pitch>>8;
    pPitch.data[2] = pitch>>16;
    pPitch.data[3] = pitch>>24;
    pPitch.length = 7;
    cf.sendParam(pPitch);

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

int ConvertCelsiusToFahrenheit(int degreesCelsius) {
  int fahrenheit = (degreesCelsius * 9/5) + 32;
  return fahrenheit;
}


// void ai1() {
//   if (digitalRead(4) == LOW) {
//     // counter++;
//     currentinHg -= 0.01;
//   } else {
//     // counter--;
//     currentinHg += 0.01;
//   }
// }

// void sendPitchAngle(float angle) {
//   CFParameter p;
//   int x;

//   p.type = 0x180;
//   p.index = 0;
//   p.fcb = 0x00;
//   x = angle*100;
//   p.data[0] = x;
//   p.data[1] = x>>8;
//   p.length = 5;

//   cf.sendParam(p);
// }

// void sendRollAngle(float angle) {
//   CFParameter p;
//   int x;

//   p.type = 0x181;
//   p.index = 0;
//   p.fcb = 0x00;
//   x = angle*100;
//   p.data[0] = x;
//   p.data[1] = x>>8;
//   p.length = 5;

//   cf.sendParam(p);
// }


