#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <Wire.h>
#include <SPI.h>

// #include <Adafruit_BMP280.h>
// #define BMP280_ADDRESS 0x76
// Adafruit_BMP280 bmp;

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

  // if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK)
  //   Serial.println("MCP2515 Initialized Successfully!");
  // else
  //   Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT);
  // pinMode(3, INPUT_PULLUP);
  // pinMode(4, INPUT_PULLUP);

  // attachInterrupt(0, ai0, RISING);
  //attachInterrupt(1, ai1, RISING);

  // Serial.println("MCP2515 Initialized Successfully!");
  now = millis();
  lasttime = now;
  airspeed = 30;

  unsigned status;
  // status = bmp.begin(0x76);
}

void loop() {
  // Set the "now" variable to the number of seconds since power 
  // was applied to the Arduino device
  now = millis();

  // check the "now" variable against another one called "lasttime" which
  // is set the "last time" this code block was run. If it's been more 
  // than 150ms, run this block again. If not, don't do anything.
  if (now - lasttime > 150) {

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
    
    pIndicatedAirspeed.data[0] = airspeed;
    pIndicatedAirspeed.data[1] = airspeed>>8;

    // Now we indicate how many total Bytes of data we plan to send to the FiX Gateway. 1 for message type (IAS), 1 for index, 1 for FCB,
    // and 2 for the actual data. That's a total of 5 Bytes
    
    pIndicatedAirspeed.length = 5;

    // Now that our CFParameter named "pIndicatedAirspeed" is complete, we'll send it out to the CAN bus where the FiX Gateway will
    // injest it and the pyEfis screen will display it.
    
    cf.sendParam(pIndicatedAirspeed);

    // Now we're just using the exact same data, but changing the type from IAS to True Airspeed (TAS) and resending it.
    // Note, IAS and TAS are likely not the same, depending on your altitude and air density. This is only a demonstration of
    // how to update a single property in CFParameter object and resend it.
    pIndicatedAirspeed.type = 0x18D;
    cf.sendParam(pIndicatedAirspeed);

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

    // float meters = bmp.readAltitude(currentMillibars);

    // signed long indicatedAltitude = meters * 3.2804; //convert to feet

    // Serial.print("currentkPa: ");
    // Serial.print(currentkPa);
    // Serial.print("\tmeters: ");
    // Serial.print(meters);
    // Serial.print("\tindicatedaltitude: ");
    // Serial.println(indicatedaltitude);


    // CFParameter pIndicatedAltitude;
    // pIndicatedAltitude.type = 0x184;
    // pIndicatedAltitude.index = 0x00;
    // pIndicatedAltitude.fcb = 0x00;
    // pIndicatedAltitude.data[0] = indicatedAltitude;
    // pIndicatedAltitude.data[1] = indicatedAltitude>>8;
    // pIndicatedAltitude.data[2] = indicatedAltitude>>16;
    // pIndicatedAltitude.data[3] = indicatedAltitude>>24;
    // pIndicatedAltitude.length = 7;
    // cf.sendParam(pIndicatedAltitude);

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


void ai1() {
  if (digitalRead(4) == LOW) {
    // counter++;
    currentinHg -= 0.01;
  } else {
    // counter--;
    currentinHg += 0.01;
  }
}

void sendPitchAngle(float angle) {
  CFParameter p;
  int x;

  p.type = 0x180;
  p.index = 0;
  p.fcb = 0x00;
  x = angle*100;
  p.data[0] = x;
  p.data[1] = x>>8;
  p.length = 2;

  cf.sendParam(p);
}

void sendRollAngle(float angle) {
  CFParameter p;
  int x;

  p.type = 0x181;
  p.index = 0;
  p.fcb = 0x00;
  x = angle*100;
  p.data[0] = x;
  p.data[1] = x>>8;
  p.length = 2;

  cf.sendParam(p);
}


