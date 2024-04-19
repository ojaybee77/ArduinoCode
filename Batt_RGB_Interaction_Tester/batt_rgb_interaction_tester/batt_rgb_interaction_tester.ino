#include "Wire.h"
#include "string.h" //memcpy

#define AD_PIN_SETTING 0    // 0 --> address 0xD0, 1--> address 0xD6
#define BQ40Z50_SMBUS_ADDRESS 0x0B

/*
Wire.endTransmission() return codes:
------------------------------------
0: success. 
1: data too long to fit in transmit buffer. 
2: received NACK on transmit of address.
3: received NACK on transmit of data.
4: other error.
5: timeout
*/


/******************* GLOBALS   *********************************/
const int AD_PIN = 2;
const uint8_t enableChip[2] = {0x00,0x20};
const uint8_t redFull[2] = {0x04,0xFF};
const uint8_t greenFull[2] = {0x05,0xFF};
const uint8_t blueFull[2] = {0x06,0xFF};
const uint8_t update[2] = {0x07,0x00};
const uint8_t setCurrent10mA[2] = {0x03,0x04};
const uint8_t setCurrent18mA[2] = {0x03,0x10};
const uint8_t allLedsOn[2] = {0x1d,0x03};
uint8_t device_address;
uint8_t rgbData[2];
const char compile_date[] =  "Compiled: " __DATE__ " " __TIME__;

/******************* FUNCTIONS *********************************/
uint8_t rgbInit (uint8_t addr) {
  Wire.beginTransmission(addr);
  memcpy(rgbData, enableChip,2);
  Wire.write(rgbData,2);
  memcpy(rgbData, setCurrent18mA,2);
  Wire.write(rgbData,2);
  memcpy(rgbData, allLedsOn,2);
  Wire.write(rgbData,2);
  memcpy(rgbData, redFull,2);
  Wire.write(rgbData,2);
  memcpy(rgbData, greenFull,2);
  Wire.write(rgbData,2);
  memcpy(rgbData, blueFull,2);
  Wire.write(rgbData,2);
  memcpy(rgbData, update,2);
  Wire.write(rgbData,2);
  return (Wire.endTransmission());
}

uint8_t rgbRandColours(uint8_t addr) {
  Wire.beginTransmission(addr);
  rgbData[0] = 0x04;
  rgbData[1] =  random(2) * 0xFF;
  Wire.write(rgbData,2);
  rgbData[0] = 0x05;
  rgbData[1] =  random(2) * 0xFF;
  Wire.write(rgbData,2);
  rgbData[0] = 0x06;
  rgbData[1] =  random(2) * 0xFF;
  Wire.write(rgbData,2);
  memcpy(rgbData, update,2);
  Wire.write(rgbData,2);
  return (Wire.endTransmission());
}

uint8_t battery(uint8_t smbus_addr) {
  // check if battery is present with a simple write to address only
  Wire.beginTransmission(smbus_addr);
  return (Wire.endTransmission());
}

/******************* INITIALIZATION******************************/
void setup() {
  Serial.begin(115200);
  delay(4000);
  Serial.println(compile_date);
  Wire.begin();
  pinMode(AD_PIN, OUTPUT);   
}

/******************* MAIN LOOP  *********************************/
void loop() {
  int count = 1000, status;

  if (AD_PIN_SETTING)  {
    device_address = 0x6B;
    digitalWrite(AD_PIN, HIGH);
  }
  else {
    device_address = 0x68;
    digitalWrite(AD_PIN, LOW);
  }

  delay(10);
  // talk to RGB controller (IS31FL3193)
  Serial.println(rgbInit(device_address));    

  // talk to battery (BQ40Z50)
  /*
  status = battery(BQ40Z50_SMBUS_ADDRESS);
  if (status > 0) {
    Serial.println(status);
    Serial.println("Battery comms failure!");
    goto STOP;
  }
  */

  while(--count > 0) {
    // do lots of RGB comms
    status = rgbRandColours(device_address);
    if (status > 0) {
      Serial.println(status);
      Serial.println (count);
      Serial.println("Ah ha, gotcha!");
      goto STOP;
    }
    delay(random(100)+200);
  }

STOP:
  while(1);
}


