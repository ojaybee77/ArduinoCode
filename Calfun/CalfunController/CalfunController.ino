/*
* CALFUN CONTROLLER
* 
* Connections:
* Serial0 UART:
* ESP32 NANO RX0 (D1) to Propius Main Board UART_TX
* ESP32 NANO TX0 (D0) to Propius Main Board UART_RX
* ESP32 NANO GND to Propius Main Board VSS (0V/GND)
*/



/* *************************     DEFINES     *******************************************/
#define TERATERM 1    // Adds 0x7F (DEL) to backspace for Teraterm only
#define DEBUG 0       // set to 1 to enable debug printing of UART TX/RX data

/* *************************     INCLUDES    *******************************************/
#include <math.h>
#include <string.h>
#include <AutoPID.h>

const char compile_date[] =  "Compiled: " __DATE__ " " __TIME__;

#define XTR2_RESPONSE_TIME_MS 15
#define MAX_TEST_TIME_MS 60000
#define MEASUREMENT_INTERVAL_MS 2000
#define COMMAND_LENGTH 22
#define OFF 0
#define ON 1

// Smoke element
#define SMOKE_START_TEMP      300
#define SMOKE_MAX_TEMP        450
#define SMOKE_START_DUTY      20
#define SMOKE_MAX_DUTY        40

#define SMOKE_START_TEMP_PARAM_NUM  1
#define SMOKE_MAX_TEMP_PARAM_NUM  2
#define SMOKE_START_DUTY_PARAM_NUM  3
#define SMOKE_MAX_DUTY_PARAM_NUM  4

// ADC and NTC settings
#define ADC_FULL_SCALE      4095.0
#define MAX_OBS_RANGE       8.7
#define MAX_USABLE_ADC      (int)(ADC_FULL_SCALE * 0.95)
#define AUTO_PRIME_COMPLETE_ADC (int)(ADC_FULL_SCALE * 0.8)

// PID settings and gains
#define LOOP_UPDATE_TIME_MS 500
#define PID_PUMP_MIN        0
#define PID_PUMP_MAX        3 
#define KP                  3
#define KI                  0.25
#define KD                  0.0

/* *************************     PINS         *******************************************/



/* *************************     GLOBAL VARS  *******************************************/
uint8_t ledOnCommand[22] = {0x00, 0x07, 0x16, 
                            0x00, 0x00, 0x00, 0x01,   // param-1:
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x05}; 
 
uint8_t ledOffCommand[22] = {0x00, 0x07, 0x16, 
                            0x00, 0x00, 0x00, 0x00,   // param-1:
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x05}; 

uint8_t smokeElementOn[22] = {0x00, 0x15, 0x16, 
                            0x00, 0x00, 0x00, 0x00,   // param-1: start temp (uint16)
                            0x00, 0x00, 0x00, 0x00,   // param-2: max temp (uint16)
                            0x00, 0x00, 0x00, 0x00,   // param-3: start duty (uint8)
                            0x00, 0x00, 0x00, 0x00,   // param-4: max duty (uint8)
                            0x05};

uint8_t smokeElementOff[22] = {0x00, 0x16, 0x16, 
                              0x00, 0x00, 0x00, 0x00,   // param-1: start temp (uint16)
                              0x00, 0x00, 0x00, 0x00,   // param-2: max temp (uint16)
                              0x00, 0x00, 0x00, 0x00,   // param-3: start duty (uint8)
                              0x00, 0x00, 0x00, 0x00,   // param-4: max duty (uint8)
                              0x05};

uint8_t getTestStatus[22] = {0x00, 0x22, 0x16, 
                            0x00, 0x00, 0x00, 0x00,   // param-1:
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x0a};                             

uint8_t startSmokeTest[22] = {0x00, 0x08, 0x16, 
                            0x00, 0x00, 0x00, 0x00,   // param-1:
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x0a}; 

uint8_t stopTest[22] =     {0x00, 0x09, 0x16, 
                            0x00, 0x00, 0x00, 0x00,   // param-1:
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x0a};

uint8_t startFan[22] =     {0x00, 0x0b, 0x16, 
                            0x00, 0x00, 0x00, 0x00,   // param-1: units are in RPM
                            0x00, 0x00, 0x07, 0x6c,   // param-2: 0x076c --> 1,900 RPM (about 30%)
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x05};

uint8_t stopFan[22] =     {0x00, 0x0c, 0x16, 
                            0x00, 0x00, 0x00, 0x00,   // param-1:
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x05};

uint8_t setFanPwm[22] =    {0x00, 0x0e, 0x16, 
                            0x00, 0x00, 0x00, 0x64,   // param-1: PWM 0..100% 
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x05};

uint8_t setFanRpm[22] =    {0x00, 0x0f, 0x16, 
                            0x00, 0x00, 0x07, 0x6c,   // param-1: 0x076C --> 1,900 RPM
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x05};

uint8_t getFanStatus[22] = {0x00, 0x0d, 0x16, 
                            0x00, 0x00, 0x00, 0x00,   // param-1: return value PWM
                            0x00, 0x00, 0x00, 0x00,   // param-2: return value RPM
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4: Byte 0--> running, Byte 1--> stalled
                            0x05};  

uint8_t torchOn[22] =      {0x00, 0x62, 0x16, 
                            0x00, 0x00, 0x00, 0x01,   // param-1: 0x01 --> ON, 0x04 --> FLASHING
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x10};


uint8_t torchFlash[22] =   {0x00, 0x62, 0x16, 
                            0x00, 0x00, 0x00, 0x04,   // param-1: 0x01 --> ON, 0x04 --> FLASHING
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x10};  

uint8_t torchOff[22] =     {0x00, 0x62, 0x16, 
                            0x00, 0x00, 0x00, 0x00,   // param-1: 
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x10};  

uint8_t getSensBrdStatus[22] = {0x00, 0x3E, 0x16, 
                            0x00, 0x00, 0x00, 0x00,   // param-1: 
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x10};  

// stepper motor 85 steps clockwise
uint8_t startStepper[22] = {0x00, 0x1b, 0x16, 
                            0x00, 0x00, 0x00, 0x55,   // param-1: 0x55 --> 85 steps
                            0x00, 0x00, 0x00, 0x01,   // param-2: 0x00 --> anticlockwise, 0x01--> clockwise
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x05};

uint8_t stopStepper[22] = { 0x00, 0x1c, 0x16, 
                            0x00, 0x00, 0x00, 0x00,   // param-1: 
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x05};

// stepper motor continously running anticlockwise
 uint8_t anticlockwiseStepper[22] = { 0x00, 0x1b, 0x16, 
                            0x00, 0xFF, 0xFF, 0xFF,   // param-1: *a lot* of steps, effectively free running!
                            0x00, 0x00, 0x00, 0x00,   // param-2: 
                            0x00, 0x00, 0x00, 0x00,   // param-3:
                            0x00, 0x00, 0x00, 0x00,   // param-4
                            0x05};   
 





const char ASCII_BACKSPACE = 0x08;
const char ASCII_DELETE = 0x7F;
double pctPerFoot, setPoint;
double pump = 0;
char formattedVal[32];          // %03d -->  'xxx' degC with leading zero padding
uint8_t responseData[22] = {0};

// input/output variables passed by reference, so they are updated automatically
// PID uses PWM 1..255 insterad of 1..100% duty
AutoPID pidController(&pctPerFoot, &setPoint, &pump, PID_PUMP_MIN, PID_PUMP_MAX, KP, KI, KD);

/* *************************     FUNCTIONS    *******************************************/
uint32_t getParam32Bit(int paramNum , uint8_t reply[]) {
  // return four bytes of requested parameter number
  int idx = 3 + (4 * (paramNum-1)); 
  return reply[idx]<<24 + reply[idx+1]<<16 + reply[idx+2]<<8 + reply[idx+3];
}

uint16_t getParam16Bit(int paramNum , uint8_t reply[]) {
  // return two bytes of requested parameter number 
  int idx = 5 + (4 * (paramNum-1));
  return reply[idx+2]<<8 + reply[idx+3];
}

uint8_t getParam8Bit(int paramNum , uint8_t reply[]) {
  // return  one byte of requested parameter number
  int idx = 6 + (4 * (paramNum-1));
  return reply[idx+3];
}

void modifyParam(int paramNum, uint32_t paramData, uint8_t command[]) {
  int index, i;
  uint8_t temp;

  // range check the parameter number
  if (paramNum < 1 || paramNum > 4) return;

  // the index of the parameter LSB
  index = 6 + (4 * (paramNum-1));
   
  for (i=4; i>0; i--) {
    // mask of each parameter byte in turn
    temp = (uint8_t)(paramData & 0xFF);
    paramData = paramData >> 8;
    command[index--] = temp;
  }
}


float convertToPercentPerFoot(uint16_t adc) {
  // Total range is about 8.7% for ADC full scale, so assume 0 ADC gives 0%/ft obscuration
  return MAX_OBS_RANGE * (float)(adc)/ADC_FULL_SCALE;
}


uint16_t printAdcReading() {
  uint16_t adcMeasurement = 0; 
  if (sendAndCheckCommand(getTestStatus, responseData)) {
       // response was OK so let's check bytes 01 and 19 match the command sent
      if (responseData[1] == 0x22 && responseData[19] == 0x0A) {
        // otherwise all is OK so read ADC (bytes 11 and 12)
        adcMeasurement = responseData[11] << 8 | responseData[12]; 
        sprintf(formattedVal, "%04d", adcMeasurement); 
        Serial.print(formattedVal);
      }
      else {
        //Serial.print("ERR!");
      }
  }
  return adcMeasurement;
}


void printBackspaces(uint8_t n) { 
  for (uint8_t i=0; i<n; i++) {
     Serial.write(ASCII_BACKSPACE);   
     if (TERATERM) Serial.write(ASCII_DELETE);
  }
}


void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
        if (!DEBUG) return;

        Serial.print("0x"); 
        for (int i=0; i<length; i++) { 
          if (data[i]<0x10) {Serial.print("0");} 
          Serial.print(data[i],HEX); 
          Serial.print(" "); 
        }
        Serial.println("");
}

bool isFanStalled() {
  uint32_t fan_status;
  bool result = sendAndCheckCommand(getFanStatus, responseData);
   // if command to read fan fails, assume fan is stalled - for safety!
  if (!result) return true;

  // now parse response to get fan status 
  // fan_status = getParam32Bit(1, responseData);
  // Serial.println(fan_status, HEX);
  // fan_status = getParam32Bit(2, responseData);
  // Serial.println(fan_status, HEX);
  //  fan_status = getParam32Bit(3, responseData);
  // Serial.println(fan_status, HEX);
   fan_status = getParam32Bit(4, responseData);
  Serial.println(fan_status, HEX);
  // check if second byte of fourth param is set (= stall)
  if (fan_status & 0x00FF0000) return true;

  // else fan was not stalled
  return false;
}

bool sendAndCheckCommand(uint8_t command[], uint8_t* responseData) {
  // add the CRC bytes
  uint16_t crc = crc16(command);
  // add MSB of CRC
  command[20] = (crc & 0xFF00) >> 8;
  // add LSB or CRC as final byte
  command[21] = crc  &0xFF;

  // send the command
  Serial0.write(command, COMMAND_LENGTH);
  PrintHex8(command, COMMAND_LENGTH);

  // wait for the XTR2 to process it and reply
  delay(XTR2_RESPONSE_TIME_MS);
  
  // read in response bytes (if any!)
  if (Serial0.available() >= COMMAND_LENGTH) { // Ensure the entire array is available
    // read the bytes into the array
    if (Serial0.readBytes(responseData, COMMAND_LENGTH) != COMMAND_LENGTH) {
      // not enough bytes received, ERROR!   
      return false;
    };
    // all bytes received OK
    //Serial.println("sendAndCheckCommand OK: ");
    PrintHex8(responseData, COMMAND_LENGTH);
    return true;
  }
  else {
    // something wrong, ERROR!
    return false;
  }
}


uint16_t crc16(const uint8_t buffer[]) {
  uint16_t crcValue = 0xFFFF;
  uint16_t index, j;

  for (index = 0; index < 20; index++) {
    crcValue = crcValue ^ ((uint16_t)buffer[index]);
    for (j = 0; j < 8; j++) {
      if (crcValue & 0x0001) {
        crcValue = (crcValue >> 1) ^ 0x8408;
      } else {
        crcValue = (crcValue >> 1);
      }
    }
  }
  return ~crcValue;
}



/* *************************     INITIALISATION  *******************************************/
void setup() {
  // for debug, user info:
  Serial.begin(115200);

  // ESP32 Nano dedicated XTR2 UART:
  Serial0.begin(115200);

  // PID set-up
  pidController.stop();
  //pidController.setBangBang(100);       // try not to use Bang-Bang control?!
  pidController.setTimeStep(LOOP_UPDATE_TIME_MS);
  
  // Wait for board to connect
  delay(4000);
 }


/* *************************     MAIN LOOP       *******************************************/
void loop() {
  int  i, uartInt, adcMeasurement, len;
  uint16_t rpm = 0;
  uint8_t modifiedCommand[22];

  Serial.println(compile_date);
  
  
MENU:
  // send a dummy/no action command to "flush" the response buffer
  sendAndCheckCommand(ledOffCommand, responseData);
  Serial.println();
  Serial.println("**********************");
  Serial.println("Calfun Command-O-Matic");
  Serial.println("**********************");
  Serial.println("");
  Serial.println("1: LED  Check\t2: Fan  Check\t3: ADC calibration");  
  Serial.println("4: Auto Prime\t5: Smoke Test\t6: Clearing");  
  Serial.println("10: Quit");
  Serial.println("");

  do {
    uartInt = Serial.parseInt();  // returns 0 after short time out
    if (uartInt == 10) goto QUIT;
    if (uartInt == 1 ) goto LED_CHECK;
    if (uartInt == 2) goto FAN_CHECK;
    if (uartInt == 3) goto ADC_CAL;
    if (uartInt == 4) goto AUTO_PRIME;  
    if (uartInt == 5) goto SMOKE_TEST;
    if (uartInt == 6) goto CLEARING;
  } while (uartInt < 1);

QUIT:
  Serial.println("***** STOPPED ******");
  while(1);

LED_CHECK:
  Serial.println("1: Quit");
  memcpy(modifiedCommand, ledOnCommand, sizeof(ledOnCommand));

  for (i=1; i<11; i++) {
    // scroll colours enum of LED by changing the 7th byte
    modifiedCommand[6] = i;
    sendAndCheckCommand(modifiedCommand, responseData);
    delay(200);
  }

  sendAndCheckCommand(ledOffCommand, responseData);
  goto MENU;


FAN_CHECK:
  memcpy(modifiedCommand, setFanRpm, sizeof(setFanRpm));
  Serial.println("");
  Serial.println("Enter PWM 10..100%\t\t1: Quit");

  // starts fan at 100%
  sendAndCheckCommand(startFan, responseData);   
  do {
      uartInt = Serial.parseInt();
      if (uartInt >= 10 && uartInt <= 100) {
        rpm = uartInt * 70;
        modifiedCommand[5] = (uint8_t)((rpm & 0xFF00)>>8);
        modifiedCommand[6] = (uint8_t)(rpm & 0xFF);
        sendAndCheckCommand(modifiedCommand, responseData);
        delay(333);
        printBackspaces(3);
        if (uartInt != 1) uartInt=0;
      }
      //isFanStalled();
    } while (uartInt != 1);
    sendAndCheckCommand(stopFan, responseData);
    goto MENU;


ADC_CAL:
  Serial.println("");
  Serial.println("Ensure cup is clear from smoke!!");
  Serial.println("Adjust LED current (R7) to set ADC reading to *just* above zero");
  Serial.println("(TRANS_OUT pin should be approx. 4.27V)");
  Serial.println("1: Quit");

    do {
      uartInt = Serial.parseInt();
      printAdcReading(); 
      delay(333);
      printBackspaces(4); 
    } while (uartInt != 1); 
    goto MENU;


AUTO_PRIME:
  memcpy(modifiedCommand, smokeElementOn, sizeof(smokeElementOn));
  Serial.println("");
  Serial.println("Priming until smoke saturates Obscuration Sensor!\t\t1: Quit");

  // fan to 30%
  sendAndCheckCommand(startFan, responseData);
  sendAndCheckCommand(setFanRpm, responseData);

  // pump anticlockwise
  sendAndCheckCommand(anticlockwiseStepper, responseData);

  // smoke element on
  modifyParam(1,SMOKE_START_TEMP, modifiedCommand);
  modifyParam(2,SMOKE_MAX_TEMP, modifiedCommand);
  modifyParam(3,SMOKE_START_DUTY, modifiedCommand);
  modifyParam(4,SMOKE_MAX_DUTY, modifiedCommand);
  sendAndCheckCommand(modifiedCommand, responseData);
  
  do {
    uartInt = Serial.parseInt();
    // exit if fan is stalled
    /*
    if (isFanStalled()) {
      break;
    }
    */
    
    adcMeasurement = printAdcReading();
    if (adcMeasurement > AUTO_PRIME_COMPLETE_ADC) {
      // exit when smoke has saturated cup
      break;
    }

    Serial.print(" ");
    Serial.print(convertToPercentPerFoot(adcMeasurement));
    delay(333);
    Serial.println("");
    //printBackspaces(4);
    } while (uartInt != 1);

  // smoke element off
  sendAndCheckCommand(smokeElementOff, responseData);

  // fan off
  sendAndCheckCommand(stopFan, responseData);

  // pump off
  sendAndCheckCommand(stopStepper, responseData);
  goto MENU;


SMOKE_TEST: 
  Serial.println("");
  Serial.println("Running a smoke test according to set profile...\t\t1: Quit");
  Serial.println("ADC reading;  Percent-per-foot");
  sendAndCheckCommand(startSmokeTest,responseData);
  do {
    uartInt = Serial.parseInt();
    adcMeasurement = printAdcReading();
    Serial.print(" ");
    Serial.print(convertToPercentPerFoot(adcMeasurement));
    delay(333);
    Serial.println("");
    //printBackspaces(4);
  } while (uartInt != 1);
  sendAndCheckCommand(stopTest,responseData);
  goto MENU;

CLEARING:
 int numLoops=30;
 Serial.println("Clearing cup\t\t1: Quit");
  sendAndCheckCommand(startFan, responseData);  
  sendAndCheckCommand(setFanPwm, responseData);    
  do {
      sprintf(formattedVal, "%02d", numLoops); 
      Serial.print(formattedVal);
      delay(333);
      printBackspaces(2);
    } while (numLoops-- > 0);
    sendAndCheckCommand(stopFan, responseData);
    goto MENU;
}
