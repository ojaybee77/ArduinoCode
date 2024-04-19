/*
Dual NPN Anemometer Using MCP4251-103 Digipot (10k)

Connections:
SIGNAL    ARDUNIO   MCP4251 
SPICLK    13        2
MISO      12        13
MOSI      11        3
CS        10        1
SHDN      9         12
WP        n/c       11 <--- to + 5V
WIPER1    A0        6 (P1W)  <--- 2k2 / R1 node (ZERO)
WIPER0    A1        9 (P0W)  <---1k / R2 node (CAL)
P0A       n/c       8 <--- connect to wiper PWB pin 9 (and Q3 collector)
P0B       n/c       10 <--- connect to 1k 
P1A       n/c       7 <--- connect to wiper PWB pin 6 (and +5V)
P1B       n/c       5 <--- connect to 2.2k resistors

FAN: 
* WHITE  -  PWM out; Pin 6
* YELLOW -  Tacho (RPM/30) in; Pin 5
* RED    -  Ext. 12V (Can use Arduino 5V just to check operation)
* BLACK  -  Arduino GND and Fan 12V PSU GND (tie together)

P0 ,-- CAL
P1 <-- ZERO
5V is put through both 10k pots (0.5mA)
and the wipers are sensed by the ADC channels

Pots are 10k, 257 steps. Power-up wiper value is mid-way. 
Incrementing the wiper moves it towards PxA (pins 7/8)
Decrementing the wiper moves it towards PxB (pins 5/10)

Fan PWM frequency see https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency

Fan Tacho frequency read see https://www.pjrc.com/teensy/td_libs_FreqCount.html
*/


#include "MCP4XXX.h"



#define TERATERM          0     // to define backspace behavious
//#define PACKAGE_TYPE      TO92  // comment out if using SOT-23

#ifdef PACKAGE_TYPE = TO92
  #define SETTLING_TIME_MS  8000
#else
  #define SETTLING_TIME_MS  6000
#endif

#define OFF_STATE HIGH
#define MAX_PWM_FOR_SPEED_INDICATOR  40
#define MAX__HZ_FOR_SPEED_INDICATOR 300

#define ZERO_POT WIPER1
#define CAL_POT WIPER0

#define ADC_FULL_SCALE    1023.0
#define SUPPLY_VOLTAGE    5.0

// Bisection Search Limits (don't make too tight!)
#define ZERO_POT_MAX_CODE 255
#define ZERO_POT_MIN_CODE 0
  /*CAUTION: ZERO_POT_MAX is actually the *lowest* resistance setting.
            Could overheat BJT Q1. Set carefully! */
#define CAL_POT_MAX_CODE 255
#define CAL_POT_MIN_CODE 0

#define ZERO_FREQ_MIN    2
#define ZERO_FREQ_MAX    8

#define CAL_FREQ_MIN    290
#define CAL_FREQ_MAX    300

#define MAX_BISECTIONS 7

#define CAL_TARGET_HZ_FULL_FAN 300
#define FULL_FAN_PERCENT 100
#define MIN_FAN_PERCENT 20

// Airspeed and Fan PWM models (y=mx+c)  - USE EXCEL TO DETERMINE
#define AIRSPEED_SLOPE  0.238
#define AIRSPEED_OFFS   2.7

#define FAN_PWM_SLOPE    0.283
#define FAN_PWM_OFFS    4.7

#define FREQ_LOW_CUT_OFF 5          // below this freq, consider air and fanspeed to be zero.

// Constraining Macro
#define CONSTRAIN(x,y,z) x<y ? x=y : x>z ? x=z : x=x;

const char ASCII_DELETE = 0x7F;
const char ASCII_BACKSPACE = 0x08;

// Digipot object with pin10 chip select
MCP4XXX digipot = MCP4XXX(10);

const char compile_date[] =  "Compiled: " __DATE__ " " __TIME__;

const int VQ1 = A0;           
const int VQ2 = A1;           

const int FAN_PWM_PIN = 3;    // must be pin 3 for TCCR02B timer
const int POT_SHUTDOWN_PIN = 4;
const int LEDR_PIN = 5;
const int LEDG_PIN = 6;
const int FAN_TACHO_PIN = 7;
const int AIRFLOW_FREQ_PIN = 8;
const int LEDB_PIN = 9;

char formattedVal[32];  
uint16_t zeroPot = 128, calPot = 128;


enum ledColours_t {RED, GREEN, BLUE};
/********************************     FUNCTIONS     ***********************************************/
void setLedPwm(ledColours_t colour, uint8_t pwm) {
  switch (colour) {
    case RED:
      analogWrite(LEDR_PIN, 255-pwm);
      break;
    case GREEN:
      analogWrite(LEDG_PIN, 255-pwm);
      break;
    case BLUE:
      analogWrite(LEDB_PIN, 255-pwm);
      break;
  }
}

void airSpeedToColour(int airSpeedFreqHz) {
  rgbLedsOff();
  if (airSpeedFreqHz > CAL_TARGET_HZ_FULL_FAN) {
    setLedPwm(RED, 4);
  } 
  else {
    CONSTRAIN(airSpeedFreqHz,1, MAX__HZ_FOR_SPEED_INDICATOR);
    switch (airSpeedFreqHz/(MAX__HZ_FOR_SPEED_INDICATOR/3)) {
      case 0:
          setLedPwm(GREEN, ((airSpeedFreqHz%100)*100)/MAX_PWM_FOR_SPEED_INDICATOR);
          break;
      case 1:
          setLedPwm(BLUE,  ((airSpeedFreqHz%100)*100)/MAX_PWM_FOR_SPEED_INDICATOR);
          setLedPwm(GREEN, ((airSpeedFreqHz%100)*100)/MAX_PWM_FOR_SPEED_INDICATOR);
          break;
      case 2:
          setLedPwm(BLUE, ((airSpeedFreqHz%100)*100)/MAX_PWM_FOR_SPEED_INDICATOR);
          break;  
      default:
          break;
    }
  }
}

void rgbLedsOff() {
  digitalWrite(LEDR_PIN, OFF_STATE);
  digitalWrite(LEDG_PIN, OFF_STATE);
  digitalWrite(LEDB_PIN, OFF_STATE);
}

int getVoltage(int adcChan) {
  int adc = analogRead(adcChan);
  float voltage = SUPPLY_VOLTAGE * ((float)(adc) / ADC_FULL_SCALE);
  return (int)(voltage*1000);   // mV
}

void printVoltage(int adcChan) {
  sprintf(formattedVal, "%4d", getVoltage(adcChan));
  Serial.print(formattedVal);  
  Serial.print("\t");
}

void printAirSpeed(int F) {
  float U = AIRSPEED_SLOPE * (float)F + AIRSPEED_OFFS;
  if (F < FREQ_LOW_CUT_OFF || F > CAL_FREQ_MAX) U = 0.0;
  sprintf(formattedVal, "%5.2f", U);
  Serial.print("\t");
  Serial.print(U);
  Serial.print("\t");
}

void printFanPwm(int F) {
  float P = FAN_PWM_SLOPE  * (float)F + FAN_PWM_OFFS;
  if (F < FREQ_LOW_CUT_OFF || F > CAL_FREQ_MAX) P = 0;
  sprintf(formattedVal, "%3d", P);
  Serial.print(P);
  Serial.print("\t");
}

void printBackspaces(uint8_t n) { 
  for (uint8_t i=0; i<n; i++) {
     Serial.write(ASCII_BACKSPACE);   
     if (TERATERM) Serial.write(ASCII_DELETE);
  }
}

void printIntToThisWidth(int val, uint8_t numChars) {
  // default format string
  char* format = "%05d"; 

  // limit number of chars to print
  if (numChars>0 && numChars<8) {
    *(format+2) = numChars + 48;
  }
  // print it
  sprintf(formattedVal, format, val); 
  Serial.print(formattedVal);
}

void printFloatToThisWidth(float val, uint8_t leadingChars, uint8_t decimalPlaces) {
  // default format string
  char* format = "%5.2f"; 

  // modify defualt format string
  *(format + 1) = leadingChars + decimalPlaces + 1 + 48;
  *(format + 3) = decimalPlaces + 48;

  // print it
  sprintf(formattedVal, format, val); 
  Serial.print(formattedVal);
}


void setFanDuty(uint8_t requestedFanDuty) {
  // Input requestedFanDuty is 0..100
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  float pwmConversion = 2.55 * (float)requestedFanDuty;
  analogWrite(FAN_PWM_PIN, (uint8_t)pwmConversion);
}

int measureFreq(bool showFreq) {
  // brute force count transitions in F pin over half a second
  int freq = 0;
  bool pinState = digitalRead(AIRFLOW_FREQ_PIN);
  long endTime=millis() + 500;

  while (millis() < (endTime)) {
    if ( digitalRead(AIRFLOW_FREQ_PIN) != pinState ) {
      pinState = !pinState;
      ++freq;
    }
  }
  // rising and falling edges in 0.5s = full cycles in 1s!

  if (showFreq) {
    Serial.print("Frq = ");
    Serial.println(freq);
  }

  return freq;
}

void autoZero() {
  uint16_t freq;
  //uint16_t midPointC = (ZERO_POT_MAX_CODE + ZERO_POT_MIN_CODE) / 2;
  uint16_t endPointA = ZERO_POT_MIN_CODE;
  uint16_t endPointB = ZERO_POT_MAX_CODE;
  int iterations = 0;
  Serial.println("\n\rstarting AutoZero bisection search...\n");

  // turn on pots 
  digitalWrite(POT_SHUTDOWN_PIN, HIGH);
  delay(100);
  freq = measureFreq(1);

  // set Zero pot to 3/4-point of allowable range,  and leave Cal pot at power-up mid-point (code 128)
  zeroPot = (3*endPointB + endPointA)/4;
  digipot.writeWiper(ZERO_POT, zeroPot);
  delay(SETTLING_TIME_MS);

  // bisection search on Zero pot for frequency in 1-10Hz range
 
  freq = measureFreq(1);
  airSpeedToColour(freq);
  while ((freq < ZERO_FREQ_MIN || freq > ZERO_FREQ_MAX) && iterations < MAX_BISECTIONS) {
    // if sign(f(c)) = sign(f(a)) then a ← c else b ← c // new interval
    if (freq < ZERO_FREQ_MIN) {
      // lower pot code (inc. R) since output locked high
      endPointB = zeroPot;
      zeroPot = (zeroPot + endPointA) / 2;
      Serial.print("lowering zero pot code to ");
      Serial.println(zeroPot);
    }
    else if (freq > ZERO_FREQ_MAX) {
      // increase pot code (dec. R) since freq too high
      endPointA = zeroPot;
      zeroPot = (zeroPot + endPointB) / 2;
      Serial.print("increasing zero pot code to ");
      Serial.println(zeroPot);
    }
    // set pot and allow time for thermal settling before freq measurement
    digipot.writeWiper(ZERO_POT, zeroPot);
    delay(SETTLING_TIME_MS/(iterations + 1));
    freq = measureFreq(1);
    iterations++;
  }
  Serial.println("\nAutoZero complete.\n");
}

void calibrate() {
  // set fan to 100% and adjust CAL pot till frequency is within target range
  // e.g. 300Hz +/-10Hz
  int iterations = 0;
  uint16_t freq;
  //uint16_t midPointC = (CAL_POT_MAX_CODE + CAL_POT_MIN_CODE) / 2;
  uint16_t endPointA = CAL_POT_MIN_CODE;
  uint16_t endPointB = CAL_POT_MAX_CODE;
  calPot = (3*endPointB + endPointA)/4;

  setFanDuty(100);
  delay(SETTLING_TIME_MS);
  freq = measureFreq(1);
  airSpeedToColour(freq);
  while ((freq < CAL_FREQ_MIN || freq > CAL_FREQ_MAX) && iterations < MAX_BISECTIONS) {
    // if sign(f(c)) = sign(f(a)) then a ← c else b ← c // new interval
    if (freq < CAL_FREQ_MIN) {
      // lower pot code (inc. R) since output locked high
      endPointB = calPot;
      calPot = (calPot + endPointA) / 2;
      Serial.print("lowering zero pot code to ");
      Serial.println(calPot);
    }
    else if (freq > CAL_FREQ_MAX) {
      // increase pot code (dec. R) since freq too high
      endPointA = calPot;
      calPot = (calPot + endPointB) / 2;
      Serial.print("increasing zero pot code to ");
      Serial.println(calPot);
    }
    // set pot and allow time for thermal settling before freq measurement
    digipot.writeWiper(CAL_POT, calPot);
    delay(SETTLING_TIME_MS/(iterations + 1));
    freq = measureFreq(1);
    iterations++;
  }
  Serial.println("\nCalibration complete.\n");


  setFanDuty(0);

}

void showPots() {
  Serial.println("");
  Serial.print("ZERO pot: ");
  Serial.print(zeroPot);
  Serial.print("  CAL  pot: ");
  Serial.println(calPot);
}

void setup() {
  Serial.begin(115200);

  // Define pin directions
  pinMode(VQ1, INPUT);
  pinMode(VQ2, INPUT);
  pinMode(FAN_TACHO_PIN, INPUT);
  pinMode(AIRFLOW_FREQ_PIN,INPUT);

  pinMode(FAN_PWM_PIN,OUTPUT);
  pinMode(POT_SHUTDOWN_PIN, OUTPUT);

  // Set output pin states
  digitalWrite(FAN_PWM_PIN, LOW);   // fan off
  digitalWrite(POT_SHUTDOWN_PIN, LOW); // pot off

  // for PWM frequency of 31372.55 Hz of Fan PWM Pin 3 
  TCCR2B = TCCR2B & B11111000 | B00000001; 
}

void loop() {
  int i, uartInt, freq;
  Serial.println(compile_date);

  PRINT_MENU:
  Serial.println();
  Serial.println("*****************************");
  Serial.println(" AirFlow Meter Command-O-Matic");
  Serial.println("*****************************");
  Serial.println("");
  Serial.println("1:Pots off 2: Set ZERO pot\t3:Set CAL pot");  
  Serial.println("4:Set Fan\t5:Fan off\t6: Show Pot Settings"); 
  Serial.println("7:Auto Zero\t8:Auto Zero & Cal"); 
  Serial.println("9:Show Menu\t10: Quit");
  Serial.println("     ");

  MENU:
  Serial.println("");
  Serial.println(" VQ1\tVQ2\tF(Hz)\tU(cm/s)\tFan(%)");
  do {
    uartInt = Serial.parseInt();  // returns 0 after short time out
    if (uartInt == 10) goto QUIT;
    if (uartInt == 1) goto POTS_OFF;
    if (uartInt == 2) goto SET_ZERO_POT;
    if (uartInt == 3 ) goto SET_CAL_POT;
    if (uartInt == 4 ) goto SET_FAN;
    if (uartInt == 5) goto FAN_OFF;
    if (uartInt == 6) goto SHOW_POTS;
    if (uartInt == 7) goto AUTO_ZERO;
    if (uartInt == 8) goto AUTO_ZERO_AND_CAL;
    if (uartInt == 9) goto PRINT_MENU;
    printBackspaces(50);
    // VQ1 and VQ2 should be set to equal each other when ZEROing with F=0 Hz output
    printVoltage(VQ1);
    printVoltage(VQ2);
    Serial.print(" ");
    freq = measureFreq(0);
    printIntToThisWidth(freq, 9);  // blocking, takes one second...
    printAirSpeed(freq);
    printFanPwm(freq);
    airSpeedToColour(freq);
  } while (1);

QUIT:
  Serial.println("***** STOPPED ******");
  rgbLedsOff();
  while(1);

POTS_OFF:
  digitalWrite(POT_SHUTDOWN_PIN, LOW);
  goto MENU;

SET_ZERO_POT:
  digitalWrite(POT_SHUTDOWN_PIN, HIGH);
  uartInt = 0;
  do {
    uartInt = Serial.parseInt(); 
    if (uartInt > 255) uartInt = 255;
    if (uartInt < 0) uartInt = 0;
  } while (uartInt < 1);
  zeroPot = uartInt;
  digipot.writeWiper(ZERO_POT, zeroPot);
  Serial.println(zeroPot);
  goto MENU;

SET_CAL_POT:
  digitalWrite(POT_SHUTDOWN_PIN, HIGH);
  uartInt = 0;
  do {
    uartInt = Serial.parseInt(); 
    if (uartInt > 255) uartInt = 255;
    if (uartInt < 0) uartInt = 0;
  } while (uartInt < 1);
  calPot = uartInt;
  digipot.writeWiper(CAL_POT, calPot);
  Serial.println(calPot);
  goto MENU;

SET_FAN:
  Serial.println("PWM: 2..100% or 1: Quit");
  //digitalWrite(FAN_PWM_PIN,HIGH);
  do {
    uartInt = Serial.parseInt();
    if (uartInt > 1) setFanDuty(uartInt);
    if (uartInt > 100) uartInt = 100;
    delay(200);
    printBackspaces(8);  
  } while (uartInt < 1);
  goto MENU;

FAN_OFF:
  digitalWrite(FAN_PWM_PIN, LOW);
  goto MENU;

SHOW_POTS:
  showPots();
  goto MENU;

AUTO_ZERO:
  autoZero();
  showPots();
  goto MENU;

AUTO_ZERO_AND_CAL:
  autoZero();
  showPots();
  calibrate();
  showPots();
  goto MENU;
}

