#include <math.h>             // for natural log and exponent in NTC calcs
#include <AutoPID.h>          // for PID control of heater
#include <stdlib.h>           // for string printing
#include <MCP48xx.h>          // for DAC SPI control
/*
TCND5000 Reflective Optical Sensor on 401706-0B Calfun Obs Sensor Board
         - Heater Controller using Arduino Uno R3 (5V)

This code allows control of the heater on 401706-OB (or the ObsSensorWithHeater board), as well as 
control of the MCP4811 DAC current source.

The heater is to keep the TCND5000 at a relatively constant temperature during a 2 minute Calfun test.
This is to minimize sensor drift during test due mainly to the IR LED efficiency dropping
rapidly as temperature increase (approx 0.6%/C).

Target temp is about 38degC (since this should be slightly above maximum operating ambient
of 35degC). Since we cannot cool the sensor actively, we must set it to the top of our range.

The heater will require a warm up period of some tens of seconds prior to the zeroing and test.
But once warm, it can be kept warm with lower power than required in the initial warm-up ramp.
Connections:

SIGNAL        ARDUNIO   
------------------
HEATER        3     (8-bit PWM 409Hz to 3 x 47R 1206 resistors in parallel)
LED_SHUTDOWN  4     HIGH --> turn off IR LED current (0mA), LOW --> enable DAC current source
NTC           A1    From 10k 0402 NTC (A0 to GND) with fixed 4k7 from 5V to A0
OBS_ADC       A0    From 401706-OB "ADC_OUT" 0..3V
SI            11    SPI MISO for MCP4811 DAC
SCK           13    SPI CLK for  MCP4811 DAC
CS            10    SPI ChipSel  MCP4811 DAC
GND           GND
5V            5V      (*** ONLY if using Arduino stand-alone without XTR2 snesor board ***)
3V            3.3V    (*** ONLY if using Arduino stand-alone without XTR2 sensor board ***) 

The current source min and max currents are set by R1/R3/R22. Use the spreadsheet
"DAC_Current_Source_Calcs.xlsx" that is included in the 401706-OB Altium job, or stored
I:\TECH\Projects\CalFun\_GATE 1 Build\PCBs\401706\Rev.0B

The code will attempt to auto-zero the obscuration sensor to a nominal "zero voltage" e.g. 
in the range 0.0 to 0.3V ish (settable), by using a binomial search of the current source
whilst measuring the ADC_OUT.


NOTE: For best serial monitor performance, it is recommended to use TeraTerm
which can be downloaded from:  https://github.com/TeraTermProject/teraterm/releases
*/

const char compile_date[] =  "\n\n\rCompiled: " __DATE__ " " __TIME__;

// System Configuration defines
#define STAND_ALONG_MODE  // define this if using Arduino Uno stand-alone with no XTR2 sensor board (5V/3V supply)
#define LOGGING_MODE    // define this if you want a CSV output to copy and paste.
                        // otherwise the output is over-printed on one line
#define TERATERM            1         // to define backspace behaviours



// Macro defines
#define CONSTRAIN(x,y,z) x<y ? x=y : x>z ? x=z : x=x;
#define LED(x) digitalWrite(LED_SHDN_PIN, x);         // macro to turn IR LED on or off
#define ON 1
#define OFF 0

// DAC and Compensation Settings
#define DAC_RESOLUTION      10         // for MCP4811 default part on 401706-0B. Could use 8 or 12.
#define LED_COEFFICIENT_PCT_PER_DEG_C -0.5    // generic value that hopefully works for all samples of TCND5000 for residual temp comp

// Autozero Parameters
#define PCT_PER_FOOT_OFFSET 1.0        
#define TYPICAL_STARTING_DAC 745       


// ADC and NTC settings
#define ADC_FULL_SCALE      1023
#define ADC_FULLSCALE_VOLTS 5.0
#define OBS_FULL_RANGE_PCT_PER_FT  8.7
#define OBS_FULL_RANGE_ADC    668   // 3.3V out of 5.0V ---> (3.3/5.0) * 1023 = 675, but we are missing the top few codes
#define BETA                3380.0    // TDK NTCG103JF103FT1S
#define RNOM                10000.0   // Nominal resistance of the NTC at TNOM as lower resistor
#define R_UPPER             4700.0    // Fixed resistance in the upper arm of the NTC potential divider
#define TNOM                298.0
#define K_TO_C              273.15
#define NUM_AVGS            4
#define RINF                (RNOM * exp (-BETA/TNOM))

// Heater/Temperature Settings
#define MIN_TARGET_TEMP     28
#define MAX_TARGET_TEMP     45
#define MAX_HEATER_PWM      255


// PID settings and gains
#define PID_PWM_MIN         1
#define PID_PWM_MAX         MAX_HEATER_PWM
#define KP                  30.0        // 20.0 on small Heater-only PCB
#define KI                  0.5         // 0.33 on small Heater-only PCB3
#define KD                  1.0

#define TEST_DURATION_MS    10000000      // 120000 --> 2mins; 300000 --> 5 mins

/**************************************************************************************************** 
************************                GLOBALS            ******************************************
*****************************************************************************************************/
const int HEATER_PIN = 3;               // must be a PWM pin (3,5,6,9,10,11)  - 490Hz on Pin 3
const int NTC_PIN = A1;
const int ADC_OUT = A0;                 // obscuration sensor output 0..3V if using sensor board, 0..3.3V if Arduino stand-alone
const int DAC_CS_PIN  = 10;
const int LED_SHDN_PIN  = 4;

char formattedVal[40];  
double targetTemp = 38.0;
int uartInt = 0;
char uartChar;
const char ASCII_DELETE = 0x7F;
const char ASCII_BACKSPACE = 0x08;
double temperature, setPoint, htrPwm = PID_PWM_MAX;
long startTime_ms,elapsedTime_ms;
bool ledState = OFF;                    // IR LED state is inverted, OFF --> means LED is oN
bool runHeater = OFF;
bool isTestRunning = false;
int dacCode = 512;

enum mode_t {MANUAL, PID};

// input/output variables passed by reference, so they are updated automatically
// PID uses PWM 1..255 insterad of 1..100% duty
AutoPID pidController(&temperature, &setPoint, &htrPwm, PID_PWM_MIN, PID_PWM_MAX, KP, KI, KD);

// DAC control object
MCP4811 dac(DAC_CS_PIN);

/**************************************************************************************************** 
************************                FUNCTIONS          ******************************************
*****************************************************************************************************/
int autoZero() {
  return TYPICAL_STARTING_DAC;
}


int getAdcReading(int adcPin, int numSamples, int delayMs) {
  // Returns the average (median) of numSamples readings
  // with delayMs between each reading
  CONSTRAIN(numSamples,1,16);
  int adc[16]= {0};

  for (int i=0; i<numSamples; i++)  {
    adc[i] += analogRead(adcPin);
    delay(delayMs);
  }
  
  return median(adc, numSamples);
}

int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}

int median(int* samples, int n) {
  // sort samples in order
  qsort(samples, n, sizeof(int), compare);
  // and pick middle(ish) one
  if (n < 3) return samples[0];
  else return samples[n/2];
}



double getNtcTemperature()  {
  double div_ratio, ntc_res;
  int adc = getAdcReading(NTC_PIN, 6, 0);
  div_ratio = (float)adc / ADC_FULL_SCALE;
  ntc_res = div_ratio * R_UPPER / (1.0 - div_ratio);
  return BETA / log(ntc_res / RINF) - K_TO_C;
}

int getSensorAdc()  {
  // turn off heater whilst taking measurement
  heaterOff();
  double div_ratio, ntc_res;
  int adc = getAdcReading(ADC_OUT, 6, 5);
  if (runHeater == ON) setHeaterPwm(htrPwm);
  return adc;
}

void printTemperature() {
  double temp = getNtcTemperature();
  Serial.print(temp);
  Serial.print(" degC");
}

void printTemperature(double temp) {
  Serial.print(temp);
  Serial.print(" degC");
}

uint16_t printSensor() {
  int obsAdc = getSensorAdc();
  Serial.print("\t");
  Serial.print(obsAdc);
  Serial.print(" ADC");
  return obsAdc;
}

float printPercentPerFoot(uint16_t obsAdc, uint16_t offset)  {
  if (obsAdc < offset) obsAdc = offset;
  float pctPerFoot = OBS_FULL_RANGE_PCT_PER_FT * (obsAdc - offset) / OBS_FULL_RANGE_ADC;
  Serial.print("\t");
  Serial.print(pctPerFoot);
  Serial.print("%/ft");
  return pctPerFoot;
}

void printPwm() {
  Serial.print("\t\t");
  if (runHeater == OFF) htrPwm = 0;
  Serial.print((uint8_t)htrPwm);
  Serial.print(" PWM");
}

void printBackspaces(uint8_t n) { 
  for (uint8_t i=0; i<n; i++) {
     Serial.write(ASCII_BACKSPACE);   
     if (TERATERM) Serial.write(ASCII_DELETE);
  }
}

void heaterOff() {
  digitalWrite(HEATER_PIN, LOW);
}

void setHeaterPwm (uint8_t pwm)  {
  analogWrite(HEATER_PIN, pwm);
}

void setHeaterPower(int power_mW) {
  //  uint8_t pwm = 
  // TBD

}

/**************************************************************************************************** 
************************           INITIALIZATION          ******************************************
*****************************************************************************************************/
void setup() {
  Serial.begin(115200);

  // DAC initialization (CS pin state, GAIN x 1 --> output range 0..2.048V)
  dac.init();
  dac.setGainA(MCP4811::Low);
  dac.setCodeA(512);
  dac.turnOnChannelA();
  dac.updateDAC();
  
  // PID set-up
  pidController.stop();

  // Define pin directions
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(NTC_PIN, INPUT);
  pinMode(LED_SHDN_PIN, OUTPUT);
 
  // Set output pin states
  digitalWrite(HEATER_PIN, LOW);        // Heater off
  LED(ledState);                        // IR LED off (0mA)
}


/**************************************************************************************************** 
************************                MAIN LOOP          ******************************************
*****************************************************************************************************/
void loop() {
  int i, uartInt, freq;
  uint16_t obsAdc;
  Serial.println(compile_date);

  PRINT_MENU:
  Serial.println();
  Serial.println("Calfun 401706-OB Heater and Current Source Command-O-Matic");
  Serial.println("**********************************************************");
  Serial.println("");
  Serial.println("1: Get Heater Target\t2: Set Heater Target");  
  Serial.println("3: Toggle Heater");  
  Serial.println("4: Get DAC\t\t5:Set DAC");
  Serial.println("6: Auto-Zero Sensor\t7: Run Test");
  Serial.println("8: Toggle IR LED");
  Serial.println("9: Show Menu\t\t10: Quit");
  Serial.println("     ");

  MENU:
  Serial.println("");
  do {
    uartInt = Serial.parseInt();  // returns 0 after one second
    if (uartInt == 10) goto QUIT;
    if (uartInt == 1) goto GET_TARGET;
    if (uartInt == 2) goto SET_TARGET;
    if (uartInt == 3 ) goto TOGGLE_HEATER;
    if (uartInt == 4) goto GET_DAC;
    if (uartInt == 5) goto SET_DAC;
    if (uartInt == 6) goto AUTO_ZERO;
    if (uartInt == 7) goto RUN_TEST;
    if (uartInt == 8) goto TOGGLE_LED;
    if (uartInt == 9) goto PRINT_MENU;
    temperature = getNtcTemperature();
    printBackspaces(60);
    printTemperature(temperature);
    obsAdc = printSensor();
    printPercentPerFoot(obsAdc, 0);
    printPwm();
    if (runHeater == ON) pidController.run();
    if(isTestRunning) {
      Serial.print(".");
    }
    setHeaterPwm((uint8_t)htrPwm);
    
  } while (1);

QUIT:
  Serial.println("\n\r***** STOPPED ******");
  heaterOff();
  while(1);

SET_TARGET:
  do {
    uartInt = Serial.parseInt(); 
    CONSTRAIN(uartInt,MIN_TARGET_TEMP, MAX_TARGET_TEMP);
  } while (uartInt < 1);
  targetTemp = uartInt;

GET_TARGET:
  Serial.print("\n\rTemp. target is ");
  Serial.print(targetTemp);
  Serial.print(" degC");
  goto MENU;


TOGGLE_HEATER:
  runHeater = !runHeater;
  if (runHeater == ON)  {
    Serial.println("\n\rHEATER is ON");
    setPoint = targetTemp;
    pidController.run();
  }
  else {
    Serial.println("\n\rHEATER is OFF");
    pidController.stop();
    heaterOff();
  }
  goto MENU;


SET_DAC:
  Serial.println("\n\rEnter DAC code (1..1023) for current source\n\n");
  uartInt = Serial.parseInt(); 
  CONSTRAIN(uartInt, 1, (1u << DAC_RESOLUTION) - 1);
  dac.setCodeA(uartInt);
  dac.updateDAC();
  Serial.println(uartInt);
  printBackspaces(8);
  dacCode = uartInt;
  goto MENU;

GET_DAC:
  Serial.println("\n");
  Serial.println(dacCode);
  Serial.println("");
  goto MENU;

AUTO_ZERO:
  dacCode = autoZero();
  if (dacCode < 0)  {
    Serial.println("******************\nAuto Zero Failed!\n******************");
  }
  else {
    Serial.print("\n\rAuto-Zero completed with DAC code ");
    Serial.println(dacCode);
  }
  goto MENU;

RUN_TEST:
  // Auto-zero the sensor at test start and rememeber the initial DAC setting
  int startingDac = autoZero();
  dac.setCodeA(startingDac);
  dac.updateDAC();
  if (isTestRunning == false) {
    Serial.println("\n\rStarting test - press 7 again to abort\n\n");
    isTestRunning = true;
  }
  else {
    Serial.println("\n\rStopping test\n\n");
    isTestRunning = false;
  }  
  goto MENU;

TOGGLE_LED:
  ledState = !ledState;
  LED(ledState);
  if (ledState == OFF) Serial.println("\n\rIR LED is ON");
  else Serial.println("\n\rIR LED is OFF");
  goto MENU;
}

/*

bool hitTarget = false;
  mode_t mode = PID;
  long testDurationMs = TEST_DURATION_MS;
  startTime_ms = millis();
  setPoint = targetTemp;
  uartInt = 0;
  htrPwm = 1;
  Serial.println("\nPress 'p' to run PID or 'm' for Manual control\n");

  do {
    uartChar = Serial.read();
  }
  while (uartChar != 'p' && uartChar != 'm');

  if (uartChar == 'm') {
    mode = MANUAL;
    htrPwm = MAX_HEATER_PWM;
    testDurationMs = 9999999;
    Serial.println("Manual Mode");
  }

  Serial.println("\nPress Any Key To Stop...\n");
  elapsedTime_ms = 0;

#ifdef LOGGING_MODE
   Serial.println("\nTime(s),PWM(byte),Temp (degC)\n");
#else
  sprintf(formattedVal, "Target Temp x10: +%d degC", (int)(targetTemp*10));
  Serial.println(formattedVal);
#endif

  while(elapsedTime_ms < testDurationMs && uartInt < 32) {
    temperature = getNtcTemperature();
    if (temperature > targetTemp && !hitTarget) {
      hitTarget = true;
      sprintf(formattedVal, "\n\rTest reached target after %04u seconds\n", elapsedTime_ms/1000);
      Serial.println(formattedVal);
    } 
    if (mode == PID) {
      pidController.run();
    }   
    else { // MANUAL
      pidController.stop();
    }
    setHeaterPwm((uint8_t)htrPwm);

#ifdef LOGGING_MODE
    Serial.print((float)(elapsedTime_ms)/1000.0);
    Serial.print(",\t");
    Serial.print((uint8_t)htrPwm);
    Serial.print(",\t");
    Serial.println(temperature);
#else
    sprintf(formattedVal, "Time: %04u", elapsedTime_ms/1000);
    Serial.print(formattedVal);
    sprintf(formattedVal, " PWM: %03u", (uint8_t)htrPwm);
    Serial.print(formattedVal);
    sprintf(formattedVal, " Temp x10: %03d degC", (int)(temperature * 10));
    Serial.print(formattedVal);
    printBackspaces(40);
#endif

    delay(1000);
    elapsedTime_ms = millis()-startTime_ms;
    uartInt = Serial.read();
  }
  heaterOff();
  pidController.stop();
  pidController.reset();
  Serial.println("\n\rHeater off\n");
  goto MENU;


  TEMP_MONITOR:
  elapsedTime_ms = 0;
  startTime_ms = millis();
  Serial.println("\nPress Any Key To Stop...\n\n");
#ifdef LOGGING_MODE
   Serial.println("\nTime (s),Temp (degC)\n");
#endif
 
  uartInt = 0;
  while(uartInt < 32) {
    temperature = getNtcTemperature();
    uartInt = Serial.read();
    delay(2000);

#ifdef LOGGING_MODE
    Serial.print((float)(elapsedTime_ms)/1000.0);
    Serial.print(",\t");
    Serial.println(temperature);
#else
    sprintf(formattedVal, "Temp x10: %03d degC", (int)(temperature * 10));
    Serial.print(formattedVal);
    delay(1000);
    printBackspaces(20);
#endif

  elapsedTime_ms = millis()-startTime_ms;
  }
  goto MENU;
*/

 