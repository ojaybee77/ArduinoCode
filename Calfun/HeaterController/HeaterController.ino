#include <math.h>
#include <AutoPID.h>
#include <stdlib.h>
/*
TCND5000 Reflective Optical Sensor - Heater Controller using Arduino Uno R3 (5V)

A heater to keep the TCND5000 at a relatively constant temperature during a 2 minute Calfun test.
This is to minimize sensor drift during test due mainly to the IR LED efficiency dropping
rapidly as temperature increase (approx 0.6%/C).

Target temp is about 38degC (since this should be slightly above maximum operating ambient
of 35degC). Since we cannot cool the sensor actively, we must set it to the top of our range.

The heater will require a warm up period of some tens of seconds prior to the zeroing and test.
But once warm, it can be kept warm with lower power than required in the initial warm-up ramp.
Connections:

SIGNAL    ARDUNIO   
------------------
HEATER    3     (8-bit PWM 409Hz to 3 x 47R 1206 resistors in parallel)
NTC       A0    From 10k 0402 NTC (A0 to GND) with fixed 4k7 from 5V to A0

Use a constant current in to the LED.
Measure photodiode current (directly or via Calfun Obs Sensor Board)
Target photodiode current is 1uA, and LED is about 10mA.

NOTE: For best serial monitor performance, it is recommended to use TeraTerm
which can be downloaded from:  https://github.com/TeraTermProject/teraterm/releases
*/

const char compile_date[] =  "\n\n\rCompiled: " __DATE__ " " __TIME__;

#define LOGGING_MODE    // define this if you want a CSV output to copy and paste.
                        // otherwise the output is over-printed on one line

#define CONSTRAIN(x,y,z) x<y ? x=y : x>z ? x=z : x=x;
#define TERATERM            1         // to define backspace behavious

// ADC and NTC settings
#define ADC_FULL_SCALE      1023
#define BETA                3380.0    // TDK NTCG103JF103FT1S
#define RNOM                10000.0   // Nominal resistance of the NTC at TNOM as lower resistor
#define R_UPPER             4700.0    // Fixed resistance in the upper arm of the NTC potential divider
#define TNOM                298.0
#define K_TO_C              273.15
#define NUM_AVGS            4
#define RINF                (RNOM * exp (-BETA/TNOM))

// Temperature Settings
#define MIN_TARGET_TEMP     28
#define MAX_TARGET_TEMP     45

#define HEATER_RESISTANCE   47          // resistance of each heater resistor (in parallel arrangement)
#define MAX_HEATER_POWER_MW 500         // maximum (over-rated!) power that can be applied...
#define MAX_POWER_DURATION_MS 10000     // ...and how long for
#define HEATER_VOLTAGE_MV   5000        // used to work out heater power at a given PWM duty cycle

// Simple algortihm settings
#define STAGE_A_POWER_MW  MAX_HEATER_POWER_MW
#define STAGE_B_POWER_MW  250
#define STAGE_C_POWER_MW  100
#define STAGE_A_DURATION_MS 10000
#define STAGE_B_DURATION_MS 5000

// PID settings and gains
#define PID_PWM_MIN         1
#define PID_PWM_MAX         255
#define KP                  20.0
#define KI                  0.33
#define KD                  1.0

#define TEST_DURATION_MS    300000

const int HEATER_PIN = 3;               // must be a PWM pin (3,5,6,9,10,11)
const int NTC_PIN = A0;

char formattedVal[40];  
double targetTemp = 38.0;
int uartInt = 0;
char uartChar;
const int HEATER_POWER_AT_FULL_DUTY_MW  = (int)((float)(pow(HEATER_VOLTAGE_MV/31.6228,2) / HEATER_RESISTANCE));
const char ASCII_DELETE = 0x7F;
const char ASCII_BACKSPACE = 0x08;
double temperature, setPoint, htrPwm = PID_PWM_MAX;
long startTime_ms,elapsedTime_ms;

enum mode_t {MANUAL, PID};

// input/output variables passed by reference, so they are updated automatically
// PID uses PWM 1..255 insterad of 1..100% duty
AutoPID pidController(&temperature, &setPoint, &htrPwm, PID_PWM_MIN, PID_PWM_MAX, KP, KI, KD);


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

void printTemperature() {
  double temp = getNtcTemperature();
  Serial.print(temp);
  Serial.print(" degC");
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


void setup() {
  Serial.begin(115200);
  
  // PID set-up
  pidController.stop();

  // check heater power settings
  if (MAX_HEATER_POWER_MW > HEATER_POWER_AT_FULL_DUTY_MW) {
    Serial.println("\n\rERROR:  MAX_HEATER_POWER_MW greater than available power!");
    while(1);
  }

  // Define pin directions
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(NTC_PIN, INPUT);
 
  // Set output pin states
  digitalWrite(HEATER_PIN, LOW);        // Heater off
}


void loop() {
  int i, uartInt, freq;
  Serial.println(compile_date);

  PRINT_MENU:
  Serial.println();
  Serial.println("Calfun TCND5000 Sensor Heater Command-O-Matic");
  Serial.println("*********************************************");
  Serial.println("");
  Serial.println("1: Set Target\t2: Get Target");  
  Serial.println("3: Run Heater\t4:Temp Monitor");  
  Serial.println("9:Show Menu\t10: Quit");
  Serial.println("     ");

  MENU:
  Serial.println("");
  do {
    uartInt = Serial.parseInt();  // returns 0 after short time out
    if (uartInt == 10) goto QUIT;
    if (uartInt == 1) goto SET_TARGET;
    if (uartInt == 2) goto GET_TARGET;
    if (uartInt == 3 ) goto RUN_HEATER;
    if (uartInt == 4 ) goto TEMP_MONITOR;
    if (uartInt == 9) goto PRINT_MENU;
    printBackspaces(10);
    printTemperature();
  } while (1);

QUIT:
  Serial.println("***** STOPPED ******");
  heaterOff();
  while(1);

SET_TARGET:
  do {
    uartInt = Serial.parseInt(); 
    CONSTRAIN(uartInt,MIN_TARGET_TEMP, MAX_TARGET_TEMP);
  } while (uartInt < 1);
  targetTemp = uartInt;

GET_TARGET:
  Serial.print("\nTemp. target is ");
  Serial.print(targetTemp);
  Serial.print(" degC");
  goto MENU;


RUN_HEATER:
  bool hitTarget = false;
  mode_t mode = PID;
  long testDurationMs = TEST_DURATION_MS;
  startTime_ms = millis();
  setPoint = targetTemp;
  uartInt = 0;
  Serial.println("\nPress 'p' to run PID or 'm' for Manual control\n");

  do {
    uartChar = Serial.read();
  }
  while (uartChar != 'p' && uartChar != 'm');

  if (uartChar == 'm') {
    mode = MANUAL;
    htrPwm = 255;
    testDurationMs = 9999999;
    Serial.println("Manual mode, running at full bhoona...");
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
      //Serial.println(formattedVal);
    }
    if (mode == PID) {
      pidController.run();  
      setHeaterPwm((uint8_t)htrPwm);
    }
    else { // MANUAL
      pidController.stop();
      setHeaterPwm((uint8_t)htrPwm);
    }

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
}

 