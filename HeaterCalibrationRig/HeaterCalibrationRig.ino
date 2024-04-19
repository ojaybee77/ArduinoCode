/*
* HEATER CALIBRATION TEST RIG
* Using HongFei HB-5015HB12B Fan PWM Control and Heater Assembly in Cup with thermocouple

*  TBD Description of Set-Up and version history

PID tuned using this method: https://pidexplained.com/how-to-tune-a-pid-controller/
*/

/* *************************     INCLUDES    *******************************************/
#include <math.h>
#include <AutoPID.h>
#include <Fuzzy.h>
#include <QuickStats.h>
#include <TimerOne.h>
//#include "FuzzyRules.h"

const char compile_date[] =  "Compiled: " __DATE__ " " __TIME__;
/* *************************     DEFINES     *******************************************/
// Fan tacho / PWM settings
#define MONOSTABLE_PULSE_MS     3.63E-3   // milliseconds pulse length of Freq-to-Volt monostable
#define V_555_OUTPUT_HIGH       4.2
#define LTC6992_MAX_DUTY_VOLTS  1.0

// General test rig settings
#define RED_LED(s)          digitalWrite(RED_LED_PIN, s)
#define ORANGE_LED(s)       digitalWrite(ORG_LED_PIN, s)
#define RAIL_5V0            4.96
#define RAIL_12V0           11.96
#define OFFICE_TEMP         22.0
#define ON                  1
#define OFF                 0
#define FAN_PWM_IN_TEST     30

// ADC and NTC settings
#define ADC_FULL_SCALE      1023
#define BETA                3892.0    // Littelfuse 104JG1J
#define RNOM                100000.0  // Nominal resistance of the NTC at TNOM as lower resistor
#define R_UPPER             10000.0   // Fixed resistance in the upper arm of the NTC potential divider
#define TNOM                298.0
#define K_TO_C              273.15
#define NUM_AVGS            4
#define RINF                (RNOM * exp (-BETA/TNOM))

// Heater parameters
#define MAX_HEATER_DUTY     40        // Max current ~= 12/1Ohm = 12A at 100% duty!!! (144W)
#define MIN_HEATER_DUTY     1
#define START_DUTY          20
#define MAX_NTC_TEMP        200       // Turn off Heater if this is exceeded
#define NORMAL              0
#define HIGH                1
#define LOOP_UPDATE_TIME_MS 500 // In XTR2, 8 loops @ 250ms per loop = 2s update time

// Fan parameters
#define MAX_FAN_RPM         7000
#define FAN_PCT_TOLERANCE   10
#define FREQ_TO_VOLTS_FUDGE_FACTOR  0.934   // to scale 100% tacho to 7000RPM

// Existing heater algorithm settings
#define MAX_TEST_TIME_MS    60000
// TBD

// PID settings and gains
#define PID_PWM_MIN         1
#define PID_PWM_MAX        (AS_PWM(MAX_HEATER_DUTY))
#define KP                  3
#define KI                  0.25
#define KD                  0.0

// fuzzy Logic settings (runs in PWM 1..255 not Duty 1-100%)
#define HISTORY_BUFFER_SIZE 8
#define SAMPLE_PERIOD_MS 500
#define MAX_NEG_PWM_CHANGE 5
#define MAX_POS_PWM_CHANGE 4
#define AS_PWM(d) (uint8_t)((float)d*2.55)
#define AS_DUTY(p) (uint8_t)((float)p/2.55)



/* *************************     PINS         *******************************************/
const int TACHO_PIN = A0;         // from Pan tacho (via freq-to-volt conversion)
const int NTC_PIN = A1;           // from NTC with fixed resistor potential divider
const int VBATT_PIN = A2;         // 12V nominal rail representing the Li-Ion batt. pack
const int THERMOCOUPLE_PIN = A3;  // from thermocouple breakout board
const int HTR_PWM_PIN = 10;       // (TimerOne) PWM 50Hz to NFET to drive HTR from 12V
const int FAN_PWM_PIN = 3;        // to Fan PWM 
const int RED_LED_PIN = 8;        // for indicating heater activity (active high)
const int ORG_LED_PIN = 9;        // for indicating control loop activity (active high)


/* *************************     GLOBAL VARS  *******************************************/
const char ASCII_BACKSPACE = 0x08;
int heatSetPoint = 120;
int hiHeatSetPoint = 115;
bool greenLedState = OFF;
bool redLedState = OFF;
double temperature, setPoint, htrPwm = AS_PWM(START_DUTY);

// input/output variables passed by reference, so they are updated automatically
// PID uses PWM 1..255 insterad of 1..100% duty
AutoPID pidController(&temperature, &setPoint, &htrPwm, PID_PWM_MIN, PID_PWM_MAX, KP, KI, KD);

// instantiating a Fuzzy object
Fuzzy *fuzzy = new Fuzzy();

// FuzzyInput Membership Functions
FuzzySet *tooCool = new FuzzySet(heatSetPoint-16, heatSetPoint-16, heatSetPoint-10, heatSetPoint-4);
//FuzzySet *desired = new FuzzySet(heatSetPoint-10, heatSetPoint-5, heatSetPoint+4, heatSetPoint+8);
FuzzySet *tooWarm = new FuzzySet(heatSetPoint+2, heatSetPoint+4, heatSetPoint+8, heatSetPoint+8);

// FuzzyOutput
FuzzySet *reduceHeat = new FuzzySet(-MAX_NEG_PWM_CHANGE, -MAX_NEG_PWM_CHANGE, -4, -2);
//FuzzySet *noChange = new FuzzySet(-2, -1, 1, 2);
FuzzySet *increaseHeat = new FuzzySet(0, 3, MAX_POS_PWM_CHANGE, MAX_POS_PWM_CHANGE);


/* *************************     FUNCTIONS    *******************************************/
double getNtcTemp() {
  double div_ratio, ntc_res;
  int adc = getAdcReading(NTC_PIN, 4, 0);
  div_ratio = (double)adc / ADC_FULL_SCALE;
  ntc_res = div_ratio * R_UPPER / (1.0 - div_ratio);
  return BETA / log(ntc_res / RINF) - K_TO_C;
}

double getThermocoupleTemp()  {
  double temp=25;
  int adc = getAdcReading(THERMOCOUPLE_PIN, 4, 0);
  //TBD conversion
  return temp;
}

double getVbatt()  {
  int adc = getAdcReading(VBATT_PIN, 4, 0);
  double temp = (double)(adc) * (56+27) * RAIL_5V0/(ADC_FULL_SCALE * 27); 
  return temp;
}

int getAdcReading(int adcPin, int numSamples, int delayMs) {
  // Returns the average of numSamples readings
  // with delayMs between each reading
  int adc = 0;
  for (int i=0; i<numSamples; i++)  {
    adc += analogRead(adcPin);
    delay(delayMs);
  }
  adc /= numSamples;

  return adc;
}

void setFanDuty(uint8_t requestedFanDuty) {
  // Input requestedFanDuty is 0..100
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  float pwmConversion = 2.55 * (float)requestedFanDuty;
  analogWrite(FAN_PWM_PIN, (uint8_t)pwmConversion);
  
  delay(1500);   // allow time for fan to get up to speed
}


int getFanRpm() {
  // FREQ_TACHO ~= V_TACHO / (MONOSTABLE_PULSE_LENGTH * V_555_OUTPUT_HIGH)
  int tachoFreqToVolts = getAdcReading(TACHO_PIN, 4, 10);
  float vTacho = RAIL_5V0 * ((float)tachoFreqToVolts/ ADC_FULL_SCALE);
  float rpm = FREQ_TO_VOLTS_FUDGE_FACTOR * 30.0 * vTacho / (MONOSTABLE_PULSE_MS * V_555_OUTPUT_HIGH);

  return (int)rpm;    
}


int getFanPercent() {
  int fanPercent = 100 * getFanRpm() / 7000;
  if (fanPercent > 100) fanPercent = 100;

  return fanPercent;
}


void setHeaterDuty(uint8_t requestedHtrDuty) {
  // limit PWM - write 0 to turn off heater
  if (requestedHtrDuty > MAX_HEATER_DUTY) {
    requestedHtrDuty = MAX_HEATER_DUTY;
  }
  float asPwm = (float)requestedHtrDuty * 255.0 / 100.0;

  //analogWrite(HTR_PWM_PIN, (uint8_t)asPwm);
  Timer1.pwm(HTR_PWM_PIN, asPwm * 4);

   // turn on red LED if PWM is > 0
  requestedHtrDuty? RED_LED(ON) : RED_LED(OFF);;
}


void setHeaterPwm(uint8_t requestedHtrPwm) {
  // limit PWM - write 0 to turn off heater
  if (requestedHtrPwm > (uint8_t)(MAX_HEATER_DUTY * 2.55)) {
    requestedHtrPwm = (uint8_t)(MAX_HEATER_DUTY *2.55);
  }

  //analogWrite(HTR_PWM_PIN, requestedHtrPwm);
  Timer1.pwm(HTR_PWM_PIN, requestedHtrPwm * 4);

   // turn on red LED if PWM is > 0
  requestedHtrPwm ? RED_LED(ON) : RED_LED(OFF);
}


/* *************************     INITIALISATION  *******************************************/
void setup() {
  pinMode(FAN_PWM_PIN,0);
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz

  Timer1.initialize(20000);  // 20000 us --> 50Hz Heater PWM

  pinMode(NTC_PIN, INPUT); 
  pinMode(TACHO_PIN, INPUT);
  pinMode(THERMOCOUPLE_PIN, INPUT);
  pinMode(HTR_PWM_PIN, OUTPUT);
  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(ORG_LED_PIN, OUTPUT);
  digitalWrite(HTR_PWM_PIN, LOW);       // Make sure heater is OFF!
  digitalWrite(FAN_PWM_PIN, LOW); 
  digitalWrite(RED_LED_PIN, HIGH); 
  digitalWrite(ORG_LED_PIN, HIGH); 

  Serial.begin(115200);
  Serial.println(compile_date);

 
  // PID set-up
  pidController.stop();
  //pidController.setBangBang(100);       // try not to use Bang-Bang control?!
  pidController.setTimeStep(LOOP_UPDATE_TIME_MS);


  // Fuzzy Logic set-up 
  // FuzzyInput Variable  - must be in setup() 
  FuzzyInput *f_temperature = new FuzzyInput(1);
  f_temperature->addFuzzySet(tooCool);
  //f_temperature->addFuzzySet(desired);
  f_temperature->addFuzzySet(tooWarm);
  fuzzy->addFuzzyInput(f_temperature);

  // FuzzyOutput Variable
  FuzzyOutput *heater = new FuzzyOutput(1);
  heater->addFuzzySet(reduceHeat);
  //heater->addFuzzySet(noChange);
  heater->addFuzzySet(increaseHeat);
  fuzzy->addFuzzyOutput(heater);


  // Building FuzzyRule "IF temp = tooCool THEN heater = increase"
  FuzzyRuleAntecedent *ifTempTooCool = new FuzzyRuleAntecedent();
  ifTempTooCool->joinSingle(tooCool);
  FuzzyRuleConsequent *thenHeaterIncrease = new FuzzyRuleConsequent();
  thenHeaterIncrease->addOutput(increaseHeat);
  FuzzyRule *fuzzyRule01 = new FuzzyRule(1, ifTempTooCool, thenHeaterIncrease);
  fuzzy->addFuzzyRule(fuzzyRule01);

  // Building FuzzyRule "IF temp = desired THEN heater = noChange"
  /*
  FuzzyRuleAntecedent *ifTempOk = new FuzzyRuleAntecedent();
  ifTempOk->joinSingle(desired);
  FuzzyRuleConsequent *thenHeaterNoChange = new FuzzyRuleConsequent();
  thenHeaterNoChange->addOutput(noChange);
  FuzzyRule *fuzzyRule02 = new FuzzyRule(1, ifTempOk, thenHeaterNoChange);
  fuzzy->addFuzzyRule(fuzzyRule02);
  */

  // Building FuzzyRule "IF temp = too warm THEN heater = reduce"
  FuzzyRuleAntecedent *ifTempTooWarm = new FuzzyRuleAntecedent();
  ifTempTooWarm->joinSingle(tooWarm);
  FuzzyRuleConsequent *thenHeaterReduce = new FuzzyRuleConsequent();
  thenHeaterReduce->addOutput(reduceHeat);
  FuzzyRule *fuzzyRule02 = new FuzzyRule(1, ifTempTooWarm, thenHeaterReduce);
  fuzzy->addFuzzyRule(fuzzyRule02);
}


/* *************************     MAIN LOOP       *******************************************/
void loop() {
  int  uartInt;
  char formattedVal[5];          // %03d -->  'xxx' degC with leading zero padding

  // give cooling fan blast
  coolingFanBlast(1.5);
  RED_LED(OFF);
  ORANGE_LED(OFF);

  Serial.print("VBATT: ");
  Serial.print(getVbatt());
  Serial.println("V");

MENU:
  Serial.println("");
  Serial.println("1: Fan Check\t2: NTC Check\t3: Thermocouple Check");   
  Serial.println("4: Heater Check\t5: Heat Test\t6: Hi-Heater Test");
  Serial.println("7: Heater Cal.\t8: Heat set\t9: Hi-Heat set\t10: Quit");
  Serial.println("");
  do {
    uartInt = Serial.parseInt();  // returns 0 after short time out
    if (uartInt == 10) goto QUIT;
    if (uartInt == 1 ) goto FAN_CHECK;
    if (uartInt == 2) goto NTC_CHECK;
    if (uartInt == 3) goto THERMOCOUPLE_CHECK;
    if (uartInt == 4) goto HEATER_CHECK;
    if (uartInt == 5) goto HEATER_TEST;
    if (uartInt == 6) goto HI_HEATER_TEST;
    if (uartInt == 7) goto HEATER_CALIBRATION;
    if (uartInt == 8) goto HEAT_SETPOINT;
    if (uartInt == 9) goto HI_HEAT_SETPOINT;

  } while (uartInt < 1);

QUIT:
  Serial.println("***** STOPPED ******");
  while(1);

FAN_CHECK:
  Serial.println("PWM: 2..100% or 1: Quit");
  //digitalWrite(FAN_PWM_PIN,HIGH);
  do {
    uartInt = Serial.parseInt();
    if (uartInt > 100) uartInt = 100;
    if (uartInt > 1) setFanDuty(uartInt);
    sprintf(formattedVal, "%04d", (int)getFanRpm()); 
    Serial.print(formattedVal);
    Serial.print(" ");
    sprintf(formattedVal, "%03d", (int)getFanPercent()); 
    Serial.print(formattedVal);
    delay(200);
    printBackspaces(8);
   
  } while (uartInt != 1);
  setFanDuty(0);
  //digitalWrite(FAN_PWM_PIN,LOW);
  goto MENU;

NTC_CHECK:
 Serial.println("1: Quit");
  do {
    sprintf(formattedVal, "%03d", (int)getNtcTemp()); 
    Serial.print(formattedVal);
    delay(200);
    printBackspaces(3);  
    uartInt = Serial.parseInt();
  } while (uartInt != 1);
  goto MENU;

THERMOCOUPLE_CHECK:
 Serial.println("1: Quit");
  do {
    sprintf(formattedVal, "%03d", (int)getThermocoupleTemp()); 
    Serial.print(formattedVal);
    delay(200);
    printBackspaces(3);
    uartInt = Serial.parseInt();
  } while (uartInt != 1);
  goto MENU;

HEATER_CHECK:
  Serial.println("Duty%: 2..100   or 1: Quit");
  setFanDuty(40);
  do {
    uartInt = Serial.parseInt();
    if (uartInt > 100) uartInt = 100;
    if (uartInt > 1) setHeaterDuty(uartInt);
    sprintf(formattedVal, "%03d", (int)getNtcTemp()); 
    Serial.print(formattedVal);
    delay(200);
    printBackspaces(3);
  } while (uartInt != 1);
  setHeaterDuty(0);
  setFanDuty(0);

  goto MENU;
  
HEATER_TEST:
  Serial.println("");
  Serial.println("1: Existing\t2:PID\t3: Fuzzy Logic\t9: Go Back"); 
  Serial.println("");
  do {
    uartInt = Serial.parseInt();  // returns 0 after short time out
    if (uartInt == 9) goto MENU;
    if (uartInt == 1 ) goto EXISTING_HEAT_TEST;
    if (uartInt == 2) goto PID_CONTROL_HEAT_TEST;
    if (uartInt == 3) goto FUZZY_LOGIC_HEAT_TEST;
  } while (1);

EXISTING_HEAT_TEST:
  setFanDuty(FAN_PWM_IN_TEST);
  existingHeaterTest(NORMAL);
  setFanDuty(0);
  goto MENU;

PID_CONTROL_HEAT_TEST:
  setFanDuty(FAN_PWM_IN_TEST);
  pidTest(NORMAL);
  setFanDuty(0);
  goto MENU;

FUZZY_LOGIC_HEAT_TEST:
  setFanDuty(FAN_PWM_IN_TEST);
  fuzzyLogicTest(NORMAL);
  setFanDuty(0);
  goto MENU;

HI_HEATER_TEST:
  Serial.println("");
  Serial.println("1: Existing\t2:PID\t3: Fuzzy Logic\t9: Go Back"); 
  Serial.println("");
  do {
    uartInt = Serial.parseInt();  // returns 0 after short time out
    if (uartInt == 9) goto MENU;
    if (uartInt == 1 ) goto EXISTING_HI_HEAT_TEST;
    if (uartInt == 2) goto PID_CONTROL_HI_HEAT_TEST;
    if (uartInt == 3) goto FUZZY_LOGIC_HI_HEAT_TEST;
  } while (1);

EXISTING_HI_HEAT_TEST:
  setFanDuty(FAN_PWM_IN_TEST);
  existingHeaterTest(HIGH);
  setFanDuty(0);
  goto MENU;

PID_CONTROL_HI_HEAT_TEST:
  setFanDuty(FAN_PWM_IN_TEST);
  pidTest(HIGH);
  setFanDuty(0);
  goto MENU;

FUZZY_LOGIC_HI_HEAT_TEST:
  setFanDuty(FAN_PWM_IN_TEST);
  fuzzyLogicTest(HIGH);
  setFanDuty(0);
  goto MENU;


HEATER_CALIBRATION:
  Serial.println("");
  Serial.println("1: Existing Method\t2: Fuzzy Logic\t3: PID\t9: Quit");   
  Serial.println("");
  setFanDuty(FAN_PWM_IN_TEST);
  do {
    uartInt = Serial.parseInt();  // returns 0 after short time out
    if (uartInt == 9) goto MENU;
    if (uartInt == 1 ) existingHeaterCalibration();
    if (uartInt == 2) fuzzyLogicCalibration();
    if (uartInt == 3) pidCalibration();
  } while (uartInt < 1);
  setFanDuty(0);
  goto MENU;

HEAT_SETPOINT:
  Serial.print("1:Quit - NTC Heat Target currently = ");
  Serial.println(heatSetPoint);
  do {
    uartInt = Serial.parseInt(SKIP_WHITESPACE);
    if (uartInt > 1) {
    if (uartInt > MAX_NTC_TEMP) heatSetPoint = MAX_NTC_TEMP;
    if (uartInt < OFFICE_TEMP ) heatSetPoint = OFFICE_TEMP;
    heatSetPoint = uartInt;
    }
  } while (uartInt != 1);
  goto MENU;

HI_HEAT_SETPOINT:
  Serial.print("1:Quit - NTC Hi-Heat Target currently = ");
  Serial.println(hiHeatSetPoint);
  do {
     uartInt = Serial.parseInt(SKIP_WHITESPACE);
    if (uartInt > 1) {
    if (uartInt > MAX_NTC_TEMP) hiHeatSetPoint = MAX_NTC_TEMP;
    if (uartInt < OFFICE_TEMP ) hiHeatSetPoint = OFFICE_TEMP;
    hiHeatSetPoint = uartInt;
    }
  } while (uartInt != 1);
  goto MENU;
}  // end loop()


void printBackspaces(uint8_t n) { 
  for (uint8_t i=0; i<n; i++) {
     Serial.write(ASCII_BACKSPACE);   
  }
}

bool isFanStalled(int targetPercent) {
  int fanSpeed = getFanPercent();
  if (fanSpeed < (targetPercent - FAN_PCT_TOLERANCE) || fanSpeed > ( targetPercent + FAN_PCT_TOLERANCE)) {
    return true;
  }
  return false;
}

void existingHeaterCalibration() {
  // As per Propius Viewer calibration method:

}

void existingHeaterTest(bool normalHigh) {
  // Implementing void HeatElementControl(HeatElementControl_t command, void const *vp_Context) 
  // from ElementControl.c

  // This routine is to be run every 250ms.
  // The NTC has a thermal time constant of approx 5sec in air.
  // The ProcessTime is the frequency of the temp control algorithm.
  // So ProcessTime = 8 is 8x250ms = 2s
  int temp, previousTemp = OFFICE_TEMP, setPoint, tempProcessError, tempGradient;
  int dutyChange, newDuty = START_DUTY;
  bool tempTargetTrigger = false, overHeated = false;
  long startTime, elapsedTimeTotal = 0, loopProcessTime, loopStartTime;

  if (normalHigh == HIGH)  setPoint = hiHeatSetPoint;
  else setPoint = heatSetPoint;

  startTime = millis();

  while(!overHeated && elapsedTimeTotal < MAX_TEST_TIME_MS) {
    int fanSpeed;
    loopStartTime = millis();

    // check fan not stalled
    if (isFanStalled(FAN_PWM_IN_TEST)) {
      overHeated = true;
      continue;
    }

    // calc temperature error
    temp = (int)getNtcTemp();
    tempProcessError = setPoint - temp;

    // calc rate of change of temperature rise (degC/s)
    tempGradient = (temp  - previousTemp) * (1000 / LOOP_UPDATE_TIME_MS);
    previousTemp = temp;	

    // proportional change of duty based on error
    dutyChange = 0;  
    if(tempProcessError > 5)dutyChange = 1;   
    if(tempProcessError > 40)dutyChange = 2;
    
    if(tempProcessError < -5)dutyChange = -1;
    if(tempProcessError < -20)dutyChange = -2;
    if(tempProcessError < -40) dutyChange = -3;

    // this controls the initial ramp to target
    if(tempTargetTrigger == false)
    {
      if(temp >= setPoint)
      {
        tempTargetTrigger = true;  
        // TBD: Ambient compensation could go here???
      }
             
      // limit the max rate of temp rise (due to high battery voltage)
      if(tempGradient > 13)
      {
        dutyChange = 0;
      }	
                
      // back off the duty when approaching a percentage of the setpoint					
      if( (temp >= (setPoint - setPoint/5)) && (tempGradient >= 4))  // -20%
      {
        if(dutyChange > 0) dutyChange = 0;
      }								
    }

    // apply duty change and check limits   						
    if(dutyChange != 0)
    {		
      newDuty += dutyChange;						

      if(newDuty > MAX_HEATER_DUTY) newDuty = MAX_HEATER_DUTY;
      if(newDuty < MIN_HEATER_DUTY) newDuty = MIN_HEATER_DUTY;

      // check if NTC temp is beyond safe operating region
      if (temp > MAX_NTC_TEMP) {
        newDuty = 0;
        overHeated = true;
      }
      setHeaterDuty(newDuty); 
    }

     plotterPrint(ON, elapsedTimeTotal, newDuty, temp, int(getThermocoupleTemp()));
    

    // toggle Green LED
    digitalWrite(ORG_LED_PIN, greenLedState);
    greenLedState ? OFF : ON;

    // work out how long to wait to make loop time constant
    loopProcessTime = millis() - loopStartTime;
    delay(LOOP_UPDATE_TIME_MS - loopProcessTime); 
    elapsedTimeTotal = millis() - startTime;
    
  }
  Serial.println("Existing Heater Test Completed.");
  setHeaterPwm(0);
  ORANGE_LED(OFF);
  coolingFanBlast(3); 
}


void fuzzyLogicCalibration() {

}

void fuzzyLogicTest(bool normalHigh) {
  long startTime, elapsedTimeTotal = 0, loopProcessTime, loopStartTime;
  bool overHeated = false;
  int fanSpeed, heaterPwm = AS_PWM(START_DUTY);
  double temperature;

  if (normalHigh == HIGH)  setPoint = hiHeatSetPoint;
  else setPoint = heatSetPoint;

  startTime = millis();

  while(!overHeated && elapsedTimeTotal < MAX_TEST_TIME_MS) {
    loopStartTime = millis();

    // check fan not stalled
    if (isFanStalled(FAN_PWM_IN_TEST))  {
      overHeated = true;
      continue;
    }

    temperature = getNtcTemp();
    fuzzy->setInput(1, temperature);
    fuzzy->fuzzify();
    heaterPwm += fuzzy->defuzzify(1);
    // clamp heater duty locally
    if (heaterPwm < AS_PWM(MIN_HEATER_DUTY)) heaterPwm = AS_PWM(MIN_HEATER_DUTY);
    if (heaterPwm > AS_PWM(MAX_HEATER_DUTY)) heaterPwm = AS_PWM(MAX_HEATER_DUTY);
    setHeaterPwm(heaterPwm);
    digitalWrite(ORG_LED_PIN, temperature > (setPoint - 2) && temperature < (setPoint + 2)); 

    if (temperature > MAX_NTC_TEMP) {
      overHeated = true;
      continue;
    }

    plotterPrint(ON, elapsedTimeTotal, (int)(AS_DUTY(heaterPwm)), (int)temperature, int(getThermocoupleTemp()));
    
    // work out how long to wait to make loop time constant
    loopProcessTime = millis() - loopStartTime;
    delay(LOOP_UPDATE_TIME_MS - loopProcessTime); 
    elapsedTimeTotal = millis() - startTime;
  }
  Serial.println("Fuzzy Logic Heater Test Completed.");
  setHeaterPwm(0);
  ORANGE_LED(OFF);
  coolingFanBlast(3); 
}

void pidCalibration() {

}

void pidTest(bool normalHigh) {
  long startTime = millis(), elapsedTimeTotal = 0; 
  long loopProcessTime, loopStartTime;
  int fanSpeed;
  bool overHeated = false;

  if (normalHigh == HIGH)  setPoint = hiHeatSetPoint;
  else setPoint = heatSetPoint;

  while(!overHeated && elapsedTimeTotal < MAX_TEST_TIME_MS) {
  loopStartTime = millis();

  // check fan not stalled
  
  if (isFanStalled(FAN_PWM_IN_TEST))  {
    overHeated = true;
    continue;
  }
  

  temperature = getNtcTemp();
  pidController.run();  
  setHeaterPwm(htrPwm);

  digitalWrite(ORG_LED_PIN, pidController.atSetPoint(2)); 
  if (temperature > MAX_NTC_TEMP) {
    overHeated = true;
    Serial.println("Overheated NTC!");
    continue;
  }
  plotterPrint(ON, elapsedTimeTotal, (int)(AS_DUTY(htrPwm)), (int)temperature, int(getThermocoupleTemp()));

  // work out how long to wait to make loop time constant
  loopProcessTime = millis() - loopStartTime;
  delay(LOOP_UPDATE_TIME_MS - loopProcessTime); 
  elapsedTimeTotal = millis() - startTime;
  }
  Serial.println("PID Hester Test Completed.");
  setHeaterPwm(0);
  ORANGE_LED(OFF);
  coolingFanBlast(3);
}


void coolingFanBlast(int seconds) {
  // make sure HEATER is off and give a FAN blast
  setFanDuty(100);
  setHeaterDuty(0);
  delay(seconds * 1000);
  setFanDuty(0);
}

void plotterPrint(bool showTime, long time_ms, int duty, int ntcTemp, int thermoTemp) {
  if (showTime) {
    Serial.print((int)(time_ms/1000));
    Serial.print(", ");
  }
  Serial.print(duty);
  Serial.print(", ");
  Serial.print(ntcTemp);
  Serial.print(", ");
  Serial.println(thermoTemp); 
}


