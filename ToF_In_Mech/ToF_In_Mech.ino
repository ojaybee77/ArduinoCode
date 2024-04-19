/*
* Propius Time-of-Flight Sensitivity to Mechanical Seating/Alignment Investigation

  Oli Bailey Jan '24 
  See https://teams.microsoft.com/l/entity/com.microsoft.teamspace.tab.planner/tt.c_19:f10124937b5541a189935154cde944ac@thread.tacv2_p_NmpkOWk8jkWg5zteHdKc05cACRgW_h_1620718335817?tenantId=38b8bc6c-036b-438c-8369-f3cd0f022456&webUrl=https%3A%2F%2Ftasks.teams.microsoft.com%2Fteamsui%2FpersonalApp%2Falltasklists&context=%7B%22subEntityId%22%3A%22%2Fboard%2Ftask%2F65sJgTJbl06RK6DCxAlf4JcAI_UO%22%2C%22channelId%22%3A%2219%3Af10124937b5541a189935154cde944ac%40thread.tacv2%22%7D


  Using Arduino Uno and ToF in Cup (see 'ToF_accuracy_and_robustness_against_dust_investigation.docx')

  This source referes extensively to the VL6180X datasheet and AN4545 "VL6180X basic ranging application note"
  Uses a modified Adafruit VL6180X library, stored along with this source file.

  This sketch is menu driven to allow the user to calibrate, change the part to part offset value and take measurements.
  
  The concern is a large spread seen in XTR2 Final Assembly Test of ToF range measurements after a ToF calibration at
  power up of the XTR2 unit

*/

#include <Wire.h>
#include "QuickStats.h"
#include "Adafruit_VL6180X_MOD.h" // C:\Users\...\Documents\Arduino\libraries\Adafruit_VL6180X

// Conditional flow control flags (comment in/out)

//#define USE_PLOTTER
#define CONTINUE_EVEN_WHEN_CAL_FAILS 


// Status values
#define ON 1
#define OFF 0
#define SUCCESS 1
#define FAIL 0
#define SILENT 1
#define VERBOSE 0

// Parameters & Thresholds
#define STATS_TOTAL 10          // number of measurements to take in Multi Measurement mode for stats summary
#define XTALK_COUNT 4           // number of measurement to average during cross-talk calibration (lower=faster, datasheet recommends 10)
#define XTALK_LOWER_LIMIT 0.0   // Mcps, cross-talk can't be negative. Nothing wrong with zero cross talk though at 120mm range.
#define XTALK_UPPER_LIMIT 1.2   // Mcps, anymore than this, cup is likely to have dust on ToF lens or something in the cup.

const char compile_date[] =  "Compiled: " __DATE__ " " __TIME__;

// Pin definitions
const int RST_PIN = 2;
const int PWR_PIN = 3;

// Global Vars
const char ASCII_BACKSPACE = 0x08;
bool DetectorIsInCup = false;
float emptyCupCalRange = 123.0;     // target calibration range

// The sensor object
Adafruit_VL6180X vl = Adafruit_VL6180X();

// Statistic object
QuickStats stats; 

/****************************************************************************
*                              FUNCTIONS                                    *
*****************************************************************************/
void compensate(bool silent_verbose) {
    // cross talk compensation process (AN4545 Section 5.1.1)
  if (doCrossTalkCompensation(silent_verbose) == FAIL) {
    Serial.println("");
    Serial.println("Crosstalk too high!! Check cup is empty and ToF lens is clean");
    Serial.println("");
#ifndef CONTINUE_EVEN_WHEN_CAL_FAILS
    while(1); // stop here
#endif
  }
}

bool showError() {
  uint8_t status = vl.readRangeStatus();

  // if no error then return
  if (status == VL6180X_ERROR_NONE) return false;

  // otherwise some error occurred, print it out!
    if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  }

  Serial.println("");
  return true;
}


bool doCrossTalkCompensation(bool silent) {
  bool calibrationResult = SUCCESS;
  uint8_t i;
  uint8_t range_vals[XTALK_COUNT];
  uint16_t avg_range = 0;
  float mcps_vals[XTALK_COUNT], avg_mcps = 0.0;

#ifndef USE_PLOTTER
  if (!silent)  Serial.println("Crosstalk compensation:");
#endif

  // ensure SYSRANGE__CROSSTALK_COMPENSATION_RATE {0x1e} is set to 0.
  vl.setCrosstalkCompensationRate(0);


  // take readings and derive averages
  for(i = 0;i < XTALK_COUNT; i++) {
    // measure range and return rate
    range_vals[i] = vl.readRange();
    avg_range += range_vals[i];
    mcps_vals[i] = vl.readReturnRate();
    avg_mcps += mcps_vals[i];
    if (!silent) {
      Serial.print("Measurement ");
      Serial.println(i);
    }
    delay(30);
  }

  // compute averages
  avg_range /= XTALK_COUNT;
  avg_mcps /= (float)XTALK_COUNT;

   // calculate the cross talk (see AN4545 section 5.1.1)
  float cross_talk = avg_mcps * (1.0 - (float)(avg_range) / emptyCupCalRange);

   // lower limit check : (NOTE: cross-talk can't be negative)
  if (cross_talk < XTALK_LOWER_LIMIT) {
    cross_talk = 0;
  }

  // upper limit check: too high and cup possibly not empty and/or dust on ToF lens
  if (cross_talk > XTALK_UPPER_LIMIT)  {
    calibrationResult = FAIL;
  }

  // convert to 9.7 format ready to write to register
  uint16_t cross_talk_9_7_format = (uint16_t)(cross_talk * 128.0);

  // write result to register
  vl.setCrosstalkCompensationRate(cross_talk_9_7_format);

  #ifndef USE_PLOTTER
  if (!silent) {
    Serial.print("Avg.Range ");
    Serial.print(avg_range);
    Serial.print(" Avg.Mcps ");
    Serial.print(avg_mcps);
    Serial.print(" X-tlk ");
    Serial.print(cross_talk);
    Serial.print(" X-talk(9.7f) ");
    Serial.println(cross_talk_9_7_format);
    Serial.println("");
  }
#endif

  return calibrationResult;
}

void printRange() {
  // serial plotter output format
  // Serial.print(lux);
  // Serial.print(" "); 
  // Serial.print(raw_range);
  // Serial.print(" "); 
  // Serial.print(range);
  // Serial.print(" "); 
  // Serial.println(range_p2p);  

}

void sensorPower(bool onNoff) {
  digitalWrite(PWR_PIN, onNoff);
  delay(10);
  if (onNoff == ON) resetSensor();
}

void resetSensor() {
  digitalWrite(RST_PIN, LOW);
  delay(10);
  digitalWrite(RST_PIN, HIGH);
  // magic register settings (won't work without this, see AN4545 page 24 "SR03 Settings")
  vl.loadSettings();
}


/****************************************************************************
*                               INITIALISATION                              *
*****************************************************************************/
void setup() {
  Serial.begin(115200);
  pinMode(PWR_PIN, OUTPUT);

  // apply power to the VL6180X from an I/O pin
  sensorPower(ON);

  // prepare serial plotter
  Serial.println("");
  Serial.println("Min:0,Max:255");
  Serial.println("lux:,raw:,range_xtlk:,range_xtlk_p2p:");  

  // reset the VL6180X, and wait for its MCU to boot
  pinMode(RST_PIN, OUTPUT);
  resetSensor();




  if (!vl.begin()) {
    Serial.println("***** Failed to find sensor, aborting! *****");
    sensorPower(OFF);
    while (1);  // and stop...
  }
  

#ifndef USE_PLOTTER
  Serial.println("");
  Serial.print("P2P factory offset (mm): ");
  Serial.println(vl.getPartToPartOffset());
  Serial.println("");
#endif

  // initial post-reset/power-on cross-talk compensation
  compensate(VERBOSE);

  // set range threshholds - what the hell are these used for?
  vl.setRangeThresholds(20,110);
}

/****************************************************************************
*                               MAIN LOOP                                   *
*****************************************************************************/
void loop() {
  uint8_t range;
  int uartInt;
  int8_t p2p_offset_mm;
  float lux;
  int current;
  float results[STATS_TOTAL] = {0.0};
  uint8_t raw_range = vl.readRawRange();
  
   if (showError())  {
    Serial.println("*** Sensor error, aborting! ***");
    goto QUIT;
   };

  MENU:
  Serial.println("");
  Serial.println("**************************************************");
  Serial.println(compile_date);
  Serial.println("1: Get P2P Offset\t2: Set P2P Offset\t3: Calibrate");   
  Serial.println("4: Single Meas.\t\t5: Multi Meas.\t\t6: Read ALS");
  Serial.println("7: Get Cal. Range\t8:Set Cal. Range\t9: Reset Sensor");
  Serial.println("10: Sensor Off\t\t\t11: Sensor On\t\t12: Quit");
  Serial.println("");
  do {
    uartInt = Serial.parseInt();  // returns 0 after short time out
    if (uartInt == 12) goto QUIT;
    if (uartInt == 1 ) goto GET_P2P_OFFSET;
    if (uartInt == 2) goto SET_P2P_OFFSET;
    if (uartInt == 3) goto CALIBRATE;
    if (uartInt == 4) goto SINGLE_MEAS;
    if (uartInt == 5) goto MULTI_MEAS;
    if (uartInt == 6) goto GET_ALS;
    if (uartInt == 7) goto GET_TARGET_RANGE;
    if (uartInt == 8) goto SET_TARGET_RANGE;
    if (uartInt == 9) goto RESET_SENSOR;
    if (uartInt == 10) goto SENSOR_OFF;
    if (uartInt == 11) goto SENSOR_ON;
  } while (uartInt < 1);

QUIT:
  sensorPower(OFF);
  Serial.println("***** STOPPED ******");
  while(1);

SENSOR_OFF:
  sensorPower(OFF);
  goto MENU;

SENSOR_ON:
  sensorPower(ON);
  goto MENU;

GET_P2P_OFFSET:
  p2p_offset_mm = vl.getPartToPartOffset();
  Serial.print("P2P Offset = ");
  Serial.print(p2p_offset_mm);
  Serial.println("mm");
  goto MENU;

SET_P2P_OFFSET:
  Serial.println("p2p Offset: -20..20mm or 100 to set zero, 99: Quit");
  do {
    uartInt = Serial.parseInt();
    if (uartInt >= -20 && uartInt <= 20) {
      if (uartInt != 0) vl.setPartToPartOffset((int8_t)uartInt);
    }  
    if (uartInt == 100) vl.setPartToPartOffset(0); 
  } while (uartInt != 99);
  goto MENU;


CALIBRATE:
  compensate(VERBOSE);
  goto MENU;

SINGLE_MEAS:
  Serial.print(vl.readRange());
  Serial.println("mm");
  goto MENU;


MULTI_MEAS:
  Serial.println("1: Quit"); 
  current = STATS_TOTAL;
  vl.readRange();
  do {
    results[--current] = vl.readRange();
    Serial.print(results[current]);
    Serial.print("mm; RR = ");
    Serial.print(vl.readReturnRate());
    Serial.println("mcps");
    delay(1000);   
  } while (current > 0 && uartInt != 1);

  // now show the statistics:
  Serial.print("Average: ");
  Serial.println(stats.average(results,STATS_TOTAL));
  Serial.print("Minimum: ");
  Serial.println(stats.minimum(results,STATS_TOTAL));
  Serial.print("Maximum: ");
  Serial.println(stats.maximum(results,STATS_TOTAL));
  Serial.print("Standard Deviation: ");
  Serial.println(stats.stdev(results,STATS_TOTAL));
  Serial.print("Median: ");
  Serial.println(stats.median(results,STATS_TOTAL));
  Serial.print("Mode: ");
  Serial.println(stats.mode(results,STATS_TOTAL,0.00001));
  goto MENU;


GET_ALS:
  lux = vl.readLux(VL6180X_ALS_GAIN_5);
  Serial.print("Lux Level = ");
  Serial.println(lux);
  goto MENU;


GET_TARGET_RANGE:
 Serial.print("Calibration Target Range = ");
 Serial.print(emptyCupCalRange);
 Serial.println("mm");
 goto MENU;

SET_TARGET_RANGE:
  Serial.println("Range: 100..200mm or 1: Quit");
  do {
    uartInt = Serial.parseInt();
    if (uartInt >= 100 && uartInt <= 200) {
      emptyCupCalRange = uartInt;
    }   
  } while (uartInt != 1);
  goto MENU;

RESET_SENSOR:
  resetSensor();
  Serial.println("Device reset");
  goto MENU;
}



