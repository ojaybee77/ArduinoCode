/*
* Propius Time-of-Flight Dust Detection Demonstration

  Oli Bailey  June '23

  Using Arduino Uno and ToF in Cup (see 'ToF_accuracy_and_robustness_against_dust_investigation.docx')

  This source referes extensively to the VL6180X datasheet and AN4545 "VL6180X basic ranging application note"
  Uses a modified Adafruit VL6180X library, stored along with this source file.



*/

#include <Wire.h>
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
#define XTALK_COUNT 4           // number of measurement to average during cross-talk calibration (lower=faster, datasheet recommends 10)
#define EMPTY_CUP_MM 120.0      // ToF range for empty cup
#define XTALK_LOWER_LIMIT 0.0   // Mcps, cross-talk can't be negative. Nothing wrong with zero cross talk though at 120mm range.
#define XTALK_UPPER_LIMIT 1.2   // Mcps, anymore than this, cup is likely to have dust on ToF lens or something in the cup.

#define MAX_LUX_WHEN_CUP_AGAINST_DETECTOR 2    // Lux maximum threshold to help determine when detector is properly within cup to start test
#define MAX_RANGE_WHEN_CUP_AGAINST_DETECTOR 66  // mm  maximum threshold to help determine when detector is properly within cup to start test
#define MIN_RANGE_WHEN_CUP_REMOVED_FROM_DUSTY_DETECTOR 30  // mm  maximum threshold to help determine when detector is properly within cup to start test

// Pin definitions
const int RST_PIN = 2;
const int PWR_PIN = 3;

// Global Vars
bool DetectorIsInCup = false;

// The sensor object
Adafruit_VL6180X vl = Adafruit_VL6180X();

/****************************************************************************
*                               INITIALISATION                              *
*****************************************************************************/
void setup() {
  Serial.begin(9600);
  // prepare serial plotter
  Serial.println("");
  Serial.println("Min:0,Max:255");
  Serial.println("lux:,raw:,range_xtlk:,range_xtlk_p2p:");
    
  // apply power to the VL6180X from an I/O pin
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(10);

  // reset the VL6180X, and wait for its MCU to boot
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN,LOW);
  delay(1);
  digitalWrite(RST_PIN,HIGH);
  delay(1);

  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }

  delay(2000);

  if (!vl.begin()) {
    Serial.println("Failed to find sensor");
    digitalWrite(PWR_PIN, LOW);
    while (1);  // and stop...
  }
  // magic register settings (won't work without this, see AN4545 page 24 "SR03 Settings")
  vl.loadSettings();

#ifndef USE_PLOTTER
  Serial.println("");
  Serial.print("P2P factory offset (mm): ");
  Serial.println(vl.getPartToPartOffset());
  Serial.println("");
#endif

  // initial post-power on cross-talk compensation
  compensate(VERBOSE);

  // set range threshholds - what the hell are these used for?
  vl.setRangeThresholds(20,110);

  // try settings to reduce/counter/ignore effect of dust on cover lens ???
  //vl.setRangeIgnore(130, 0x0100);    // height in mm, threshold in 9.7 format 
  //vl.RangeIgnoreMode(ON);

  // adjust readout averaging (default is 48 * 64.5us = 4.3ms)
  //vl.setReadOutAveragingPeriod(96);

}

/****************************************************************************
*                               MAIN LOOP                                   *
*****************************************************************************/
void loop() {
  float lux = vl.readLux(VL6180X_ALS_GAIN_5);
  uint8_t raw_range = vl.readRawRange();
  uint8_t range = vl.readRange(0);
  uint8_t range_p2p = range + vl.getPartToPartOffset();
  showError();
  
  // serial plotter output format
  Serial.print(lux);
  Serial.print(" "); 
  Serial.print(raw_range);
  Serial.print(" "); 
  Serial.print(range);
  Serial.print(" "); 
  Serial.println(range_p2p);  

  // Determine detector in cup state
  if (lux < MAX_LUX_WHEN_CUP_AGAINST_DETECTOR && range_p2p < MAX_RANGE_WHEN_CUP_AGAINST_DETECTOR) {
    if (!DetectorIsInCup)  {
      Serial.println("Cup placed around detector!?");
      DetectorIsInCup = true;
    }
  }
  else if (lux > MAX_LUX_WHEN_CUP_AGAINST_DETECTOR && range_p2p > MIN_RANGE_WHEN_CUP_REMOVED_FROM_DUSTY_DETECTOR) {
    if (DetectorIsInCup)  {
      Serial.println("Cup removed from detector!?");
      DetectorIsInCup = false;
      // small delay, then recalibrate to current "dust" levels
      delay(500);
      compensate(VERBOSE);
      
    }
  }

  // if range overflow is reported, user may have cleaned lens whilst unit powered up, so do recalibration
  if (vl.readRangeStatus() == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range Overflow detected- recalibrating...");
    compensate(VERBOSE);
  }

  delay(1333);
}


/****************************************************************************
*                               OTHER FUNCTIONS                             *
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

void showError() {
  uint8_t status = vl.readRangeStatus();

  // if no error then return
  if (status == VL6180X_ERROR_NONE) return;

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
}


bool doCrossTalkCompensation(bool silent) {
  bool calibrationResult = SUCCESS;
#ifndef USE_PLOTTER
  if (!silent)  Serial.println("Crosstalk compensation:");
#endif

  // ensure SYSRANGE__CROSSTALK_COMPENSATION_RATE {0x1e} is set to 0.
  vl.setCrosstalkCompensationRate(0);

  uint8_t i;
  uint8_t range_vals[XTALK_COUNT];
  uint16_t avg_range = 0;
  float mcps_vals[XTALK_COUNT], avg_mcps = 0.0;

  // take readings and derive averages
  for(i = 0;i < XTALK_COUNT; i++) {
    // measure range and return rate
    range_vals[i] = vl.readRange(1);
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
  float cross_talk = avg_mcps * (1 - (float)(avg_range) / EMPTY_CUP_MM);

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