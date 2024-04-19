/*
* HongFei HB-5015HB12B Fan PWM Control
* Propius Fan, requires 10-30KHz PWM on White lead (0-100% duty) to digital I/O Pin 6
* Tacho freq output from fan on Yellow lead to digital I/O Pin 2: RPM = f_tacho * 30
* 12V on Red lead. 0V (Arduino GND) on Black lead.
* Fan does not respond well to low PWM, e.g. doesn't start at duty < 20% or so.
*
* Arduino built-in PWM using AnalogWrite is too slow (<1KHz) so need direct
* PORTD writes and NOP waits to get required 10-30KHz PWM speeds.

* WHITE  -  PWM out; Pin 6
* YELLOW -  Tacho (RPM/30) in; Pin 5
* RED    -  Ext. 12V (Can use Arduino 5V just to check operation)
* BLACK  -  Arduino GND and Fan 12V PSU GND (tie together)
* BLUE   -  Connect to Arduino 5V - it pull-up Tacho via a 10K

* Typically maximum RPM is about 6800-7000RPM for this fan
* and it has a fairly linear relation between PWM and Tacho freq (RPM)
* Datasheet: C:\Users\oliver.bailey\OneDrive - No Climb Products Ltd\Documents\Datasheets\Fans\EX39112_HB-5015H12B-PWM+FG-V06 (002).pdf
*/

#include <FreqCount.h>

const uint8_t TACHO_PIN = A0;
const int FREQ_COUNT_GATE_TIME_MS = 200;    // used to scale freq count to frequency (over 1000ms!)
                                            // 100ms give usable freq, but 1000ms give more accuracy

int pwm = 40;  // FAN PWM 0-100%
int rpm = 1000;
const long int LOOPS_PER_SECOND  = 13600;
const int NOP_LOOPS = 100;    // NOPs per period for ~13KHz PWM

void setup() {
  FreqCount.begin(200);   
  pinMode(TACHO_PIN, INPUT);
  Serial.begin(115200);
  Serial.println();
  Serial.println("Arduino HongFei HB-5015HB12B Fan PWM Control using PORTD writes");
  delay(100);   // to allow UART to finish before disabling interrupts

  DDRD = B01000000;  // set PWM pin as an output
}

void loop() {
  // Runs the fan, increasing the speed every 2 seconds in a repeating sawtooth manner...

  // give fan a kick for low PWM requests
  if (pwm <= 30) kickStart(200);

  runFan(3000,pwm);  //blocking
  rpm = readTacho();  
  Serial.print("PWM: ");
  Serial.print(pwm);
  Serial.print(" RPM: ");
  Serial.println(rpm);
  runFan(5000,pwm);  //blocking   

  pwm += 10;
  if (pwm > 100) pwm = 10;

}

void runFan(int duration_ms, int duty_pct) {
  // calculate duty cycle on/off loops
  int j;
  int on_loops = (NOP_LOOPS * duty_pct) / 100;
  int off_loops = NOP_LOOPS - on_loops;
  long int i;
  long int total_cycles = duration_ms * (LOOPS_PER_SECOND / 1000);

for (i=0; i < total_cycles; i++) {
    // PWM pin set high
    PORTD = B01000000;
    for (j=0; j < on_loops ; j++) {  
      __asm__("nop\n\t");   // 62.5ns (1/16MHz) cycle delay
    }
    // PWM pin set low
    PORTD = B00000000;
    for (j=0; j < off_loops; j++) {  
      __asm__("nop\n\t");   // 62.5ns (1/16MHz) cycle delay
    }
  } 
}

int readTacho() {
  // Measure approx period of Tacho signal (T_tacho)
  // and output RPM = 30/T_tacho = 30 * f
  unsigned long count;

  if (FreqCount.available())    count = FreqCount.read();

  // limit to be positive
  if (count < 0 ) count = 0;

  // scale by 30 for fan and by gate period for freq count
  return (1000/FREQ_COUNT_GATE_TIME_MS)*30*(int)count;
}

void kickStart(int duration_ms) {
  // Use 100% duty to get the fan going
  PORTD=B01000000;
  delay(duration_ms);
  PORTD=B00000000;
}
