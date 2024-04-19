/*
  CalFun ObsSensor Board (COB)

  Connect ot 5V Arduino
  A0 = ADC OUT
  5V Arduino to COB 5V_IN
  ARDUINO GND to COB VSS

  1) Apply power with empty cup
  2) Twiddle R7 to set TRANS_OUT to 4.27V. (COB should consume about 40mA +/- 5mA)
  3) Apply smoke and monitor ADC_OUT!
  4) See 
*/

#define MAX_PCT_PER_FOOT 7.3
#define CLEAN_AIR_ADC 70

int sensorPin = A0;   // select the input pin for the obscuration sensor board ADC output 
int adc = 0;  // variable to store the value coming from the sensor
float mV = 0;
float pct_per_foot = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  adc = getAverageOfN(4);
  // calculate derived values (volts, %/ft)
  mV = adc * ( 5000/1023);
  // assume linear relationship between voltage and %/ft  for now...
  pct_per_foot = MAX_PCT_PER_FOOT * (adc - CLEAN_AIR_ADC)/ 1023.0;

  // clamp %/ft to be positive only
  if (pct_per_foot < 0 ) pct_per_foot = 0;

  Serial.print("ADC:");
  Serial.print(adc);
  Serial.print(",");
  Serial.print("mV/10:");
  Serial.print(mV/10);
  Serial.print(",");
  Serial.print("%/ft*100:");
  Serial.println(pct_per_foot*100);
  delay(1000);
}

int getAverageOfN(int N) {
  int i, adc=0;

  for (i=0; i<N; i++) {
    adc += analogRead(sensorPin);
  }
  return adc/N;

}
