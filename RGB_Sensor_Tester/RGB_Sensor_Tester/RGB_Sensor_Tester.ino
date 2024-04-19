/**************************************************************************************************
 *      RGB SENSOR MAGIC WAND TESTER DRIVER
 *      -----------------------------------
 *      O.Bailey Feb '24
 * 
 *      For testing the Resesas/ISL vs AMS sensors on 400574 vs 401160 XTR2 Sensor Boards
 *      
 *      Ability to drive RGB leds indepently or togther, with red blink test mode.
 *      * LED intensity set 0..9 with a square law relationship to PWM duty to acheive approx linear
 *        intensity increase for each step.
 *      * RGB Full colour mode to test readings for RGB together
 *      * Seperate R,G, B only test modes to test each sensor channel in isolation
 *    
 * ************************************************************************************************/
// Magic Wand is Common Cathode, leave below line in:
#define COMMON_CATHODE  // but comment out if using common anode LED rig

#ifdef COMMON_CATHODE
  #define OFF_STATE LOW
#else 
  #define OFF_STATE HIGH
#endif

#define LEDR 3
#define LEDG 5
#define LEDB 6

const char compile_date[] =  "Compiled: " __DATE__ " " __TIME__;


enum ledColours {RED=1, GREEN, BLUE};
/********************************     FUNCTIONS     ***********************************************/
uint8_t brightnessToDuty(uint8_t brightness) {
  // convert brightness on 0..9 scale to inverted PWM on 255..12 scale
  if (brightness > 9) brightness = 9;
#ifdef COMMON_CATHODE
  uint8_t duty = (3* brightness * brightness);
#else
  uint8_t duty = 255 - (3* brightness * brightness);
#endif

  return duty;
}

void rgbLedsOff() {
  digitalWrite(LEDR, OFF_STATE);
  digitalWrite(LEDG, OFF_STATE);
  digitalWrite(LEDB, OFF_STATE);
}

uint8_t getRed(int rgbCode) {
  // strips out R from 000..999 RGB colour code
  return rgbCode/100;
}

uint8_t getGreen(int rgbCode) {
  // strips out G from 000..999 RGB colour code
  return (rgbCode % 100)/10;
}

uint8_t getBlue(int rgbCode) {
  // strips out B from 000..999 RGB colour code
  return rgbCode % 10;
}

uint8_t getColour(int colourPWMCode) {
  // strips out R,G,B from 4-digit colour & PWM code
  // e.g. 1123 is 1=RED, 123=PWM;   3255 is 3=BLUE, 255=PWM
  return (uint8_t)(colourPWMCode / 1000);
}

uint8_t getPWM(int colourPWMCode) {
  // strips out PWM from 4-digit colour & PWM code
  // e.g. 1123 is 1=RED, 123=PWM;   3255 is 3=BLUE, 255=PWM
  uint8_t pwmVal;

  if (colourPWMCode < 1000) colourPWMCode = 1000;
  pwmVal = (uint8_t)(colourPWMCode % 1000);
  if (pwmVal > 255) pwmVal = 255;

#ifndef COMMON_CATHODE
  pwmVal = 255 - pwmVal;
#endif

  return pwmVal;
}


/********************************     INITIALIZATION      *****************************************/
void setup() {
  Serial.begin(115200);
  Serial.println(compile_date);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  rgbLedsOff();
}



/********************************     MAIN LOOP  / MENUS**********************************************/
void loop() {
  uint8_t brightness;
  uint8_t pwmDuty;
  int uartInt;

PRINT_MENU:
  Serial.println();
  Serial.println("**********************");
  Serial.println("RGB Sensor Magic Wand");
  Serial.println("**********************");
  Serial.println("");
  Serial.println("1: LED  Check\t2: Red \t3: Green\t4: Blue");  
  Serial.println("5: Red Blink\t6: Full Colour\t\t7:Party Mode!");  
  Serial.println("8: Fine PWM\t9: Print Menu\t10: Quit");
  Serial.println("");

MENU:
   rgbLedsOff();
   do {
    uartInt = Serial.parseInt();  // returns 0 after short time out
    if (uartInt == 10) goto QUIT;
    if (uartInt == 9 ) goto PRINT_MENU;
    if (uartInt == 1) goto LED_CHECK;
    if (uartInt == 2) goto RED_LED;
    if (uartInt == 3) goto GREEN_LED;  
    if (uartInt == 4) goto BLUE_LED; 
    if (uartInt == 5) goto RED_BLINK; 
    if (uartInt == 6) goto FULL_COLOUR; 
    if (uartInt == 7) goto PARTY_MODE; 
    if (uartInt == 8) goto FINE_PWM;
  } while (uartInt < 1);


QUIT:
  Serial.println("***** STOPPED ******");
  while(1);


LED_CHECK:
  for (brightness=0; brightness<10; brightness++) {
  pwmDuty = brightnessToDuty(brightness);
  analogWrite(LEDR, pwmDuty);
  analogWrite(LEDG, pwmDuty);
  analogWrite(LEDB, pwmDuty);
  delay(100);
  }
  for (brightness=0; brightness<10; brightness++) {
  pwmDuty = brightnessToDuty(9-brightness);
  analogWrite(LEDR, pwmDuty);
  analogWrite(LEDG, pwmDuty);
  analogWrite(LEDB, pwmDuty);
  delay(100);
  }
  goto MENU;


RED_LED:
  Serial.println("");
  Serial.println("Enter brightness (0..9)");
  Serial.println("r: Return");

  do {
    uartInt = Serial.read();
    if (uartInt > -1) analogWrite(LEDR, brightnessToDuty((uint8_t)uartInt - 48));
    delay(333);
  } while (uartInt != 114); 
  goto PRINT_MENU;


GREEN_LED:
  Serial.println("");
  Serial.println("Enter brightness (0..9)");
  Serial.println("r: Return");

   do {
    uartInt = Serial.read();
    if (uartInt > -1) analogWrite(LEDG, brightnessToDuty((uint8_t)uartInt - 48));
    delay(333);
  } while (uartInt != 114); 
  goto PRINT_MENU;


BLUE_LED:
  Serial.println("");
  Serial.println("Enter brightness (0..9)");
  Serial.println("r: Return");

  do {
    uartInt = Serial.read();
    if (uartInt > -1) analogWrite(LEDB, brightnessToDuty((uint8_t)uartInt - 48));
    delay(333);
  } while (uartInt != 114); 
  goto PRINT_MENU;


RED_BLINK:
  int period, onTime;
  Serial.println("");
    // get period, and do some limit checks
  Serial.println("Enter period (ms)");
  do {
      uartInt = Serial.parseInt();
    } while (uartInt == 0); 
  period = uartInt;
  if (period < 0)  period = 1000;
  if (period > 10000) period = 1000;

  // get on time, and do some limit checks
  Serial.println("Enter period (ms)");
  do {
      uartInt = Serial.parseInt();
    } while (uartInt == 0); 
  onTime = uartInt;
  if (onTime < 0)  onTime = 100;
  if (onTime > 1000) onTime = 999;
  if (onTime > period) onTime = period / 10;

  // get intensity, and do some limit checks
  Serial.println("Enter brightness 1..9");
  do {
      uartInt = Serial.parseInt();
    } while (uartInt == 0);
  brightness = uartInt;
  if (brightness > 9) brightness = 9;
  if (brightness < 1) brightness = 1;
  
  Serial.println("Any key to Quit");

  // blink loop (on first , then off till period complete)
  do {
    uartInt = Serial.read();
    long timer = millis();
    do {
      analogWrite(LEDR, brightnessToDuty(brightness));
    } while (millis() - timer < (onTime));

    do {
      digitalWrite(LEDR, OFF_STATE);
    } while (millis() - timer < (period));
  } while (uartInt == -1);
  goto PRINT_MENU;



FULL_COLOUR:
  Serial.println("");
  Serial.println("Enter RGB colour 000...999");
  Serial.println("1000: Return");

  do {
    uartInt = Serial.parseInt();
    if (uartInt >= 001 && uartInt <= 999) {
      analogWrite(LEDR, brightnessToDuty(getRed(uartInt)));
      analogWrite(LEDG, brightnessToDuty(getGreen(uartInt)));
      analogWrite(LEDB, brightnessToDuty(getBlue(uartInt)));
    }    
  } while (uartInt != 1000); 

  Serial.setTimeout(1000);
  goto PRINT_MENU;


PARTY_MODE:
uint8_t i=0,j=0,k=0;
Serial.println("Any key to Quit");
do {
    uartInt = Serial.read();
    analogWrite(LEDR, brightnessToDuty(9-j));
    analogWrite(LEDG, brightnessToDuty(k));
    analogWrite(LEDB, brightnessToDuty(i));
    i++;
    if (i == 10) {
      i = 0;
      j++;
    }
    if (j == 10) {
      j = 0;
      k++;
    }
     if (k == 10) {
      k = 0;
     }
    delay(33);
} while (uartInt == -1);

goto PRINT_MENU;


FINE_PWM:
  Serial.println("");
  Serial.println("Enter colour(R=1,G=2,B=3) & PWM(0..255) as 4 digits:");
  Serial.println("1: Return");

  do {
    uartInt = Serial.parseInt();
    if (uartInt >= 1000 && uartInt <= 3255) {
      if (getColour(uartInt)==RED)  {
        analogWrite(LEDR, getPWM(uartInt));
        digitalWrite(LEDG, OFF_STATE);
        digitalWrite(LEDB, OFF_STATE);
      }
      if (getColour(uartInt)==GREEN)  {
        analogWrite(LEDG, getPWM(uartInt));
        digitalWrite(LEDR, OFF_STATE);
        digitalWrite(LEDB, OFF_STATE);
      }
      if (getColour(uartInt)==BLUE)  {
        analogWrite(LEDB, getPWM(uartInt));
        digitalWrite(LEDR, OFF_STATE);
        digitalWrite(LEDG, OFF_STATE);
      }
    }    
  } while (uartInt != 1); 

  goto PRINT_MENU;
}

