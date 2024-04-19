const uint8_t ledOnCommand[] = {0x00, 0x07, 0x16, 0x00, 0x00, 0x00, 0x01, 
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xdf, 0xe4};

const uint8_t ledOffCommand[] = {0x00, 0x07, 0x16, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xa1, 0x0e};


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN,OUTPUT);
}
 
// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  Serial.write(ledOnCommand, 22);  //led on on Propius status LED
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(4000);                      // wait for a second
  Serial.write(ledOffCommand, 22);   // led off on Propius status LED  
}
