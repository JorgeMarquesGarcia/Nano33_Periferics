#include <Arduino.h>
#include <Wire.h>

/*This is the writer board*/
#ifndef LED_BLUE_PIN
  #if defined(LEDB)
    #define LED_BLUE_PIN          LEDB
  #else
    #define LED_BLUE_PIN          D7
  #endif
#endif


void receiveEvent(int howMany);

void setup() {
 
  Serial.begin(9600);
  while(!Serial);
  Wire.begin(8);               // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event

}

void loop() {

}



void receiveEvent(int howMany) {
    while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    if (c == '0') {
      digitalWrite(LED_BLUE_PIN, LOW);
    } else if (c == '1') {
      digitalWrite(LED_BLUE_PIN, HIGH);
    } else {
        Serial.print("Command failed");         // print the character
    }
  }
}