#include <Arduino.h>

// put function declarations here:
extern const int adcPin = P0_4;
const int delayTime = 1000;
char buffer[50];

void readADC(const int &delayTime) {
  int adcValue = analogRead(adcPin);
  // Serial.println(adcValue);
  sprintf(buffer,"ADC read value: %d", adcValue);
  Serial.println(buffer);
  // Serial.print("ADC read value: ");
  // Serial.println(adcValue);
  delay(delayTime);
}


void setup() {
  Serial.begin(9600);
  pinMode(adcPin, INPUT);
  // put your setup code here, to run once:
}

void loop() {
  readADC(delayTime);
}

// put function definitions here:




