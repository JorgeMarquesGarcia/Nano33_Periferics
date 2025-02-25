#include <Arduino.h>
#if !( ARDUINO_ARCH_NRF52840 && TARGET_NAME == ARDUINO_NANO33BLE )
  #error This code is designed to run on nRF52-based Nano-33-BLE boards using mbed-RTOS platform! Please check your Tools->Board setting.
#endif

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#include "NRF52_MBED_TimerInterrupt.h"
#include "NRF52_MBED_ISR_Timer.h"

#define HW_TIMER_INTERVAL_MS      1000  // Intervalo del temporizador de hardware en milisegundos (1 segundo)
#define TIMER_INTERVAL_10S        10000L  // Intervalo del temporizador basado en ISR en milisegundos (10 segundos)

NRF52_MBED_Timer ITimer(NRF_TIMER_3);
NRF52_MBED_ISRTimer ISR_Timer;

#ifndef LED_BLUE_PIN
  #if defined(LEDB)
    #define LED_BLUE_PIN          LEDB
  #else
    #define LED_BLUE_PIN          D7
  #endif
#endif

void TimerHandler()
{
  ISR_Timer.run();
}

extern const int adcPin = P0_4;
volatile bool adcTrigger = false;

void triggerADC() {
  adcTrigger = true;
  digitalWrite(LED_BLUE_PIN, !digitalRead(LED_BLUE_PIN));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  delay(100);

  Serial.print(F("\nStarting TimerInterruptLEDDemo on "));
  Serial.println(BOARD_NAME);
  Serial.println(NRF52_MBED_TIMER_INTERRUPT_VERSION);

  pinMode(adcPin, INPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS *1000 , TimerHandler))
  {
    Serial.print(F("Starting ITimer OK, millis() = "));
    Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));

  ISR_Timer.setInterval(TIMER_INTERVAL_10S,  triggerADC);
}

void loop() {
    if (adcTrigger) {
    adcTrigger = false;
    int adcValue = analogRead(adcPin);
    char buffer[50];
    sprintf(buffer, "ADC read value: %d", adcValue);
    Serial.println(buffer);
  }
}



