#include <Arduino.h>
#include "mbed.h"

#if !( ARDUINO_ARCH_NRF52840 && TARGET_NAME == ARDUINO_NANO33BLE )
  #error This code is designed to run on nRF52-based Nano-33-BLE boards using mbed-RTOS platform! Please check your Tools->Board setting.
#endif

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#include "NRF52_MBED_TimerInterrupt.h"
#include "NRF52_MBED_ISR_Timer.h"

#define HW_TIMER_INTERVAL_MS      1000  
#define TIMER_INTERVAL_10S        10000L  //Frecuencia de la interrupción

NRF52_MBED_Timer ITimer(NRF_TIMER_3);
NRF52_MBED_ISRTimer ISR_Timer;

#ifndef LED_BLUE_PIN
  #if defined(LEDB)
    #define LED_BLUE_PIN          LEDB
  #else
    #define LED_BLUE_PIN          D7
  #endif
#endif

extern const int adcPin = P0_4;
volatile bool adcTrigger = false;
const PinName PWM_Pin = P0_4; // PinName for PwmOut
const unsigned int PWMPeriod = 200; // us (minimum value is 2)
const float PWMDutyCycle = 0.5; // ratio
/*const char *c1 = "ADC";
const char *c2 = "ADC(x)";
const char *c3 = "PWM(x)";*/
String cmd = "";

mbed::PwmOut pwm(LED1);

void PWM_write(int &ADCvalue);
void TimerHandler();
void triggerADC();
void UART_read();
void ADCread();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  delay(100);

  Serial.print(F("\nStarting TimerInterruptLEDDemo on "));
  Serial.println(BOARD_NAME);
  Serial.println(NRF52_MBED_TIMER_INTERRUPT_VERSION);

  pinMode(adcPin, INPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

  // set the PWM period to PWMPeriod
  pwm.period_us(PWMPeriod);

  // start the PWM signal with the duty cycle set to PWMDutyCycle
  pwm.write(PWMDutyCycle);

  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler))
  {
    Serial.print(F("Starting ITimer OK, millis() = "));
    Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));



  //ISR_Timer.setInterval(TIMER_INTERVAL_10S,  triggerADC);
}

void loop() {
  // Llamar a UART_read para procesar comandos entrantes
  UART_read();
  if (adcTrigger)
  {
    int adcValue = analogRead(adcPin);
    char buffer[50];
    sprintf(buffer, "ADC read value: %d", adcValue);
    Serial.println(buffer);
    adcTrigger = false;
  }
}

void PWM_write(int ADCvalue, int cycle)
{
  // Mapear el valor del ADC (0-1023) al rango PWM (0.0-1.0)
  float pwmValue = map(ADCvalue, 0, 1023, 0, 1000) / 1000.0;
  pwm.write(pwmValue);
}

void UART_read()
{
  // Leer el puerto serie
  if (Serial.available() > 0)
  {
    cmd = Serial.readStringUntil('\n');
    cmd.trim(); // Eliminar espacios en blanco al principio y al final
    Serial.println(cmd); // Imprimir el comando recibido para depuración

    // Procesar el comando recibido
    if (cmd == "ADC"){
      ADCread(); 
    }
    else if(cmd.startsWith("ADC(")){
      String intervalStr = cmd.substring(4, cmd.length() - 1);
      unsigned long interval = intervalStr.toFloat();
      Serial.println(interval);
      if(interval != 0){
          ISR_Timer.setInterval(interval*1000,  triggerADC);
        }else ISR_Timer.deleteTimer(0);
    }else if(cmd.startsWith("PWM(")){
      String cycleStr = cmd.substring(4, cmd.length() - 1);
      float cycle = cycleStr.toFloat();
      float DutyCycle = constrain(cycle, 0.0, 1.0);
      pwm.write(DutyCycle);
    }
}
}

void TimerHandler() {
  ISR_Timer.run();
}

void triggerADC() {
  adcTrigger = true;
  //digitalWrite(LED_BLUE_PIN, !digitalRead(LED_BLUE_PIN));
}

void ADCread(){
  int adcValue = analogRead(adcPin);
  char buffer[50];
  sprintf(buffer, "ADC read value: %d", adcValue);
  Serial.println(buffer);
}