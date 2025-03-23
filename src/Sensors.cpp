#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino_LSM9DS1.h>

#if !( ARDUINO_ARCH_NRF52840 && TARGET_NAME == ARDUINO_NANO33BLE )
  #error This code is designed to run on nRF52-based Nano-33-BLE boards using mbed-RTOS platform! Please check your Tools->Board setting.
#endif
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#include "NRF52_MBED_TimerInterrupt.h"
#include "NRF52_MBED_ISR_Timer.h"

#define HW_TIMER_INTERVAL_MS      1000  // Intervalo del temporizador de hardware en milisegundos (1 segundo)
#define TIMER_INTERVAL_10S       50L  // Intervalo del temporizador basado en ISR en milisegundos (10 segundos)
NRF52_MBED_Timer ITimer(NRF_TIMER_3);
NRF52_MBED_ISRTimer ISR_Timer;
#ifndef LED_BLUE_PIN
  #if defined(LEDB)
    #define LED_BLUE_PIN          LEDB
  #else
    #define LED_BLUE_PIN          D7
  #endif
#endif

volatile bool UARTtrigger = false;

typedef struct {
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
} Vector3D;

void triggerUART();
void ReadData();
void TimerHandler();


void setup() {
  Serial.begin(115200);
  pinMode(LED_BLUE_PIN, OUTPUT);
  while (!Serial) {
    // Espera a que el puerto serial esté listo
  }
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while(1);
  }
  IMU.setContinuousMode(); // Set continuous mode for the IMU
  Serial.println("IMU initialized successfully");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyro sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.print("Magneto sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS *1000 , TimerHandler))
  {
    Serial.print(F("Starting ITimer OK, millis() = "));
    Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));

  ISR_Timer.setInterval(TIMER_INTERVAL_10S,  triggerUART);
}

Vector3D SensorData;

void loop() {
  ReadData();
  char buffer[150];
  if (UARTtrigger) {
    UARTtrigger = false;
    sprintf(buffer, "Accel X:%.2f \tY:%.2f \tZ:%.2f\n Gyro X:%.2f\t Y:%.2f\t Z:%.2f\n B: X:%2.f\t Y:%.2f\t Z:%.2f\n", 
     SensorData.acc_x, SensorData.acc_y, SensorData.acc_z, SensorData.gyro_x, SensorData.gyro_y, SensorData.gyro_z, SensorData.mag_x, SensorData.mag_y, SensorData.mag_z);
    Serial.println(buffer);
  }
}


void ReadData(){
  IMU.readAcceleration(SensorData.acc_x, SensorData.acc_y, SensorData.acc_z);
  IMU.readGyroscope(SensorData.gyro_x, SensorData.gyro_y, SensorData.gyro_z); 
  IMU.readMagneticField(SensorData.mag_x, SensorData.mag_y, SensorData.mag_z);
}

void TimerHandler()
{
  ISR_Timer.run();
}

void triggerUART()
{
  UARTtrigger = true;
  digitalWrite(LED_BLUE_PIN, !digitalRead(LED_BLUE_PIN));
}