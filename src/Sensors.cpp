/*
  Arduino LSM9DS1 - Simple Accelerometer

  This example reads the acceleration values from the LSM9DS1
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>


Adafruit_LSM9DS1 IMU = Adafruit_LSM9DS1();
void setupSensor();


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // Espera a que el puerto serial esté listo
  }
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }
  Serial.println("IMU initialized successfully");
  setupSensor();
}

void loop() {
  IMU.read();
  sensors_event_t accel, mag, gyro, temp; // Struct defined by library check Adafruit_Sensor.h
  IMU.getEvent(&accel, &mag, &gyro, &temp);
  

  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(accel.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(accel.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(mag.magnetic.x);   Serial.print(" uT");
  Serial.print("\tY: "); Serial.print(mag.magnetic.y);     Serial.print(" uT");
  Serial.print("\tZ: "); Serial.print(mag.magnetic.z);     Serial.println(" uT");

  Serial.print("Gyro X: "); Serial.print(gyro.gyro.x);   Serial.print(" rad/s");
  Serial.print("\tY: "); Serial.print(gyro.gyro.y);      Serial.print(" rad/s");
  Serial.print("\tZ: "); Serial.print(gyro.gyro.z);      Serial.println(" rad/s");
  Serial.print("Temp: "); Serial.print(temp.temperature/100); Serial.println(" deg C");

  Serial.println();
  delay(1000);
}


void setupSensor()
{
    // 1.) Set the accelerometer range
    IMU.setupAccel(IMU.LSM9DS1_ACCELRANGE_2G, IMU.LSM9DS1_ACCELDATARATE_10HZ); //100ms
    //IMU.setupAccel(IMU.LSM9DS1_ACCELRANGE_4G, IMU.LSM9DS1_ACCELDATARATE_119HZ);
    //IMU.setupAccel(IMU.LSM9DS1_ACCELRANGE_8G, IMU.LSM9DS1_ACCELDATARATE_476HZ);
    //IMU.setupAccel(IMU.LSM9DS1_ACCELRANGE_16G, IMU.LSM9DS1_ACCELDATARATE_952HZ);
    
    // 2.) Set the magnetometer sensitivity
    IMU.setupMag(IMU.LSM9DS1_MAGGAIN_4GAUSS);
    //IMU.setupMag(IMU.LSM9DS1_MAGGAIN_8GAUSS);
    //IMU.setupMag(IMU.LSM9DS1_MAGGAIN_12GAUSS);
    //IMU.setupMag(IMU.LSM9DS1_MAGGAIN_16GAUSS);

    // 3.) Setup the gyroscope
    IMU.setupGyro(IMU.LSM9DS1_GYROSCALE_245DPS);
    //IMU.setupGyro(IMU.LSM9DS1_GYROSCALE_500DPS);
    //IMU.setupGyro(IMU.LSM9DS1_GYROSCALE_2000DPS);


}