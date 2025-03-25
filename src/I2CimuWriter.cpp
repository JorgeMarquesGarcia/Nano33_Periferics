/*
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

Adafruit_LSM9DS1 IMU = Adafruit_LSM9DS1();
void setupSensor();
void captureSensorData();
void sendSensorData();

const int I2C_ADDRESS = 8;
const int SAMPLE_INTERVAL = 200; // 200 ms
const int SAMPLE_DURATION = 1000; // 1 second
const int NUM_SAMPLES = SAMPLE_DURATION / SAMPLE_INTERVAL;
const int SDC_PIN = P0_31;
const int SCL_PIN = P0_2;

struct SensorData {
  float accelX, accelY, accelZ;
  float magX, magY, magZ;
  float gyroX, gyroY, gyroZ;
  float temp;
};

SensorData sensorDataBuffer[NUM_SAMPLES];

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  } Serial.println("Started");

  IMU.begin();
  while (!IMU.begin()){
  } Serial.println("IMU initialized successfully");

  setupSensor();
  Wire.begin();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '1') {
      captureSensorData();
      sendSensorData();
    }
  }
}

void captureSensorData() {
  for (int i = 0; i < NUM_SAMPLES; i++) {
    IMU.read();
    sensors_event_t accel, mag, gyro, temp;
    IMU.getEvent(&accel, &mag, &gyro, &temp);

    sensorDataBuffer[i] = {
      accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
      mag.magnetic.x, mag.magnetic.y, mag.magnetic.z,
      gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
      temp.temperature / 100
    };

    delay(SAMPLE_INTERVAL);
  }
}

void sendSensorData() {
  for (int i = 0; i < NUM_SAMPLES; i++) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write((byte*)&sensorDataBuffer[i], sizeof(SensorData));
    Wire.endTransmission();
    delay(10); // Small delay to ensure data is sent properly
  }
}

void setupSensor() {
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
  */