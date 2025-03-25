/*#include <Arduino.h>
#include <Wire.h>

///This is the writer board
 struct SensorData {
  float accelX, accelY, accelZ;
  float magX, magY, magZ;
  float gyroX, gyroY, gyroZ;
  float temp;  
 };

 const int I2C_ADDRESS = 8;



void receiveEvent(int howMany);

void setup() {
 
  Serial.begin(9600);
  while(!Serial);
  Wire.begin(I2C_ADDRESS);               // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event

}

void loop() {

}



void receiveEvent(int howMany) {
    SensorData data;
    while (Wire.available() >= sizeof(SensorData)) {
      Wire.readBytes((char*)&data, sizeof(SensorData));
      Serial.print("Accel X: "); Serial.print(data.accelX); Serial.print(" m/s^2");
      Serial.print("\tY: "); Serial.print(data.accelY); Serial.print(" m/s^2 ");
      Serial.print("\tZ: "); Serial.print(data.accelZ); Serial.println(" m/s^2 ");
  
      Serial.print("Mag X: "); Serial.print(data.magX);   Serial.print(" uT");
      Serial.print("\tY: "); Serial.print(data.magY);     Serial.print(" uT");
      Serial.print("\tZ: "); Serial.print(data.magZ);     Serial.println(" uT");
  
      Serial.print("Gyro X: "); Serial.print(data.gyroX);   Serial.print(" rad/s");
      Serial.print("\tY: "); Serial.print(data.gyroY);      Serial.print(" rad/s");
      Serial.print("\tZ: "); Serial.print(data.gyroZ);      Serial.println(" rad/s");
      Serial.print("Temp: "); Serial.print(data.temp); Serial.println(" deg C");
  
      Serial.println();
    }
  }*/