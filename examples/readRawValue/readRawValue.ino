/**
    @author: AMOUSSOU Z. Kenneth
    @date: 07-09-2018
    @summary: Example program to retrieve raw data from MPU60x0 sensor
    @chip: tested on MPU6050
    @platform: Arduino
*/
#include <MPU60x0.h>
#include <Wire.h>

MPU60x0 mySensor;

IMU_DATA data;

void setup() {  
  Serial.begin(9600);
  mySensor.begin();
  
  /*
    Read the device I2C adress
    Should be:
        0x68    : when AD0 is connceted to GND
        0x69    : when AD0 is connected to VCC

    # Check your connection if you get different value. #
  */
  /* Check correct wiring of the sensor */
  while(mySensor.whoami() != ADDR){
    Serial.println("MPU60x0 not found!");
    delay(200);     // 200ms
  }
  Serial.print("Sensor ADDR: 0x");
  Serial.println(mySensor.whoami(), HEX);
}

void loop() {
  /**
    Read raw data from the IMU sensor
    
    These data are quite meaningless.
    
    data:
        struct{
            float accelX;
            float accelY;
            float accelZ;
            
            float temp;
            
            float gyroX;
            float gyroY;
            float gyroZ;
        }
    Use this data structure to retrieve the information needed
  */
  data = mySensor.getData();

  Serial.print("Gyroscope x, y, z: ");
  Serial.print(data.gyroX);
  Serial.print(", ");
  Serial.print(data.gyroY);
  Serial.print(", ");
  Serial.println(data.gyroZ);

  Serial.print("Accelerometer x, y, z: ");
  Serial.print(data.accelX);
  Serial.print(", ");
  Serial.print(data.accelY);
  Serial.print(", ");
  Serial.println(data.accelZ);

  Serial.print("Temperature: ");
  Serial.println(data.temp);

  Serial.println();
  
  delay(1000);

}
