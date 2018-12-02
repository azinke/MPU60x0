/**
    @author: AMOUSSOU Z. Kenneth
    @date: 07-09-2018
    @summary: Example program to retrieve computed data from MPU60x0 sensor
    @chip: tested on MPU6050
    @platform: Arduino
*/
#include <MPU60x0.h>
#include <Wire.h>

MPU60x0 mySensor;

IMU_DATA data;

void setup() {
  /* Initialize serial communication */
  Serial.begin(9600);
  
  /* Initialize sensor for measurement 
     Disable sleep mode + set up full scale range
  */
  mySensor.begin();
  
  /* Check correct wiring of the sensor */
  while(mySensor.whoami() != ADDR){
    Serial.println("MPU60x0 not found!");
    delay(200);     // 200ms
  }
  /* Read device I2C adress */
  Serial.print("Sensor ADDR: 0x");
  Serial.println(mySensor.whoami(), HEX);
}

void loop() {
  /**
    Read the IMU sensor computed data
    
    Gyroscope:      °/s (degree per second)
    Accelerometer:  m/s²
    Temperature:    °C 
  */
  data = mySensor.read();

  Serial.print("Gyroscope x, y, z: ");
  Serial.print(data.gyroX);
  Serial.print(" °/s , ");
  Serial.print(data.gyroY);
  Serial.print(" °/s , ");
  Serial.print(data.gyroZ);
  Serial.println(" °/s");

  Serial.print("Accelerometer x, y, z: ");
  Serial.print(data.accelX);
  Serial.print(" m/s² , ");
  Serial.print(data.accelY);
  Serial.print(" m/s² , ");
  Serial.print(data.accelZ);
  Serial.println(" m/s²");

  Serial.print("Temperature: ");
  Serial.print(data.temp);
  Serial.println(" °C");

  Serial.println();
  delay(1000);
}
