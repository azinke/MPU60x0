/**
    @author: AMOUSSOU Z. Kenneth
    @date: 07-10-2018
    @summary: Example program to compute Eurler's angle based on MPU60x0 sensor
    @chip: tested on MPU6050
    @platform: Arduino
*/
#include <MPU60x0.h>

MPU60x0 mySensor;

/** Data structure

    struct{
        float accelX = 0;
        float accelY = 0;
        float accelZ = 0;
        
        float temp = 0;
        
        float gyroX = 0;
        float gyroY = 0;
        float gyroZ = 0;
    } IMU_DATA 
*/
IMU_DATA data;

/**
    Variables
*/
unsigned long time;
float dt = 0.0;   // tiny time difference in ms
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

/* get gyroscope offset */
float gyro_x_offset = 0;
float gyro_y_offset = 0;
float gyro_z_offset = 0;

unsigned int i = 0;

void setup(){
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
  
  /* get Gyroscope's offset */
  for(i = 0; i <= 100; i++){
    data = mySensor.read();
    gyro_x_offset += data.gyroX;
    gyro_y_offset += data.gyroY;
    gyro_z_offset += data.gyroZ;
    delay(100);   // 100 ms
  }
  gyro_x_offset /= i;
  gyro_y_offset /= i;
  gyro_z_offset /= i;
  time = millis();
}

void loop(){
    data = mySensor.read();
    dt = millis() - time;
    yaw += (data.gyroX - gyro_x_offset) * (dt * 0.001);
    pitch += (data.gyroY - gyro_y_offset) * (dt * 0.001);
    yaw += (data.gyroZ - gyro_z_offset) * (dt * 0.001);
    time = millis();
    
    Serial.println("Yaw: " + String(yaw) + "°");
    Serial.println("Pitch: " + String(pitch) + "°");
    Serial.println("Roll: " + String(roll) + "°");
    Serial.println();
}
