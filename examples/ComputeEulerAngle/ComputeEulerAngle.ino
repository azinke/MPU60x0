/**
    @author: AMOUSSOU Z. Kenneth
    @date: 07-10-2018
    @summary: Example program to compute Eurler's angle based on MPU60x0 sensor
    @chip: tested on MPU6050
    @platform: Arduino
*/
#include <MPU60x0.h>

#define DT 0.004032// 0.002048

/**
    @function prototype
**/ 
void initTimer2();

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
volatile IMU_DATA data;

/**
    Variables
*/
unsigned long time;
float dt = 0.0;   // tiny time difference in ms
float acceleration = 0.0;
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

float accel_pitch = 0.0;
float accel_roll = 0.0;

/* get gyroscope offset */
float gyro_x_offset = 0;
float gyro_y_offset = 0;
float gyro_z_offset = 0;

unsigned int i = 0;

unsigned long counter = 0; // used to delay the display rate of data

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
  for(i = 0; i <= 2000; i++){
    data = mySensor.read();
    gyro_x_offset += data.gyroX;
    gyro_y_offset += data.gyroY;
    gyro_z_offset += data.gyroZ;
    delay(1);   // 1 ms
  }
  gyro_x_offset /= i;
  gyro_y_offset /= i;
  gyro_z_offset /= i;
  // initer timer
  initTimer2();
}

void loop(){
    
    
    /*
    accel_pitch = (float)atan2(data.accelY, data.accelZ);
    accel_pitch *= 180;
    accel_pitch /= PI;
    pitch = 0.98 * (pitch + (data.gyroY - gyro_y_offset) * (dt/1000000.0)) + 0.02 * accel_pitch;
    
    accel_roll = (float)atan2(data.accelX, sqrt(data.accelY * data.accelY + data.accelZ * data.accelZ));
    accel_roll *= 180;
    accel_roll /= PI;
    roll = 0.98 * (roll + (data.gyroZ - gyro_z_offset) * (dt/1000000.0)) + 0.02 * accel_roll;
    */
    counter++;
    if(counter >= 400){
        Serial.println("Yaw: " + String(yaw) + "°");
        Serial.println("Pitch: " + String(pitch) + "°");
        Serial.println("Roll: " + String(roll) + "°");
        Serial.println();
        counter = 0;
    }
    delay(1);   // 1 ms delay
}

/**
    @function: initTimer2
    @summary: configure the timer 2 module of the ATMEGA328 µC
    @parameters: none
    @return: none
**/
void initTimer2(){
    cli();
    TCCR2A = 0x00;
    ASSR &= ~(1<<5); // clk_io
    /**
        Fosc = 16MHz
        prescaler = 1024
        256 steps -> 16.384ms
        32 steps  -> 2.048ms (~ 488.28125 Hz)
        63 steps  -> 4.032ms (~ 248.015873 Hz)
    **/
    TCNT2 = (255 - 63);
    GTCCR &= ~(1<<7);   // disable timer 2 prescaler reset
    TCCR2B = 0x07;
    TIFR2 = 0x00;
    TIMSK2 = 0x01;
    sei();
}

SIGNAL(TIMER2_OVF_vect){
    cli();                      // disable interrupt
    TIFR2 = 0x00;               // clear interrupt flag
    data = mySensor.read();     // read new data
    // Process new data to get Euler's angles
    yaw += (data.gyroZ - gyro_z_offset) * DT;
    pitch += (data.gyroY - gyro_y_offset) * DT;
    roll += (data.gyroX - gyro_x_offset) * DT;    
    TCNT2 = (255 - 63);         // reset timer2 counter register
    sei();                      // enable interrupt
}
