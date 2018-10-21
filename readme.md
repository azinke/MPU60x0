# Inertial Measurement Unit (IMU)

The current library is based on the `PMU60x0` IMU from `InvenSense Inc.`. 
Developped around the `GY-521` DIY module, this library is tested this module 
which get the MPU6050 IC.

The MPU-6050 sensor module contains an accelerometer and a gyro in a single 
chip. It is very accurate, as it contains 16-bits analog to digital conversion 
hardware for each channel. Therefore it captures the x, y, and z channel at the 
same time. The sensor uses the I2C-bus to interface with the Arduino.

## Sensor features

- Use the chip: MPU-6050.
- Power supply: 3-5v (internal low dropout regulator).
- Communication modes: standard IIC communications protocol.
- Chip built-in 16bit AD converter, 16-bit data output.
- Immersion Gold PCB machine welding process to ensure quality.
- Tri-Axis angular rate sensor (gyro) with a sensitivity up to 131 LSBs/dps 
and a fullscale range of ±250, ±500, ±1000, and ±2000dps
- Tri-Axis accelerometer with a programmable full scale range of ±2g, ±4g, ±8g
 and ±16g

## How to use the library

While preparing a more detailed Wiki page, some basics example of usage are in 
the `examples` folder. However the table below show the most important data 
structure and methods available with this library.

### Basics

Library | Descriptions
:-------|:-----------
`MPU60x0` | Constructor
`IMU_DATA` | Data structure used to hold sensor's data

As shown below, the `IMU_DATA` is a `struct` which contains all data available 
with the sensor.

```C++
    struct{
        float accelX;
        float accelY;
        float accelZ;
        
        float temp;
        
        float gyroX;
        float gyroY;
        float gyroZ;
    } IMU_DATA
```

### Useful methods

Methods | Descriptions
:-------|:--------------
`begin()` | Initialize the sensor with default configurations
`whoami()` | Read the sensor's I2C address - Could be used to check correct wiring of the sensor
`read()` | read pre-processed data from the sensor (angular velocity in `°/s`, acceleration in `m/s²`, temperature in `°C`)
`getData()` | read raw data from the sensor
`getGyroX()` | read gyroscope x-axis angular velocity
`getGyroY()` | read gyroscope y-axis angular velocity
`getGyroY()` | read gyroscope z-axis angular velocity
`getAccelX()` | get acceleration along the x-axis
`getAccelY()` | get acceleration along the y-axis
`getAccelZ()` | get acceleration along the z-axis
`getTemp()` | get temperature

