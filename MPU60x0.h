/**
    Powered by SmartTech Benin
    @author: AMOUSSOU Z. Kenneth
    @date: 19-08-2019
    @version: 1.0
    
    @external-library:
        Wire: I2C communication library
*/
#ifndef H_MPU60x0
#define H_MPU60x0

#include <Wire.h>

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

#define DEBUG

/** Device I2C address **/
#define ADDR_L
#ifdef ADDR_L
    #define ADDR 0x68   //AD0: Low
#else
    #define ADDR 0x69   //AD0: High
#endif

#define SELF_TEST_X         13
#define SELF_TEST_Y         14
#define SELF_TEST_Z         15
#define SELF_TEST_A         16

#define SMPLRT_DIV          25
#define CONFIG              26
#define GYRO_CONFIG         27
#define ACCEL_CONFIG        28
#define FIFO_EN             35

#define I2C_MST_CTRL        36
#define I2C_SLV0_ADDR       37
#define I2C_SLV0_REG        38
#define I2C_SLV0_CTRL       39
#define I2C_SLV1_ADDR       40
#define I2C_SLV1_REG        41
#define I2C_SLV1_CTRL       42
#define I2C_SLV2_ADDR       43
#define I2C_SLV2_REG        44
#define I2C_SLV2_CTRL       45
#define I2C_SLV3_ADDR       46
#define I2C_SLV3_REG        47
#define I2C_SLV3_CTRL       48
#define I2C_SLV4_ADDR       49
#define I2C_SLV4_REG        50
#define I2C_SLV4_DO         51
#define I2C_SLV4_CTRL       52
#define I2C_SLV4_DI         53
#define I2C_MST_STATUS      54

#define INT_PIN_CFG         55
#define INT_ENABLE          56
#define INT_STATUS          58

#define ACCEL_XOUT_H        59
#define ACCEL_XOUT_L        60
#define ACCEL_YOUT_H        61
#define ACCEL_YOUT_L        62
#define ACCEL_ZOUT_H        63
#define ACCEL_ZOUT_L        64

#define TEMP_OUT_H          65
#define TEMP_OUT_L          66

#define GYRO_XOUT_H         67
#define GYRO_XOUT_L         68
#define GYRO_YOUT_H         69
#define GYRO_YOUT_L         70
#define GYRO_ZOUT_H         71
#define GYRO_ZOUT_L         72

#define EXT_SENS_DATA_00    73
#define EXT_SENS_DATA_01    74
#define EXT_SENS_DATA_02    75
#define EXT_SENS_DATA_03    76
#define EXT_SENS_DATA_04    77
#define EXT_SENS_DATA_05    78
#define EXT_SENS_DATA_06    79
#define EXT_SENS_DATA_07    80
#define EXT_SENS_DATA_08    81
#define EXT_SENS_DATA_09    82
#define EXT_SENS_DATA_10    83
#define EXT_SENS_DATA_11    84
#define EXT_SENS_DATA_12    85
#define EXT_SENS_DATA_13    86
#define EXT_SENS_DATA_14    87
#define EXT_SENS_DATA_15    88
#define EXT_SENS_DATA_16    89
#define EXT_SENS_DATA_17    90
#define EXT_SENS_DATA_18    91
#define EXT_SENS_DATA_19    92
#define EXT_SENS_DATA_20    93
#define EXT_SENS_DATA_21    94
#define EXT_SENS_DATA_22    95
#define EXT_SENS_DATA_23    96

#define I2C_SLV0_DO         99
#define I2C_SLV1_DO         100
#define I2C_SLV2_DO         101
#define I2C_SLV3_DO         102

#define I2C_MST_DELAY_CTRL  103
#define SIGNAL_PATH_RESET   104
#define USER_CTRL           106
#define PWR_MGMT_1          107
#define PWR_MGMT_2          108

#define FIFO_COUNT_H        114
#define FIFO_COUNT_L        115
#define FIFO_R_W            116

#define WHO_AM_I            117

#define GRAVITY             9.81


typedef struct{
    float accelX = 0;
    float accelY = 0;
    float accelZ = 0;
    
    float temp = 0;
    
    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;
} IMU_DATA; 

class MPU60x0{
    public:
        MPU60x0();
        void begin();
        
        void configure(uint8_t ext_sync, uint8_t digital_low_pass_filter);
        bool reset();   // Reset the sensor
        uint8_t whoami();
        
        void setSampleRateDivider(uint8_t value);
        float getGyroSampleRate();
        float getAccelSampleRate();
        
        void setGyroFSR(uint8_t value); // FSR: Full Scale Range
        uint8_t getGyroFSR();
        void setAccelFSR(uint8_t value);
        uint8_t getAccelFSR();
        
        int16_t getGyroX();
        int16_t getGyroY();
        int16_t getGyroZ();
        bool gyroReset();
        
        int16_t getAccelX();
        int16_t getAccelY();
        int16_t getAccelZ();
        bool accelReset();
        
        float getTemp();
        bool disableTemp();
        bool enableTemp();
        bool tempReset();
        
        bool enableGyroTest();
        bool enableAccelTest();
        
        bool enableXgFifo();
        bool enableYgFifo();
        bool enableZgFifo();
        bool enableAccelFifo();
        bool enableTempFifo();
        bool enableSlave0Fifo();
        bool enableSlave1Fifo();
        bool enableSlave2Fifo();
        bool enableSlave3Fifo();
        int16_t readFifo();
        bool resetFifo();
        bool enableFifo();
        bool disableFifo();
        
        bool enableSleepMode();
        bool disableSleepMode();
        
        void setClock(uint8_t value);
        
        uint8_t gyroXSelfTest();
        uint8_t gyroYSelfTest();
        uint8_t gyroZSelfTest();
        
        uint8_t accelXSelfTest();
        uint8_t accelYSelfTest();
        uint8_t accelZSelfTest();
        
        IMU_DATA getData(); // get all the data from the sensor
        IMU_DATA read();    // compute sensor data and return them back
        
        bool i2cMultiMasterEnable();
        bool i2cMultiMasterDisable();
        bool i2cMasterClock(uint8_t clock);
        bool i2cSlave0Enable();
        bool i2cSlave0Disable();
    private:
        void _write(uint8_t registerAddr, uint8_t data);
        int8_t _read(uint8_t registerAddr);
        void _readBytes(uint8_t startAddr, uint8_t *buffer, uint8_t size);
        uint8_t _buffer;
        const uint16_t  ACCEL_SENSITIVITY[4] = { 16384 , 8192, 4096 , 2048 };
        const uint16_t  GYRO_SENSITIVITY[4] = { 1310 , 655 , 328 , 164 }; // x10
};

#endif
