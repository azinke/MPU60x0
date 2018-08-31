/**
    Powered by SmartTech Benin
    @author: AMOUSSOU Z. Kenneth
    @date: 19-08-2019
    @version: 1.0
    
    @external-library:
        Wire: I2C communication library
*/
#include "MPU60x0.h"

MPU60x0::MPU60x0(){
    Wire.begin();
}

/**
    function: begin
    @summary: initialize and configure the sensor
    @parameter: none
    @return: none
*/
void MPU60x0::begin(){
    configure(0, 1);
    disableSleepMode();
    setClock(1);
    setGyroFSR(2);
    setAccelFSR(2);
}

/**
    function: _write
    @summary: internal function to write a byte on Two Wire Interface (TWI)
    @parameter:
        data: byte to write
    @return: none
*/
void MPU60x0::_write(uint8_t registerAddr, uint8_t data){
    Wire.beginTransmission(ADDR);
    Wire.write(registerAddr);
    Wire.write(data);
    Wire.endTransmission(true);
}

/**
    function: _read
    @summary: internal function to read a byte from Two Wire Interface (TWI)
    @parameter:none
    @return:
        uint8_t: byte read form the sensor
*/
int8_t MPU60x0::_read(uint8_t registerAddr){
    Wire.beginTransmission(ADDR);
    Wire.write(registerAddr);
    Wire.endTransmission(true);
    
    Wire.requestFrom(ADDR, 1, true);
    //while(!Wire.available()){};
    _buffer = Wire.read();
    Wire.endTransmission(true);
    return _buffer;
}

/**
    function: _readBytes
    @summary: internal function to read some bytes from Two Wire Interface (TWI)
    @parameter:
        startAddr: beginning adress to read
        buffer: location to load data
        size: number of byte to read
    @return: none
*/
void MPU60x0::_readBytes(uint8_t startAddr, uint8_t *buffer, uint8_t size){
    Wire.beginTransmission(ADDR);
    Wire.write(startAddr);
    Wire.endTransmission(false);
    
    Wire.requestFrom(ADDR, size, true);
    uint8_t i = 0;
    while(Wire.available() && i < size){
        buffer[i++] = Wire.read();    
    }
    Wire.endTransmission(true);
}

/**
    function: configure
    @summary:
        configure the digital low pass filter and the external frame 
        synchronisation signal
    @parameter:
        ext_sync: external frame sync value
                    range: 0 - 7
        digital_low_pass_filter: filter's value
                    range: 0 - 7
                    0 or 7: disable the digital filter
                    else: set a specific cutoff frequency - see datasheet
    @return:
        uint8_t: byte read form the sensor
*/
void MPU60x0::configure(uint8_t ext_sync, uint8_t digital_low_pass_filter){
    _buffer = (ext_sync << 3) + digital_low_pass_filter;
    _write(CONFIG, _buffer);
}

/**
    function: reset
    @summary: reset the sensor
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::reset(){
    _buffer = _read(PWR_MGMT_1);
    _buffer &= ~(1<<7);
    _write(PWR_MGMT_1, _buffer);
    return 1;
}

/**
    function: whoami
    @summary: read and return the sensor I2C adress
    @parameter: none
    @return:
        uint8_t: sensor address
*/
uint8_t MPU60x0::whoami(){
    _buffer = _read(WHO_AM_I);
    _buffer &= ~(1);
    _buffer &= ~(1<<7);
    #ifndef ADDR_L
        _buffer += 1;
    #endif
    return _buffer;    
}

/**
    function: setSampleRateDivider
    @summary: set the sample rate divider for the sensor
    @parameter:
        value: value of the divider
    @return: none
*/
void MPU60x0::setSampleRateDivider(uint8_t value){
    _write(SMPLRT_DIV, value);
}

/**
    function: getGyroSampleRate
    @summary: compute and return the sample rate of the gyroscope sensor
    @parameter: none
    @return:
        float: sample rate of the gyroscope in Hertz (Hz)
*/
float MPU60x0::getGyroSampleRate(){
    _buffer = _read(SMPLRT_DIV);
    // lpf: low pass filter
    uint8_t digital_lpf =  _read(CONFIG) && 0x07;
    if((digital_lpf == 0) || (digital_lpf == 7)){
        return 8000/(1+_buffer);
    }else{
        return 1000/(1+_buffer);
    }
}

/**
    function: getAcceSampleRate
    @summary: return the sample rate of the accelerometer sensor
    @parameter: none
    @return:
        float: sample rate of the accelerometer in Hertz (Hz)
*/
float MPU60x0::getAccelSampleRate(){
    return 1000;
}

/**
    function: setGyroSFR
    @summary: set the full scale range of the gyroscope
    @parameter:
        value: full scale range value
               range: 0 - 3
    @return: none
*/
void MPU60x0::setGyroFSR(uint8_t value){
    _buffer = _read(GYRO_CONFIG);
    switch(value){
        case 0:{
            _buffer &= ~(1<<3);
            _buffer &= ~(1<<4);
            break;
        }
        case 1:{
            _buffer |= (1<<3);
            _buffer &= ~(1<<4);
            break;
        }
        case 2:{
            _buffer &= ~(1<<3);
            _buffer |= (1<<4);
            break;
        }
        case 3:{
            _buffer |= (1<<3);
            _buffer |= (1<<4);
            break;
        }
    }
    _write(GYRO_CONFIG, _buffer);  
}

/**
    function: getGyroSFR
    @summary: read the full scale range of the gyroscope
    @parameter: none
    @return:
        uint8_t: ±250°/s | ±500°/s | ±1000°/s | ±2000°/s
          range:   0          1         2         3
*/
uint8_t MPU60x0::getGyroFSR(){
    return ((_read(GYRO_CONFIG) && 0x18) >> 3);
}

/**
    function: setAccelSFR
    @summary: set the full scale range of the accelerometer
    @parameter:
        value: full scale range value
               range: 0 - 3
    @return: none
*/
void MPU60x0::setAccelFSR(uint8_t value){
    _buffer = _read(ACCEL_CONFIG);
    switch(value){
        case 0:{
            _buffer &= ~(1<<3);
            _buffer &= ~(1<<4);
            break;
        }
        case 1:{
            _buffer |= (1<<3);
            _buffer &= ~(1<<4);
            break;
        }
        case 2:{
            _buffer &= ~(1<<3);
            _buffer |= (1<<4);
            break;
        }
        case 3:{
            _buffer |= (1<<3);
            _buffer |= (1<<4);
            break;
        }
    }
    _write(ACCEL_CONFIG, _buffer);  
}

/**
    function: getAccelFSR
    @summary: read the full scale range of the accelerometer
    @parameter:
    @return:
        uint8_t: ±2g | ±4g | ±8g | ±16g
          range:  0     1     2      3
*/
uint8_t MPU60x0::getAccelFSR(){
    return ((_read(ACCEL_CONFIG) && 0x18) >> 3);
}

/**
    function: getAccelX
    @summary: read the accelerometer X-axis value from memory
    @parameter: none
    @return:
        unsigned int: value
*/
int16_t MPU60x0::getAccelX(){
    int16_t buffer = 0;
    buffer = (_read(ACCEL_XOUT_H) << 8);
    return buffer + _read(ACCEL_XOUT_L);    
}

/**
    function: getAcceY
    @summary: read the accelerometer Y-axis value from memory
    @parameter: none
    @return:
        unsigned int: value
*/
int16_t MPU60x0::getAccelY(){
    int16_t buffer = 0;
    buffer = _read(ACCEL_YOUT_H) << 8;
    return buffer + _read(ACCEL_YOUT_L);    
}

/**
    function: getAcceZ
    @summary: read the accelerometer Z-axis value from memory
    @parameter: none
    @return:
        unsigned int: value
*/
int16_t MPU60x0::getAccelZ(){
    int16_t buffer = 0;
    buffer = _read(ACCEL_ZOUT_H) << 8;
    return _buffer + _read(ACCEL_ZOUT_L);    
}

/**
    function: gyroReset
    @summary: reset the gyroscope
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::gyroReset(){
    _write(SIGNAL_PATH_RESET, 4);
    delayMicroseconds(100);
    _write(SIGNAL_PATH_RESET, 0);
    return 1;    
}

/**
    function: getGyroX
    @summary: read the gyroscope Z-axis value from memory
    @parameter: none
    @return:
        unsigned int: value
*/
int16_t MPU60x0::getGyroX(){
    int16_t buffer = 0;
    buffer = _read(GYRO_XOUT_H) << 8;
    return buffer + _read(GYRO_XOUT_L);    
}

/**
    function: getGyroY
    @summary: read the gyroscope Y-axis value from memory
    @parameter: none
    @return:
        unsigned int: value
*/
int16_t MPU60x0::getGyroY(){
    int16_t buffer = 0;
    buffer = _read(GYRO_YOUT_H) << 8;
    return buffer + _read(GYRO_YOUT_L);    
}

/**
    function: getGyroZ
    @summary: read the gyroscope Z-axis value from memory
    @parameter: none
    @return:
        unsigned int: value
*/
int16_t MPU60x0::getGyroZ(){
    int16_t buffer = 0;
    buffer = _read(GYRO_ZOUT_H) << 8;
    return buffer + _read(GYRO_ZOUT_L);    
}

/**
    function: accelReset
    @summary: reset the accelerometer
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::accelReset(){
    _write(SIGNAL_PATH_RESET, 2);
    delayMicroseconds(100);
    _write(SIGNAL_PATH_RESET, 0);
    return 1;    
}

/**
    function: getTemp
    @summary: read the temperature value from memory
    @parameter: none
    @return:
        unsigned int: value
*/
float MPU60x0::getTemp(){
    int16_t buffer = 0;
    buffer = _read(TEMP_OUT_H) << 8;
    buffer += _read(TEMP_OUT_L);
    return (float)(buffer/340 + 36.53 );    
}

/**
    function: disableTemp
    @summary: disable the temperature sensor
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::disableTemp(){
    _buffer = _read(PWR_MGMT_1);
    _buffer &= ~(1 << 3);
    _write(PWR_MGMT_1, _buffer);
    return 1;    
}

/**
    function: enableTemp
    @summary: enable the temperature sensor
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableTemp(){
    _buffer = _read(PWR_MGMT_1);
    _buffer |= (1 << 3);
    _write(PWR_MGMT_1, _buffer);
    return 1;    
}

/**
    function: gyroReset
    @summary: reset the gyroscope
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::tempReset(){
    _write(SIGNAL_PATH_RESET, 1);
    delayMicroseconds(100);
    _write(SIGNAL_PATH_RESET, 0);
    return 1;    
}

/**
    function: enableSleepMode
    @summary: enable the sleep mode of the sensor
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSleepMode(){
    _buffer = _read(PWR_MGMT_1);
    _buffer |= (1<<6);
    _write(PWR_MGMT_1, _buffer);
    return 1; 
}

/**
    function: disableSleepMode
    @summary: enable the sleep mode of the sensor
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::disableSleepMode(){
    _buffer = _read(PWR_MGMT_1);
    _buffer &= ~(1<<6);
    _write(PWR_MGMT_1, _buffer);
    return 1; 
}

/**
    function: setClock
    @summary: configure the clock source of the sensor
    @parameter:
        value: clock source number
               range: 0 ~ 7
               0: internal 8MHz
               1: PLL with X axis gyroscope reference
               2: PLL with Y axis gyroscope reference
               3: PLL with Z axis gyroscope reference
               4: PLL with external 32.768KHz reference
               5: PLL with external 19.2MHz reference
               6: reserved
               7: stop clock
    @return:
        bool: return 1 on success
*/
void MPU60x0::setClock(uint8_t value){
    _buffer = _read(PWR_MGMT_1);
    _buffer = _buffer >> 3;
    _buffer = (_buffer << 3) + (0x07 && value);
    _write(PWR_MGMT_1, _buffer);    
}

/**
    =================================================
                        FIFO
    =================================================
**/
/**
    function: enableFifo
    @summary: enable the FIFO usage on the sensors
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableFifo(){
    _buffer = _read(USER_CTRL);
    _buffer |= (1 << 6);
    _write(USER_CTRL, _buffer);
    return 1; 
}

/**
    function: disableFifo
    @summary: disable the FIFO usage on the sensors
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::disableFifo(){
    _buffer = _read(USER_CTRL);
    _buffer &= ~(1 << 6);
    _write(USER_CTRL, _buffer);
    return 1; 
}

/**
    function: resetFifo
    @summary: reset the FIFO register
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::resetFifo(){
    disableFifo();
    _buffer = _read(USER_CTRL);
    _buffer |= (1 << 2);
    _write(USER_CTRL, _buffer);
    return 1; 
}

/**
    function: enableTempFifo
    @summary: enable  temperature measurement to be send into Fifo
              Only one sensor is allowed to be enabled at a time to access FIfo.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableTempFifo(){
    _buffer = (1 << 7);
    _write(FIFO_EN, _buffer);
    return 1; 
}

/**
    function: enableXgFifo
    @summary: enable  gyroscope X measurement to be send into Fifo
              Only one sensor is allowed to be enabled at a time to access FIfo.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableXgFifo(){
    _buffer = (1 << 6);
    _write(FIFO_EN, _buffer);
    return 1; 
}

/**
    function: enableYgFifo
    @summary: enable  gyroscope Y measurement to be send into Fifo
              Only on sensor is allowed to be enabled at a time to access FIfo.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableYgFifo(){
    _buffer = (1 << 5);
    _write(FIFO_EN, _buffer);
    return 1; 
}

/**
    function: enableZgFifo
    @summary: enable  gyroscope Z measurement to be send into Fifo
              Only one sensor is allowed to be enabled at a time to access FIfo.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableZgFifo(){
    _buffer = (1 << 4);
    _write(FIFO_EN, _buffer);
    return 1; 
}

/**
    function: enableAccelFifo
    @summary: enable  accelerometer measurement to be send into Fifo
              Only one sensor is allowed to be enabled at a time to access FIfo.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableAccelFifo(){
    _buffer = (1 << 3);
    _write(FIFO_EN, _buffer);
    return 1; 
}

/**
    function: enableSlave3Fifo
    @summary: enable  I2C slave 3 data to be send into Fifo
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave3Fifo(){
    _buffer = _read(I2C_MST_CTRL);
    _buffer |= (1 << 5);
    _write(FIFO_EN, _buffer);
    return 1;
}

/**
    function: enableSlave2Fifo
    @summary: enable  I2C slave 2 data to be send into Fifo
              Only one sensor is allowed to be enabled at a time to access FIfo.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave2Fifo(){
    _buffer = (1 << 2);
    _write(FIFO_EN, _buffer);
    return 1; 
}

/**
    function: enableSlave1Fifo
    @summary: enable  I2C slave 1 data to be send into Fifo
              Only one sensor is allowed to be enabled at a time to access FIfo.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave1Fifo(){
    _buffer = (1 << 1);
    _write(FIFO_EN, _buffer);
    return 1; 
}

/**
    function: enableSlave0Fifo
    @summary: enable  I2C slave 0 data to be send into Fifo
              Only one sensor is allowed to be enabled at a time to access FIfo.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave0Fifo(){
    _buffer = 1;
    _write(FIFO_EN, _buffer);
    return 1; 
}

/**
    function: readFifo
    @summary: read out a single word (16-bit) from Fifo
    @parameter: none
    @return:
        int16_t: return the data read
*/
int16_t MPU60x0::readFifo(){
    /* read Fifo */
    int16_t buffer = _read(FIFO_R_W) << 8;
    return (buffer + _read(FIFO_R_W));
}

/**
    ====================================================
                         READ DATA
    ====================================================
**/

/**
    function: getData
    @summary: read all the measured data from the sensor
    @parameter: none
    @return:
        DATA: return a struct that contain all the required data
*/
IMU_DATA MPU60x0::getData(){
    struct {
        uint8_t accelX_H;
        uint8_t accelX_L;
        uint8_t accelY_H;
        uint8_t accelY_L;
        uint8_t accelZ_H;
        uint8_t accelZ_L;
        uint8_t temp_H;
        uint8_t temp_L;
        uint8_t gyroX_H;
        uint8_t gyroX_L;
        uint8_t gyroY_H;
        uint8_t gyroY_L;
        uint8_t gyroZ_H;
        uint8_t gyroZ_L;
    } registers;
    _readBytes(ACCEL_XOUT_H, (uint8_t*) &registers, sizeof(registers));
    IMU_DATA buffer;   
    buffer.accelX = (registers.accelX_H << 8) + registers.accelX_L;
    buffer.accelY = (registers.accelY_H << 8) + registers.accelY_L;
    buffer.accelZ = (registers.accelZ_H << 8) + registers.accelZ_L;
    
    buffer.temp = (registers.temp_H << 8) + registers.temp_L;
    
    buffer.gyroX = (registers.gyroX_H << 8) + registers.gyroX_L;
    buffer.gyroY = (registers.gyroY_H << 8) + registers.gyroY_L;
    buffer.gyroZ = (registers.gyroZ_H << 8) + registers.gyroZ_L;
    
    /**
    buffer.accelX = getAcceX();ACCEL_XOUT_H
    buffer.accelY = getAcceY();
    buffer.accelZ = getAcceZ();
    
    buffer.temp = getTemp();
    
    buffer.gyroX = getGyroX();
    buffer.gyroY = getGyroY();
    buffer.gyroZ = getGyroZ();
    **/
    return buffer;
}

/**
    function: read
    @summary: read all the measured data from the sensor and process them
    @parameter: none
    @return:
        DATA: return proccessed IMU (Inertial Measurment Unit) data
        
        Acceleration:   in m/s²
        Temperature:    in °C
        Gyrocope:       in °/s    
*/
IMU_DATA MPU60x0::read(){
    IMU_DATA buffer = getData();
    /* read FSR */
    uint8_t _gyroFsr = getGyroFSR();
    uint8_t _accelFsr = getAccelFSR();
    buffer.accelX = (float)(buffer.accelX/ACCEL_SENSITIVITY[_accelFsr]);
    buffer.accelY = (float)(buffer.accelY/ACCEL_SENSITIVITY[_accelFsr]);
    buffer.accelZ = (float)(buffer.accelY/ACCEL_SENSITIVITY[_accelFsr]);
    
    buffer.gyroX = (float)(buffer.gyroX * 10.0)/GYRO_SENSITIVITY[_gyroFsr];
    buffer.gyroY = (float)(buffer.gyroY * 10.0)/GYRO_SENSITIVITY[_gyroFsr];
    buffer.gyroZ = (float)(buffer.gyroZ * 10.0)/GYRO_SENSITIVITY[_gyroFsr];

    buffer.temp = (float)(buffer.temp/340 + 36.53);
    return buffer;
}

/**
    ============================================================
                            SELF TEST
    ============================================================
**/

/**
    function: gyroXSelfTest
    @summary: read the gyroscope self test X value from the sensor
    @parameter: none
    @return:
        uint8_t: return the self test value
*/
uint8_t MPU60x0::gyroXSelfTest(){
    uint8_t test_value = 0;
    uint8_t fsr = getGyroFSR();
    setGyroFSR(0); // set FSR to ±250°/s
    // Enable selt test of gyro X-axis
    _buffer = _read(GYRO_CONFIG);
    _buffer |= (1 << 7);
    _write(GYRO_CONFIG, _buffer);
    test_value = _read(SELF_TEST_X) && 0x1F;
    _buffer &= ~(1 << 7);
    _write(GYRO_CONFIG, _buffer);
    setGyroFSR(fsr); // set FSR to the default value
    return test_value;
}

/**
    function: gyroYSelfTest
    @summary: read the gyroscope self test Y value from the sensor
    @parameter: none
    @return:
        uint8_t: return the self test value
*/
uint8_t MPU60x0::gyroYSelfTest(){
    uint8_t test_value = 0;
    uint8_t fsr = getGyroFSR();
    setGyroFSR(0); // set FSR to ±250°/s
    // Enable selt test of gyro X-axis
    _buffer = _read(GYRO_CONFIG);
    _buffer |= (1 << 6);
    _write(GYRO_CONFIG, _buffer);
    test_value = _read(SELF_TEST_Y) && 0x1F;
    _buffer &= ~(1 << 6);
    _write(GYRO_CONFIG, _buffer);
    setGyroFSR(fsr); // set FSR to the default value
    return test_value;
}

/**
    function: gyroZSelfTest
    @summary: read the gyroscope self test Z value from the sensor
    @parameter: none
    @return:
        uint8_t: return the self test value
*/
uint8_t MPU60x0::gyroZSelfTest(){
    uint8_t test_value = 0;
    uint8_t fsr = getGyroFSR();
    setGyroFSR(0); // set FSR to ±250°/s
    // Enable selt test of gyro X-axis
    _buffer = _read(GYRO_CONFIG);
    _buffer |= (1 << 5);
    _write(GYRO_CONFIG, _buffer);
    test_value = _read(SELF_TEST_Z) && 0x1F;
    _buffer &= ~(1 << 5);
    _write(GYRO_CONFIG, _buffer);
    setGyroFSR(fsr); // set FSR to the default value
    return test_value;
}

/**
    function: accelXSelfTest
    @summary: read the accelerometer self test X value from the sensor
    @parameter: none
    @return:
        uint8_t: return the self test value
*/
uint8_t MPU60x0::accelXSelfTest(){
    uint8_t test_value = 0;
    uint8_t fsr = getAccelFSR();
    setAccelFSR(2); // set FSR to ±8g
    // Enable selt test of gyro X-axis
    _buffer = _read(ACCEL_CONFIG);
    _buffer |= (1 << 7);
    _write(GYRO_CONFIG, _buffer);
    test_value = (_read(SELF_TEST_X) && 0xE0) >> 3;
    test_value += (_read(SELF_TEST_A) && 0x30) >> 4;
    _buffer &= ~(1 << 7);
    _write(GYRO_CONFIG, _buffer);
    setAccelFSR(fsr); // set FSR to the default value
    return test_value;
}

/**
    function: accelYSelfTest
    @summary: read the accelerometer self test Y value from the sensor
    @parameter: none
    @return:
        uint8_t: return the self test value
*/
uint8_t MPU60x0::accelYSelfTest(){
    uint8_t test_value = 0;
    uint8_t fsr = getAccelFSR();
    setAccelFSR(2); // set FSR to ±8g
    // Enable selt test of gyro X-axis
    _buffer = _read(ACCEL_CONFIG);
    _buffer |= (1 << 6);
    _write(GYRO_CONFIG, _buffer);
    test_value = (_read(SELF_TEST_Y) && 0xE0) >> 3;
    test_value += (_read(SELF_TEST_A) && 0x0C) >> 2;
    _buffer &= ~(1 << 6);
    _write(GYRO_CONFIG, _buffer);
    setAccelFSR(fsr); // set FSR to the default value
    return test_value;
}

/**
    function: accelZSelfTest
    @summary: read the accelerometer self test Y value from the sensor
    @parameter: none
    @return:
        uint8_t: return the self test value
*/
uint8_t MPU60x0::accelZSelfTest(){
    uint8_t test_value = 0;
    uint8_t fsr = getAccelFSR();
    setAccelFSR(2); // set FSR to ±8g
    // Enable selt test of gyro X-axis
    _buffer = _read(ACCEL_CONFIG);
    _buffer |= (1 << 5);
    _write(GYRO_CONFIG, _buffer);
    test_value = (_read(SELF_TEST_Y) && 0xE0) >> 3;
    test_value += (_read(SELF_TEST_A) && 0x03);
    _buffer &= ~(1 << 5);
    _write(GYRO_CONFIG, _buffer);
    setAccelFSR(fsr); // set FSR to the default value
    return test_value;
}


