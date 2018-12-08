/**
    @author: AMOUSSOU Z. Kenneth
    @date: 19-08-2018
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
    // disable sleep mode
    disableSleepMode();
    setClock(0); // internal 8MHz
    setGyroFSR(0);
    setAccelFSR(0);
    _gyroFsr = 0;
    _accelFsr = 0;
    // read sensitivity from PROGMEM
    _accel_sensitivity = ACCEL_SENSITIVITY[_accelFsr];
    _gyro_sensitivity = GYRO_SENSITIVITY[_gyroFsr];
    // disable sensor's sensitivity read on every data pulling
    _isFSRUpdated = false;
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
uint8_t MPU60x0::_read(uint8_t registerAddr){
    Wire.beginTransmission(ADDR);
    Wire.write(registerAddr);
    Wire.endTransmission(false);
    
    Wire.requestFrom(ADDR, 1, true);
    _buffer = Wire.read();
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
    _buffer = (ext_sync << 3) | digital_low_pass_filter;
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
    _isFSRUpdated = true;
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
    _isFSRUpdated = true;
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
    return buffer | _read(ACCEL_XOUT_L);    
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
    return buffer | _read(ACCEL_YOUT_L);    
}

/**
    function: getAccelZ
    @summary: read the accelerometer Z-axis value from memory
    @parameter: none
    @return:
        unsigned int: value
*/
int16_t MPU60x0::getAccelZ(){
    int16_t buffer = 0;
    buffer = _read(ACCEL_ZOUT_H) << 8;
    return buffer | _read(ACCEL_ZOUT_L);    
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
    return buffer | _read(GYRO_XOUT_L);    
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
    return buffer | _read(GYRO_YOUT_L);    
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
    return buffer | _read(GYRO_ZOUT_L);    
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
    buffer = _read(TEMP_OUT_H);
    buffer <<= 8;
    buffer |= _read(TEMP_OUT_L);
    #ifdef DEBUG
        Serial.print("raw temperature: ");
        Serial.println(buffer);
    #endif
    return ((float)buffer/340.0 + 36.53 );    
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
    _buffer = (_buffer << 3) | (0x07 && value);
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
    return (buffer | _read(FIFO_R_W));
}

/**
    ====================================================
                         READ DATA
    ====================================================
**/

/**
    function: getData
    @summary: read all the measured data from the sensor. These are raw data
    @parameter: none
    @return:
        DATA: return a struct that contain all the required data
              These are not really meaning full.
              
    # Should use this function if you know how to handle raw value #
*/
IMU_DATA MPU60x0::getData(){
    struct {
        int8_t accelX_H;
        uint8_t accelX_L;
        int8_t accelY_H;
        uint8_t accelY_L;
        int8_t accelZ_H;
        uint8_t accelZ_L;
        int8_t temp_H;
        uint8_t temp_L;
        int8_t gyroX_H;
        uint8_t gyroX_L;
        int8_t gyroY_H;
        uint8_t gyroY_L;
        int8_t gyroZ_H;
        uint8_t gyroZ_L;
    } registers;
    _readBytes(ACCEL_XOUT_H, (uint8_t*) &registers, sizeof(registers));
    IMU_DATA buffer;   
    buffer.accelX = (registers.accelX_H << 8) | registers.accelX_L;
    buffer.accelY = (registers.accelY_H << 8) | registers.accelY_L;
    buffer.accelZ = (registers.accelZ_H << 8) | registers.accelZ_L;
    
    buffer.temp = (registers.temp_H << 8) | registers.temp_L;
    
    buffer.gyroX = (registers.gyroX_H << 8) | registers.gyroX_L;
    buffer.gyroY = (registers.gyroY_H << 8) | registers.gyroY_L;
    buffer.gyroZ = (registers.gyroZ_H << 8) | registers.gyroZ_L;
    return buffer;
}

/**
    function: read
    @summary: read all the measured data from the sensor and process them
    @parameter: none
    @return:
        DATA: return proccessed IMU (Inertial Measurment Unit) data
        
        Acceleration:   in g (with g in m/s²) g: earth gravity
        Temperature:    in °C
        Gyrocope:       in °/s
    @dependency: pgmspace.h
                 pgm_read_word: to read sensor's sensitivity from PROGMEM 
*/
IMU_DATA MPU60x0::read(){
    IMU_DATA buffer = getData();
    /* read FSR */
    if(_isFSRUpdated){
        _gyroFsr = getGyroFSR();
        _accelFsr = getAccelFSR();
        _accel_sensitivity = ACCEL_SENSITIVITY[_accelFsr];
        _gyro_sensitivity = GYRO_SENSITIVITY[_gyroFsr];
    }
    #ifdef DEBUG
        Serial.print("Accel sensitivity: ");
        Serial.println(_accel_sensitivity);
        Serial.print("Gyro sensitivity: ");
        Serial.println(_gyro_sensitivity);
    #endif
    buffer.accelX = (float)(buffer.accelX/_accel_sensitivity);
    buffer.accelY = (float)(buffer.accelY/_accel_sensitivity);
    buffer.accelZ = (float)(buffer.accelZ/_accel_sensitivity);
    
    buffer.gyroX = (float)((buffer.gyroX * 10.0)/_gyro_sensitivity);
    buffer.gyroY = (float)((buffer.gyroY * 10.0)/_gyro_sensitivity);
    buffer.gyroZ = (float)((buffer.gyroZ * 10.0)/_gyro_sensitivity);

    buffer.temp = ((float)buffer.temp/340.0) + 36.53;
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
    test_value |= (_read(SELF_TEST_A) && 0x30) >> 4;
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
    test_value |= (_read(SELF_TEST_A) && 0x0C) >> 2;
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
    test_value |= (_read(SELF_TEST_A) && 0x03);
    _buffer &= ~(1 << 5);
    _write(GYRO_CONFIG, _buffer);
    setAccelFSR(fsr); // set FSR to the default value
    return test_value;
}

/**
    ============================================================
                            I2C MASTER
    ============================================================
**/

/**
    function: i2cMultiMasterEnable
    @summary: enable multi master capability on the sensor
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::i2cMultiMasterEnable(){
    _buffer = _read(I2C_MST_CTRL);
    _buffer |= (1 << 7);
    _write(I2C_MST_CTRL, _buffer);
    return 1;
}

/**
    function: i2cMultiMasterDisable
    @summary: enable multi master capability on the sensor
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::i2cMultiMasterDisable(){
    _buffer = _read(I2C_MST_CTRL);
    _buffer &= ~(1 << 7);
    _write(I2C_MST_CTRL, _buffer);
    return 1;
}

/**
    function: i2cMasterClok
    @summary: enable multi master capability on the sensor
    @parameter:
        clock: set the master clock by choosing between 0 and 15
            [00] I2C_MST_CLK_0: 348kHz
            [01] I2C_MST_CLK_1: 333kHz
            [02] I2C_MST_CLK_2: 320kHz
            [03] I2C_MST_CLK_3: 308kHz
            [04] I2C_MST_CLK_4: 296kHz
            [05] I2C_MST_CLK_5: 286kHz
            [06] I2C_MST_CLK_6: 276kHz
            [07] I2C_MST_CLK_7: 267kHz
            [08] I2C_MST_CLK_8: 258kHz
            [09] I2C_MST_CLK_9: 500kHz
            [10] I2C_MST_CLK_10: 471kHz
            [11] I2C_MST_CLK_11: 444kHz
            [12] I2C_MST_CLK_12: 421kHz
            [13] I2C_MST_CLK_13: 400kHz
            [14] I2C_MST_CLK_14: 381kHz
            [15] I2C_MST_CLK_15: 364kHz
    @return:
        bool: return true on success
*/
bool MPU60x0::i2cMasterClock(uint8_t clock){
	// TODO: set a contrain checking for clock value
    _buffer = _read(I2C_MST_CTRL);
    _buffer = (_buffer && 0xF0) | clock;    
    _write(I2C_MST_CTRL, _buffer);
    return 1;
}

/**
    function: i2cSetMasterDelay
    @summary: configures the reduced access rate of I2C slaves relative to 
              the Sample Rate
    @parameter:
        divider: the value to compute the slaves access rate
        
        Slave Access Sample Rate = 1 / (1 + divider)
    @return:
        bool: return 1 on success
*/
bool MPU60x0::i2cSetMasterDelay(uint8_t divider){
    _buffer = _read(I2C_SLV4_CTRL);
    _buffer = (_buffer && 0x70) | (divider && 0x1F);
    _write(I2C_SLV4_CTRL, _buffer);
    return 1;
}

/**
    function: i2cGetMasterDelay
    @summary: configures the reduced access rate of I2C slaves relative to 
              the Sample Rate
    @parameter: none
    @return:
        uint8_t: return the delay
*/
uint8_t MPU60x0::i2cGetMasterDelay(){
    return (_read(I2C_SLV4_CTRL) && 0x1F);
}

/**
    function: i2cGetSampleRate
    @summary: compute de sample rate on which slaves are accessed
              Slave Access Sample Rate = 1 / (1 + I2C_MST_DLY[4:0])
    @see: i2cGetMasterDelay - to get I2C_MST_DLY[4:0]
    @parameter: none
    @return:
        float: return the sample rate of slaves in Hertz (Hz)
*/
float MPU60x0::i2cGetSampleRate(){
    return (float)(1 / (1 + i2cGetMasterDelay()));
}

/**
    function: i2cMasterStatus
    @summary: read the status of the interrupt generating signals in the I2C 
              Master within the MPU60X0
    @parameter: none
    @return:
        uint8_t: 
            [0]: I2C slave 0 NACK (sets to 1 when the I2C Master receives a 
                                    NACK in a transaction with Slave 0.)
            [1]: I2C slave 1 NACK (sets to 1 when the I2C Master receives a 
                                    NACK in a transaction with Slave 1.)
            [2]: I2C slave 2 NACK (sets to 1 when the I2C Master receives a 
                                    NACK in a transaction with Slave 2.)
            [3]: I2C slave 3 NACK (sets to 1 when the I2C Master receives a 
                                    NACK in a transaction with Slave 3.)
            [4]: I2C slave 4 NACK (sets to 1 when the I2C Master receives a 
                                    NACK in a transaction with Slave 4.)
            [5]: I2C_LOST_ARB (sets to 1 when the I2C Master lost arbitration of 
                                    the auxillary I2C master)
            [6]: I2C_SLV4_DONE (sets to 1 when the slave 4 transaction has 
                                completed)
            [7]: PASS_THROUGH (This bit reflects the status of the FSYNC interrupt 
                               from an external device into the MPU-60X0.)

           # It is important to notice that this register (I2C_MST_STATUS) is 
             automatically cleared on reading operation #
            
*/
uint8_t MPU60x0::i2cMasterStatus(){
    return _read(I2C_MST_STATUS);
}

// Slave 0
/**
    function: slave0Enable
    @summary: enable slave 0 for writing and reading operations
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::slave0Enable(){
    _buffer = _read(I2C_SLV0_CTRL);
    _buffer |= (1 << 7);
    _write(I2C_SLV0_CTRL, _buffer);
    return 1;
}

/**
    function: slave0Disable
    @summary: disable slave 0
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::slave0Disable(){
    _buffer = _read(I2C_SLV0_CTRL);
    _buffer &= ~(1 << 7);
    _write(I2C_SLV0_CTRL, _buffer);
    return 1;
}

/**
    function: slave0Write
    @summary: write a single byte on slave 0
    @parameter:
        address: The slave 0 device address
        start_register: the address of the first register to be written
        size: The number of byte to write
    @return:
        uint16_t: number of bytes written on the slave device

    TODO test this methods to be sure that it's working
*/
void MPU60x0::slave0Write(uint8_t address, uint8_t _register, uint8_t data){
    _write(I2C_SLV0_ADDR, (address && 0x7F));
    _write(I2C_SLV0_REG, _register);
    _write(I2C_SLV0_DO, data);
    _write(I2C_SLV0_CTRL, 0x81); // Slave 0 enable + data lenght = 1
    _write(I2C_SLV0_CTRL, 0x00);
}

/**
    function: setSlave0DataLength
    @summary: set slave 0 data length for reading or writing operation
    @parameter:
        length: number of byte to read or write
    @return: none
*/
void MPU60x0::setSlave0DataLength(uint8_t length){
    _buffer = _read(I2C_SLV0_CTRL);
    _buffer |= (length && 0x0F);
    _write(I2C_SLV0_CTRL, _buffer);
}

/**
    function: getSlave0DataLenght
    @summary: get slave 0 data length for reading or writing operation
    @parameter: none
    @return:
        uint8_t: number of byte
*/
uint8_t MPU60x0::getSlave0DataLenght(){
    return (_read(I2C_SLV0_CTRL) && 0x0F);
}

/**
    function: enableSlave0ByteSwap
    @summary: configure byte swapping of word pairs.
    @see: Checkout group pairing (setSlave0WordGrouping)
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave0ByteSwap(){
    _buffer = _read(I2C_SLV0_CTRL);
    _buffer |= (1<<6);
    _write(I2C_SLV0_CTRL, _buffer);
}

/**
    function: setSlave0WordGrouping
    @summary: configure the grouping order of word pairs received from registers.
    @parameter:
        order:
            Value: 0 | 1
            [0]: bytes from register addresses 0 and 1, 2 and 3, etc 
                 (even, then odd register addresses) are paired to form a word.
            [1]: bytes from register addresses are paired 1 and 2, 3 and 4, etc.
                 (odd, then even register addresses) are paired to form a word.
    @return: none
*/
void MPU60x0::setSlave0WordGrouping(uint8_t order){
    if(order == 1){
        _buffer = _read(I2C_SLV0_CTRL);
        _buffer |= (1<<4);
    }else if(order == 0){
        _buffer = _read(I2C_SLV0_CTRL);
        _buffer &= ~(1<<4);
    }else return;
    _write(I2C_SLV0_CTRL, _buffer);
}

/**
    function: enableSlave0Register
    @summary: configure the transaction to read or write data only
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave0Register(){
    _buffer = _read(I2C_SLV0_CTRL);
    _buffer |= (1<<5);
    _write(I2C_SLV0_CTRL, _buffer);
    return 1;
}

/**
    function: disableSlave0Register
    @summary: set the I2C_SLV0_REG_DIS bit to 0. Therefore, the transaction 
              will write a register address prior to reading or writing data.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::disableSlave0Register(){
    _buffer = _read(I2C_SLV0_CTRL);
    _buffer &= ~(1<<5);
    _write(I2C_SLV0_CTRL, _buffer);
    return 1;
}

/**
    function: setSlave0Register
    @summary: set the address of slave 0 we want to write/read data in/from
    @parameter:
        address: the register address
    @return:
        bool: return 1 on success
*/
bool MPU60x0::setSlave0Register(uint8_t address){
    _write(I2C_SLV0_REG, address);
    return 1;
}

/**
    function: getSlave0Register
    @summary: read slave 0 address form the sensor
    @parameter: none
    @return:
        uint8_t: return the address of slave 0
*/
uint8_t MPU60x0::getSlave0Register(){
    return _read(I2C_SLV0_REG);
}

// Slave 1
/**
    function: slave1Enable
    @summary: enable slave 1 for writing and reading operations
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::slave1Enable(){
    _buffer = _read(I2C_SLV1_CTRL);
    _buffer |= (1 << 7);
    _write(I2C_SLV1_CTRL, _buffer);
    return 1;
}

/**
    function: slave1Disable
    @summary: disable slave 1
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::slave1Disable(){
    _buffer = _read(I2C_SLV1_CTRL);
    _buffer &= ~(1 << 7);
    _write(I2C_SLV1_CTRL, _buffer);
    return 1;
}

/**
    function: slave1Write
    @summary: write a single byte on slave 1
    @parameter:
        address: The slave 1 device address
        start_register: the address of the first register to be written
        size: The number of byte to write
    @return:
        uint16_t: number of bytes written on the slave device

    TODO test this methods to be sure that it's working
*/
void MPU60x0::slave1Write(uint8_t address, uint8_t _register, uint8_t data){
    _write(I2C_SLV1_ADDR, (address && 0x7F));
    _write(I2C_SLV1_REG, _register);
    _write(I2C_SLV1_DO, data);
    _write(I2C_SLV1_CTRL, 0x81); // Slave 0 enable + data lenght = 1
    _write(I2C_SLV1_CTRL, 0x00);
}

/**
    function: setSlave1DataLength
    @summary: set slave 1 data length for reading or writing operation
    @parameter:
        length: number of byte to read or write
    @return: none
*/
void MPU60x0::setSlave1DataLength(uint8_t length){
    _buffer = _read(I2C_SLV1_CTRL);
    _buffer |= (length && 0x0F);
    _write(I2C_SLV1_CTRL, _buffer);
}

/**
    function: getSlave1DataLenght
    @summary: get slave 1 data length for reading or writing operation
    @parameter: none
    @return:
        uint8_t: number of byte
*/
uint8_t MPU60x0::getSlave1DataLenght(){
    return (_read(I2C_SLV1_CTRL) && 0x0F);
}

/**
    function: enableSlave1ByteSwap
    @summary: configure byte swapping of word pairs.
    @see: Checkout group pairing (setSlave0WordGrouping)
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave1ByteSwap(){
    _buffer = _read(I2C_SLV1_CTRL);
    _buffer |= (1<<6);
    _write(I2C_SLV1_CTRL, _buffer);
}

/**
    function: setSlave1WordGrouping
    @summary: configure the grouping order of word pairs received from registers.
    @parameter:
        order:
            Value: 0 | 1
            [0]: bytes from register addresses 0 and 1, 2 and 3, etc 
                 (even, then odd register addresses) are paired to form a word.
            [1]: bytes from register addresses are paired 1 and 2, 3 and 4, etc.
                 (odd, then even register addresses) are paired to form a word.
    @return: none
*/
void MPU60x0::setSlave1WordGrouping(uint8_t order){
    if(order == 1){
        _buffer = _read(I2C_SLV1_CTRL);
        _buffer |= (1<<4);
    }else if(order == 0){
        _buffer = _read(I2C_SLV1_CTRL);
        _buffer &= ~(1<<4);
    }else return;
    _write(I2C_SLV1_CTRL, _buffer);
}

/**
    function: enableSlave1Register
    @summary: configure the transaction to read or write data only
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave1Register(){
    _buffer = _read(I2C_SLV1_CTRL);
    _buffer |= (1<<5);
    _write(I2C_SLV1_CTRL, _buffer);
    return 1;
}

/**
    function: disableSlave1Register
    @summary: set the I2C_SLV1_REG_DIS bit to 0. Therefore, the transaction 
              will write a register address prior to reading or writing data.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::disableSlave1Register(){
    _buffer = _read(I2C_SLV1_CTRL);
    _buffer &= ~(1<<5);
    _write(I2C_SLV1_CTRL, _buffer);
    return 1;
}

/**
    function: setSlave1Register
    @summary: set the address of slave 1 we want to write/read data in/from
    @parameter:
        address: the register address
    @return:
        bool: return 1 on success
*/
bool MPU60x0::setSlave1Register(uint8_t address){
    _write(I2C_SLV1_REG, address);
    return 1;
}

/**
    function: getSlave1Register
    @summary: read slave 1 address form the sensor
    @parameter: none
    @return:
        uint8_t: return the address of slave 0
*/
uint8_t MPU60x0::getSlave1Register(){
    return _read(I2C_SLV1_REG);
}

// Slave 2
/**
    function: slave2Enable
    @summary: enable slave 2 for writing and reading operations
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::slave2Enable(){
    _buffer = _read(I2C_SLV2_CTRL);
    _buffer |= (1 << 7);
    _write(I2C_SLV2_CTRL, _buffer);
    return 1;
}

/**
    function: slave2Disable
    @summary: disable slave 2
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::slave2Disable(){
    _buffer = _read(I2C_SLV2_CTRL);
    _buffer &= ~(1 << 7);
    _write(I2C_SLV2_CTRL, _buffer);
    return 1;
}

/**
    function: slave2Write
    @summary: write a single byte on slave 2
    @parameter:
        address: The slave 2 device address
        start_register: the address of the first register to be written
        size: The number of byte to write
    @return:
        uint16_t: number of bytes written on the slave device

    TODO test this methods to be sure that it's working
*/
void MPU60x0::slave2Write(uint8_t address, uint8_t _register, uint8_t data){
    _write(I2C_SLV2_ADDR, (address && 0x7F));
    _write(I2C_SLV2_REG, _register);
    _write(I2C_SLV2_DO, data);
    _write(I2C_SLV2_CTRL, 0x81); // Slave 0 enable + data lenght = 1
    _write(I2C_SLV2_CTRL, 0x00);
}

/**
    function: setSlave2DataLength
    @summary: set slave 2 data length for reading or writing operation
    @parameter:
        length: number of byte to read or write
    @return: none
*/
void MPU60x0::setSlave2DataLength(uint8_t length){
    _buffer = _read(I2C_SLV2_CTRL);
    _buffer |= (length && 0x0F);
    _write(I2C_SLV2_CTRL, _buffer);
}

/**
    function: getSlave2DataLenght
    @summary: get slave 2 data length for reading or writing operation
    @parameter: none
    @return:
        uint8_t: number of byte
*/
uint8_t MPU60x0::getSlave2DataLenght(){
    return (_read(I2C_SLV2_CTRL) && 0x0F);
}

/**
    function: enableSlave2ByteSwap
    @summary: configure byte swapping of word pairs.
    @see: Checkout group pairing (setSlave0WordGrouping)
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave2ByteSwap(){
    _buffer = _read(I2C_SLV2_CTRL);
    _buffer |= (1<<6);
    _write(I2C_SLV2_CTRL, _buffer);
}

/**
    function: setSlave2WordGrouping
    @summary: configure the grouping order of word pairs received from registers.
    @parameter:
        order:
            Value: 0 | 1
            [0]: bytes from register addresses 0 and 1, 2 and 3, etc 
                 (even, then odd register addresses) are paired to form a word.
            [1]: bytes from register addresses are paired 1 and 2, 3 and 4, etc.
                 (odd, then even register addresses) are paired to form a word.
    @return: none
*/
void MPU60x0::setSlave2WordGrouping(uint8_t order){
    if(order == 1){
        _buffer = _read(I2C_SLV2_CTRL);
        _buffer |= (1<<4);
    }else if(order == 0){
        _buffer = _read(I2C_SLV2_CTRL);
        _buffer &= ~(1<<4);
    }else return;
    _write(I2C_SLV2_CTRL, _buffer);
}

/**
    function: enableSlave2Register
    @summary: configure the transaction to read or write data only
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave2Register(){
    _buffer = _read(I2C_SLV2_CTRL);
    _buffer |= (1<<5);
    _write(I2C_SLV2_CTRL, _buffer);
    return 1;
}

/**
    function: disableSlave2Register
    @summary: set the I2C_SLV2_REG_DIS bit to 0. Therefore, the transaction 
              will write a register address prior to reading or writing data.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::disableSlave2Register(){
    _buffer = _read(I2C_SLV2_CTRL);
    _buffer &= ~(1<<5);
    _write(I2C_SLV2_CTRL, _buffer);
    return 1;
}

/**
    function: setSlave2Register
    @summary: set the address of slave 2 we want to write/read data in/from
    @parameter:
        address: the register address
    @return:
        bool: return 1 on success
*/
bool MPU60x0::setSlave2Register(uint8_t address){
    _write(I2C_SLV2_REG, address);
    return 1;
}

/**
    function: getSlave2Register
    @summary: read slave 2 address form the sensor
    @parameter: none
    @return:
        uint8_t: return the address of slave 0
*/
uint8_t MPU60x0::getSlave2Register(){
    return _read(I2C_SLV2_REG);
}

// Slave 3
/**
    function: slave3Enable
    @summary: enable slave 3 for writing and reading operations
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::slave3Enable(){
    _buffer = _read(I2C_SLV3_CTRL);
    _buffer |= (1 << 7);
    _write(I2C_SLV3_CTRL, _buffer);
    return 1;
}

/**
    function: slave3Disable
    @summary: disable slave 3
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::slave3Disable(){
    _buffer = _read(I2C_SLV3_CTRL);
    _buffer &= ~(1 << 7);
    _write(I2C_SLV3_CTRL, _buffer);
    return 1;
}

/**
    function: slave3Write
    @summary: write a single byte on slave 3
    @parameter:
        address: The slave 3 device address
        start_register: the address of the first register to be written
        size: The number of byte to write
    @return:
        uint16_t: number of bytes written on the slave device

    TODO test this methods to be sure that it's working
*/
void MPU60x0::slave3Write(uint8_t address, uint8_t _register, uint8_t data){
    _write(I2C_SLV3_ADDR, (address && 0x7F));
    _write(I2C_SLV3_REG, _register);
    _write(I2C_SLV3_DO, data);
    _write(I2C_SLV3_CTRL, 0x81); // Slave 0 enable + data lenght = 1
    _write(I2C_SLV3_CTRL, 0x00);
}

/**
    function: setSlave3DataLength
    @summary: set slave 2 data length for reading or writing operation
    @parameter:
        length: number of byte to read or write
    @return: none
*/
void MPU60x0::setSlave3DataLength(uint8_t length){
    _buffer = _read(I2C_SLV3_CTRL);
    _buffer |= (length && 0x0F);
    _write(I2C_SLV3_CTRL, _buffer);
}

/**
    function: getSlave3DataLenght
    @summary: get slave 3 data length for reading or writing operation
    @parameter: none
    @return:
        uint8_t: number of byte
*/
uint8_t MPU60x0::getSlave3DataLenght(){
    return (_read(I2C_SLV3_CTRL) && 0x0F);
}

/**
    function: enableSlave3ByteSwap
    @summary: configure byte swapping of word pairs.
    @see: Checkout group pairing (setSlave0WordGrouping)
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave3ByteSwap(){
    _buffer = _read(I2C_SLV3_CTRL);
    _buffer |= (1<<6);
    _write(I2C_SLV3_CTRL, _buffer);
}

/**
    function: setSlave2WordGrouping
    @summary: configure the grouping order of word pairs received from registers.
    @parameter:
        order:
            Value: 0 | 1
            [0]: bytes from register addresses 0 and 1, 2 and 3, etc 
                 (even, then odd register addresses) are paired to form a word.
            [1]: bytes from register addresses are paired 1 and 2, 3 and 4, etc.
                 (odd, then even register addresses) are paired to form a word.
    @return: none
*/
void MPU60x0::setSlave3WordGrouping(uint8_t order){
    if(order == 1){
        _buffer = _read(I2C_SLV3_CTRL);
        _buffer |= (1<<4);
    }else if(order == 0){
        _buffer = _read(I2C_SLV3_CTRL);
        _buffer &= ~(1<<4);
    }else return;
    _write(I2C_SLV3_CTRL, _buffer);
}

/**
    function: enableSlave3Register
    @summary: configure the transaction to read or write data only
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave3Register(){
    _buffer = _read(I2C_SLV3_CTRL);
    _buffer |= (1<<5);
    _write(I2C_SLV3_CTRL, _buffer);
    return 1;
}

/**
    function: disableSlave3Register
    @summary: set the I2C_SLV3_REG_DIS bit to 0. Therefore, the transaction 
              will write a register address prior to reading or writing data.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::disableSlave3Register(){
    _buffer = _read(I2C_SLV3_CTRL);
    _buffer &= ~(1<<5);
    _write(I2C_SLV3_CTRL, _buffer);
    return 1;
}

/**
    function: setSlave3Register
    @summary: set the address of slave 3 we want to write/read data in/from
    @parameter:
        address: the register address
    @return:
        bool: return 1 on success
*/
bool MPU60x0::setSlave3Register(uint8_t address){
    _write(I2C_SLV3_REG, address);
    return 1;
}

/**
    function: getSlave3Register
    @summary: read slave 3 address form the sensor
    @parameter: none
    @return:
        uint8_t: return the address of slave 0
*/
uint8_t MPU60x0::getSlave3Register(){
    return _read(I2C_SLV3_REG);
}

// Slave 4
/**
    function: slave4Enable
    @summary: enable slave 4 for writing and reading operations
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::slave4Enable(){
    _buffer = _read(I2C_SLV4_CTRL);
    _buffer |= (1 << 7);
    _write(I2C_SLV4_CTRL, _buffer);
    return 1;
}

/**
    function: slave4Disable
    @summary: disable slave 4
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::slave4Disable(){
    _buffer = _read(I2C_SLV4_CTRL);
    _buffer &= ~(1 << 7);
    _write(I2C_SLV4_CTRL, _buffer);
    return 1;
}

/**
    function: enableSlave4Register
    @summary: configure the transaction to read or write data only
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave4Register(){
    _buffer = _read(I2C_SLV4_CTRL);
    _buffer |= (1<<5);
    _write(I2C_SLV4_CTRL, _buffer);
    return 1;
}

/**
    function: disableSlave4Register
    @summary: set the I2C_SLV4_REG_DIS bit to 0. Therefore, the transaction 
              will write a register address prior to reading or writing data.
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::disableSlave4Register(){
    _buffer = _read(I2C_SLV4_CTRL);
    _buffer &= ~(1<<5);
    _write(I2C_SLV4_CTRL, _buffer);
    return 1;
}

/**
    function: setSlave4Register
    @summary: set the address of slave 4 we want to write/read data in/from
    @parameter:
        address: the register address
    @return:
        bool: return 1 on success
*/
bool MPU60x0::setSlave4Register(uint8_t address){
    _write(I2C_SLV4_REG, address);
    return 1;
}

/**
    function: getSlave4Register
    @summary: read slave 4 address form the sensor
    @parameter: none
    @return:
        uint8_t: return the address of slave 0
*/
uint8_t MPU60x0::getSlave4Register(){
    return _read(I2C_SLV4_REG);
}

/**
    function: enableSlave4Interrupt
    @summary: enable interrupt for slave 4
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::enableSlave4Interrupt(){
    _buffer = _read(I2C_SLV4_CTRL);
    _buffer |= (1<<6);
}

/**
    function: disableSlave4Interrupt
    @summary: disable interrupt for slave 4
    @parameter: none
    @return:
        bool: return 1 on success
*/
bool MPU60x0::disableSlave4Interrupt(){
    _buffer = _read(I2C_SLV4_CTRL);
    _buffer &= ~(1<<6);
}


// Read external sensor data
/**
    function: getExternalSensorData
    @summary: read a byte from an external sensor register given it's index
    @parameter:
        index: the relative position of the register from EXT_SENS_DATA_00
               range: 0 - 23
    @see: EXT_SENS_DATA_00 (in MPU60x0.h file)
    @return:
        int: the value of the choosen register
             [-1]: wrong position given
*/
int MPU60x0::getExternalSensorData(uint8_t index){
    if((index < 0) || (index > 23)) return -1;
    return _read(EXT_SENS_DATA_00 + index);
}

/**
    ============================================================
                            STANDBY
    ============================================================
**/

/**
    function: enableAccelStandby
    @summary: enable accelerometer standby for all axis
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableAccelStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer |= 0x38;
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: enableAccelXStandby
    @summary: enable accelerometer X standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableAccelXStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer |= (1 << 5);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: enableAccelYStandby
    @summary: enable accelerometer Y standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableAccelYStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer |= (1 << 4);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: enableAccelZStandby
    @summary: enable accelerometer Z standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableAccelZStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer |= (1 << 3);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: enableGyroStandby
    @summary: enable gyroscope standby for all axis
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableGyroStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer |= 0x07;
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: enableGyroXStandby
    @summary: enable gyroscope X standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableGyroXStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer |= (1 << 2);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: enableGyroYStandby
    @summary: enable gyroscope Y standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableGyroYStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer |= (1 << 1);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: enableGyroZStandby
    @summary: enable gyroscope Z standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableGyroZStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer |= 1;
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: disableAccelStandby
    @summary: disable accelerometer standby for all axis
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableAccelStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer = (_buffer && 0xC7);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: disableAccelXStandby
    @summary: disable accelerometer X standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableAccelXStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer &= ~(1 << 5);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: disableAccelYStandby
    @summary: disable accelerometer Y standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableAccelYStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer &= ~(1 << 4);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: disableAccelZStandby
    @summary: disable accelerometer Z standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableAccelZStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer &= ~(1 << 3);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: disableGyroStandby
    @summary: disable gyroscope standby for all axis
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableGyroStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer = (_buffer && 0xF8);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: disableGyroXStandby
    @summary: disable gyroscope X standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableGyroXStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer &= ~(1 << 2);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: disableGyroYStandby
    @summary: disable gyroscope Y standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableGyroYStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer &= ~(1 << 1);
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    function: disableGyroZStandby
    @summary: disable gyroscope Z standby
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableGyroZStandby(){
    _buffer = _read(PWR_MGMT_2);
    _buffer &= ~1;
    _write(PWR_MGMT_2, _buffer);
    return true;
}

/**
    ============================================================
                           INTERRUPT
    ============================================================
**/
/**
    function: setInterruptLevel
    @summary: configure the interrupt level
    @parameter:
        level:
            [0]: active HIGH
            [1]: active LOW
    @return:
        bool: return true on success
*/
bool MPU60x0::setInterruptLevel(bool level){
    _buffer = _read(INT_PIN_CFG);
    if(level) _buffer |= (1 << 7);
    else _buffer &= ~(1 << 7);
    _write(INT_PIN_CFG, _buffer);
    return 1;
}

/**
    function: setInterruptPinState
    @summary: configure interrupt pin state on the sensor
    @parameter:
        state:
            [0]: push-pull
            [1]: open drain
    @return:
        bool: return true on success
*/
bool MPU60x0::setInterruptPinState(bool state){
    _buffer = _read(INT_PIN_CFG);
    if(state) _buffer |= (1 << 6);
    else _buffer &= ~(1 << 6);
    _write(INT_PIN_CFG, _buffer);
    return 1;
}

/**
    function: enableInterruptPinLatch
    @summary: enable interrupt pin latch
              The INT pin is held high until the interrupt is cleared.
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableInterruptPinLatch(){
    _buffer = _read(INT_PIN_CFG);
    _buffer |= (1 << 5);
    _write(INT_PIN_CFG, _buffer);
    return 1;
}

/**
    function: disableInterruptPinLatch
    @summary: enable interrupt pin latch
              The INT pin emits a 50us long pulse.
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableInterruptPinLatch(){
    _buffer = _read(INT_PIN_CFG);
    _buffer &= ~(1 << 5);
    _write(INT_PIN_CFG, _buffer);
    return 1;
}

/**
    function: clearInterruptOn
    @summary: configure the operation required to clear interrupt
    @parameter:
        reading_status:
            [0]: interrupt status bits are cleared on any read operation.
            [1]: interrupt status bits are cleared only by reading INT_STATUS
    @return:
        bool: return true on success
*/
bool MPU60x0::clearInterruptOn(bool reading_status){
    _buffer = _read(INT_PIN_CFG);
    if(reading_status) _buffer &= ~(1 << 4);
    else _buffer |= (1 << 4);
    _write(INT_PIN_CFG, _buffer);
    return 1;
}

/**
    function: enableFsyncInterrupt
    @summary: enable the FSYNC pin to cause interrupt on the host processor
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableFsyncInterrupt(){
    _buffer = _read(INT_PIN_CFG) | 1;
    _write(INT_PIN_CFG, _buffer);
    return 1;
}

/**
    function: setFsyncInterruptLevel
    @summary: configure the interrupt level on FSYNC pin
    @parameter:
        level:
            [0]: active HIGH
            [1]: active LOW
    @return:
        bool: return true on success
*/
bool MPU60x0::setFsyncInterruptLevel(bool level){
    _buffer = _read(INT_PIN_CFG);
    if(level) _buffer |= (1 << 1);
    else _buffer &= ~(1 << 1);
    _write(INT_PIN_CFG, _buffer);
    return 1;
}

// Enable interrupts
/**
    function: enableMotionDetectionInterrupt
    @summary: enable motion detection to generate an interrupt
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableMotionDetectionInterrupt(){
    _buffer = _read(INT_ENABLE);
    _buffer |= (1 << 6);
    _write(INT_ENABLE, _buffer);
    return 1;
}

/**
    function: disableMotionDetectionInterrupt
    @summary: unable motion detection to generate an interrupt
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableMotionDetectionInterrupt(){
    _buffer = _read(INT_ENABLE);
    _buffer &= ~(1 << 6);
    _write(INT_ENABLE, _buffer);
    return 1;
}

/**
    function: enableFifoOverflowInterrupt
    @summary: enable FIFO to generate an interrupt on overflow
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableFifoOverflowInterrupt(){
    _buffer = _read(INT_ENABLE);
    _buffer |= (1 << 4);
    _write(INT_ENABLE, _buffer);
    return 1;
}

/**
    function: disableFifoOverflowInterrupt
    @summary: disable FIFO to generate an interrupt on overflow
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableFifoOverflowInterrupt(){
    _buffer = _read(INT_ENABLE);
    _buffer &= ~(1 << 4);
    _write(INT_ENABLE, _buffer);
    return 1;
}

/**
    function: enableI2cMasterInterrupt
    @summary: enables any of the I2C Master interrupt sources to generate an 
              interrupt.
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableI2cMasterInterrupt(){
    _buffer = _read(INT_ENABLE);
    _buffer |= (1 << 3);
    _write(INT_ENABLE, _buffer);
    return 1;
}

/**
    function: disableI2cMasterInterrupt
    @summary: disables any of the I2C Master interrupt sources to generate an 
              interrupt.
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableI2cMasterInterrupt(){
    _buffer = _read(INT_ENABLE);
    _buffer &= ~(1 << 3);
    _write(INT_ENABLE, _buffer);
    return 1;
}

/**
    function: enableDataReadyInterrupt
    @summary: enables the data ready interrupt.
              This occurs each time a write operation to all of the sensor 
              registers has been completed.
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::enableDataReadyInterrupt(){
    _buffer = _read(INT_ENABLE);
    _buffer |= 1;
    _write(INT_ENABLE, _buffer);
    return 1;
}

/**
    function: disableDataReadyInterrupt
    @summary: disables the data ready interrupt.
              This occurs each time a write operation to all of the sensor 
              registers has been completed.
    @parameter: none
    @return:
        bool: return true on success
*/
bool MPU60x0::disableDataReadyInterrupt(){
    _buffer = _read(INT_ENABLE);
    _buffer &= ~1;
    _write(INT_ENABLE, _buffer);
    return 1;
}

/**
    function: getInterruptStatus
    @summary: read the interrupt status register
    @parameter: none
    @return:
        uint8_t: 
                bit-fileds
            [0]: Data ready interrupt
            [3]: I2C master interrupt
            [4]: Fifo overflow interrupt
            [6]: Motion detection interrupt
        
        # Ready the status register clear each interrupt bit #
*/
uint8_t MPU60x0::getInterruptStatus(){
    return _read(INT_STATUS);
}
