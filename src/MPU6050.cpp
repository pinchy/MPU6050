#include <MPU6050.h>

MPU6050::MPU6050()
{
    this->setDeviceAddress(MPU_I2C_ADDRESS_AD0);
}

MPU6050::MPU6050(byte addr)
{
    this->setDeviceAddress(addr);
}

bool MPU6050::begin(range_t range)
{
    this->_attitude = {0,0,0};
    this->_accOffset = {0,0,0};
    this->_gyroOffset = {0,0,0};

    this->_range = range;

    this->write(MPU6050_SMPLRT_DIV, 0x00);
    this->write(MPU6050_CONFIG, 0x00);
    this->write(MPU6050_GYRO_CONFIG, 0x00);
    this->write(MPU6050_ACCEL_CONFIG, 0x00);
    this->write(MPU6050_PWR_MGMT_1, 0x01);

    this->_readMPU();
    this->_lastSampleTime = millis();

    return this->available();
}


void MPU6050::calibrate(void)
{
    // clear the offsets (as we'll be recalculating these)
    this->_attitude = {0,0,0};
    this->_accOffset = {0,0,0};
    this->_gyroOffset = {0,0,0};

    // local variables to calculate new offsets
    vector_t gyroOffset = {0,0,0};
    vector_t accOffset = {0,0,0};

    for(int i = 0; i < CALIBRATION_COUNT; i ++)
    {
        this->_readMPU();
        
        gyroOffset.x += this->_gyro.x;
        gyroOffset.y += this->_gyro.y;
        gyroOffset.z += this->_gyro.z;

        accOffset.x += atan2(this->_acc.y, this->_acc.z);
        accOffset.y += atan2(-this->_acc.x, sqrt(pow(this->_acc.y, 2) + pow(this->_acc.z, 2)));
    }

    this->_accOffset.x = accOffset.x / CALIBRATION_COUNT;
    this->_accOffset.y = accOffset.y / CALIBRATION_COUNT;

    this->_gyroOffset.x = gyroOffset.x / CALIBRATION_COUNT;
    this->_gyroOffset.y = gyroOffset.y / CALIBRATION_COUNT;
    this->_gyroOffset.z = gyroOffset.z / CALIBRATION_COUNT;

    this->_attitude.roll = this->_accOffset.x;
    this->_attitude.pitch = this->_accOffset.y;
}



void MPU6050::update()
{
    // complementary filter http://www.pieter-jan.com/node/11
    attitude_t accAngle;
    this->_readMPU();

    accAngle.roll = atan2(this->_acc.y, this->_acc.z + abs(this->_acc.x)) * RAD2DEG;
    accAngle.pitch = atan2(this->_acc.x, this->_acc.z + abs(this->_acc.y)) * -RAD2DEG;

    float dt = (millis() - this->_lastSampleTime) * 0.001;
    
    this->_attitude.roll = (0.98 * (this->_attitude.roll + this->_gyro.x * dt)) + (0.02 * accAngle.roll);
    this->_attitude.pitch = (0.98 * (this->_attitude.pitch + this->_gyro.y * dt)) + (0.02 * accAngle.pitch);
    this->_attitude.yaw += this->_gyro.z * dt;

    this->_lastSampleTime = millis();
}


void MPU6050::_readMPU(void)
{
    byte buffer[14];
    this->read(MPU6050_ACCEL_OUT, 14, buffer);

    raw_t acc;
    raw_t gyro;

    acc.x = buffer[0] << 8 | buffer[1];
    acc.y = buffer[2] << 8 | buffer[3];
    acc.z = buffer[4] << 8 | buffer[5];

    uint16_t temperature = buffer[6] << 8 | buffer[7];

    gyro.x = buffer[8] << 8 | buffer[9];
    gyro.y = buffer[10] << 8 | buffer[11];
    gyro.z = buffer[12] << 8 | buffer[13];
    
    this->_temperature = ((float) temperature / 340.0) + 36.53;

    float accScale = this->_getAccScale();
    this->_acc.x = ((float) acc.x) / accScale;
    this->_acc.y = ((float) acc.y) / accScale;
    this->_acc.z = ((float) acc.z) / accScale;

    float gyroScale = this->_getGyroScale();
    this->_gyro.x = (((float) gyro.x) / gyroScale) - this->_gyroOffset.x;
    this->_gyro.y = (((float) gyro.y) / gyroScale) - this->_gyroOffset.y;
    this->_gyro.z = (((float) gyro.z) / gyroScale) - this->_gyroOffset.z;
}


bool MPU6050::available(void)
{
    byte buffer[1];
    this->read(MPU6050_WHO_AM_I, 1, buffer);
    return (buffer[0] == MPU6050_DEVICE_ID);
}



attitude_t MPU6050::getAttitude(units_t units) 
{
    attitude_t attitude;
    attitude.roll = this->_normalise(this->_attitude.roll, this->_range);
    attitude.pitch = this->_normalise(this->_attitude.pitch, this->_range);
    attitude.yaw = this->_normalise(this->_attitude.yaw, this->_range); 

    // attitude is saved in degrees, convert to radians 
    if(units == UNITS_RADIANS)
    {
        attitude.roll *= DEG2RAD;
        attitude.pitch *= DEG2RAD;
        attitude.yaw *= DEG2RAD;
    }

    return attitude;
};


float MPU6050::_normalise(float x, range_t range)
{
    if(range == RANGE_THROUGH) return x;

    float offset = (range == RANGE_ABSOLUTE) ? 0 : 180; // set to 0 (RANGE_ABSOLUTE) for [0,360], to pi (RANGE_RELATIVE) for [-180,180]
    x = fmod(x + offset, 360);
    if(x < 0) x += 360;
    return x - offset;
}



void MPU6050::setGyroConfig(byte c)
{
    this->write(MPU6050_GYRO_CONFIG, c);
}

byte MPU6050::getGyroConfig(void)
{
    byte buf[1];
    this->read(MPU6050_GYRO_CONFIG, 1, buf);
    return buf[0];
}
        

void MPU6050::setAccConfig(byte c)
{
    this->write(MPU6050_ACCEL_CONFIG, c);
}

byte MPU6050::getAccConfig(void)
{
    byte buf[1];
    this->read(MPU6050_ACCEL_CONFIG, 1, buf);
    return buf[0];
}
        


float MPU6050::_getAccScale(void)
{
    byte config = this->getAccConfig();
    float scale = 1;
    if (config == MPU6050_ACC_FULL_SCALE_2_G) 
        scale = 16384;
    else if (config == MPU6050_ACC_FULL_SCALE_4_G) 
        scale = 8192;
    else if (config == MPU6050_ACC_FULL_SCALE_8_G) 
        scale = 4096;
    else if (config == MPU6050_ACC_FULL_SCALE_16_G) 
        scale = 2048;

    return scale;
}

float MPU6050::_getGyroScale(void)
{
    byte config = this->getGyroConfig();
    float scale = 1;
    if (config == MPU6050_GYRO_FULL_SCALE_250_DPS) 
        scale = 131;
    else if (config == MPU6050_GYRO_FULL_SCALE_500_DPS) 
        scale = 65.5;
    else if (config == MPU6050_GYRO_FULL_SCALE_1000_DPS) 
        scale = 32.8;
    else if (config == MPU6050_GYRO_FULL_SCALE_2000_DPS) 
        scale = 16.4;

    return scale;
}

void MPU6050::read(byte reg, uint8_t len, byte* data)
{
    Wire.beginTransmission(this->_addr);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(this->_addr, len); 
    uint8_t i = 0;
    while (Wire.available())
        data[i++] = Wire.read();
}


void MPU6050::write(byte reg, byte data)
{
    Wire.beginTransmission(this->_addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}