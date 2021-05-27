#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

// device addresses (based on address pin LOW (ADO) or HIGH (AD1))
#define MPU_I2C_ADDRESS_AD0                 0x68
#define MPU_I2C_ADDRESS_AD1                 0x69

// config registers
#define MPU6050_SMPLRT_DIV                  0x19  ///< sample rate divisor register
#define MPU6050_CONFIG                      0x1A      ///< General configuration register
#define MPU6050_GYRO_CONFIG                 0x1B ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG                0x1C ///< Accelerometer specific configration register
#define MPU6050_INT_PIN_CONFIG              0x37    ///< Interrupt pin configuration register
#define MPU6050_SIGNAL_PATH_RESET           0x68 ///< Signal path reset register
#define MPU6050_USER_CTRL                   0x6A         ///< FIFO and I2C Master control register
#define MPU6050_PWR_MGMT_1                  0x6B        ///< Primary power/sleep control register
#define MPU6050_PWR_MGMT_2                  0x6C ///< Secondary power/sleep control register
#define MPU6050_WHO_AM_I                    0x75          ///< Divice ID register
#define MPU6050_DEVICE_ID                   0x68

// data registers
#define MPU6050_TEMP_H                      0x41     ///< Temperature data high byte register
#define MPU6050_TEMP_L                      0x42     ///< Temperature data low byte register
#define MPU6050_ACCEL_OUT                   0x3B  ///< base address for sensor data reads

// interrupt registers
#define MPU6050_MOT_THR                     0x1F        // Motion detection threshold bits [7:0]
#define MPU6050_MOT_DUR                     0x20 // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MPU6050_MOT_DETECT_CTRL             0x69
#define MPU6050_INT_ENABLE                  0x38

// gyro configuration values
#define MPU6050_GYRO_FULL_SCALE_250_DPS     0x00  // (default)
#define MPU6050_GYRO_FULL_SCALE_500_DPS     0x08
#define MPU6050_GYRO_FULL_SCALE_1000_DPS    0x10
#define MPU6050_GYRO_FULL_SCALE_2000_DPS    0x18

// accelerometer configuration values
#define MPU6050_ACC_FULL_SCALE_2_G          0x00  // (default)
#define MPU6050_ACC_FULL_SCALE_4_G          0x08
#define MPU6050_ACC_FULL_SCALE_8_G          0x10
#define MPU6050_ACC_FULL_SCALE_16_G         0x18


// conversions and constants
#define RAD2DEG (57.29577793F)       
#define DEG2RAD (0.017453293F)
#define GRAVITY_STANDARD (9.80665F)
#define CALIBRATION_COUNT 1000
#define TWO_PI (6.2831853072F)
#define PI (3.141592657F)


typedef enum {
    UNITS_DEGREES,
    UNITS_RADIANS
} units_t;

typedef enum {
    RANGE_RELATIVE, // normalise between [pi,pi] ([-180,180])
    RANGE_ABSOLUTE  // normalise between [0,2pi] ([0,360])
} range_t;

typedef struct {
    float x;
    float y;
    float z;
} vector_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} raw_t;


class MPU6050
{
    private:
        byte _addr;

        attitude_t _attitude = {0,0,0};         // attitude in degrees
        vector_t _acc = {0,0,0};                // raw acceleration in m/s/s
        vector_t _gyro = {0,0,0};               // raw gyro in deg/s

        // calibration offsets
        vector_t _accOffset = {0,0,0};
        vector_t _gyroOffset = {0,0,0};

        // normalisation
        range_t _range = RANGE_RELATIVE;

        unsigned long _lastSampleTime = 0;

        float _temperature;

        void _readMPU(void);
        float _getAccScale(void);
        float _getGyroScale(void);
        float _normalise(float x, range_t range = RANGE_RELATIVE);
        
    public:

        MPU6050(void);
        MPU6050(byte addr);

        /*
            reset MPU and program variables to defaults.
        */ 
        bool begin(range_t range = RANGE_RELATIVE);


        /*
            Returns true if the device is found
        */
        bool available(void);

        /*
            Performs CALIBRATION_COUNT iterations to calculate average offset. Do not move device during
            calibration!
        */
        void calibrate(void);

        /*
            update raw accelerometer values and calculate attitude using complementary filter.  
        */
        void update(void);


        // i2c stuff
        void setDeviceAddress(byte addr) {this->_addr = addr;};
        void read(byte reg, uint8_t len, byte* data);
        void write(byte reg, byte data);


        // interrupts
        void enableMotionInterrupt(bool en);
        void setMotionInterruptThreshold(byte t);
        void setMotionInterruptDuration(byte d);
        void enableFreeFallInterrupt(bool en);
        
        
        // register settings
        void setGyroConfig(byte c);
        byte getGyroConfig(void);
        void setAccConfig(byte c);
        byte getAccConfig(void);


        // setters
        void setRange(range_t range) { this->_range = range;};

        // getters
        // Call update() first
        attitude_t getAttitude(units_t units = UNITS_RADIANS);
        vector_t getAcceleration(void) {return this->_acc;};
        vector_t getGyro(void) { return this->_gyro;};
        float getTemperature(void) { return this->_temperature;};

};


#endif;