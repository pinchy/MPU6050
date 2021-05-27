# MPU6050 library
Lightweight flexible Arduino library for easy communication with the MPU6050 IMU using compensation algorithm for attitude estimation.

## Datasheets
Links to reference documents:
 * [Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
 * [Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

## Dependencies
Requires I2C Wire library to be loaded and configured, assumes I2C is available via the global `Wire` class.

## Usage
See example sketches [examples](https://github.com/pinchy/MPU6050/tree/master/examples)  

Minimum code example:
```
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu();

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    mpu.begin();
    mpu.calibrate();
}

void loop()
{
    // read the latest values from the IMU
    mpu.update();

    // get the attitude (roll, pitch, yaw)
    attitude_t attitude = mpu.getAttitude();

    // do something with the data
    Serial.print("Roll: ");
    Serial.print(attitude.roll);
    Serial.print("\tPitch: ");
    Serial.print(attitude.pitch);
    Serial.print("\tYaw: ");
    Serial.println(attitude.yaw);
    
    delay(100);
}
```

### Device Address
By default the device assumes `ADO` pin is `LOW` and the device address `MPU_I2C_ADDRESS_AD0 = 0x68` is used.  However if the alternative (`ADO = HIGH`) is desired, then this can be achieved in two ways:

 * via the constructor: 
   `MPU6050 mpu(MPU_I2C_ADDRESS_AD1);`

 * or during runtime via the `setDeviceAddress(byte)` method:
   `setDeviceAddress(MPU_I2C_ADDRESS_AD1);`


### Auto calibration
The Gyro values can be calibrated by calling `calibrate()`, this takes the average of `CALIBRATION_COUNT` samples and deducts these values from future readings.

## Advanced functionality

### Range normalisation
By default, Attitude readings are normalised between 0 and 180 degrees, however three range normalisation congiguratios are available.  These can be configured by calling the `setRange(range_t)` method with one of the following values:

 * `RANGE_THROUGH` pass-through, no normalisation.
 * `RANGE_ABSOLUTE` absolute values, normalised between 0 and 360 degrees
 * `RANGE_RELATIVE` relative values from the normal, normalised between -180 and 180 degrees (default)


### Handy radian/degrees conversion
By default, attitude from the `getAttitude(units_t)` method returns attitude in degrees. If radians are prefered pass unit type `UNITS_RADIANS` in the method, e.g. `getAttitude(UNITS_RADIANS)`


### IMU register configuration
Gryo and Accelerometer range registers can be configured directly using the methods `setGyroConfig(byte)` and `setAccConfig(byte)`.

**Acceptable gyro configuration values**
 * MPU6050_GYRO_FULL_SCALE_250_DPS (default)
 * MPU6050_GYRO_FULL_SCALE_500_DPS
 * MPU6050_GYRO_FULL_SCALE_1000_DPS
 * MPU6050_GYRO_FULL_SCALE_2000_DPS

**Acceptabl;e accelerometer configuration values**
 * MPU6050_ACC_FULL_SCALE_2_G (default)
 * MPU6050_ACC_FULL_SCALE_4_G 
 * MPU6050_ACC_FULL_SCALE_8_G 
 * MPU6050_ACC_FULL_SCALE_16_G 


## Licence
MIT

## Author

[pinchy](https://github.com/pinchy)
