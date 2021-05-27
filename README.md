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

 * or during runtime via the `setDeviceAddress()` method:
   `setDeviceAddress(MPU_I2C_ADDRESS_AD1);`


### Auto calibration
The Gyro values can be calibrated by calling `calibrate()`, this takes the average of `CALIBRATION_COUNT` samples and deducts these values from future readings.

## Advanced functionality


### IMU register configuration
Gryo and Accelerometer range registers can be configured directly using
setGyroConfig() and setAccConfig()

### Handy radian/degrees conversion
By default, attitude from the `getAttitude()` method returns attitude in degrees. If radians are prefered pass unit type `UNITS_RADIANS` in the method, e.g. `getAttitude(UNITS_RADIANS)`


## Licence
MIT

## Author

[pinchy](https://github.com/pinchy)
