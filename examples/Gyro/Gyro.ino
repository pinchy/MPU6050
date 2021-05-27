#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu6050;

void setup(void)
{
    Serial.begin(115200);
    Wire.begin();
    mpu6050.begin();

    if(!mpu6050.available())
    {
        Serial.print("Unable to connect to MPU!");
        while(1);
    }
    
    // set the scaling registers
    mpu6050.setAccConfig(MPU6050_ACC_FULL_SCALE_2_G);
    mpu6050.setGyroConfig(MPU6050_GYRO_FULL_SCALE_500_DPS);

    // run the self calibration function, must not move the MPU during this function call!
    mpu6050.calibrate();
}


void loop(void)
{
    mpu6050.update();
    vector_t acc = mpu6050.getAcceleration();
    Serial.print("ax: ");
    Serial.print(acc.x);
    Serial.print("\tay:");
    Serial.print(acc.y);
    Serial.print("\taz:");
    Serial.print(acc.z);
 

    vector_t gyro = mpu6050.getGyro();
    Serial.print("\tgx: ");
    Serial.print(gyro.x);
    Serial.print("\tgy: ");
    Serial.print(gyro.y);
    Serial.print("\tgz: ");
    Serial.println(gyro.z);

    delay(500);
}
