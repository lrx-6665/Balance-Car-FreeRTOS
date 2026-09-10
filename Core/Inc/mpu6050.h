#ifndef MPU6050_H
#define MPU6050_H
#include <stdint.h>

uint8_t MPU6050_Init(void);
uint8_t MPU6050_GetWhoAmI(void);
void MPU6050_Calibrate(void);
void MPU6050_SetPitch(float pitch);
void MPU6050_Update(float dt, float *pitch, float *gyro_y, float *gyro_z);

#endif
