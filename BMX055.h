/*
 * BMX055.h
 *
 *  Created on: Sep 9, 2023
 *      Author: MUHAMMAD HELMI BIN ROZAIN
 */

#include <main.h>

typedef struct {
	float x;
	float y;
	float z;
} POS;

extern POS GYRO_POS;
extern POS GYRO_MOVEMENT;
extern POS VELOCITY;
extern POS ACCEL;

// ACCELEROMETER SETUP
#define ACCEL_CHIP_ID_REG    0x00
#define ACCEL_CHIP_ID_DEFAULT    0xFA //default chip id reg
#define ACCEL_ADDR1  0x18 //default address
#define ACCEL_ADDR2  0x19
#define ACCEL_X_LSB     0x02
#define ACCEL_X_MSB     0x03
#define ACCEL_Y_LSB     0x04
#define ACCEL_Y_MSB     0x05
#define ACCEL_Z_LSB     0x06
#define ACCEL_Z_MSB     0x07
#define ACCEL_RANGE_REG   0x0F
#define ACCEL_RANGE_2G  0x03  // 0011 0.98mg/LSB
#define ACCEL_RANGE_4G  0x05  // 0101 1.95mg/LSB
#define ACCEL_RANGE_8G  0x08  // 1000 3.91mg/LSB
#define ACCEL_RANGE_16G 0x0C  // 1100 7.81mg/LSB

// GYRO SETUP
#define GYRO_CHIP_ID_REG    0x00
#define GYRO_CHIP_ID_DEFAULT    0x0F
#define GYRO_ADDR1  0x68 //default address
#define GYRO_ADDR2  0x69
#define GYRO_X_LSB 0x02
#define GYRO_X_MSB 0x03
#define GYRO_Y_LSB 0x04
#define GYRO_Y_MSB 0x05
#define GYRO_Z_LSB 0x06
#define GYRO_Z_MSB 0x07

#define MGNT_ADDR  0x10 //default address

// GYRO FUNCTION
int GyroCheck(I2C_HandleTypeDef *hi2c1);
void ReadGyro(I2C_HandleTypeDef *hi2c1, float deltaTime); // deltaTime in seconds 10ms = 0.01s
char* getGyroPosAsString();
char* getGyroMovementAsString();

// ACCELERATOR FUNCTION
int AccelCheck(I2C_HandleTypeDef *hi2c1, uint8_t ACCEL_G, int *infoCode);
void ReadAccel(I2C_HandleTypeDef *hi2c1, float deltaTime);
char* getAccelAsString();
char* getVelocityAsString();
