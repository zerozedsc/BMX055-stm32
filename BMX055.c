/*
 * BMX055.c
 *
 *  Created on: Sep 9, 2023
 *      Author: MUHAMMAD HELMI BIN ROZAIN
 */

#include <stdio.h>
#include <string.h>
#include <BMX055.h>
#include <main.h>


int16_t raw_x, raw_y, raw_z;

int16_t ReadRawData(I2C_HandleTypeDef *hi2c, uint8_t SENSOR_ADDR, uint8_t lsbAddr, uint8_t msbAddr) {
    uint8_t lsb = 0;
    uint8_t msb = 0;

    HAL_I2C_Mem_Read(hi2c, SENSOR_ADDR << 1, lsbAddr, 1, &lsb, 1, 1000);
    HAL_I2C_Mem_Read(hi2c, SENSOR_ADDR << 1, msbAddr, 1, &msb, 1, 1000);

    return (int16_t) ((msb << 8) | lsb);
}


// gyroscope function
float POS_X = 0.0;
float POS_Y = 0.0;
float POS_Z = 0.0;
POS GYRO_POS = {0.0f, 0.0f, 0.0f};
POS GYRO_MOVEMENT = {0.0f, 0.0f, 0.0f};

uint8_t GYRO_ADDR = GYRO_ADDR1;

int GyroCheck(I2C_HandleTypeDef *hi2c1){
	uint8_t chipID = 0;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c1, GYRO_ADDR << 1, GYRO_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK && chipID == GYRO_CHIP_ID_DEFAULT) return 1;

	GYRO_ADDR = GYRO_ADDR2;
	status = HAL_I2C_Mem_Read(hi2c1, GYRO_ADDR << 1, GYRO_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK && chipID == GYRO_CHIP_ID_DEFAULT) return 1;

	return 0;
}

void UpdateRawGyroXYZ(I2C_HandleTypeDef *hi2c1){
	raw_x = ReadRawData(hi2c1, GYRO_ADDR, GYRO_X_LSB, GYRO_X_MSB);
	raw_y = ReadRawData(hi2c1, GYRO_ADDR, GYRO_Y_LSB, GYRO_Y_MSB);
	raw_z = ReadRawData(hi2c1, GYRO_ADDR, GYRO_Z_LSB, GYRO_Z_MSB);
}

void ReadGyro(I2C_HandleTypeDef *hi2c1, float deltaTime){
	UpdateRawGyroXYZ(hi2c1);

	float sensitivity = 0.0038; // This is hypothetical. Get the exact value from the datasheet.

	GYRO_MOVEMENT.x = raw_x * sensitivity;
	GYRO_MOVEMENT.y = raw_y * sensitivity;;
	GYRO_MOVEMENT.z = raw_z * sensitivity;;

	POS_X += GYRO_MOVEMENT.x * deltaTime;
	POS_Y += GYRO_MOVEMENT.y * deltaTime;
	POS_Z += GYRO_MOVEMENT.z * deltaTime;

	GYRO_POS.x = POS_X;
	GYRO_POS.y = POS_Y;
	GYRO_POS.z = POS_Z;

}

char* getGyroPosAsString() {
    char *buffer = malloc(100 * sizeof(char));
    size_t bufferSize = 100;
    int x_int = (int)GYRO_POS.x;
    int x_frac = (int)(100 * (GYRO_POS.x - x_int));

    int y_int = (int)GYRO_POS.y;
    int y_frac = (int)(100 * (GYRO_POS.y - y_int));

    int z_int = (int)GYRO_POS.z;
    int z_frac = (int)(100 * (GYRO_POS.z - z_int));

    snprintf(buffer, bufferSize, "GYRO_POS (degree) x: %d.%02d y: %d.%02d z: %d.%02d \n", x_int, x_frac, y_int, y_frac, z_int, z_frac);
    return buffer;
}

char* getGyroMovementAsString() {
    char *buffer = malloc(100 * sizeof(char));
    size_t bufferSize = 100;
    int x_int = (int)GYRO_POS.x;
    int x_frac = (int)(100 * (GYRO_MOVEMENT.x - x_int));

    int y_int = (int)GYRO_POS.y;
    int y_frac = (int)(100 * (GYRO_MOVEMENT.y - y_int));

    int z_int = (int)GYRO_POS.z;
    int z_frac = (int)(100 * (GYRO_MOVEMENT.z - z_int));

    snprintf(buffer, bufferSize, "GYRO_MOVEMENT (degree\s) x: %d.%02d y: %d.%02d z: %d.%02d \n", x_int, x_frac, y_int, y_frac, z_int, z_frac);
    return buffer;
}

// accelerometer function
POS VELOCITY = {0.0f, 0.0f, 0.0f};
POS ACCEL = {0.0f, 0.0f, 0.0f};

uint8_t ACCEL_ADDR = ACCEL_ADDR1;

int AccelCheck(I2C_HandleTypeDef *hi2c1, uint8_t ACCEL_G, int *infoCode){
	uint8_t chipID = 0;
	infoCode = 0;

	if (HAL_I2C_Mem_Write(hi2c1, ACCEL_ADDR<<1, ACCEL_RANGE_REG, 1, &ACCEL_G, 1, 1000) != HAL_OK) {
	   infoCode = 2;
	}

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c1, ACCEL_ADDR << 1, ACCEL_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK && chipID == ACCEL_CHIP_ID_DEFAULT) {
		infoCode = 1;
		return infoCode;
	}

	GYRO_ADDR = ACCEL_ADDR2;
	status = HAL_I2C_Mem_Read(hi2c1, GYRO_ADDR << 1, ACCEL_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK && chipID == ACCEL_CHIP_ID_DEFAULT) {
		infoCode = 1;
		return infoCode;
	}

	return infoCode;
}

void UpdateRawAccelXYZ(I2C_HandleTypeDef *hi2c1){
	raw_x = ReadRawData(hi2c1, ACCEL_ADDR, ACCEL_X_LSB, ACCEL_X_MSB);
	raw_y = ReadRawData(hi2c1, ACCEL_ADDR, ACCEL_Y_LSB, ACCEL_Y_MSB);
	raw_z = ReadRawData(hi2c1, ACCEL_ADDR, ACCEL_Z_LSB, ACCEL_Z_MSB);
}

void ReadAccel(I2C_HandleTypeDef *hi2c1, float deltaTime){
	UpdateRawAccelXYZ(hi2c1);

	float scaleFactor = (2.0 * 9.81) / 32768.0;
	ACCEL.x = raw_x * scaleFactor;
	ACCEL.y = raw_y * scaleFactor;
	ACCEL.z = raw_z * scaleFactor;

	VELOCITY.x += ACCEL.x * deltaTime;
	VELOCITY.y += ACCEL.y * deltaTime;
	VELOCITY.z += ACCEL.z * deltaTime;
}

char* getAccelAsString() {
    char *buffer = malloc(100 * sizeof(char));
    size_t bufferSize = 100;
    int x_int = (int)GYRO_POS.x;
    int x_frac = (int)(100 * (ACCEL.x - x_int));

    int y_int = (int)GYRO_POS.y;
    int y_frac = (int)(100 * (ACCEL.y - y_int));

    int z_int = (int)GYRO_POS.z;
    int z_frac = (int)(100 * (ACCEL.z - z_int));

    snprintf(buffer, bufferSize, "ACCELERATION (m/s^2) x: %d.%02d y: %d.%02d z: %d.%02d \n", x_int, x_frac, y_int, y_frac, z_int, z_frac);
    return buffer;
}

char* getVelocityAsString() {
    char *buffer = malloc(100 * sizeof(char));
    size_t bufferSize = 100;
    int x_int = (int)GYRO_POS.x;
    int x_frac = (int)(100 * (VELOCITY.x - x_int));

    int y_int = (int)GYRO_POS.y;
    int y_frac = (int)(100 * (VELOCITY.y - y_int));

    int z_int = (int)GYRO_POS.z;
    int z_frac = (int)(100 * (VELOCITY.z - z_int));

    snprintf(buffer, bufferSize, "VELOCITY (m/s) x: %d.%02d y: %d.%02d z: %d.%02d \n", x_int, x_frac, y_int, y_frac, z_int, z_frac);
    return buffer;
}

