/*
 * BMX055.c
 *
 *  Created on: Sep 9, 2023
 *      Author: MUHAMMAD HELMI BIN ROZAIN
 *
 *  C file for BMX055 9 AXIS BOSCH SENSOR
 */

#include <stdio.h>
#include <string.h>
#include <BMX055.h>
#include <math.h>
#include <main.h>
#include <float.h>

// test function
void Imu_init(I2C_HandleTypeDef hi2c1)
{
	HAL_I2C_Mem_Write(&hi2c1, 0x19<<1, 0x0F, I2C_MEMADD_SIZE_8BIT, 0x03, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, 0x19<<1, 0x10, I2C_MEMADD_SIZE_8BIT, 0x08, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, 0x19<<1, 0x11, I2C_MEMADD_SIZE_8BIT, 0x00, 1, 1000);

	HAL_I2C_Mem_Write(&hi2c1, 0x69<<1, 0x0F, I2C_MEMADD_SIZE_8BIT, 0x04, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, 0x69<<1, 0x10, I2C_MEMADD_SIZE_8BIT, 0x07, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, 0x69<<1, 0x11, I2C_MEMADD_SIZE_8BIT, 0x00, 1, 1000);

	HAL_I2C_Mem_Write(&hi2c1, 0x13<<1, 0x4B, I2C_MEMADD_SIZE_8BIT, 0x83, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, 0x13<<1, 0x4B, I2C_MEMADD_SIZE_8BIT, 0x01, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, 0x13<<1, 0x4C, I2C_MEMADD_SIZE_8BIT, 0x00, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, 0x13<<1, 0x4E, I2C_MEMADD_SIZE_8BIT, 0x84, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, 0x13<<1, 0x51, I2C_MEMADD_SIZE_8BIT, 0x04, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, 0x13<<1, 0x52, I2C_MEMADD_SIZE_8BIT, 0x16, 1, 1000);
}

// misc function
void floatToString(float value, char* buffer, int precision) {
    int integerPart = (int)value;
    int decimalPart = (value - integerPart) * pow(10, precision);

    // Correct for negative values of decimalPart when the float is negative.
    if (value < 0 && decimalPart != 0) {
        decimalPart = -decimalPart;
    }

    snprintf(buffer, 20, "%d.%0*d", integerPart, precision, decimalPart);  // Assuming buffer is at least 20 characters
}

// function for all sensor
int16_t raw_x, raw_y, raw_z, offset_x, offset_y, offset_z;

int16_t ReadRawData(I2C_HandleTypeDef *hi2c, uint8_t SENSOR_ADDR, uint8_t lsbAddr, uint8_t msbAddr) {
    uint8_t lsb = 0;
    uint8_t msb = 0;

    HAL_I2C_Mem_Read(hi2c, SENSOR_ADDR << 1, lsbAddr, 1, &lsb, 1, 1000);
    HAL_I2C_Mem_Read(hi2c, SENSOR_ADDR << 1, msbAddr, 1, &msb, 1, 1000);

    return (int16_t) ((msb << 8) | lsb);
}

int16_t ReadRawDataBurst(I2C_HandleTypeDef *hi2c, uint8_t SENSOR_ADDR, uint8_t lsbAddr) {
    uint8_t data[2];

    // Read both LSB and MSB in a single transaction
    HAL_I2C_Mem_Read(hi2c, SENSOR_ADDR << 1, lsbAddr, 1, data, 2, 1000);

    return (int16_t)((data[1] << 8) | data[0]);  // MSB is data[1] and LSB is data[0]
}

int16_t ReadOffsetData(I2C_HandleTypeDef *hi2c, uint8_t SENSOR_ADDR, uint8_t firstReg, uint8_t secondReg) {
    uint8_t firstByte = 0;
    uint8_t secondByte = 0;

    // Read the first and second byte
    HAL_I2C_Mem_Read(hi2c, SENSOR_ADDR << 1, firstReg, 1, &firstByte, 1, 1000);
    HAL_I2C_Mem_Read(hi2c, SENSOR_ADDR << 1, secondReg, 1, &secondByte, 1, 1000);

    // Construct the 12-bit offset value. Please adjust this according to the register's data arrangement.
    int16_t offsetValue = ((secondByte << 4) & 0x0FF0) | ((firstByte >> 3) & 0x000F);

    // Considering the data is 2's complement, adjusting for the negative values.
    if (offsetValue & 0x0800) {  // Checking the sign bit
        offsetValue |= 0xF000;  // Extending the sign bit
    }

    return offsetValue;
}

// gyroscope function
uint8_t GYRO_ADDR = GYRO_ADDR1;

POS GYRO_POS = {0.0f, 0.0f, 0.0f};
POS GYRO_MOVEMENT = {0.0f, 0.0f, 0.0f};
POS GYRO_OFFSET = {200.0f, 216.0f, 410.0f};
POS GYRO_RAW = {0.0f, 0.0f, 0.0f};

int checkGyro = 0, calibrateGyro = 0;

int GyroCheck(I2C_HandleTypeDef *hi2c){
	uint8_t chipID = 0;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, GYRO_ADDR << 1, GYRO_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK) return 1;
	else if (status == HAL_OK && chipID != GYRO_CHIP_ID_DEFAULT){
				return 2;
		}

	GYRO_ADDR = GYRO_ADDR2;
	status = HAL_I2C_Mem_Read(hi2c, GYRO_ADDR << 1, GYRO_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK) return 1;
	else if (status == HAL_OK && chipID != GYRO_CHIP_ID_DEFAULT){
				return 2;
		}

	return 0;
}

void GyroFilter(int16_t *raw_x, int16_t *raw_y, int16_t *raw_z){
	if (-GYRO_OFFSET.x < *raw_x && *raw_x < GYRO_OFFSET.x) {
	        *raw_x = 0;
	    } else {
	        *raw_x = (*raw_x > 0) ? (*raw_x - GYRO_OFFSET.x) : (*raw_x + GYRO_OFFSET.x);
	    }


	    if (-GYRO_OFFSET.y < *raw_y && *raw_y < GYRO_OFFSET.y) {
	        *raw_y = 0;
	    } else {
	        *raw_y = (*raw_y > 0) ? (*raw_y - GYRO_OFFSET.y) : (*raw_y + GYRO_OFFSET.y);
	    }


	    if (-GYRO_OFFSET.z < *raw_z && *raw_z < GYRO_OFFSET.z) {
	        *raw_z = 0;
	    } else {
	        *raw_z = (*raw_z > 0) ? (*raw_z - GYRO_OFFSET.z) : (*raw_z + GYRO_OFFSET.z);
	    }

}

void UpdateRawGyroXYZ(I2C_HandleTypeDef *hi2c1){
//    raw_x = ReadRawDataBurst(hi2c1, GYRO_ADDR, GYRO_X_LSB);
//    raw_y = ReadRawDataBurst(hi2c1, GYRO_ADDR, GYRO_Y_LSB);
//    raw_z = ReadRawDataBurst(hi2c1, GYRO_ADDR, GYRO_Z_LSB);

    raw_x = ReadRawData(hi2c1, GYRO_ADDR, GYRO_X_LSB, GYRO_X_MSB);
    raw_y = ReadRawData(hi2c1, GYRO_ADDR, GYRO_Y_LSB, GYRO_Y_MSB);
    raw_z = ReadRawData(hi2c1, GYRO_ADDR, GYRO_Z_LSB, GYRO_Z_MSB);

    GyroFilter(&raw_x, &raw_y, &raw_z);


    GYRO_RAW.x = raw_x;
    GYRO_RAW.y = raw_y;
    GYRO_RAW.z = raw_z;
}

void ReadGyro(I2C_HandleTypeDef *hi2c, float deltaTime){
	UpdateRawGyroXYZ(hi2c);


	float sensitivity = GYRO_SENSITIVITY; // This is hypothetical. Get the exact value from the datasheet.

	    // Convert the corrected values into movements with sensitivity scaling
	GYRO_MOVEMENT.x = (raw_x * sensitivity * GYRO_ERROR_MULTIPLIER) + GYRO_ERROR_CONSTANT;
	GYRO_MOVEMENT.y = (raw_y * sensitivity * GYRO_ERROR_MULTIPLIER) + GYRO_ERROR_CONSTANT;
	GYRO_MOVEMENT.z = (raw_z * sensitivity * GYRO_ERROR_MULTIPLIER) + GYRO_ERROR_CONSTANT;

	    // Integrate the rate of change to get the change in orientation
	GYRO_POS.x += GYRO_MOVEMENT.x * deltaTime;
	GYRO_POS.y += GYRO_MOVEMENT.y * deltaTime;
	GYRO_POS.z += GYRO_MOVEMENT.z * deltaTime;

}

const char* getGyroPosAsString() {
    char *buffer = malloc(100 * sizeof(char));
    size_t bufferSize = 100;
    char x[10], y[10], z[10];
    floatToString(GYRO_POS.x, x, 4);
    floatToString(GYRO_POS.y, y, 4);
    floatToString(GYRO_POS.z, z, 4);


    snprintf(buffer, bufferSize, "GYRO_POS (degree) x: %s y: %s z: %s", x, y, z);
    return buffer;
}

const char* getGyroMovementAsString() {
    char *buffer = malloc(100 * sizeof(char));
    size_t bufferSize = 100;
    char x[10], y[10], z[10];
    floatToString(GYRO_MOVEMENT.x, x, 4);
    floatToString(GYRO_MOVEMENT.y, y, 4);
    floatToString(GYRO_MOVEMENT.z, z, 4);

    snprintf(buffer, bufferSize, "GYRO_MOVEMENT (degreePsec) x: %s y: %s z: %s", x, y, z);
    return buffer;
}

// accelerometer function
uint8_t ACCEL_ADDR = ACCEL_ADDR1;

POS VELOCITY = {0.0f, 0.0f, 0.0f};
POS PREVIOUS_VELOCITY = {0.0f, 0.0f, 0.0f};
POS ACCEL = {0.0f, 0.0f, 0.0f};
POS PREVIOUS_ACCEL = {0.0f, 0.0f, 0.0f};
POS ACCEL_OFFSET = {-10.0f, 420.0f, 4090.0f};
POS ACCEL_RAW = {0.0f, 0.0f, 0.0f};

int checkAccel = 0, accelCode = 0;

int AccelCheck(I2C_HandleTypeDef *hi2c, uint8_t ACCEL_G){
	uint8_t chipID = 0;

	if (HAL_I2C_Mem_Write(hi2c, ACCEL_ADDR<<1, ACCEL_RANGE_REG, 1, &ACCEL_G, 1, 1000) != HAL_OK) {
		accelCode = 3;
	}


	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, ACCEL_ADDR << 1, ACCEL_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK && chipID == ACCEL_CHIP_ID_DEFAULT) {
		if (accelCode == 0) accelCode = 1;
		return 1;
	}
	else if (status == HAL_OK && chipID != ACCEL_CHIP_ID_DEFAULT){
			if (accelCode == 0) accelCode = 2;
			return 2;
	}

	ACCEL_ADDR = ACCEL_ADDR2;
	status = HAL_I2C_Mem_Read(hi2c, ACCEL_ADDR << 1, ACCEL_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK && chipID == ACCEL_CHIP_ID_DEFAULT) {
		if (accelCode == 0) accelCode = 1;
		return 1;
	}
	else if (status == HAL_OK && chipID != ACCEL_CHIP_ID_DEFAULT){
			if (accelCode == 0) accelCode = 2;
			return 2;
	}

	return 0;
}

void AccelFilter(int16_t *raw_x, int16_t *raw_y, int16_t *raw_z){
	const float upper_x = 2.0f; // because x data is below 0, we not use ACCEL_OFFSET
	    const float lower_x = -247.0f;

	    if (lower_x < *raw_x && *raw_x < upper_x) {
	        *raw_x = 0;
	    } else {
	    	if (*raw_x < lower_x) *raw_x -= lower_x;
	    	else if (*raw_x > upper_x) *raw_x -= upper_x;
	    }


	    if (-ACCEL_OFFSET.y < *raw_y && *raw_y < ACCEL_OFFSET.y) {
	        *raw_y = 0;
	    } else {
	        *raw_y = (*raw_y > 0) ? (*raw_y - ACCEL_OFFSET.y) : (*raw_y + ACCEL_OFFSET.y);
	    }


	    if (-ACCEL_OFFSET.z < *raw_z && *raw_z < ACCEL_OFFSET.z) {
	        *raw_z = 0;
	    } else {
	        *raw_z = (*raw_z > 0) ? (*raw_z - ACCEL_OFFSET.z) : (*raw_z + ACCEL_OFFSET.z);
	    }
}


void UpdateRawAccelXYZ(I2C_HandleTypeDef *hi2c){
    raw_x = ReadRawData(hi2c, ACCEL_ADDR, ACCEL_X_LSB, ACCEL_X_MSB);
    raw_y = ReadRawData(hi2c, ACCEL_ADDR, ACCEL_Y_LSB, ACCEL_Y_MSB);
    raw_z = ReadRawData(hi2c, ACCEL_ADDR, ACCEL_Z_LSB, ACCEL_Z_MSB);

    AccelFilter(&raw_x, &raw_y, &raw_z);

    ACCEL_RAW.x = raw_x;
    ACCEL_RAW.y = raw_y;
    ACCEL_RAW.z = raw_z;
}


void ReadAccel(I2C_HandleTypeDef *hi2c, float deltaTime){
	UpdateRawAccelXYZ(hi2c);

//	ACCEL_OFFSET.x = offset_x;
//	ACCEL_OFFSET.y = offset_y;
//	ACCEL_OFFSET.z = offset_z;

	// Apply offsets to the raw values
	int16_t corrected_x = raw_x - GYRO_OFFSET.x;
	int16_t corrected_y = raw_y - GYRO_OFFSET.y;
	int16_t corrected_z = raw_z - GYRO_OFFSET.z;

	float scaleFactor = (2.0 * 9.81) / 32768.0;
	float ACCEL_X = (corrected_x * scaleFactor * ACCEL_ERROR_MULTIPLIER) + ACCEL_ERROR_CONSTANT;
	float ACCEL_Y = (corrected_y * scaleFactor * ACCEL_ERROR_MULTIPLIER) + ACCEL_ERROR_CONSTANT;
	float ACCEL_Z = (corrected_z * scaleFactor * ACCEL_ERROR_MULTIPLIER) + ACCEL_ERROR_CONSTANT;

	if (ACCEL_X != PREVIOUS_ACCEL.x || ACCEL_Y != PREVIOUS_ACCEL.y || ACCEL_Z != PREVIOUS_ACCEL.z) PREVIOUS_ACCEL = ACCEL;
	ACCEL.x = ACCEL_X;
	ACCEL.y = ACCEL_Y;
	ACCEL.z = ACCEL_Z;

	float VELO_X = ACCEL.x * deltaTime;
	float VELO_Y = ACCEL.y * deltaTime;
	float VELO_Z = ACCEL.z * deltaTime;

	if (VELO_X != PREVIOUS_VELOCITY.x || VELO_Y != PREVIOUS_VELOCITY.y || VELO_Z != PREVIOUS_VELOCITY.z) PREVIOUS_VELOCITY = VELOCITY;

	VELOCITY.x += VELO_X;
	VELOCITY.y += VELO_Y;
	VELOCITY.z += VELO_Z;
}

const char* getAccelAsString() {
    char *buffer = malloc(100 * sizeof(char));
    size_t bufferSize = 100;
    char x[10], y[10], z[10];
    floatToString(ACCEL.x, x, 4);
    floatToString(ACCEL.y, y, 4);
    floatToString(ACCEL.z, z, 4);

    snprintf(buffer, bufferSize, "ACCELERATION (m/s^2) x: %s y: %s z: %s",x,y,z);
    return buffer;
}

const char* getVelocityAsString() {
    char *buffer = malloc(100 * sizeof(char));
    size_t bufferSize = 100;
    char x[10], y[10], z[10];
    floatToString(VELOCITY.x, x, 4);
    floatToString(VELOCITY.y, y, 4);
    floatToString(VELOCITY.z, z, 4);

    snprintf(buffer, bufferSize, "VELOCITY (m/s) x: %s y: %s z: %s",x,y,z);
    return buffer;
}

// magnetometer function
uint8_t MAG_ADDR = MAG_ADDR1;

POS MAG_POS = {0.0f, 0.0f, 0.0f};
POS MAG_RAW = {0.0f, 0.0f, 0.0f};

POS AVERAGE_DEGREE10[DEGREE10] = {
		{ 0.0, 0.0, 0.0 }, // 0
		    { 0.0, 1.8, 0.0 }, // 10
		    { -2.0, 3.2, 0.0 }, // 20
		    { -9.8, 0.0, 0.0 }, // 30
		    { -18.0, 5.2, 0.0 }, // 40
		    { -42.0, 3.2, 0.0 }, // 50
		    { -46.8, 6.4, 0.0 }, // 60
		    { -52.8, 5.6, 0.0 }, // 70
		    { -75.4, 0.0, 0.0 }, // 80
		    { -79.2, -2.2, 0.0 }, // 90
		    { -98.0, 0.0, 0.0 }, // 100
		    { -110.0, -4.4, 0.0 }, // 110
		    { -114.8, -19.8, 0.0 }, // 120
		    { -111.6, -34.2, 0.0 }, // 130
		    { -114.8, -46.2, 0.0 }, // 140
		    { -115.4, -59.8, 0.0 }, // 150
		    { -114.8, -76.6, 0.0 }, // 160
		    { -98.0, -86.2, 0.0 }, // 170
		    { -80.0, -100.2, 0.0 }, // 180
		    { -71.6, -105.8, 0.0 }, // 190
		    { -53.4, -115.0, 0.0 }, // 200
		    { -41.2, -119.8, 0.0 }, // 210
		    { -20.4, -115.0, 0.0 }, // 220
		    { -14.2, -113.4, 0.0 }, // 230
		    { -5.6, -119.0, 0.0 }, // 240
		    { 0.0, -110.2, 0.0 }, // 250
		    { 0.0, -103.0, 0.0 }, // 260
		    { 0.0, -85.8, 0.0 }, // 270
		    { 0.0, -79.8, 0.0 }, // 280
		    { 0.0, -60.0, 0.0 }, // 290
		    { 0.0, -57.0, 0.0 }, // 300
		    { 0.0, -45.0, 0.0 }, // 310
		    { 0.0, -36.0, 0.0 }, // 320
		    { 0.0, -14.4, 0.0 }, // 330
		    { 0.0, -8.4, 0.0 }, // 340
		    { 0.0, -1.8, 0.0 } // 350
};

float MAGDEGREE=0, heading=0;
int checkMag= 0, magCounter=0;
float magRaw10[2][10] = {0};

void InitMagnetometer(I2C_HandleTypeDef *hi2c) {
    uint8_t regValue = 0;

    // Soft reset
    regValue = 0x82; // Setting both soft reset bits
    HAL_I2C_Mem_Write(hi2c, MAG_ADDR << 1, 0x4B, 1, &regValue, 1, HAL_MAX_DELAY);
    HAL_Delay(50); // Wait for reset to complete

    // Power up the magnetometer to Sleep mode (transition step to Active mode)
    regValue = 0x01; // Setting the power control bit to 1 (Sleep mode)
    HAL_I2C_Mem_Write(hi2c, MAG_ADDR << 1, 0x4B, 1, &regValue, 1, HAL_MAX_DELAY);
    HAL_Delay(10); // Wait to stabilize
}

HAL_StatusTypeDef CheckAndSetActiveModeMag(I2C_HandleTypeDef *hi2c) {
    uint8_t opMode = NORMAL_MODE; // Set to Normal Mode

    // Write the mode to the operation mode register
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, (MAG_ADDR << 1), MAG_OP_MODE_REG, 1, &opMode, 1, HAL_MAX_DELAY);

    return status;
}

int MagCheck(I2C_HandleTypeDef *hi2c) {
    uint8_t chipID = 0;
    int infoCode = 0;
    HAL_StatusTypeDef status;
    uint8_t addresses[] = {MAG_ADDR1, MAG_ADDR2, MAG_ADDR3, MAG_ADDR4}; // assuming these are defined globally
    int num_addresses = sizeof(addresses) / sizeof(addresses[0]);

    for (int i = 0; i < num_addresses; i++) {
        MAG_ADDR = addresses[i];
        InitMagnetometer(hi2c);
        status = CheckAndSetActiveModeMag(hi2c);

        if (status != HAL_OK) {
            continue; // If there's an error setting the mode, try the next address
        }

        status = HAL_I2C_Mem_Read(hi2c, MAG_ADDR << 1, MAG_CHIP_ID_REG, 1, &chipID, 1, HAL_MAX_DELAY);
        if (status == HAL_OK && chipID == MAG_CHIP_ID_DEFAULT) {
            infoCode = 1; // Successfully found and initialized the magnetometer
            break;
        }
    }

    return infoCode;
}

void ReadRawBurstMag(I2C_HandleTypeDef *hi2c, int16_t *dataX, int16_t *dataY, int16_t *dataZ) {
    uint8_t rawData[6] = {0}; // 2 bytes each for X, Y, and Z

    // Burst read from the magnetometer
    if (HAL_I2C_Mem_Read(hi2c, MAG_ADDR << 1, MAG_X_LSB, 1, rawData, 6, 1000) != HAL_OK) {
        *dataX = *dataY = *dataZ = -200;
        return;
    }

    // Extract X data
       *dataX = ((int16_t)rawData[1] << 5) | (rawData[0] & 0x1F);
       if (*dataX & (1 << 12)) { // If 12th bit is set, the number is negative
           *dataX -= (1 << 13); // Convert to two's complement
       }

       // Extract Y data
       *dataY = ((int16_t)rawData[3] << 5) | (rawData[2] & 0x1F);
       if (*dataY & (1 << 12)) {
           *dataY -= (1 << 13);
       }

       // Extract Z data
       *dataZ = ((int16_t)rawData[5] << 7) | (rawData[4] & 0x7F);
       if (*dataZ & (1 << 14)) {
           *dataZ -= (1 << 15);
       }
}

void MagFilter(int16_t *raw_x, int16_t *raw_y, int16_t *raw_z){
	const float upper_x = 0.0, lower_x=-65.0, //-70.0, 0
				upper_y = 45.0, lower_y = 0.0, //60.0, 0
				upper_z = -100.0, lower_z = -300.0; //-100, -300

		if (lower_x < *raw_x && *raw_x < upper_x) {
		        *raw_x = 0;
		    } else {
		    	if (*raw_x < lower_x) *raw_x -= lower_x;
		    	else if (*raw_x > upper_x) *raw_x -= upper_x;
		    }

		if (lower_y < *raw_y && *raw_y < upper_y) {
		        *raw_y = 0;
		    } else {
		    	if (*raw_y < lower_y) *raw_y -= lower_y;
		    	else if (*raw_y > upper_y) *raw_y -= upper_y;
		    }

		if (lower_z < *raw_z && *raw_z < upper_z) {
		        *raw_z = 0;
		    } else {
		    	if (*raw_z < lower_z) *raw_z -= lower_z;
		    	else if (*raw_z > upper_z) *raw_z -= upper_z;
		    }
}

void UpdateRawMagXYZ(I2C_HandleTypeDef *hi2c){
	ReadRawBurstMag(hi2c, &raw_x, &raw_y, &raw_z);
	MagFilter(&raw_x, &raw_y, &raw_z);

	MAG_RAW.x = raw_x;
	MAG_RAW.y = raw_y;
	MAG_RAW.z = raw_z;

}

const int DetermineOrientation(float x, float y, float threshold) {
	float min_difference = FLT_MAX;
	int matching_key = -1;
	float X=0, Y=0;
	magRaw10[0][magCounter%10] = x;
	magRaw10[1][magCounter%10] = y;

	for (int j=0; j<10; j++){
		X+=magRaw10[0][j];
		Y+=magRaw10[1][j];
	}
	X/=10;
	Y/=10;

	for (int i=0; i<DEGREE10; i++){
		float difference_x = X - AVERAGE_DEGREE10[i].x;
		difference_x = difference_x >= 0 ? difference_x:difference_x*-1;
		float difference_y = Y - AVERAGE_DEGREE10[i].y;
		difference_y = difference_y >= 0 ? difference_y:difference_y*-1;

		float total_difference = difference_x + difference_y;

		if (total_difference < threshold){
			matching_key = i * 10;
			break;
		}
		else if (total_difference < min_difference){
			min_difference = total_difference;
			matching_key = i * 10;
		}

	}

	return matching_key;

}

void ReadMag(I2C_HandleTypeDef *hi2c) {
	UpdateRawMagXYZ(hi2c);

	MAG_POS.x = raw_x;
	MAG_POS.y = raw_y;
	MAG_POS.z = raw_z;

    MAGDEGREE = (float)DetermineOrientation(MAG_POS.x, MAG_POS.y, 5.0);
    magCounter++;
}


const char* getOrientationAsString(){
//	if (orientation == 1) return "Orientation: NORTH";
//	if (orientation == 12) return "Orientation: NORTH-EAST";
//	if (orientation == 13) return "Orientation: NORTH-WEST";
//	if (orientation == 2) return "Orientation: EAST";
//	if (orientation == 3) return "Orientation: WEST";
//	if (orientation == 4) return "Orientation: SOUTH";
//	if (orientation == 42) return "Orientation: SOUTH-EAST";
//	if (orientation == 43) return "Orientation: SOUTH-WEST";
	char *buffer = malloc(256 * sizeof(char));
	size_t bufferSize = 256;
	char degreeStr[10];
	floatToString(MAGDEGREE, degreeStr, 2);
	snprintf(buffer, bufferSize, "DEGREE : %s",degreeStr);

	return buffer;

}

const char* getMagPosAsString() {
    char *buffer = malloc(256 * sizeof(char));
    size_t bufferSize = 256;
    char x[10], y[10], z[10];
    floatToString(MAG_POS.x, x, 4);
    floatToString(MAG_POS.y, y, 4);
    floatToString(MAG_POS.z, z, 4);

    snprintf(buffer, bufferSize, "MAG_POS (degree) : %s y: %s z: %s",x,y,z);
    return buffer;
}

// function that use all 3 sensor(data) in one time
POS CURRENT_POS = {0.0f, 0.0f, 0.0f};
POS PREVIOUS_POS = {0.0f, 0.0f, 0.0f};

float currentHeading = 0.0f;
int ERROR_PRINT[3] = {0,0,0};
float BMX055_data[BMX055_SIZE], BMX055_raw[9];

void BMX055_init(I2C_HandleTypeDef *hi2c, float timer){ //initialization for BMX055
	checkGyro = GyroCheck(hi2c);
	checkAccel = AccelCheck(hi2c, ACCEL_RANGE_8G);
	checkMag = MagCheck(hi2c);

	if (checkGyro == 1) ReadGyro(hi2c, timer);
	if (checkAccel == 1) ReadAccel(hi2c, timer);
	if (checkMag == 1) ReadMag(hi2c);

//	BMX055_data[0] = GYRO_MOVEMENT.x;
//	BMX055_data[1] = GYRO_MOVEMENT.y;
//	BMX055_data[2] = GYRO_MOVEMENT.z;
//	BMX055_data[3] = GYRO_POS.x;
//	BMX055_data[4] = GYRO_POS.y;
//	BMX055_data[5] = GYRO_POS.z;
//	BMX055_data[6] = ACCEL.x;
//	BMX055_data[7] = ACCEL.y;
//	BMX055_data[8] = ACCEL.z;
//	BMX055_data[9] = VELOCITY.x;
//	BMX055_data[10] = VELOCITY.y;
//	BMX055_data[11] = VELOCITY.z;
//	BMX055_data[12] = MAG_POS.x;
//	BMX055_data[13] = MAG_POS.y;
//	BMX055_data[14] = MAG_POS.z;
//	BMX055_data[15] = GYRO_OFFSET.x;
//	BMX055_data[16] = GYRO_OFFSET.y;
//	BMX055_data[17] = GYRO_OFFSET.z;
//	BMX055_data[18] = ACCEL_OFFSET.x;
//	BMX055_data[19] = ACCEL_OFFSET.y;
//	BMX055_data[20] = ACCEL_OFFSET.z;
//	BMX055_data[21] = degree;

	BMX055_data[0] = MAGDEGREE;

	BMX055_raw[0] = GYRO_RAW.x;
	BMX055_raw[1] = GYRO_RAW.y;
	BMX055_raw[2] = GYRO_RAW.z;
	BMX055_raw[3] = ACCEL_RAW.x;
	BMX055_raw[4] = ACCEL_RAW.y;
	BMX055_raw[5] = ACCEL_RAW.z;
	BMX055_raw[6] = MAG_RAW.x;
	BMX055_raw[7] = MAG_RAW.y;
	BMX055_raw[8] = MAG_RAW.z;
}

const RobotActivity DetectActivity() {

    // Calculate magnitudes
    float accel_magnitude = sqrt(ACCEL.x*ACCEL.x + ACCEL.y*ACCEL.y + ACCEL.z*ACCEL.z);
    float gyro_magnitude = sqrt(GYRO_POS.x*GYRO_POS.x + GYRO_POS.y*GYRO_POS.y + GYRO_POS.z*GYRO_POS.z);

    // Stationary check: Gyroscope and Accelerometer readings are low.
    if(accel_magnitude < ACCEL_NOISE_THRESHOLD && gyro_magnitude < GYRO_NOISE_THRESHOLD) {
        return STATIONARY;
    }

    // Collision check: A sudden sharp acceleration.
    if(accel_magnitude > COLLISION_THRESHOLD) {
        return COLLISION;
    }

    // Flipping check: High rotation on one of the axes.
    if(fabs(GYRO_POS.x) > FLIPPING_THRESHOLD || fabs(GYRO_POS.y) > FLIPPING_THRESHOLD || fabs(GYRO_POS.z) > FLIPPING_THRESHOLD) {
        return FLIPPING;
    }

    // Falling check: The Z acceleration is different from gravity and the total acceleration is close to zero.
    float total_without_gravity = accel_magnitude - GRAVITY;
    if(fabs(ACCEL.z - GRAVITY) > FALLING_THRESHOLD_Z && total_without_gravity < FALLING_THRESHOLD_TOTAL) {
       return FALLING;
    }

    // Turning check: Gyroscope detects rotation but no significant linear acceleration.
    if(gyro_magnitude > GYRO_NOISE_THRESHOLD && accel_magnitude < ACCEL_NOISE_THRESHOLD) {
        return TURNING;
    }

    // Moving straight check: Linear acceleration but no significant rotation.
    if(accel_magnitude > ACCEL_NOISE_THRESHOLD && gyro_magnitude < GYRO_NOISE_THRESHOLD) {
        return MOVING_STRAIGHT;
    }

    // If none of the above, return unknown.
    return UNKNOWN;
}

const char* getDetectActivityAsString(RobotActivity activity){
	switch (activity) {
	        case STATIONARY:      return "Stationary";
	        case MOVING_STRAIGHT: return "Moving Straight";
	        case TURNING:         return "Turning";
	        case COLLISION:       return "Collision";
	        case UNKNOWN:         return "Unknown";
	        default:              return "Invalid";  // Just in case
	    }
}

void UpdatePosition(float deltaTime) {
    // Update the heading based on the gyroscope data (z-axis, assuming yaw rotation)
	currentHeading += GYRO_MOVEMENT.z * deltaTime;

    // Convert the heading to radians for trigonometry functions
    float heading_rad = currentHeading * M_PI / 180.0f;

    // Integrate velocity to get displacement
    float dx = VELOCITY.x * deltaTime;
    float dy = VELOCITY.y * deltaTime;

    // Update the position based on the heading
    float POS_X = dx * cos(heading_rad) - dy * sin(heading_rad);
    float POS_Y = dx * sin(heading_rad) + dy * cos(heading_rad);

    if (POS_X != PREVIOUS_POS.x || POS_Y != PREVIOUS_POS.y) PREVIOUS_POS = CURRENT_POS;

    CURRENT_POS.x += POS_X;
    CURRENT_POS.y += POS_Y;
}

const float CalculateDistanceTravelled() {
    float dx = CURRENT_POS.x;
    float dy = CURRENT_POS.y;
    return sqrt(dx*dx + dy*dy);  // Pythagoras theorem for distance in 2D
}

int isClimbing() {
    // Calculate the total acceleration magnitude
    float total_accel = sqrt(ACCEL.x*ACCEL.x + ACCEL.y*ACCEL.y + ACCEL.z*ACCEL.z);

    // Check if the robot is tilted by comparing the Z acceleration to the magnitude of gravity.
    // Note: We assume that there's not much external acceleration, which might not always be the case (like when starting or stopping).
    if(fabs(total_accel - GRAVITY) > TILT_THRESHOLD) {
        return 1;  // The robot is tilted which might imply it's climbing.
    }
    return 0;  // The robot is not climbing.
}

WheelStatus DetectWheelSlip(int encoderCount) {

    // Calculate change in accelerometer data
    float deltaAccelX = fabs(ACCEL.x - PREVIOUS_ACCEL.x);
    float deltaAccelY = fabs(ACCEL.y - PREVIOUS_ACCEL.y);

    // Check if wheels are turning but accelerometer data shows little to no movement
    if (encoderCount > ENCODER_THRESHOLD && (deltaAccelX < ACCEL_THRESHOLD && deltaAccelY < ACCEL_THRESHOLD)) {
        return SLIPPING;
    }

    return NOT_SLIPPING;
}

const char* getWheelSlipAsString(WheelStatus wheel){
	switch (wheel) {
	        case NOT_SLIPPING:      return "NOT_SLIPPING";
	        case SLIPPING:          return "SLIPPING";
	        default:                return "Invalid";  // Just in case
	    }
}

const char* getCurrentPosAsString() {
    char *buffer = malloc(100 * sizeof(char));
    size_t bufferSize = 100;
    char x[10], y[10], z[10];
    floatToString(CURRENT_POS.x, x, 4);
    floatToString(CURRENT_POS.y, y, 4);
    floatToString(CURRENT_POS.z, z, 4);

    snprintf(buffer, bufferSize, "CURRENT POS (m/s) x: %s y: %s z: %s",x,y,z);
    return buffer;
}

const char* getPreviousPosAsString() {
    char *buffer = malloc(100 * sizeof(char));
    size_t bufferSize = 100;
    char x[10], y[10], z[10];
    floatToString(PREVIOUS_POS.x, x, 4);
    floatToString(PREVIOUS_POS.y, y, 4);
    floatToString(PREVIOUS_POS.z, z, 4);

    snprintf(buffer, bufferSize, "PREV POS (m/s) x: %s y: %s z: %s",x,y,z);
    return buffer;
}


