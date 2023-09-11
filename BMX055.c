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


int16_t raw_x, raw_y, raw_z;

int16_t ReadRawData(I2C_HandleTypeDef *hi2c, uint8_t SENSOR_ADDR, uint8_t lsbAddr, uint8_t msbAddr) {
    uint8_t lsb = 0;
    uint8_t msb = 0;

    HAL_I2C_Mem_Read(hi2c, SENSOR_ADDR << 1, lsbAddr, 1, &lsb, 1, 1000);
    HAL_I2C_Mem_Read(hi2c, SENSOR_ADDR << 1, msbAddr, 1, &msb, 1, 1000);

    return (int16_t) ((msb << 8) | lsb);
}

// gyroscope function
uint8_t GYRO_ADDR = GYRO_ADDR1;

POS GYRO_POS = {0.0f, 0.0f, 0.0f};
POS GYRO_MOVEMENT = {0.0f, 0.0f, 0.0f};

int checkGyro = 0;

int GyroCheck(I2C_HandleTypeDef *hi2c1){
	uint8_t chipID = 0;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c1, GYRO_ADDR << 1, GYRO_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK && chipID == GYRO_CHIP_ID_DEFAULT) return 1;
	else if (status == HAL_OK && chipID != GYRO_CHIP_ID_DEFAULT){
				return 2;
		}

	GYRO_ADDR = GYRO_ADDR2;
	status = HAL_I2C_Mem_Read(hi2c1, GYRO_ADDR << 1, GYRO_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK && chipID == GYRO_CHIP_ID_DEFAULT) return 1;
	else if (status == HAL_OK && chipID != GYRO_CHIP_ID_DEFAULT){
				return 2;
		}

	return 0;
}

void UpdateRawGyroXYZ(I2C_HandleTypeDef *hi2c1){
	raw_x = ReadRawData(hi2c1, GYRO_ADDR, GYRO_X_LSB, GYRO_X_MSB);
	raw_y = ReadRawData(hi2c1, GYRO_ADDR, GYRO_Y_LSB, GYRO_Y_MSB);
	raw_z = ReadRawData(hi2c1, GYRO_ADDR, GYRO_Z_LSB, GYRO_Z_MSB);
}

void ReadGyro(I2C_HandleTypeDef *hi2c1, float deltaTime){
	UpdateRawGyroXYZ(hi2c1);

	float sensitivity = GYRO_SENSITIVITY; // This is hypothetical. Get the exact value from the datasheet.

	GYRO_MOVEMENT.x = (raw_x * sensitivity * GYRO_ERROR_MULTIPLIER) + GYRO_ERROR_CONSTANT;
	GYRO_MOVEMENT.y = (raw_y * sensitivity * GYRO_ERROR_MULTIPLIER) + GYRO_ERROR_CONSTANT;
	GYRO_MOVEMENT.z = (raw_z * sensitivity * GYRO_ERROR_MULTIPLIER) + GYRO_ERROR_CONSTANT;

	// Integrate the rate of change to get the change in orientation (in degrees)
	GYRO_POS.x += GYRO_MOVEMENT.x * deltaTime;
	GYRO_POS.y += GYRO_MOVEMENT.y * deltaTime;
	GYRO_POS.z += GYRO_MOVEMENT.z * deltaTime;

}

const char* getGyroPosAsString() {
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

const char* getGyroMovementAsString() {
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
uint8_t ACCEL_ADDR = ACCEL_ADDR1;

POS VELOCITY = {0.0f, 0.0f, 0.0f};
POS PREVIOUS_VELOCITY = {0.0f, 0.0f, 0.0f};
POS ACCEL = {0.0f, 0.0f, 0.0f};
POS PREVIOUS_ACCEL = {0.0f, 0.0f, 0.0f};

int checkAccel = 0, accelCode = 0;

int AccelCheck(I2C_HandleTypeDef *hi2c1, uint8_t ACCEL_G){
	uint8_t chipID = 0;

	if (HAL_I2C_Mem_Write(hi2c1, ACCEL_ADDR<<1, ACCEL_RANGE_REG, 1, &ACCEL_G, 1, 1000) != HAL_OK) {
		accelCode = 3;
	}


	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c1, ACCEL_ADDR << 1, ACCEL_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK && chipID == ACCEL_CHIP_ID_DEFAULT) {
		if (accelCode == 0) accelCode = 1;
		return 1;
	}
	else if (status == HAL_OK && chipID != ACCEL_CHIP_ID_DEFAULT){
			if (accelCode == 0) accelCode = 2;
			return 2;
	}

	ACCEL_ADDR = ACCEL_ADDR2;
	status = HAL_I2C_Mem_Read(hi2c1, ACCEL_ADDR << 1, ACCEL_CHIP_ID_REG, 1, &chipID, 1, 1000);
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

void UpdateRawAccelXYZ(I2C_HandleTypeDef *hi2c1){
	raw_x = ReadRawData(hi2c1, ACCEL_ADDR, ACCEL_X_LSB, ACCEL_X_MSB);
	raw_y = ReadRawData(hi2c1, ACCEL_ADDR, ACCEL_Y_LSB, ACCEL_Y_MSB);
	raw_z = ReadRawData(hi2c1, ACCEL_ADDR, ACCEL_Z_LSB, ACCEL_Z_MSB);
}

void ReadAccel(I2C_HandleTypeDef *hi2c1, float deltaTime){
	UpdateRawAccelXYZ(hi2c1);

	float scaleFactor = (2.0 * 9.81) / 32768.0;
	float ACCEL_X = (raw_x * scaleFactor * ACCEL_ERROR_MULTIPLIER) + ACCEL_ERROR_CONSTANT;
	float ACCEL_Y = (raw_y * scaleFactor * ACCEL_ERROR_MULTIPLIER) + ACCEL_ERROR_CONSTANT;
	float ACCEL_Z = (raw_z * scaleFactor * ACCEL_ERROR_MULTIPLIER) + ACCEL_ERROR_CONSTANT;

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
    int x_int = (int)GYRO_POS.x;
    int x_frac = (int)(100 * (ACCEL.x - x_int));

    int y_int = (int)GYRO_POS.y;
    int y_frac = (int)(100 * (ACCEL.y - y_int));

    int z_int = (int)GYRO_POS.z;
    int z_frac = (int)(100 * (ACCEL.z - z_int));

    snprintf(buffer, bufferSize, "ACCELERATION (m/s^2) x: %d.%02d y: %d.%02d z: %d.%02d \n", x_int, x_frac, y_int, y_frac, z_int, z_frac);
    return buffer;
}

const char* getVelocityAsString() {
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

// magnetometer function
POS MAG_POS = {0.0f, 0.0f, 0.0f};

float orientation=0, heading=0;
int checkMag= 0;

int MagCheck(I2C_HandleTypeDef *hi2c1){
	uint8_t chipID = 0;
	int infoCode = 0;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c1, MAG_ADDR << 1, MAG_CHIP_ID_REG, 1, &chipID, 1, 1000);
	if (status == HAL_OK && chipID == MAG_CHIP_ID_DEFAULT) {
		infoCode = 1;
		return infoCode;
	}
	else if (status == HAL_OK && chipID != MAG_CHIP_ID_DEFAULT){
		infoCode = 2;
	}

	return infoCode;

}

void UpdateRawMagXYZ(I2C_HandleTypeDef *hi2c1){
	raw_x = ReadRawData(hi2c1, MAG_ADDR, MAG_X_LSB, MAG_X_MSB);
	raw_y = ReadRawData(hi2c1, MAG_ADDR, MAG_Y_LSB, MAG_Y_MSB);
	raw_z = ReadRawData(hi2c1, MAG_ADDR, MAG_Z_LSB, MAG_Z_MSB);
}

const int DetermineOrientation(float x, float y) {
    heading = atan2(y, x) * (180.0 / M_PI); // Calculate the angle

    if (heading < 0)
        heading += 360;

    if ((heading >= 0 && heading < 45) || (heading >= 315 && heading <= 360))
        return 1; // North
    if (heading >= 45 && heading < 135)
        return 2; // East
    if (heading >= 135 && heading < 225)
        return 3; // South
    if (heading >= 225 && heading < 315)
        return 4; // West

    return 0;
}

void ReadMag(I2C_HandleTypeDef *hi2c1) {
	UpdateRawMagXYZ(hi2c1);

	MAG_POS.x = raw_x;
	MAG_POS.y = raw_y;
	MAG_POS.z = raw_z;

    orientation = DetermineOrientation(MAG_POS.x, MAG_POS.y);
}

const char* getOrientationAsString(){
	if (orientation == 1) return "Orientation: NORTH\n";
	if (orientation == 2) return "Orientation: EAST\n";
	if (orientation == 3) return "Orientation: SOUTH\n";
	if (orientation == 4) return "Orientation: WEST\n";
	return "Orientation: UNKNOWN\n";
}

const char* getMagPosAsString() {
    char *buffer = malloc(256 * sizeof(char));
    size_t bufferSize = 256;
    int x_int = (int)MAG_POS.x;
    int x_frac = (int)(100 * (MAG_POS.x - x_int));

    int y_int = (int)MAG_POS.y;
    int y_frac = (int)(100 * (MAG_POS.y - y_int));

    int z_int = (int)MAG_POS.z;
    int z_frac = (int)(100 * (MAG_POS.z - z_int));

    snprintf(buffer, bufferSize, "MAG_POS (degree) x: %d.%02d y: %d.%02d z: %d.%02d \n", x_int, x_frac, y_int, y_frac, z_int, z_frac);
    return buffer;
}

// function that use all 3 sensor(data) in one time
POS CURRENT_POS = {0.0f, 0.0f, 0.0f};
POS PREVIOUS_POS = {0.0f, 0.0f, 0.0f};

float currentHeading = 0.0f;

void BMX055_init(I2C_HandleTypeDef *hi2c){ //initialization for BMX055
	checkGyro = GyroCheck(hi2c);
	checkAccel = AccelCheck(hi2c, ACCEL_RANGE_8G);
	checkMag = MagCheck(hi2c);

	if (checkGyro == 1) ReadGyro(hi2c, 0.01f);
	if (checkAccel == 1) ReadAccel(hi2c, 0.01f);
	if (checkMag == 1) ReadMag(hi2c);
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
