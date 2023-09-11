/*
 * BMX055.h
 *
 *  Created on: Sep 9, 2023
 *      Author: MUHAMMAD HELMI BIN ROZAIN
 *
 *      Header File for BMX055 9 AXIS BOSCH SENSOR
 */

#include <main.h>

typedef struct {
	float x;
	float y;
	float z;
} POS;

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
#define ACCEL_ERROR_MULTIPLIER 1.0f
#define ACCEL_ERROR_CONSTANT 0.0f

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
#define GYRO_SENSITIVITY 0.0038f;
#define GYRO_ERROR_MULTIPLIER 1.0f
#define GYRO_ERROR_CONSTANT 0.0f

// MAGNETOMETER SETUP
#define MAG_CHIP_ID_REG    0x40
#define MAG_CHIP_ID_DEFAULT    0x32
#define MAG_ADDR  0x10 //default address
#define MAG_X_LSB 0x42
#define MAG_X_MSB 0x43
#define MAG_Y_LSB 0x44
#define MAG_Y_MSB 0x45
#define MAG_Z_LSB 0x46
#define MAG_Z_MSB 0x47
#define MAG_ERROR_MULTIPLIER 1.0f
#define MAG_ERROR_CONSTANT 0.0f

// 3 SENSOR IN ONE TIME SETUP
// Thresholds
#define ACCEL_NOISE_THRESHOLD 0.2f   // Small threshold to account for noise
#define GYRO_NOISE_THRESHOLD  0.2f   // Threshold for gyroscope noise
#define COLLISION_THRESHOLD   5.0f   // Sudden acceleration change for collisions
#define CORRECTION_FACTOR 0.1f // adjust as needed
#define HEADING_THRESHOLD 5.0f // degrees of allowable deviation
#define SLOPE_THRESHOLD 10.0f // Adjust this based on your robot's sensitivity
#define GRAVITY 9.81f  // Earth gravity in m/s^2
#define TILT_THRESHOLD 0.2f  // Adjust this based on your requirement. It's the threshold for considering that the robot is tilted.
#define ACCEL_THRESHOLD 0.2f // Set a threshold for accelerometer readings (m/s^2). Adjust as per your robot's characteristics.
#define ENCODER_THRESHOLD 5  // Set a threshold for motor encoder readings (counts or rotations). Adjust as needed.
#define FLIPPING_THRESHOLD 30.0f //This threshold would determine a significant change in the orientation of the robot.
#define FALLING_THRESHOLD_Z 4.5f //This represents the deviation from the gravitational acceleration (in the z-axis) that indicates the robot is falling.
#define FALLING_THRESHOLD_TOTAL 15.0f // This represents the magnitude of the acceleration in all directions.


// GYRO FUNCTION
extern POS GYRO_POS; // Degrees per second (°/s) or Radians per second (rad/s)
extern POS GYRO_MOVEMENT; // Degrees per second (°/s) or Radians per second (rad/s)

extern int checkGyro;

int GyroCheck(I2C_HandleTypeDef *hi2c1);
void ReadGyro(I2C_HandleTypeDef *hi2c1, float deltaTime); // deltaTime in seconds 10ms = 0.01s
const char* getGyroPosAsString();
const char* getGyroMovementAsString();

// ACCELERATOR FUNCTION
extern POS VELOCITY; // Meters per second (m/s), ACCEL * deltaTime
extern POS PREVIOUS_VELOCITY;
extern POS ACCEL; // Meters per second squared (m/s^2) or G-forces (where 1G = 9.81 m/s^2)
extern POS PREVIOUS_ACCEL;

extern int checkAccel, accelCode;

int AccelCheck(I2C_HandleTypeDef *hi2c1, uint8_t ACCEL_G);
void ReadAccel(I2C_HandleTypeDef *hi2c1, float deltaTime);
const char* getAccelAsString();
const char* getVelocityAsString();

// MAGNETOMETER FUNCTION
extern POS MAG_POS; // microteslas (µT) or milligauss (mG) -> unit measurement

extern int checkMag;
extern float orientation, heading; //heading and orientation value from magnetometer

int MagCheck(I2C_HandleTypeDef *hi2c1);
void ReadMag(I2C_HandleTypeDef *hi2c1);
const char* getOrientationAsString();
const char* getMagPosAsString();

// BMX055 3 SENSOR IN ONE TIME FUNCTION (NEED TO INITIALIZE 3 SENSOR BEFORE USE THESE FUNCTION/ OR IT WILL NOT WORKING WELL)
extern POS CURRENT_POS;
extern POS PREVIOUS_POS;
typedef enum {
    STATIONARY,
    MOVING_STRAIGHT,
    TURNING,
    COLLISION,
	FLIPPING,
	FALLING,
    UNKNOWN
} RobotActivity;
typedef enum {
    NOT_SLIPPING,
    SLIPPING
} WheelStatus;

extern float currentHeading;

void BMX055_init(I2C_HandleTypeDef *hi2c);
void UpdatePosition(float deltaTime);
const RobotActivity DetectActivity(); // detect robot activity
const WheelStatus DetectWheelSlip(int encoderCount);
const char* getDetectActivityAsString(RobotActivity activity);
const char* getWheelSlipAsString(WheelStatus wheel);
const float CalculateDistanceTravelled();
const int isClimbing();

