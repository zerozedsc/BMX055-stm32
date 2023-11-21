# BMX055-stm32

## Overview

BMX055-stm32 is a project designed to interface with the BMX055 9-axis Bosch sensor using the STM32 microcontroller. This project is currently being tested with the STM32 NUCLEO-F446RE board and is intended for use with ROS (Robot Operating System) and Micro-ROS.

## BMX055 Sensor

The BMX055 is a 9-axis sensor from Bosch, consisting of a gyroscope, accelerometer, and magnetometer.

## Compatibility

- **Tested Platform:** STM32 NUCLEO-F446RE board.
- **Dependencies:** Requires ROS and Micro-ROS for integration.

## Features

- Successfully reads raw data for all three sensors in the BMX055:
  - Gyroscope
  - Accelerometer
  - Magnetometer

- Implements a filter for raw data.

- Gyroscope Section: Currently no specific features.

- Accelerometer Section: Currently no specific features.

- Magnetometer Section:
  - Successfully converts raw data to degree values.
  - Reads values from 0 to 350, achieving 85% accuracy.
  - Ongoing improvements.

- Combination of 3 Sensors: Currently no specific features.

- Python File:
  - Used for raw data analysis.
  - Subscribes to data from the microcontroller with ROS.

## Hardware Setup

- Connect SDA and SCK with any i2c SDA SDK pin
- Connect 3.3v tp 3.3 and ground to ground on STM32 board
- short(solder) jp7 on BMX055 sensor


## Installation

### 1. Download Source Files

Download the source files and include them in the appropriate folders of your `F446RE project`.

- Download `bmx055.c` and include it in the `Core/Src` folder inside the F446RE project folder.
- Download `bmx055.h` and include it in the `Core/Inc` folder inside the F446RE project folder.

### 2. Sensor Initialization 

Add the following code for BMX055 initialization inside the `StartTask02()` loop.

```c
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    BMX055_init(&hi2c1, deltaTime); // put this inside the loop
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}
```

### 3. Set Publisher Function

Define the BMX055 publisher function inside `main.c`.

```c
// inside main.c
rcl_publisher_t publisher, pub_float_array, publisher_int, pub_bmx055_raw;
std_msgs__msg__Float32MultiArray pub_arr_msg;

void Publish_Bmx055(){
    pub_arr_msg.data.size = BMX055_SIZE;
    pub_arr_msg.data.data = BMX055_data;
    rcl_publish(&pub_float_array, &pub_arr_msg, NULL);

    pub_arr_msg.data.size = 9;
    pub_arr_msg.data.data = BMX055_raw;
    rcl_publish(&pub_bmx055_raw, &pub_arr_msg, NULL);
}
```

### 4. Set up Publisher in StartDefaultTask()

Inside the ```StartDefaultTask()``` function, create the BMX055 publishers.

```c
// inside StartDefaultTask()
// create publisher
RCCHECK(rclc_publisher_init_best_effort(
                 &pub_float_array,
                 &node,
                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                 "/BMX055_DATA"));

RCCHECK(rclc_publisher_init_best_effort(
                 &pub_bmx055_raw,
                 &node,
                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                 "/BMX055_RAW"));
```

### 5. Configure I2C Connection

Set up `I2C connection settings` (default with a clock speed of 100 kHz) in the `.ioc file`.

### 6. Python ROS Setup

For the `bmx055.py` file, set it up inside the Python ROS package. In `setup.py`, include the entry point for the bmx055 script.

```python
setup(
    ...
    entry_points={
        'console_scripts': [
            ...
            'bmx055=lec_1.bmx055:main',
        ],
    },
)
```

Use ```colcon build``` to build the Python ROS package.

Run the Python file with ROS:

```bash
ros2 run your_python_package_name bmx055
```

Replace placeholders such as `your_python_package_name` with actual information specific to your project.

## VARIABLE AND FUNCTION EXPLANATION

**`typedef struct POS`**:  
struct contains `float x`, `float y` and `float z`

**`float MAGDEGREE`**:  
contain converted magnetometer rawdata to degree value from range 0 to 350

## Reference

- BMX055 DATASHEET https://www.mouser.com/datasheet/2/783/BST_BMX055_DS000-1509552.pdf

## Contact

Feel free to [report issues](https://github.com/yourusername/BMX055-stm32/issues) or [contribute](https://github.com/yourusername/BMX055-stm32/pulls) to this project!
