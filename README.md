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
 
## Installation

### 1. Download Source Files

Download the source files and include them in the appropriate folders of your F446RE project.

- Download `bmx055.c` and include it in the `Core/Src` folder inside the F446RE project folder.
- Download `bmx055.h` and include it in the `Core/Inc` folder inside the F446RE project folder.

### 2. Sensor Initialization 

Add the following code for BMX055 initialization inside the StartTask02 loop.

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


