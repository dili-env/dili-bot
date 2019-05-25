/// @file sdriver.h
/// @brief Driver for MPU sensor declaration

#ifndef _SDRIVER_H_
#define _SDRIVER_H_

#include "stdint.h"
#include "inc/sd_hal_mpu6050.h"

typedef enum {
  S_ERROR = -1,
  S_SUCCESS = 0,
} PROCESS_STATUS_t;

typedef struct {
  int16_t Accelerometer_X; /*!< Accelerometer value X axis */
  int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
  int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
  int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
  int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
  int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
} MPU_DATA_t;


/// Interface function prototype
PROCESS_STATUS_t imu_Init(I2C_HandleTypeDef *hi2c);
PROCESS_STATUS_t imu_getData(I2C_HandleTypeDef *hi2c, MPU_DATA_t* data);

#endif
