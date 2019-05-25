/// @file sdriver.c
/// @brief Driver sensor MPU 6050 function definition

#include "sdriver.h"


/* Static variable */
static SD_MPU6050 s_mpu_data;


/* Static function declaration */
/// @brief Static function for getting MPU data
static PROCESS_STATUS_t _get_MPU_Data (I2C_HandleTypeDef *hi2c) {
  SD_MPU6050_Result result;
  result  = SD_MPU6050_ReadGyroscope(hi2c, &s_mpu_data);
  result += SD_MPU6050_ReadAccelerometer(hi2c, &s_mpu_data);
  if (result != SD_MPU6050_Result_Ok) {
    return S_ERROR;
  }
  return S_SUCCESS;
}

/* Interface function */

/// @brief Initialize MPU 6050
/// @param Handle of valid I2C
/// @return Process status (error/success)
PROCESS_STATUS_t imu_Init(I2C_HandleTypeDef *hi2c) {
  SD_MPU6050_Result result;
  result = SD_MPU6050_Init(hi2c, &s_mpu_data,
                          SD_MPU6050_Device_0,
                          SD_MPU6050_Accelerometer_2G,
                          SD_MPU6050_Gyroscope_250s);
  if (SD_MPU6050_Result_Ok != result) {
    return S_ERROR;
  }
  
  return S_SUCCESS;
}

/// @brief Get full data from MPU6050
/// @param [in] hi2c I2C handle
/// @param [out] data pointer to result output
/// @return status of getting data (success/error)
PROCESS_STATUS_t imu_getData(I2C_HandleTypeDef *hi2c, MPU_DATA_t *data) {
  MPU_DATA_t raw_mpu_data;
  if (_get_MPU_Data(hi2c) == S_ERROR)
    return S_ERROR;
  raw_mpu_data.Accelerometer_X = s_mpu_data.Accelerometer_X;
  raw_mpu_data.Accelerometer_Y = s_mpu_data.Accelerometer_Y;
  raw_mpu_data.Accelerometer_Z = s_mpu_data.Accelerometer_Z;
  raw_mpu_data.Gyroscope_X = s_mpu_data.Gyroscope_X;
  raw_mpu_data.Gyroscope_Y = s_mpu_data.Gyroscope_Y;
  raw_mpu_data.Gyroscope_Z = s_mpu_data.Gyroscope_Z;

  *data = raw_mpu_data;
  return S_SUCCESS;
}


