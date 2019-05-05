/// @file calib_sensor.c
/// @brief Sensor investigate function definition

#include "sd_hal_mpu6050.h"
#include "driver.h"
#include "timer.h"
#include "calib_sensor.h"


#define BIAS_X          271.4644f
#define BIAS_Y          -329.8503f

#define BIAS_ANGLE_X    1.4019f
#define BIAS_ANGLE_Y    2.8933f
#define BIAS_ANGLE_Z    0.0f

#define CUTOFF_GYRO     5.0f          // Cut off frequency = 5Hz
#define CUTOFF_ANGLE    5.0f          // Cut off frequencu = 5Hz
#define FS              200.0f        // Sampling frequency = 200Hz
#define PI              3.14159f

#define UPDATE_TIME_TENTH_MS    50    // 50 tenths of ms -> 5ms
#define MAGIC                   1200  // Convert Udk to PWM

extern float imu_value_f[3];
extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart4;

static SD_MPU6050 mpu1;
static float RC_gyro, RC_angle, dt, alpha_gyro, alpha_angle;

static TIMER_ID Sensor_TimerID = INVALID_TIMER_ID;
static bool Sensor_Flag = false;
static IMU_Data_t s_imu_sensor;


/* Static function prototype *************************************************/
static float frequency_filter(float cur_value, float pre_value, float alpha);
void ballbot_process_(void);

/* Static function definition ************************************************/
static void sensor_stopTimeout(void) {
  if (Sensor_TimerID != INVALID_TIMER_ID)
    TIMER_UnregisterEvent(Sensor_TimerID);
  Sensor_TimerID = INVALID_TIMER_ID;
}

static TIMER_ID sensor_runTimeout(TIMER_CALLBACK_FUNC callback_func,
                                          unsigned long tenth_of_ms) {
  sensor_stopTimeout();
  Sensor_TimerID = TIMER_RegisterEvent(callback_func, tenth_of_ms);
  return Sensor_TimerID;
}

static void sensor_timeoutCallback(void) {
  Sensor_TimerID = INVALID_TIMER_ID;
  Sensor_Flag = true;
  sensor_runTimeout(&sensor_timeoutCallback, UPDATE_TIME_TENTH_MS);
}

/// @brief Frequency alpha filter theory
//  *Note: out[i] = out[i-1] + alpha*(in[i] - out[i-1])
static float frequency_filter(float cur_value, float pre_value, float alpha) {
  return (pre_value + alpha*(cur_value - pre_value));
}

/******************************************************************************
 *          Global function definition                                        *
 *****************************************************************************/

/// @brief Get sensor "net" data (after pre-processing data)
/// @return Usefull sensor data for controller
IMU_Data_t sensor_GetData(void) {
  return s_imu_sensor;
}

/// @brief Initialize sensor calibrate and pre-processing
//  *Note: If problem occur durring initialize error messaage will be returned
void sensor_calibInit(void) {
  SD_MPU6050_Result result;
  uint8_t mpu_ok[15] = {"MPU WORK FINE\n"};
  uint8_t mpu_not[17] = {"MPU NOT WORKING\n"};

  result = SD_MPU6050_Init (&hi2c3, &mpu1,
                            SD_MPU6050_Device_0,
                            SD_MPU6050_Accelerometer_2G,
                            SD_MPU6050_Gyroscope_250s );
  HAL_Delay(500);
  if(result == SD_MPU6050_Result_Ok) {
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)mpu_ok, 14);
  } else {
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)mpu_not, 16);
  }
  
  /// Calculate filtering parameter
  RC_gyro  = 1/(CUTOFF_GYRO*2*PI);
  RC_angle = 1/(CUTOFF_ANGLE*2*PI);
  dt = 1/FS;
  alpha_gyro  = dt/(RC_gyro+dt);
  alpha_angle = dt/(RC_angle+dt);
  
  /// Initialize callback function
  sensor_runTimeout(&sensor_timeoutCallback, UPDATE_TIME_TENTH_MS);
}

//#ifdef LOG_SYSTEM
/* #Test *****************************************************/
//IMU_Data_t g_IMU_DataTest[MAX_BUFFER_STORE];
//uint32_t   g_IMU_DataTest_Idx = 0;
/* End test **************************************************/
//#endif

/// @brief Process raw sensor value and filtering
//  *Note: Handling on global variable
SensorHeartBeat_t sensor_calibProcess(void) {
  SensorHeartBeat_t status = RUN;
  static bool initialized = true;
  static IMU_Data_t pre_sensor;
  
  IMU_Data_t tmp_sensor;
  SD_MPU6050_Result result;
  float imu_angle_x, imu_angle_y, imu_angle_z;
  
  if (true == Sensor_Flag) {
    /// Update sensor tick flag
    Sensor_Flag = false;
    
    //printf("Calib process\n");
    
    imu_angle_x = (imu_value_f[2] - BIAS_ANGLE_Y) * PI / 180.0f;
    imu_angle_y = (imu_value_f[1] - BIAS_ANGLE_X) * PI / 180.0f;
    imu_angle_z = (imu_value_f[0])                * PI / 180.0f;

    /// Get data from gyro
    result = SD_MPU6050_ReadGyroscope(&hi2c3, &mpu1);
    
    
    /// Check result return
    if (SD_MPU6050_Result_Ok == result) {
      // *Note: Exchange Gx Gy for hardware compliance
      tmp_sensor.gx = -((float)mpu1.Gyroscope_Y - BIAS_Y)*(250.0f/32767.0f) * (3.14159f/180.0f);
      tmp_sensor.gy = ((float)mpu1.Gyroscope_X - BIAS_X)*(250.0f/32767.0f) * (3.14159f/180.0f);
      tmp_sensor.ax = imu_angle_x;
      tmp_sensor.ay = -imu_angle_y;
    } else {
      return STOP;
    }
    /// If first data, set output as input
    if (initialized) {
      initialized = false;
      s_imu_sensor = tmp_sensor;
    } else {
      /// From the second time, apply filtering
      s_imu_sensor.ax = frequency_filter(tmp_sensor.ax, pre_sensor.ax, alpha_angle);
      s_imu_sensor.ay = frequency_filter(tmp_sensor.ay, pre_sensor.ay, alpha_angle);
      s_imu_sensor.gx = frequency_filter(tmp_sensor.gx, pre_sensor.gx, alpha_gyro);
      s_imu_sensor.gy = frequency_filter(tmp_sensor.gy, pre_sensor.gy, alpha_gyro);
    }
    pre_sensor = s_imu_sensor;
    
//#ifdef LOG_SYSTEM
    /****************************************************************
    // TODO: This is for test only, "heart have to beat" forever ^^
    //       Data in sensor calibrate need not to be logged!
    ****************************************************************/
//    g_IMU_DataTest[g_IMU_DataTest_Idx++] = s_imu_sensor;
//    if (g_IMU_DataTest_Idx == MAX_BUFFER_STORE) {
//      status = STOP;
//      // printf("End log\n");
//    }
    /* End test ****************************************************/
//#endif
  }
  return status;
}
