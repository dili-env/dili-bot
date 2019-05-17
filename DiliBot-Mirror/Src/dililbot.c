/// @file calib_sensor.c
/// @brief Sensor investigate function definition

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "driver.h"
#include "sd_hal_mpu6050.h"

#include "dilibot.h"

#include "timer.h"

#include "../Matlab/pid_my_ert_rtw/pid_my.h"

//#define _PID_CONTROL_
#define _CHECK_TOUT_
//#define _CHECK_IMU_
//#define _CHECK_PHI_
//#define _CHECK_PID_
//#define _CHECK_ENC_
//#define _CHECK_SYS_STATE_

/* Good at least 
#define K_thetax        -107.9184f
#define K_thetax_dot     -8.8686f

#define K_thetay        -107.9184f
#define K_thetay_dot     -8.8686f

#define K_phix          0.11531f
#define K_phix_dot      0.5b790f

#define K_phiy          0.11531f
#define K_phiy_dot      0.5790f
*/
/*
#define K_thetax        -167.9184f
#define K_thetax_dot     -18.8686f

#define K_thetay        -167.9184f
#define K_thetay_dot     -18.8686f

#define K_phix          0.0051531f
#define K_phix_dot      2.5790f

#define K_phiy          0.0051531f
#define K_phiy_dot      2.5790f
*/

/*bua
#define K_thetax        -236.9315f
#define K_thetax_dot     -16.2332f

#define K_thetay        -236.9315f
#define K_thetay_dot     -16.2332f

#define K_phix          0.022452f
#define K_phix_dot      3.0329f

#define K_phiy          0.022452f
#define K_phiy_dot      3.0329f

*/


#define PI              3.14159f

#define MAGIC           120.0f          /** Magic LRQ from voltage to duty  */
//#define MAGIC           5000.0f         /** Magic SLIDING from voltage to duty  */

#ifdef _PID_CONTROL_
  #define Kp            0.05f
  #define Ki            0.03f
  #define phix_dot_setpoint (0.0f*PI/180.0f)
  #define phiy_dot_setpoint (0.0f*PI/180.0f)
#endif /* _PID_CONTROL_ */


#define K_thetax        -236.9315f
#define K_thetax_dot     -16.2332f

#define K_thetay        -236.9315f
#define K_thetay_dot     -16.2332f

#define K_phix          0.0022452f
#define K_phix_dot      3.0329f

#define K_phiy          0.0022452f
#define K_phiy_dot      3.0329f

#define rW              0.029
#define rK              0.1225
#define SCALE_PHI       0.23673f        /** rw/rk [-] rk = 0.1225(bong) rw = 0.029 (ominiwheel)*/

#define BIAS_X          271.4644f
#define BIAS_Y          -329.8503f

#define BIAS_ANGLE_Y    (0.8f)
#define BIAS_ANGLE_X    (2.1f)
#define BIAS_ANGLE_Z    0.0f

//#define ENC2PHI         0.004327263f    /** 1452 xung / vong (2pi radian) <-> 2PI/1452 */

#define ENC2PHI1        0.0034599f        /** 1816 xung / vong (2pi rad)  <-> 2PI/1816 */
#define ENC2PHI2        0.0034868f        /** 1802 xung / vong (2pi rad)  <-> 2PI/1802 */
#define ENC2PHI3        0.0047420f        /** 1325 xung / vong (2pi rad)  <-> 2PI/1325 */

#define GYRO2RATE       0.000133162f    /** (250.0f/32767.0f) * (3.14159f/180.0f) */

#define SEPERATE        ','
#define ENDLINE         '\n'

#define CUTOFF_GYRO     5.0f            /** Cut off frequency = 5Hz     */
#define CUTOFF_ANGLE    190.0f            /** Cut off frequency = 5Hz     */
#define CUTOFF_PHI      10.0f           /** Cut off frequency = 10Hz    */
#define FS              200.0f          /** Sampling frequency = 200Hz  */
#define TS              0.005f          /** 1/FS = 5ms (0.005s)         */

#define UPDATE_TIME_TENTH_MS  50        /** 50 tenths of ms -> 5ms      */

extern float imu_value_f[3];            /** Global angle data from UART DMA receive */
extern I2C_HandleTypeDef hi2c3;         /** I2C handle for angle rate get           */

static SD_MPU6050 mpu1;                 /** MPU data struct */
static float RC_gyro, RC_angle, RC_phi, dt, alpha_gyro, alpha_angle, alpha_phi;  /** Filter param  */

static TIMER_ID Sensor_TimerID = INVALID_TIMER_ID;
static bool Sensor_Flag = false;

static boolean_T OverrunFlag = 0;

SENSOR_Data_t g_imu_sensor;
uint8_t chart_buff_x[12] = {0};
uint8_t chart_buff_y[12] = {0};
uint8_t chart_buff_T[15] = {0};
uint8_t chart_buff_L[25] = {0};

uint8_t chart_sysstate[] = {0};

/* Static function prototype *************************************************/
static float frequency_filter(float cur_value, float pre_value, float alpha);

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

/// @brief This function convert decimal with number of digit input @n
///        to ascii displayable and store it in buffer
void dec2ascii(int32_t dec, uint8_t num_digit, uint8_t* ascii_buff) {
    uint8_t ascii_i, offset = 0;
    int32_t dec_left;
    if (dec < 0) {
        offset = 1;
        dec_left = -dec;
        *ascii_buff = '-';
    } else {
        dec_left = dec;
    }

    for (int i = num_digit - 1; i > 0; i--) {
        ascii_i = dec_left % 10;
        *(ascii_buff + i + offset) = ascii_i + '0';
        dec_left = (dec_left - ascii_i) / 10;
    }
    *(ascii_buff + offset) = dec_left + '0';
}

/// @brief Decimal to ascii 5 digit include sign of dec
int8_t dec2ascii5(int32_t dec, uint8_t* ascii_buff) {
  int32_t tmp_dec = dec;
  int8_t idx = 1;
  /// Constrain dec from -999 to 0999
  if (dec < -9999) {
    for (int i = 0; i < 5; i++)
      *(ascii_buff+i) = '-';
    return 5;
  } else if (dec > 9999) {
    for (int i = 0; i < 5; i++)
      *(ascii_buff+i) = '+';
    return 5;
  }
  /// Sign position
  if (dec < 0) {
      *ascii_buff = '-';
      tmp_dec = -dec;
  } else *ascii_buff = '0';
  *(ascii_buff + (idx++)) = (tmp_dec / 1000) + '0';
  tmp_dec = tmp_dec % 1000;
  *(ascii_buff + (idx++)) = (tmp_dec / 100)  + '0';
  tmp_dec = tmp_dec % 100;
  *(ascii_buff + (idx++)) = (tmp_dec / 10) + '0';
  *(ascii_buff + (idx++)) = (tmp_dec % 10) + '0';
  return idx;
}

/// @brief Prepare drawing contrain 3 data
// *Note: [data1],[data2],[data3],[data4][\n]
//        Buffsize = 24
uint8_t g_char_buff_state[24] = {0};
void dma_chart_draw_state(int32_t data1, int32_t data2, int32_t data3, int32_t data4) {
  int idx = 0;
  /// Add to buffer
  idx += dec2ascii5(data1, g_char_buff_state);
  *(g_char_buff_state +  (idx++)) = SEPERATE;
  idx += dec2ascii5(data2, g_char_buff_state + idx);
  *(g_char_buff_state + (idx++)) = SEPERATE;
  idx += dec2ascii5(data3, g_char_buff_state + idx);
  *(g_char_buff_state + (idx++)) = SEPERATE;
  idx += dec2ascii5(data4, g_char_buff_state + idx);
  *(g_char_buff_state + (idx++)) = ENDLINE;
  /// DMA-Send uart
  dma_printf(g_char_buff_state, idx);
}

uint8_t g_char_buff_control[18] = {0};
void dma_chart_draw_control(int32_t data1, int32_t data2, int32_t data3) {
  int idx = 0;
  /// Add to buffer
  idx += dec2ascii5(data1, g_char_buff_control);
  *(g_char_buff_control +  (idx++)) = SEPERATE;
  idx += dec2ascii5(data2, g_char_buff_control + idx);
  *(g_char_buff_control + (idx++)) = SEPERATE;
  idx += dec2ascii5(data3, g_char_buff_control + idx);
  *(g_char_buff_control + (idx++)) = ENDLINE;
  /// DMA-Send uart
  dma_printf(g_char_buff_control, idx);
}

float get_phix_dot2(float psi1_dot, float psi2_dot, float psi3_dot) {
  return SCALE_PHI*(0.9428f*psi1_dot - 0.4714f*psi2_dot - 0.4714f*psi3_dot);
}

float get_phiy_dot2(float psi1_dot, float psi2_dot, float psi3_dot) {
    return SCALE_PHI*(-0.8165f*psi2_dot + 0.8165f*psi3_dot);
}

void rt_OneStep(void)
{
  if (OverrunFlag++) {
    rtmSetErrorStatus(pid_my_M, "Overrun");
    return;
  }
  /* Step the model */
  pid_my_step();
  /* Indicate task complete */
  OverrunFlag--;
}

float get_phix_dot( float theta_x_dot, 
                    float psi1_dot,
                    float psi2_dot,
                    float psi3_dot,
                    float theta_x,
                    float theta_y
) {
  // Note: sqrt(6) = 2.4495, sqrt(2) = 1.4142
  // imu_value_rad[1], imu_value_rad[2] unit is radian
  return (1.0f/(3.0f*rK))*( 2.4495f*rW*sin(theta_x)*sin(theta_y)*(-psi2_dot + psi3_dot) + \
                           1.4142f*rW*cos(theta_x)*sin(theta_y)*(psi1_dot + psi2_dot + psi3_dot) + \
                           cos(theta_y)*(1.4142f*rW*(-2.0f*psi1_dot + psi2_dot + psi3_dot) + 3.0f*rK*theta_x_dot) );
}

float get_phiy_dot( float theta_y_dot,
                    float psi1_dot,
                    float psi2_dot,
                    float psi3_dot,
                    float theta_x,
                    float theta_y
) {
  return (1.0f/(3.0f*rK))*( 2.4495f*rW*cos(theta_x)*(-psi2_dot + psi3_dot) - \
                           1.4142f*rW*sin(theta_x)*(psi1_dot + psi2_dot + psi3_dot) +\
                           3.0f*rK*theta_y_dot);
}

/******************************************************************************
 ********************* Global function definition *****************************
 *****************************************************************************/

/// @brief Get sensor "net" data (after pre-processing data)
/// @return Usefull sensor data for controller
SENSOR_Data_t sensor_GetData(void) {
  return g_imu_sensor;
}

/// @brief Initialize sensor calibrate and pre-processing
//  *Note: If problem occur durring initialize error messaage will be returned
uint8_t dma_mpu_ok[15] = {"MPU WORK FINE\n"};
uint8_t dma_mpu_not[17] = {"MPU NOT WORKING\n"};
int dilibot_sensorCalibInit(void) {
  SD_MPU6050_Result result;

  /// Try to communicate with MPU (max 10 times)
  // *Note Block system !!!
  for (int i = 0; i<10; i++) {
    HAL_Delay(500);
    result = SD_MPU6050_Init (&hi2c3, &mpu1,
                              SD_MPU6050_Device_0,
                              SD_MPU6050_Accelerometer_2G,
                              SD_MPU6050_Gyroscope_250s );

    if(result == SD_MPU6050_Result_Ok) {
      dma_printf(dma_mpu_ok, 15);
      break;
    } /* In other case, repeat ten times */
  }
  
  /// After 10 times, result still not ok => return fail
  if (SD_MPU6050_Result_Ok != result) {
    dma_printf(dma_mpu_not, 17);
    return -1;
  }
  
  /// Calculate filtering parameter
  RC_gyro  = 1/(CUTOFF_GYRO *2*PI);
  RC_angle = 1/(CUTOFF_ANGLE*2*PI);
  RC_phi   = 1/(CUTOFF_PHI  *2*PI);
  dt = 1/FS;
  alpha_gyro  = dt/(RC_gyro  + dt);
  alpha_angle = dt/(RC_angle + dt);
  alpha_phi   = dt/(RC_phi   + dt);
  /// Initialize callback function
  sensor_runTimeout(&sensor_timeoutCallback, UPDATE_TIME_TENTH_MS);
  return 0;
}

/// @brief Process raw sensor value and filtering
//  *Note: Handling on global variable
SensorHeartBeat_t dilibot_process(void) {
  /// System variable declaration
  SensorHeartBeat_t status = RUN;
  static bool initialized = true;
  static int count_delay = 0;
  /***************************************/
  /// Chart data counting
  static uint8_t count = 0;
  
  /// IMU sensor data variable
  static SENSOR_Data_t pre_sensor;
  SENSOR_Data_t tmp_sensor, sensor_data;
  static float gyrox_delay[4];
  static float gyroy_delay[4];
  /// Gyro data
  SD_MPU6050_Result result;
  /// Angle estimation data
  float imu_angle_x, imu_angle_y, imu_angle_z;
  
  
  /// Encoder data
  static float pre_psi1 = 0;
  static float pre_psi2 = 0;
  static float pre_psi3 = 0;
  float psi1, psi2, psi3, psi1_dot, psi2_dot, psi3_dot;
  float phix_dot, phiy_dot;
  static float phix, phiy;

#ifdef _PID_CONTROL_
  /// PID  check phi
  static float pre_e_phix_dot = 0;
  static float pre_e_phiy_dot = 0;
  float e_phix_dot, e_phiy_dot, T_phix, T_phiy;
#endif /* _PID_CONTROL_ */

  /// Control signal
  volatile float Tx, Ty;
  volatile int32_t T1, T2, T3;

  /* System process **********************/
  if (true == Sensor_Flag) {
    /// Update sensor tick flag
    Sensor_Flag = false;
    
    /// Calibrate sensor
    imu_angle_y = (imu_value_f[2] - BIAS_ANGLE_Y) * PI / 180.0f;
    imu_angle_x = (imu_value_f[1] - BIAS_ANGLE_X) * PI / 180.0f;
    imu_angle_z = (imu_value_f[0])                * PI / 180.0f;
    /// Get data from gyro BLOCKING resource!!!
    result = SD_MPU6050_ReadGyroscope(&hi2c3, &mpu1);
    /// Check result return
    if (SD_MPU6050_Result_Ok == result) {
      // *Note: Exchange Gx Gy for hardware compliance
      tmp_sensor.s_gyro_y = -((float)mpu1.Gyroscope_Y - BIAS_Y)*GYRO2RATE;
      tmp_sensor.s_gyro_x =  ((float)mpu1.Gyroscope_X - BIAS_X)*GYRO2RATE;
      tmp_sensor.s_imu_ay =  imu_angle_y;
      tmp_sensor.s_imu_ax = -imu_angle_x;
    } else {
      return STOP;
    }
    
    /// Read current encoder
    psi1 = (float)encoder_ReadMotor(MOTOR_1)*ENC2PHI1;
    psi2 = (float)encoder_ReadMotor(MOTOR_2)*ENC2PHI2;
    psi3 = (float)encoder_ReadMotor(MOTOR_3)*ENC2PHI3;
    psi1_dot = (psi1 - pre_psi1) / TS;
    psi2_dot = (psi2 - pre_psi2) / TS;
    psi3_dot = (psi3 - pre_psi3) / TS;

    /// Initialize process
    if (initialized) {
      initialized = false;
      /// - At the first time set sensor data as current sensor data
      sensor_data = tmp_sensor;
    } else {
      /// - From the second time, apply filtering
      sensor_data.s_imu_ax = frequency_filter(tmp_sensor.s_imu_ax, pre_sensor.s_imu_ax, alpha_angle);
      sensor_data.s_imu_ay = frequency_filter(tmp_sensor.s_imu_ay, pre_sensor.s_imu_ay, alpha_angle);
      sensor_data.s_gyro_x = frequency_filter(tmp_sensor.s_gyro_x, pre_sensor.s_gyro_x, alpha_gyro);
      sensor_data.s_gyro_y = frequency_filter(tmp_sensor.s_gyro_y, pre_sensor.s_gyro_y, alpha_gyro);
      psi1 = frequency_filter(psi1, pre_psi1, alpha_phi);
      psi2 = frequency_filter(psi2, pre_psi2, alpha_phi);
      psi3 = frequency_filter(psi3, pre_psi3, alpha_phi);
    }

    /// Simple calculate phix_dot, phiy_dot and integral calculate phix, phiy
    phix_dot = get_phix_dot2(psi1_dot, psi2_dot, psi3_dot);
    phiy_dot = get_phiy_dot2(psi1_dot, psi2_dot, psi3_dot);
    phix += phix_dot*TS;
    phiy += phiy_dot*TS;



    /// Ok we can check IMU sensor data here!


#ifdef _CHECK_ENC_
    if (++count == 5) {
      count = 0;
      dma_chart_draw_state(
        (int32_t)(psi1_dot * 180.0f / PI),
        (int32_t)(psi2_dot * 180.0f / PI),
        (int32_t)(psi3_dot * 180.0f / PI),
        0
      );
    }
#endif /* CHECK_ENC_ */

#ifdef _CHECK_PHI_
    if (++count == 5) {
      count = 0;
      dma_chart_draw_state(
        (int32_t)(phix      * 180.0f / PI),
        (int32_t)(phix_dot  * 180.0f / PI),
        (int32_t)(phiy      * 180.0f / PI),
        (int32_t)(phiy_dot  * 180.0f / PI)
      );
    }
#endif /* _CHECK_PHI_ */
//    
    /// LQR Control theory *************************************************
    Tx = -(  K_thetax*sensor_data.s_imu_ax  + K_thetax_dot*sensor_data.s_gyro_x
           + K_phix  *phix                  + K_phix_dot  *phix_dot
          );

    Ty = -(  K_thetay*sensor_data.s_imu_ay  + K_thetay_dot*sensor_data.s_gyro_y
           + K_phiy  *phiy                  + K_phiy_dot  *phiy_dot
          );

#ifdef _PID_CONTROL_
    /// Example control the speed of Phix = 0 deg/s = 0 rad/s
    e_phix_dot = phix_dot_setpoint - phix_dot;
    e_phiy_dot = phiy_dot_setpoint - phiy_dot;
    pre_e_phix_dot += e_phix_dot;
    pre_e_phiy_dot += e_phiy_dot;
    // T   =  Kp*e            + Ki*
    T_phix = Kp*e_phix_dot + Ki*pre_e_phix_dot;
    T_phiy = Kp*e_phiy_dot + Ki*pre_e_phiy_dot;
    
    Tx += T_phix;
    Ty += T_phiy;
#endif /* _PID_CONTROL_ */

    /// Convert to real system
    T1 = (int32_t)(MAGIC*      (0.9428f)*( Tx))              ;
    T2 = (int32_t)(MAGIC*0.97f*(0.4714f)*(-Tx - 1.7321f*Ty)) ;
    T3 = (int32_t)(MAGIC*1.01f*(0.4714f)*(-Tx + 1.7321f*Ty)) ;

#ifdef _CHECK_PID_
    if (++count == 5) {
      count = 0;
      dma_chart_draw_state(0, 0, (int32_t)(sensor_data.s_imu_ax*100.0f), (int32_t)(sensor_data.s_imu_ay*100.0f));
    }
#endif /* _CHECK_PID_ */

    
#ifdef _CHECK_TOUT_
    if (++count == 5) {
      count = 0;
      dma_chart_draw_control(T1, T2, T3);
    }
#endif /* _CHECK_TOUT_ */
    
#ifdef _CHECK_SYS_STATE_
    if (++count == 5) {
      count = 0;
      dma_chart_draw_state(phix*180.0f/PI,
                           phiy*180.0f/PI,
                           sensor_data.s_imu_ax*1800.0f/PI,
                           sensor_data.s_imu_ay*1800.0f/PI);
    }
#endif /* _CHECK_SYS_STATE_ */

    if (count_delay < 20) {
      count_delay++;
      return status;
    }
    
    motor_SetSpeed(MOTOR_1, T1);
    motor_SetSpeed(MOTOR_2, T2);
    motor_SetSpeed(MOTOR_3, T3);

#ifdef _CHECK_IMU_
    if (++count == 5) {
      count = 0;
      dma_chart_draw_state(
        (int32_t)(sensor_data.s_imu_ax * 180.0f / PI * 100.0f),
        (int32_t)(sensor_data.s_gyro_x * 180.0f / PI * 100.0f),
        (int32_t)(sensor_data.s_imu_ay * 180.0f / PI * 100.0f),
        (int32_t)(sensor_data.s_gyro_y * 180.0f / PI * 100.0f)
      );
    }
#endif /* CHECK_IMU */
    
    /// Remember previous state
    pre_psi1 = psi1; pre_psi2 = psi2; pre_psi3 = psi3;
    pre_sensor = sensor_data;
  }
  return status;
}
