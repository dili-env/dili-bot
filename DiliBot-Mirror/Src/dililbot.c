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

#define MAGIC           100.0f          /** Magic LRQ from voltage to duty  */
//#define MAGIC           5000.0f         /** Magic SLIDING from voltage to duty  */

#define K_thetax        -84.5283
#define K_thetax_dot    -1.7972

#define K_thetay        -84.5283
#define K_thetay_dot    -1.7972

#define K_phix          -0.3778
#define K_phix_dot      -0.1645

#define K_phiy          -0.3778
#define K_phiy_dot      -0.1645

#define rW              0.029
#define rK              0.1225
#define SCALE_PHI       0.23673f        /** rw/rk [-] rk = 0.1225(bong) rw = 0.029 (ominiwheel)*/

#define BIAS_X          271.4644f
#define BIAS_Y          -329.8503f

#define BIAS_ANGLE_Y    1.4019f
#define BIAS_ANGLE_X    2.8933f
#define BIAS_ANGLE_Z    0.0f

#define ENC2PHI         0.004327263f    /** 1452 xung / vong (2pi radian) <-> 2PI/1452 */
#define GYRO2RATE       0.000133162f    /** (250.0f/32767.0f) * (3.14159f/180.0f) */

#define SEPERATE        ','
#define ENDLINE         '\n'

#define CUTOFF_GYRO     10.0f            /** Cut off frequency = 5Hz     */
#define CUTOFF_ANGLE    10.0f            /** Cut off frequency = 5Hz     */
#define CUTOFF_PHI      10.0f           /** Cut off frequency = 10Hz    */
#define FS              200.0f          /** Sampling frequency = 200Hz  */
#define PI              3.14159f
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

float get_phix(float psi1, float psi2, float psi3) {
  return -(0.9428f*psi1 - 0.4714f*psi2 - 0.4714f*psi3)*SCALE_PHI;
}

float get_phiy(float psi1, float psi2, float psi3) {
    return -(-0.8165f*psi2 + 0.8165f*psi3)*SCALE_PHI;
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
  return (1/(3*rK))*( 2.4495*rW*sin(theta_y)*sin(theta_x)*(-psi2_dot + psi3_dot) + \
                           1.4142*rW*cos(theta_y)*sin(theta_x)*(psi1_dot + psi2_dot + psi3_dot) + \
                           cos(theta_x)*(1.4142*rW*(-2*psi1_dot + psi2_dot + psi3_dot) + 3*rK*theta_x_dot) );
}

float get_phiy_dot( float theta_y_dot,
                    float psi1_dot,
                    float psi2_dot,
                    float psi3_dot,
                    float theta_x,
                    float theta_y
) {
  return (1/(3*rK))*( 2.4495*rW*cos(theta_y)*(-psi2_dot + psi3_dot) - \
                           1.4142*rW*sin(theta_y)*(psi1_dot + psi2_dot + psi3_dot) +\
                           3*rK*theta_y_dot);
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
  SensorHeartBeat_t status = RUN;
  static bool initialized = true;
  static SENSOR_Data_t pre_sensor;
  static PHI_ENC_Data_t pre_phi;
  
  SENSOR_Data_t tmp_sensor, sensor_data;
  PHI_ENC_Data_t tmp_phi, phi_data;

  SD_MPU6050_Result result;
  float imu_angle_x, imu_angle_y, imu_angle_z;
  float phi1, phi2, phi3, rphi_x_dot, rphi_y_dot;
  
  volatile float inv_g4, Sx, Sy, f, f2, K_sat;
  
  volatile float Tx, Ty;
  volatile int32_t T1, T2, T3;
  
  int32_t i_thetax, i_thetax_dot, i_thetay, i_thetay_dot;
  
// _PID_
  int udk;
//
  if (true == Sensor_Flag) {
    /// Update sensor tick flag
    Sensor_Flag = false;

/* _PID_
    In2 = (double)encoder_ReadMotor(MOTOR_3);
    rt_OneStep();
    Out1 = Out1*800/12;
    udk = (int)Out1;
    motor_SetSpeed(MOTOR_3, udk);
    
    dec2ascii(In2, 5, &chart_buff_y[0]);
    chart_buff_y[5] = SEPERATE;
    dec2ascii(Out1, 5, &chart_buff_y[6]);
    chart_buff_y[11] = ENDLINE;
    dma_printf(chart_buff_y, 12);
// _PID_ */


//===========================================================================
// Code WORK
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
    phi1 = (float)encoder_ReadMotor(MOTOR_1)*ENC2PHI;
    phi2 = (float)encoder_ReadMotor(MOTOR_2)*ENC2PHI;
    phi3 = (float)encoder_ReadMotor(MOTOR_3)*ENC2PHI;
    tmp_phi.phi_x = get_phix(phi1, phi2, phi3);
    tmp_phi.phi_y = get_phiy(phi1, phi2, phi3);
    
    /// If first data, set output as input
    if (initialized) {
      initialized = false;
      sensor_data = tmp_sensor;
      phi_data.phi_x_dot = 0;
      phi_data.phi_y_dot = 0;
      phi_data = tmp_phi;
    } else {
      /// From the second time, apply filtering for
      sensor_data.s_imu_ax = frequency_filter(tmp_sensor.s_imu_ax, pre_sensor.s_imu_ax, alpha_angle);
      sensor_data.s_imu_ay = frequency_filter(tmp_sensor.s_imu_ay, pre_sensor.s_imu_ay, alpha_angle);
      sensor_data.s_gyro_x = frequency_filter(tmp_sensor.s_gyro_x, pre_sensor.s_gyro_x, alpha_gyro);
      sensor_data.s_gyro_y = frequency_filter(tmp_sensor.s_gyro_y, pre_sensor.s_gyro_y, alpha_gyro);
      
      phi_data.phi_x = frequency_filter(tmp_phi.phi_x, pre_phi.phi_x, alpha_phi);
      phi_data.phi_y = frequency_filter(tmp_phi.phi_y, pre_phi.phi_y, alpha_phi);
      
      /// Calculate phi_dot
      rphi_x_dot = (phi_data.phi_x - pre_phi.phi_x) / TS;
      rphi_y_dot = (phi_data.phi_y - pre_phi.phi_y) / TS;
      phi_data.phi_x_dot = frequency_filter(rphi_x_dot, pre_phi.phi_x_dot, alpha_phi);
      phi_data.phi_y_dot = frequency_filter(rphi_y_dot, pre_phi.phi_y_dot, alpha_phi);
    }
    pre_sensor = sensor_data;
    pre_phi = phi_data;
    
    
    /// Control theory ************************************************
    Tx = -(K_thetax*sensor_data.s_imu_ax + K_thetax_dot*sensor_data.s_gyro_x 
         + K_phix*phi_data.phi_x         + K_phix_dot*phi_data.phi_x_dot);
    Ty = -(K_thetay*sensor_data.s_imu_ay + K_thetay_dot*sensor_data.s_gyro_y
         + K_phiy*phi_data.phi_y         + K_phiy_dot*phi_data.phi_y_dot);
    
#ifdef _SLIDING_
    /// Sliding calculation on two axis
//    K_sat = 0.01f;
//    inv_g4 = (cos(sensor_data.s_imu_ax)-0.7693f*(cos(sensor_data.s_imu_ax))*(cos(sensor_data.s_imu_ax))+2.3967f) / 
//               (-19.2152f-36.2312f*cos(sensor_data.s_imu_ax));
//    Sx = sensor_data.s_imu_ax + 0.5f*sensor_data.s_gyro_x;
//          // Saturating
//    if (Sx >  K_sat) Sx = K_sat;
//    if (Sx < -K_sat) Sx = -K_sat;
//    f = 0.5f*sensor_data.s_gyro_x;
//    f2 = (72.7425f*sin(sensor_data.s_imu_ax) - 0.7693f*cos(sensor_data.s_imu_ax)*sin(sensor_data.s_imu_ax)*sensor_data.s_gyro_x*sensor_data.s_gyro_x \
//          + 0.5f*sin(sensor_data.s_imu_ax)*sensor_data.s_gyro_x*sensor_data.s_gyro_x ) / 
//           (cos(sensor_data.s_imu_ax) - 0.7693f*cos(sensor_data.s_imu_ax)*cos(sensor_data.s_imu_ax) + 2.3967f);
//    Tx = -inv_g4*(Sx*2 + f + f2);


//    inv_g4 = (cos(sensor_data.s_imu_ay)-0.7693f*(cos(sensor_data.s_imu_ay))*(cos(sensor_data.s_imu_ay))+2.3967f) / 
//               (-19.2152f-36.2312f*cos(sensor_data.s_imu_ay));
//    Sy = sensor_data.s_imu_ay + 0.5f*sensor_data.s_gyro_y;
//          // Saturating
//    if (Sy >  K_sat) Sy = K_sat;
//    if (Sy < -K_sat) Sy = -K_sat;
//    f = 0.5f*sensor_data.s_gyro_y;
//    f2 = (72.7425f*sin(sensor_data.s_imu_ay) - 0.7693f*cos(sensor_data.s_imu_ay)*sin(sensor_data.s_imu_ay)*sensor_data.s_gyro_y*sensor_data.s_gyro_y \
//          + 0.5f*sin(sensor_data.s_imu_ay)*sensor_data.s_gyro_y*sensor_data.s_gyro_y ) / 
//           (cos(sensor_data.s_imu_ay) - 0.7693f*cos(sensor_data.s_imu_ay)*cos(sensor_data.s_imu_ay) + 2.3967f);
//    Ty = -inv_g4*(Sy*2 + f + f2);
#endif // _SLIDING_
    
    
    
    /// Convert to real system
    T1 = (int32_t)(MAGIC*(0.9428f)*( Tx))              ;
    T2 = (int32_t)(MAGIC*(0.4714f)*(-Tx - 1.7321f*Ty)) ;
    T3 = (int32_t)(MAGIC*(0.4714f)*(-Tx + 1.7321f*Ty)) ;
    
    motor_SetSpeed(MOTOR_1, T1);
    motor_SetSpeed(MOTOR_2, T2);
    motor_SetSpeed(MOTOR_3, T3);

    /// Draw sensor data
    i_thetay     = (int32_t)(sensor_data.s_imu_ay * 1000.0f + 1000.0f);
    i_thetay_dot = (int32_t)(sensor_data.s_gyro_y * 1000.0f + 1000.0f);
    i_thetax     = (int32_t)(sensor_data.s_imu_ax * 1000.0f + 1000.0f);
    i_thetax_dot = (int32_t)(sensor_data.s_gyro_x * 1000.0f + 1000.0f);
    
    dec2ascii(i_thetax, 4, &chart_buff_T[0]);
    chart_buff_T[4] = SEPERATE;
    dec2ascii(i_thetay, 4, &chart_buff_T[5]);
    chart_buff_T[9] = SEPERATE;
    dec2ascii((T1+1000), 4, &chart_buff_T[10]);
    chart_buff_T[14] = ENDLINE;
    dma_printf(chart_buff_T, 15);

//    dec2ascii(i_thetax, 5, &chart_buff_x[0]);
//    chart_buff_x[5] = SEPERATE;
//    dec2ascii(i_thetax_dot, 5, &chart_buff_x[6]);
//    chart_buff_x[11] = ENDLINE;
//    dma_printf(chart_buff_x, 12);

#ifdef _PRINT_T_
//    dec2ascii((T1+800), 4, &chart_buff_T[0]);
//    chart_buff_T[4] = SEPERATE;
//    dec2ascii((T2+800), 4, &chart_buff_T[5]);
//    chart_buff_T[9] = SEPERATE;
//    dec2ascii((T3+800), 4, &chart_buff_T[10]);
//    chart_buff_T[14] = ENDLINE;
//    dma_printf(chart_buff_T, 15);
#endif // _PRINT_T_ 
    
    
//    dec2ascii(phi_data.phi_x*100.0f + 1000.0f, 5, &chart_buff_y[0]);
//    chart_buff_y[5] = SEPERATE;
//    dec2ascii(phi_data.phi_x_dot*100.0f + 1000.0f, 5, &chart_buff_y[6]);
//    chart_buff_y[11] = ENDLINE;
//    dma_printf(chart_buff_y, 12);

// END CODE WORK */
  }
  return status;
}
