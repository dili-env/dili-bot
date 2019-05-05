/// @file balancing.c
/// @brief Balance ballbot by some theory function definition
#include <stdbool.h>
#include <stdint.h>

#include "timer.h"

#include "driver.h"
#include "balancing.h"
#include "calib_sensor.h"

#define UPDATE_TIME_TENTH_MS    50 /* 50 tenths of ms -> 5ms */
#define T                       0.005f
#define MAGIC                   1500

extern uint32_t ms5_tick_count;
extern UART_HandleTypeDef huart4;

static TIMER_ID Balancing_TimerID = INVALID_TIMER_ID;
static bool Balancing_Flag = false;

/* Static function definition ************************************************/
static void balancing_stopTimeout(void) {
  if (Balancing_TimerID != INVALID_TIMER_ID)
    TIMER_UnregisterEvent(Balancing_TimerID);
  Balancing_TimerID = INVALID_TIMER_ID;
}

static TIMER_ID balancing_runTimeout(TIMER_CALLBACK_FUNC callback_func,
                                            unsigned long tenth_of_ms) {
  balancing_stopTimeout();
  Balancing_TimerID = TIMER_RegisterEvent(callback_func, tenth_of_ms);
  return Balancing_TimerID;
}

static void balancing_timeoutCallback(void) {
  Balancing_TimerID = INVALID_TIMER_ID;
  Balancing_Flag = true;
  balancing_runTimeout(&balancing_timeoutCallback, UPDATE_TIME_TENTH_MS);
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
/******************************************************************************
 *          Global function definition                                        *
 *****************************************************************************/

/// @brief Initialize ballbot balancing, register callback event
void ballbot_balanceInit(void) {
  balancing_runTimeout(&balancing_timeoutCallback, UPDATE_TIME_TENTH_MS);
}

//#ifdef LOG_SYSTEM
/* #Test *********************************************************************/
//SYSTEM_Data_t trace_system_buff[MAX_BUFFER_STORE];
//uint32_t      trace_system_buff_idx = 0;
/* End test ******************************************************************/
//#endif
uint8_t chart_buff[10] = {0};
/// @brief Ballbot balancing process follow sliding control theory
void ballbot_process(void) {
  
  float theta[2], theta_dot[2];
  float inv_g4, S, f, f2;
  float Txy[2], T1, T2, T3, K_sat;
  int i = 0;
//#ifdef LOG_SYSTEM
//  SYSTEM_Data_t log_system_data;
//#endif  
  static bool initialized = true;
  /// Check event tick flag
  if (true == Balancing_Flag) {
    Balancing_Flag = false;
    /// Start sliding control when flag ticks
    /// Get Sensor (IMU and Gyro) data
    theta[0] = sensor_GetData().ax;
    theta[1] = sensor_GetData().ay;
    theta_dot[0] = sensor_GetData().gx;
    theta_dot[1] = sensor_GetData().gy;
    
    if (initialized) {
      initialized = false;
      // TODO: Reserve for encoder speed calculation
    }
    
    /// Sliding calculation on two axis
    for (i = 0; i < 2; i++) {
      inv_g4 = (cos(theta[i])-0.7693f*(cos(theta[i]))*(cos(theta[i]))+2.3967f) / 
               (-19.2152f-36.2312f*cos(theta[i]));
      // Scaling K_sat @@
      K_sat = 0.2f;
      // Chosing slinding plan
      S = theta[i] + 0.5f*theta_dot[i];
      // Saturating
      if (S > K_sat)
        S = K_sat;
      else if (S < -K_sat)
        S = -K_sat;
      
      f = 0.5f*theta_dot[i];
      
      f2 = (72.7425f*sin(theta[i]) - 0.7693f*cos(theta[i])*sin(theta[i])*theta_dot[i]*theta_dot[i] + 0.5f*sin(theta[i])*theta_dot[i]*theta_dot[i] ) / 
           (cos(theta[i]) - 0.7693f*cos(theta[i])*cos(theta[i]) + 2.3967f);
      
      Txy[i] = -inv_g4*(S + f + f2) * MAGIC;
    }
    
    /// Convert to real system
    T1 = (0.9428f)*( Txy[0])                  ;
    T2 = (0.4714f)*(-Txy[0] - 1.7321f*Txy[1]) ;
    T3 = (0.4714f)*(-Txy[0] + 1.7321f*Txy[1]) ;
    
    // dma_printf(buff, 7);
    /// Prepare chart buff
    dec2ascii((int32_t)(theta[0]*100.0f + 100.0f), 4, &chart_buff[0]);
    chart_buff[4] = ',';
    dec2ascii((int32_t)(theta_dot[0]*100.0f + 100.0f), 4, &chart_buff[5]);
    chart_buff[9] = '\n';
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)chart_buff, 10);
    /// Trigger motor
//#ifndef LOG_SYSTEM
//    motor_SetSpeed(MOTOR_1, (int)T1);
//    motor_SetSpeed(MOTOR_2, (int)T2);
//    motor_SetSpeed(MOTOR_3, (int)T3);
//#endif
    
//#ifdef LOG_SYSTEM
    /******************************************************************
    // TODO: This is for test only
    //       Log data to check by matlab
    ******************************************************************/
//    log_system_data.sensor.s_imu_ax = theta[0];
//    log_system_data.sensor.s_imu_ay = theta[1];
//    log_system_data.sensor.s_gyro_x = theta_dot[0];
//    log_system_data.sensor.s_gyro_y = theta_dot[1];
//    log_system_data.udk.Tx = (int32_t)Txy[0];
//    log_system_data.udk.Ty = (int32_t)Txy[1];
//    log_system_data.udk.T1 = (int32_t)T1;
//    log_system_data.udk.T2 = (int32_t)T2;
//    log_system_data.udk.T3 = (int32_t)T3;
//    log_system_data.sys_tick = ms5_tick_count;
//    
//    trace_system_buff[trace_system_buff_idx++]= log_system_data;
    /* End test ******************************************************/
//#endif
  }
}
