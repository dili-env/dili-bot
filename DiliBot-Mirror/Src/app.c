/// @file app.c
/// @brief Application definition, state machine definition
#include <stdio.h>

#include "stm32f4xx_hal.h"

#include "app.h"

//#include "balancing.h"
//#include "calib_sensor.h"
#include "dilibot.h"

//#ifdef LOG_SYSTEM
// #TEST
//extern IMU_Data_t g_IMU_DataTest[MAX_BUFFER_STORE];
//extern SYSTEM_Data_t trace_system_buff[MAX_BUFFER_STORE];
//#endif

static SYSTEM_STATE s_systemState = SYSTEM_INITIALIZE;


SYSTEM_STATE system_getState(void) {
  return s_systemState;
}

void system_setState(SYSTEM_STATE state) {
  s_systemState = state;
}

void send_imu_data(void) {
//#ifdef LOG_SYSTEM
//  printf  ("Ax\tAy\tGx\tGy\n");
//  for (int i = 0; i < MAX_BUFFER_STORE; i++) {
//    printf("%.06f\t%.06f\t%.06f\t%.06f\n",
//            g_IMU_DataTest[i].ax,
//            g_IMU_DataTest[i].ay,
//            g_IMU_DataTest[i].gx,
//            g_IMU_DataTest[i].gy);
//    HAL_Delay(10);
//  }
//  printf("\n\n\n");
//  printf("Thetax\tThetay\tTheta_dotx\tTheta_doty\tTx\tTy\tT1\tT2\tT3\tTick\n");
//  for (int j = 0; j < MAX_BUFFER_STORE; j++) {
//    printf("%.06f\t%.06f\t%.06f\t%.06f\t%d\t%d\t%d\t%d\t%d\t%d\n",
//            trace_system_buff[j].sensor.s_imu_ax,
//            trace_system_buff[j].sensor.s_imu_ay,
//            trace_system_buff[j].sensor.s_gyro_x,
//            trace_system_buff[j].sensor.s_gyro_y,
//            trace_system_buff[j].udk.Tx,
//            trace_system_buff[j].udk.Ty,
//            trace_system_buff[j].udk.T1,
//            trace_system_buff[j].udk.T2,
//            trace_system_buff[j].udk.T3,
//            trace_system_buff[j].sys_tick
//    );
//    HAL_Delay(10);
//  }
//#else
  //printf("Sensor error\n");
//#endif
}

void system_processSystemState(void) {
  switch (system_getState()) {
    case SYSTEM_INITIALIZE:
      system_setState(SYSTEM_CALIB_SENSOR);
      break;
    case SYSTEM_CALIB_SENSOR:
      if (dilibot_process() == STOP) {
        system_setState(SYSTEM_SEND_DATA);
      }
      break;
    case SYSTEM_SEND_DATA:
      send_imu_data();
      system_setState(SYSTEM_HALT);
      break;
    case SYSTEM_CHECK_CMD:
      // Do some thing here for checking
      system_setState(SYSTEM_BALANCE);
      break;
    case SYSTEM_BALANCE:
      // ballbot_process();
      break;
    case SYSTEM_HALT:
    default:
      // Do not thing
      break;
  }
}
