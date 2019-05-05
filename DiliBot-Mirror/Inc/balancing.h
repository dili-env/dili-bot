/// @file balancing.h
/// @brief Balance ballbot function declaration

#ifndef _BALANCING_H_
#define _BALANCING_H_

#include <math.h>
#include <stdint.h>

typedef struct sensor_data {
  float s_imu_ax;
  float s_imu_ay;
  float s_gyro_x;
  float s_gyro_y;
} SENSOR_Data_t;

typedef struct u_dk {
  int32_t Tx;
  int32_t Ty;
  int32_t T1;
  int32_t T2;
  int32_t T3;
} Udk_t;

typedef struct system_data {
  SENSOR_Data_t sensor;
  Udk_t udk;
  uint32_t sys_tick; /* sys_tick increase every 5ms */
} SYSTEM_Data_t;

void simple_control_LQR(void);
void simple_control_LQR_2(void);
void sliding_control(void);

void ballbot_balanceInit(void);
void ballbot_process(void);

#endif /* _BALANCING_H_ */
