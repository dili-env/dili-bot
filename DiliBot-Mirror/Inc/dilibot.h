/// @file dilibot.h
/// @brief Dili-bot sensor pre-process and balancing process system declaration

#ifndef _DILIBOT_H_
#define _DILIBOT_H_

#include <stdint.h>

// TODO: This is for test
#define MAX_BUFFER_STORE    500

typedef struct sensor_data {
  float s_imu_ax;
  float s_imu_ay;
  float s_gyro_x;
  float s_gyro_y;
} SENSOR_Data_t;

typedef struct phi_enc_data {
  float phi_x;
  float phi_y;
  float phi_x_dot;
  float phi_y_dot;
} PHI_ENC_Data_t;

typedef struct u_dk {
  int32_t Tx;
  int32_t Ty;
} Udk_t;

typedef struct system_data {
  SENSOR_Data_t sensor;
  Udk_t udk;
  uint32_t sys_tick; /* sys_tick increase every 5ms */
} SYSTEM_Data_t;


typedef enum sensor_heartbeat {
  STOP = 0,
  RUN  = 1,
} SensorHeartBeat_t;

/* API provided */
int dilibot_sensorCalibInit(void);
SensorHeartBeat_t dilibot_process(void);

SENSOR_Data_t sensor_GetData(void);

#endif /* _DILIBOT_H_ */
