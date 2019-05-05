/// @file calib_sensor.h
/// @brief Investiagte sensor property

#ifndef _CALIB_SENSOR_H_
#define _CALIB_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

// TODO: This is for test
#define MAX_BUFFER_STORE    500

typedef struct imu_data {
  float gx;
  float gy;
  float ax;
  float ay;
} IMU_Data_t;

typedef enum sensor_heartbeat {
  STOP = 0,
  RUN  = 1,
} SensorHeartBeat_t;

/* Sensor module APIs provided */
void sensor_calibInit(void);
SensorHeartBeat_t sensor_calibProcess(void);
IMU_Data_t sensor_GetData(void);

#endif /* _CALIB_SENSOR_H_s */
