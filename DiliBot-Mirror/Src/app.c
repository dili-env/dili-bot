/// @file app.c
/// @brief Application definition, state machine definition
#include "app.h"
#include "balancing.h"
#include "calib_sensor.h"

#include "sd_hal_mpu6050.h"

#include "driver.h"


static SYSTEM_STATE s_systemState = SYSTEM_INITIALIZE;


SYSTEM_STATE system_getState(void) {
  return s_systemState;
}

void system_setState(SYSTEM_STATE state) {
  s_systemState = state;
}

void system_processSystemState(void) {
  switch (system_getState()) {
    case SYSTEM_INITIALIZE:
      system_setState(SYSTEM_CALIB_SENSOR);
      break;
    case SYSTEM_CALIB_SENSOR:
      sensor_calibProcess();
      break;
    case SYSTEM_CHECK_CMD:
      // Do some thing here for checking
      system_setState(SYSTEM_BALANCE);
      break;
    case SYSTEM_BALANCE:
      ballbot_process();
      break;
    default:
      break;
  }
}
