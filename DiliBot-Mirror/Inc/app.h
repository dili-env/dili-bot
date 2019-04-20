/// @file app.h
/// @brief Application function declaration, statemachine declare utility function

#ifndef _APP_H_
#define _APP_H_

typedef enum {
  SYSTEM_INITIALIZE,
  SYSTEM_CHECK_CMD,
  SYSTEM_CALIB_SENSOR,
  SYSTEM_BALANCE, /* TODO: This state will be removed if robot can self-balance */
} SYSTEM_STATE;

void system_processSystemState(void);

#endif
