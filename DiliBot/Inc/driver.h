/// @file driver.h
/// @brief all driver initialization and execution function declaration

#ifndef _DRIVER_H_
#define _DIRVER_H_

#include "stm32f4xx_hal.h"

/* Private type define *******************************************************/
typedef enum {
  MOTOR_1,
  MOTOR_2,
  MOTOR_3
} MotorIndex;

typedef enum {
  MOTOR_FORWARD,
  MOTOR_BACKWARD
} MotorDirection;

/* Private enum **************************************************************/
enum {
  BIT_INA1 = 0x01, // 0B 0000.0001
  BIT_INB1 = 0x02, // 0B 0000.0010
  BIT_INA2 = 0x04, // 0B 0000.0100
  BIT_INB2 = 0x08, // 0B 0000.1000
  BIT_INA3 = 0x10, // 0B 0001.0000
  BIT_INB3 = 0x20, // 0B 0010.0000
  BIT_EN1  = 0x40, // 0B 0100.0000
  BIT_EN2  = 0x80  // 0B 1000.0000
};

void SPI_DataSend(uint8_t *data, uint16_t size);

void TEST_AllMotor(void);

/// API for motor control function delclaration
void motor_Disable(MotorIndex motor_index);
void motor_DisableAll(void);
void motor_Enable(MotorIndex motor_index);
void motor_EnableAll(void);

int motor_SetSpeed(MotorIndex motor_index, int value);




void TEST_Motor_API(void);
#endif
