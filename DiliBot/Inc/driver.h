/// @file driver.h
/// @brief all driver initialization and execution function declaration

#ifndef _DRIVER_H_
#define _DIRVER_H_

#include "stm32f4xx_hal.h"
#include "stdio.h"
/* Private type define *******************************************************/
typedef enum {
  MOTOR_1 = 0,
  MOTOR_2,
  MOTOR_3,
  MOTOR_NONE = 9
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

// Debug suport function
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int char)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void SPI_DataSend(uint8_t *data, uint16_t size);

void TEST_AllMotor(void);

// Configuration
int imu_CmdInit(void);
int imu_StartIRQ(float *buffer, uint8_t size);


/// API for motor control function delclaration
int motor_Init(void);
void motor_Disable(MotorIndex motor_index);
void motor_DisableAll(void);
void motor_Enable(MotorIndex motor_index);
void motor_EnableAll(void);
int motor_SetSpeed(MotorIndex motor_index, int value);

/// API for encoder control function declaration
int encoder_Init(void);
int encoder_ReadMotor(MotorIndex motor_idx);

void TEST_Motor_API(void);
void TEST_Encoder_API(void);
#endif
