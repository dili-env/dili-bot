/// @file driver.h
/// @brief all driver initialization and execution function declaration

#ifndef _DRIVER_H_
#define _DIRVER_H_

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#include "main.h"


#define _DEBUG_

/* Private macro define ******************************************************/
#define EN_GPIO_Port GPIOD

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


// Debug suport function
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int char)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

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



void debug_printf(const char *fmt, ...);
void dma_printf(uint8_t *buff_ptr, uint16_t buff_size);

#endif
