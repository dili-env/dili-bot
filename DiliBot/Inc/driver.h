/// @file driver.h
/// @brief all driver initialization and execution function declaration

#ifndef _DRIVER_H_
#define _DIRVER_H_

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"

#include "main.h"


#define _DEBUG_

/* Private macro define ******************************************************/
#define EN_GPIO_Port GPIOD

// Debug suport function
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int char)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

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

/* ADIS IMU definition *******************************************************/
#define NUM_BYTE_TOKEN      7
#define NUM_TOKEN           9
#define ADIS_MSG_LENGTH     65
#define DMA_BUFF_LENGTH     (ADIS_MSG_LENGTH)
#define FRAME_END           0x0D
#define FRAME_START         0x0A

typedef struct {
    int32_t angle1;
    int32_t angle2;
    int32_t angle3;
    int32_t gyro1;
    int32_t gyro2;
    int32_t gyro3;
    int32_t acce1;
    int32_t acce2;
    int32_t acce3;
} ADIS_DATA_t;

typedef struct {
  float angle_r1;
  float angle_r2;
  float gyro_r1;
  float gyro_r2;
} ADIS_RAD_DATA_t;

/* Token of frame consist of 7 bytes */
typedef struct {uint8_t bytetoken[NUM_BYTE_TOKEN]; } FrameToken;
/* Frame message consist of start byte + 9 token + end byte */
typedef struct {
  uint8_t byteStart;
  FrameToken token[NUM_TOKEN];
  uint8_t byteEnd;
} FrameData;

typedef enum {
    TOKEN_UNKNOWN = -1,
    TOKEN_NORMAL = 0,
    TOKEN_END,
} TokenType_t;


HAL_StatusTypeDef adisStartIRQ(UART_HandleTypeDef *huart);
ADIS_DATA_t adisGetData(void);
ADIS_RAD_DATA_t adisGetRadData(void);
/* Private enum **************************************************************/


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


void dma_printf(uint8_t *buff_ptr, uint16_t buff_size);
int8_t dec2ascii5(int32_t dec, uint8_t* ascii_buff);
#endif
