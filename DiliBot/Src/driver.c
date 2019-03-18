/// @file driver.c
/// @brief All dirver initialization and execution function definition

#include "stm32f4xx_hal.h"
#include "driver.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

uint8_t SPI_Buffer;

/// @brief Retarget the C library function to UART
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart4, (uint8_t*)&ch, 1, 10);
  return ch;
}

void SPI_DataSend(uint8_t *data, uint16_t size) {
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, data, size, 10);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}

void TEST_AllMotor(void) {
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);
  } else {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 00);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 00);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 00);
  }
}

void TEST_Motor_API(void) {
  static int tmp = 0;

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
    printf("Value (1) is: %d -- Value (2) is: %d -- Value (3) is: %d\r\n",
            TIM1->CNT, TIM4->CNT, TIM8->CNT);
    tmp ++;
  }
  if (tmp > 3) motor_Disable(MOTOR_1);
  if (tmp > 6) motor_Disable(MOTOR_2);
  if (tmp > 9) motor_Disable(MOTOR_3);
  if (tmp > 12) motor_Enable(MOTOR_1);
  if (tmp > 15) motor_Enable(MOTOR_2);
  if (tmp > 18) motor_Enable(MOTOR_3);
  if (tmp > 20) motor_DisableAll();
  if (tmp > 22) motor_EnableAll();
  if (tmp > 25) motor_SetSpeed(MOTOR_1, 300);
  if (tmp > 28) motor_SetSpeed(MOTOR_2, 500);
  if (tmp > 31) motor_SetSpeed(MOTOR_3, 700);
  if (tmp > 34) motor_SetSpeed(MOTOR_1, -300);
  if (tmp > 37) motor_SetSpeed(MOTOR_2, -500);
  if (tmp > 40) motor_SetSpeed(MOTOR_3, -700);
  if (tmp > 50) motor_DisableAll();
  HAL_Delay(500);
}


void TEST_Encoder_API(void) {
  static int pre_encoder[3]; // testing encoder

  if(encoder_ReadMotor(MOTOR_1) != pre_encoder[0]) {
    pre_encoder[0] = encoder_ReadMotor(MOTOR_1);
    printf("Current encoder 1 value = %d\r\n", pre_encoder[0]);
  }
  if(encoder_ReadMotor(MOTOR_2) != pre_encoder[1]) {
    pre_encoder[1] = encoder_ReadMotor(MOTOR_2);
    printf("Current encoder 2 value = %d\r\n", pre_encoder[1]);
  }
  if(encoder_ReadMotor(MOTOR_3) != pre_encoder[2]) {
    pre_encoder[2] = encoder_ReadMotor(MOTOR_3);
    printf("Current encoder 3 value = %d\r\n", pre_encoder[2]);
  }
}





/*****************************************************************************/
/* IMU GY951 configuration function definition *******************************/
/// @brief Initialize IMU GY951 by sending command
///        Data streaming is disable untill imu_StartIRQ is call
/// @ret Param assertion and HAL uart sending status
///         - Normal return HAL_OK = 0
///         - UART send problem return sum of HAL UART sending status (> 0)
/*  Note: UART receive should not be activate at this time
          Sending command procedure consist of:
            - Disable streaming data '#o0'
            - Setting data streaming output to binary mode '#ob'
            - Send request one frame for test (Considering *FIXME)
*/
int imu_CmdInit(void) {
  char uart_buffer[5];
  int command_status = 0;
  // Disable streaming output
  snprintf(uart_buffer, 4, "#o0");
  command_status += HAL_UART_Transmit(&huart5, (uint8_t*)uart_buffer, 3, 10);
  // Delay for some garbage data stream
  HAL_Delay(500);
  // Setting binary mode output
  snprintf(uart_buffer, 4, "#ob");
  command_status += HAL_UART_Transmit(&huart5, (uint8_t*)uart_buffer, 3, 10);
  // Request one frame (considering *FIXME)
  snprintf(uart_buffer, 3, "#f");
  command_status += HAL_UART_Transmit(&huart5, (uint8_t*)uart_buffer, 2, 10);
  return command_status;
}

/// @brief Start IMU GY951 by sending command
///        Enable data streaming (should be call after init command for imu)
/// @param imu_buffer data receive buffer (3x float buffer)
///        imu_size size of receive buffer must be 12 (3x float)
/// @ret Param assertion and HAL uart sending status
///         - Normal return HAL_OK = 0
///         - Assert param false return -1
///         - UART send problem return sum of HAL UART sending status (> 0)
/*  Note: IMU must be configurate before calling this function
          Buffer of this function is designed for binary data streaming
          Start reanding IMU procedure consist of:
            - Enable uart dma receive interrupt
            - Start imu data streaming by sending '#o1'
*/
int imu_StartIRQ(float *imu_buffer, uint8_t imu_size) {
  char uart_buffer[5];
  int data_transfer_status = 0;
  // Assert param and size of float
  if ((imu_size != 12) || (sizeof(float) != 4)) return -1;
  // Enable uart receive dma interrupt
  data_transfer_status = 
    HAL_UART_Receive_DMA(&huart5, (uint8_t*)imu_buffer, imu_size);
  // Delay 0.5s
  HAL_Delay(500);
  // Start data streaming
  snprintf((char*)uart_buffer, 4, "#o1");
  data_transfer_status += 
    HAL_UART_Transmit(&huart5, (uint8_t*)uart_buffer, 3, 10);
  return data_transfer_status;
}

/*****************************************************************************/
/* Motor control API function definition *************************************/

/// @brief Initialize all motor (PWM init) and configurate control motor pin
/// @return Init PWM timer status (zero mean OK, other than zero mean error)
/*  Note: Initialize motor control pin following
            - Enable all motor
            - Set all motor direction to clockwise
*/
int motor_Init(void) {
  int tim_start_status = 0;
  // Initialize motor control pin
  SPI_Buffer = 0xD5;
  SPI_DataSend(&SPI_Buffer, 1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
  // Initialize all motor PWM
  tim_start_status  = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  tim_start_status += HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  tim_start_status += HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  return tim_start_status;
}

/// @brief Disable motor with specified motor index
///        This motor will not work untill it is enabled
/// @param motor_index index of the motor going to disable (from 1 to 3)
/*  Note: In case of motor is 1/2 write 0 to BIT7/BIT6 of SPI buffer
          In case of moter is 3 set PD15 to zero
*/
void motor_Disable(MotorIndex motor_index) {
  switch (motor_index) {
    case MOTOR_1:
      SPI_Buffer &= (~BIT_EN1);
      SPI_DataSend(&SPI_Buffer, 1);
      break;
    case MOTOR_2:
      SPI_Buffer &= (~BIT_EN2);
      SPI_DataSend(&SPI_Buffer, 1);
      break;
    case MOTOR_3:
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
      break;
    default:
      break;
  }
}

/// @brief Disable all motor (1,2,3)
/// @param none
void motor_DisableAll(void) {
  motor_SetSpeed(MOTOR_1, 0);
  motor_SetSpeed(MOTOR_2, 0);
  motor_SetSpeed(MOTOR_3, 0);
  SPI_Buffer &= ~(BIT_EN1|BIT_EN2);
  SPI_DataSend(&SPI_Buffer, 1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}

/// @brief Enable motor with specifid motor index
/// @param motor_index index of motor going to enable (from 1 to 3)
/*  Note: In case of motor is 1/2 write 1 to BIT7/BIT6 of SPI buffer
          In case of moter is 3 set PD15 to 1
*/
void motor_Enable(MotorIndex motor_index) {
  switch (motor_index) {
    case MOTOR_1:
      SPI_Buffer |= BIT_EN1;
      SPI_DataSend(&SPI_Buffer, 1);
      break;
    case MOTOR_2:
      SPI_Buffer |= BIT_EN2;
      SPI_DataSend(&SPI_Buffer, 1);
      break;
    case MOTOR_3:
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
      break;
    default:
      break;
  }
}

/// @brief Enable all motor (1,2,3)
/// @param none
void motor_EnableAll(void) {
  SPI_Buffer |= (BIT_EN1|BIT_EN2);
  SPI_DataSend(&SPI_Buffer, 1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
  motor_SetSpeed(MOTOR_1, 0);
  motor_SetSpeed(MOTOR_2, 0);
  motor_SetSpeed(MOTOR_3, 0);
}

/// @brief Set speed for specified motor (from -999 to 999)
/// @param motor_index index of motor that setting speed
///        value value to set (PWM duty cycle)
/// @return assert of value input, in case of motor index or value out of range
///         return -1 else normal return 0
/*  Note: If the value to set is out of range, set the saturate value
          if value > 999  => set value = 999
          if value < -999 => set value = -999
          The negative value mean motor direction is counterclockwise
          The positive value mean motor direction is clockwise
*/
int motor_SetSpeed(MotorIndex motor_index, int value) {
  MotorDirection dir = MOTOR_FORWARD;
  int ret = 0;
  /// Configure the direction and standarize value
  if (value < 0) {
    dir = MOTOR_BACKWARD;
    value = -value;
  }
  
  /// Set saturation value
  if (value > 999) value = 999;
  
  switch (motor_index) {
    case MOTOR_1:
      if (dir == MOTOR_FORWARD) {
        SPI_Buffer |= BIT_INA1;
        SPI_Buffer &= ~BIT_INB1;
      } else {
        SPI_Buffer &= ~BIT_INA1;
        SPI_Buffer |=  BIT_INB1;
      }
      SPI_DataSend(&SPI_Buffer, 1);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, value);
      break;
    case MOTOR_2:
      if (dir == MOTOR_FORWARD) {
        SPI_Buffer |= BIT_INA2;
        SPI_Buffer &= ~BIT_INB2;
      } else {
        SPI_Buffer &= ~BIT_INA2;
        SPI_Buffer |=  BIT_INB2;
      }
      SPI_DataSend(&SPI_Buffer, 1);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, value);
      break;
    case MOTOR_3:
      if (dir == MOTOR_FORWARD) {
        SPI_Buffer |= BIT_INA3;
        SPI_Buffer &= ~BIT_INB3;
      } else {
        SPI_Buffer &= ~BIT_INA3;
        SPI_Buffer |=  BIT_INB3;
      }
      SPI_DataSend(&SPI_Buffer, 1);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, value);
      break;
    default:
      printf("Error motor index in setting speed!");
      ret = -1;
      break;
  }
  return ret;
}

/*****************************************************************************/
/* Encoder control API function definition ***********************************/
/// @brief Enable all encode/timer
/*  Note: Reset all timer counter register and enable timer
*/
int encoder_Init(void) {
  int init_timer_status = 0;
  TIM1->CNT = 0;
  init_timer_status = HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  TIM4->CNT = 0;
  init_timer_status += HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  TIM8->CNT = 0;
  init_timer_status += HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  return init_timer_status;
}

int encoder_ReadMotor(MotorIndex motor_idx) {
  static int32_t pos[3], pos_pre[3];
  static int32_t d_pos[3], cur_pulse[3];

  switch (motor_idx) {
    case MOTOR_1:
      pos[0] = (int32_t)TIM1->CNT;
      break;
    case MOTOR_2:
      pos[1] = (int32_t)TIM4->CNT;
      break;
    case MOTOR_3:
      pos[2] = (int32_t)TIM8->CNT;
      break;
    default:
      printf("Error Motor index encoder read!");
      break;
  }
  
  d_pos[motor_idx] = pos[motor_idx] - pos_pre[motor_idx];
  if (d_pos[motor_idx] > 32768) d_pos[motor_idx] -= 65536;
  else if (d_pos[motor_idx] < -32768) d_pos[motor_idx] += 65536;
  
  pos_pre[motor_idx] = pos[motor_idx];
  cur_pulse[motor_idx] += d_pos[motor_idx];
  
  return cur_pulse[motor_idx];
}


