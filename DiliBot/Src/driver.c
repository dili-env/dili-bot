/// @file driver.c
/// @brief All dirver initialization and execution function definition

#include "stm32f4xx_hal.h"
#include "driver.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;

#define OFFSET_ANGLE_1    (-48.92  )
#define OFFSET_ANGLE_2    (  8.7808)
#define OFFSET_GYRO_1     (2736.276)
#define OFFSET_GYRO_2     (2129.149)

#define PI                (3.14159265358979f)
#define ADIS2RAD          (0.000174532952f)       /* raw *(1/100)*(1/180)*PI */
/// @brief Retarget the C library function to UART
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
  return ch;
}

void dma_printf(uint8_t *buff_ptr, uint16_t buff_size) {
  HAL_UART_Transmit_DMA(&huart4, buff_ptr, buff_size);
}

/// @brief Decimal to ascii 5 digit include sign of dec
int8_t dec2ascii5(int32_t dec, uint8_t* ascii_buff) {
  int32_t tmp_dec = dec;
  int8_t idx = 1;
  /// Constrain dec from -999 to 0999
  if (dec < -9999) {
    for (int i = 0; i < 5; i++)
      *(ascii_buff+i) = '-';
    return 5;
  } else if (dec > 9999) {
    for (int i = 0; i < 5; i++)
      *(ascii_buff+i) = '+';
    return 5;
  }
  /// Sign position
  if (dec < 0) {
      *ascii_buff = '-';
      tmp_dec = -dec;
  } else *ascii_buff = '0';
  *(ascii_buff + (idx++)) = (tmp_dec / 1000) + '0';
  tmp_dec = tmp_dec % 1000;
  *(ascii_buff + (idx++)) = (tmp_dec / 100)  + '0';
  tmp_dec = tmp_dec % 100;
  *(ascii_buff + (idx++)) = (tmp_dec / 10) + '0';
  *(ascii_buff + (idx++)) = (tmp_dec % 10) + '0';
  return idx;
}

/*****************************************************************************/
/* ADIS IMU manipulate function definition ***********************************/
// Prototype
TokenType_t decodeToken(FrameToken token, int32_t *idata);
bool decodeFrame(void);

// DMA buffer is set as Frame data
static FrameData adisMessage;
// IMU data
static ADIS_DATA_t imuData;

/* API function provided ******************/
/// @brief ADIS IMU data get
ADIS_DATA_t adisGetData(void) {
  return imuData;
}

/// @brief ADIS IMU radian data get with calibration
ADIS_RAD_DATA_t adisGetRadData(void) {
  ADIS_RAD_DATA_t tmpRadData;
  
  tmpRadData.angle_r1 = (imuData.angle1 - OFFSET_ANGLE_1)*ADIS2RAD;
  tmpRadData.angle_r2 = (imuData.angle2 - OFFSET_ANGLE_2)*ADIS2RAD;
  
  tmpRadData.gyro_r1 = (imuData.gyro1 - OFFSET_GYRO_1)*ADIS2RAD;
  tmpRadData.gyro_r2 = (imuData.gyro2 - OFFSET_GYRO_2)*ADIS2RAD;

  return tmpRadData;
}

/// @brief Start DMA interupt receive
HAL_StatusTypeDef adisStartIRQ(UART_HandleTypeDef *huart) {
  HAL_StatusTypeDef status = 
    HAL_UART_Receive_DMA(huart, (uint8_t*)&adisMessage, DMA_BUFF_LENGTH);
  return status;
}


/* Private function definition ************/
/// @brief UART5 DMA RX callback handler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART5) {
    /// Check start/end frame
    if ((FRAME_START != adisMessage.byteStart) &&
         FRAME_END   != adisMessage.byteEnd) {
      HAL_UART_DMAStop(&huart5);
      printf("Frame error -> restart\n");
      HAL_UART_Receive_DMA(huart, (uint8_t*)&adisMessage, DMA_BUFF_LENGTH);
      return;
    }
    /// Decode frame
    if (false == decodeFrame()) printf("Decode fail\n");
    else { /* Do not thing if decoded success */ }
  }
}

/// @brief decode one frame
bool decodeFrame(void) {
  TokenType_t type;

  type  = decodeToken(adisMessage.token[0], &imuData.angle1);
  type += decodeToken(adisMessage.token[1], &imuData.angle2);
  type += decodeToken(adisMessage.token[2], &imuData.angle3);

  type += decodeToken(adisMessage.token[3], &imuData.gyro1);
  type += decodeToken(adisMessage.token[4], &imuData.gyro2);
  type += decodeToken(adisMessage.token[5], &imuData.gyro3);

  type += decodeToken(adisMessage.token[6], &imuData.acce1);
  type += decodeToken(adisMessage.token[7], &imuData.acce2);
  type += decodeToken(adisMessage.token[8], &imuData.acce3);

  if (TOKEN_NORMAL != type) return false;
  return true;
}

/// @brief decode each token
TokenType_t decodeToken(FrameToken token, int32_t *idata) {
  int32_t tmp = 0;
  tmp += (token.bytetoken[1] - '0')*10000;
  tmp += (token.bytetoken[2] - '0')*1000;
  tmp += (token.bytetoken[3] - '0')*100;
  tmp += (token.bytetoken[4] - '0')*10;
  tmp += (token.bytetoken[5] - '0');

  if (token.bytetoken[0] == '-') tmp = -tmp;
  *idata = tmp;
  if (token.bytetoken[6] != ' ') return TOKEN_UNKNOWN;
  return TOKEN_NORMAL;
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
  // Enable all motor and set direction to clockwise
  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin| EN2_Pin | EN3_Pin, GPIO_PIN_SET);

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
      HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_RESET);
      break;
    case MOTOR_2:
      HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_RESET);
      break;
    case MOTOR_3:
      HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_RESET);
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
  HAL_GPIO_WritePin(EN_GPIO_Port, EN1_Pin | EN2_Pin | EN3_Pin, GPIO_PIN_RESET);
}

/// @brief Enable motor with specifid motor index
/// @param motor_index index of motor going to enable (from 1 to 3)
/*  Note: In case of motor is 1/2 write 1 to BIT7/BIT6 of SPI buffer
          In case of moter is 3 set PD15 to 1
*/
void motor_Enable(MotorIndex motor_index) {
  switch (motor_index) {
    case MOTOR_1:
      HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
      break;
    case MOTOR_2:
      HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);
      break;
    case MOTOR_3:
      HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_SET);
      break;
    default:
      break;
  }
}

/// @brief Enable all motor (1,2,3)
/// @param none
void motor_EnableAll(void) {
  HAL_GPIO_WritePin(EN_GPIO_Port, EN1_Pin | EN2_Pin | EN3_Pin, GPIO_PIN_SET);
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
        HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_SET);
      }
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, value);
      break;
    case MOTOR_2:
      if (dir == MOTOR_FORWARD) {
        HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_SET);
      }
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, value);
      break;
    case MOTOR_3:
      if (dir == MOTOR_FORWARD) {
        HAL_GPIO_WritePin(INA3_GPIO_Port, INA3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(INB3_GPIO_Port, INB3_Pin, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(INA3_GPIO_Port, INA3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INB3_GPIO_Port, INB3_Pin, GPIO_PIN_SET);
      }
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

/// @brief Read current specified encoder value
/// @param motor_idx index of motor desired
/*  Note: Timer registers' value were calibrated before return
*/
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
