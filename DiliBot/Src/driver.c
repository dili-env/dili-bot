/// @file driver.c
/// @brief All dirver initialization and execution function definition

#include "stm32f4xx_hal.h"
#include "driver.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart5;
extern uint8_t SPI_Buffer;

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
  uint32_t counter1 = 0;
  uint32_t counter2 = 0;
  uint32_t counter3 = 0;
  uint8_t uart_buffer[50];
  int tmp = 0;

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
    counter1 = TIM1->CNT;
    counter2 = TIM4->CNT;
    counter3 = TIM8->CNT;

    snprintf((char*)uart_buffer, 50, "\nValue (1) is: %d -- ", counter1);
    HAL_UART_Transmit(&huart5, uart_buffer, 20, 10);
    snprintf((char*)uart_buffer, 50, "Value (2) is: %d -- ", counter2);
    HAL_UART_Transmit(&huart5, uart_buffer, 20, 10);
    snprintf((char*)uart_buffer, 50, "Value (3) is: %d\r\n", counter3);
    HAL_UART_Transmit(&huart5, uart_buffer, 20, 10);
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


/* Motor control API function definition *************************************/

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
      ret = -1;
      break;
  }
  return ret;
}
