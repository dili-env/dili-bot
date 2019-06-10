/// @file test_api.c
/// @brief Tesing API or desired feature function definition
#include "test_api.h"
#include "driver.h"
#include "mat_utility.h"
#include "../Matlab/pid_my_ert_rtw/pid_my.h"

extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim2;

// #TEST Tick 1ms and 5ms count
uint32_t ms1_tick_count = 0; /* Count each 1ms from 0 -> 4*/
uint32_t ms5_tick_count = 0; /* Count each 5ms */

void PID_MotorTestProcess(void) {
  uint8_t buff[20];
  int8_t start;
  uint8_t error_msg_buff[45] = "No motor running, can't stop anything :)\n";
  MotorIndex motor_running;
  int udk;
  static uint32_t tick_5ms;
  
  HAL_UART_Receive(&huart4, buff, 1, 1);
  if (buff[0] == 'a') start = 0;
  else if (buff[0] == 'b') start = 1;
  else if (buff[0] == 'c') start = 2;
  else if (buff[0] == 'r') start = 9;
  else if (buff[0] == 's') start = -1;
  else if (buff[0] == 'f') start = -9;
  else start = -99;
  // Check PID matlab code gen
  if (start >= 0 && start != 9) {
    motor_running = (MotorIndex)start;
    if (tick_5ms != ms5_tick_count) {
      tick_5ms = ms5_tick_count;
      In2 = (double)encoder_ReadMotor(motor_running);
      // rt_OneStep();
      Out1 = Out1*800/12;
      udk = (int)Out1;
      motor_SetSpeed(motor_running, udk);
    }
  }
  else if (start == -1) {
    if (motor_running != MOTOR_NONE)
      motor_SetSpeed(motor_running, 0);
    else
      dma_printf(error_msg_buff, 44);
  }
  else if (start == -9)
    motor_DisableAll();
  else if (start == 9)
    motor_EnableAll();
  else { // start == -99
    motor_SetSpeed(MOTOR_1, 0);
    HAL_Delay(20);
    motor_SetSpeed(MOTOR_2, 0);
    HAL_Delay(20);
    motor_SetSpeed(MOTOR_3, 0);
  }
}

/*****************************************************************************/
/* Testing API function definition                                           */
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

/*****************************************/
/* Debug mode - redirect printf function */
#ifdef _DEBUG_PRINTF_
extern UART_HandleTypeDef huart2;
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
  return ch;
}
/*****************************************/


/* Matrix manipulation *******************/
float32_t AI_f32[16];
arm_matrix_instance_f32 matAI; /* Inverse of matrix A */

float * example_matrix_manipulate(void){
  float32_t A_f32[16] = {
    0.10,    -0.32,     0.40,    -1.280,
    0.10,    0.32,     0.64,   2.0480,
    0.10,    0.16,     0.40,     6.40,
    0.10,    0.16,     0.64,   1.0240,
  };

  arm_status status;
  
  arm_matrix_instance_f32 matA;  /* Original matrix A */
  
  arm_mat_init_f32(&matA,  4, 4, A_f32);
  arm_mat_init_f32(&matAI, 4, 4, AI_f32);
  
  status = arm_mat_inverse_f32(&matA, &matAI);
  
  if (ARM_MATH_SUCCESS != status) {
    printf("Inverse status: %d\n",status);
    return NULL;
  }
  
  return matAI.pData;
}

/*****************************************/

#endif /* _DEBUG_PRINTF_ */