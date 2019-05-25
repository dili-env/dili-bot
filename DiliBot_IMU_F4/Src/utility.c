/// @file utility.c
/// @brief Utility function declaration

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "utility.h"

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
void test_matrix(void) {
  arm_matrix_instance_q15 m1;
  m1.numCols = 5;
  m1.numRows = 5;
  
}

/*****************************************/

#endif /* _DEBUG_PRINTF_ */
