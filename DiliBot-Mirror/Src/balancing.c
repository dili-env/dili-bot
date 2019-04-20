/// @file balancing.c
/// @brief Balance ballbot by some theory function definition
#include <stdbool.h>
#include <stdint.h>

#include "driver.h"
#include "balancing.h"
#include "timer.h"
#include "driver_util.h"

extern uint32_t test_count;

#define UPDATE_TIME_TENTH_MS    50 /* 50 tenths of ms -> 5ms */

static TIMER_ID Balancing_TimerID = INVALID_TIMER_ID;
static bool Balancing_Flag = false;

static void balancing_stopTimeout(void) {
  if (Balancing_TimerID != INVALID_TIMER_ID)
    TIMER_UnregisterEvent(Balancing_TimerID);
  Balancing_TimerID = INVALID_TIMER_ID;
}

static TIMER_ID balancing_runTimeout(TIMER_CALLBACK_FUNC callback_func, unsigned long tenth_of_ms) {
  balancing_stopTimeout();
  Balancing_TimerID = TIMER_RegisterEvent(callback_func, tenth_of_ms);
  return Balancing_TimerID;
}

static void balancing_timeoutCallback(void) {
  Balancing_TimerID = INVALID_TIMER_ID;
  Balancing_Flag = true;
  balancing_runTimeout(&balancing_timeoutCallback, UPDATE_TIME_TENTH_MS);
}



void ballbot_balanceInit(void) {
  balancing_runTimeout(&balancing_timeoutCallback, UPDATE_TIME_TENTH_MS);
}

void ballbot_process(void) {
  if (Balancing_Flag) {
    Balancing_Flag = false;
    // debug_printf("Systick count: %d\r\n", test_count);
    // dma_printf((uint8_t*)&test_count, 1);
    sliding_control();
  }
}
