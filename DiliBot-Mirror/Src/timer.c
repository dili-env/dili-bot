/// @file timer.c
/// @brief Timer support instance
#include "timer.h"

typedef struct {
  TIMER_CALLBACK_FUNC callback;
  unsigned long period_cnt;
} TIMEOUT_EVT;

static TIMEOUT_EVT timer_event_list[MAX_TIMEOUT_EVT];

/// @brief Timer6 interrupt service routine
/// @param none
/// @return none
void TIMER_ISR(void) {
  for(int i=0; i<MAX_TIMEOUT_EVT; i++) {
    if(timer_event_list[i].period_cnt > 0) {
      timer_event_list[i].period_cnt--;
      if(timer_event_list[i].period_cnt == 0 && timer_event_list[i].callback != NULL) {
        (timer_event_list[i].callback)();
        /*
         * Only clear timeout callback when period equal to 0
         * Another callback could be register in current timeout callback
         */
        if (timer_event_list[i].period_cnt == 0)
          timer_event_list[i].callback = NULL;
      }
    }
  }
}

/// @brief Register event
/// @param callback call back function name
/// @param tenth_ms number of tenth of one ms
/// @return Timer event ID valid (from 0 -> 9) or invalid (0xff)
TIMER_ID TIMER_RegisterEvent(TIMER_CALLBACK_FUNC callback, unsigned long tenth_of_ms) {
  int i;
  /// Find valid timer id for registration
  for (i = 0; i < MAX_TIMEOUT_EVT; i++) {
    if ((timer_event_list[i].period_cnt == 0) && 
        (timer_event_list[i].callback == NULL)) break;
  }
  
  /// Invalid timer id return if event list reach limitation
  if (i == MAX_TIMEOUT_EVT)
    return INVALID_TIMER_ID;
  
  /// Register valid timer event to event list
  timer_event_list[i].period_cnt = tenth_of_ms;
  timer_event_list[i].callback = callback;
  return (TIMER_ID)i;
}

/// @brief Unregister timer event
/// @param timer_id index of timer event desired to unregister
/// @return Unregister event result
bool TIMER_UnregisterEvent(TIMER_ID timer_id) {
  bool ret = false;
  if (timer_id < MAX_TIMEOUT_EVT) {
    timer_event_list[timer_id].period_cnt = 0;
    timer_event_list[timer_id].callback = NULL;
    ret = true;
  }
  return ret;
}

