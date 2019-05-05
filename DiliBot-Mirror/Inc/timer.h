/// @file timer.h
/// @brief timer support function declaration

#ifndef _TIMER_H_
#define _TIMER_H_

#include <stdbool.h>
#include <stddef.h>

#define TIMER_PERIOD_MS     1
#define MAX_TIMEOUT_EVT     10
#define INVALID_TIMER_ID    0xff

typedef void (*TIMER_CALLBACK_FUNC)();
typedef unsigned char TIMER_ID;

void TIMER_ISR(void);
TIMER_ID TIMER_RegisterEvent(TIMER_CALLBACK_FUNC callback, unsigned long tenth_of_ms);
bool TIMER_UnregisterEvent(TIMER_ID timer_id);



#endif /* _TIMER_H_

*/
