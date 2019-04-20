/// @file balancing.h
/// @brief Balance ballbot function declaration

#ifndef _BALANCING_H_
#define _BALANCING_H_

#include <math.h>

void simple_control_LQR(void);
void simple_control_LQR_2(void);
void sliding_control(void);

void ballbot_balanceInit(void);
void ballbot_process(void);

#endif /* _BALANCING_H_ */
