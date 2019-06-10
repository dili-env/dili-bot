/*
 * File: rt_SATURATE.h
 *
 * Real-Time Workshop code generated for Simulink model pid_my.
 *
 * Model version                        : 1.9
 * Real-Time Workshop file version      : 7.4  (R2009b)  29-Jun-2009
 * Real-Time Workshop file generated on : Sun Mar 10 22:36:01 2019
 * TLC version                          : 7.4 (Jul 14 2009)
 * C/C++ source code generated on       : Sun Mar 10 22:36:02 2019
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_rt_SATURATE_h_
#define RTW_HEADER_rt_SATURATE_h_
#define rt_SATURATE(sig,ll,ul)         (((sig) >= (ul)) ? (ul) : (((sig) <= (ll)) ? (ll) : (sig)) )
#endif                                 /* RTW_HEADER_rt_SATURATE_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
