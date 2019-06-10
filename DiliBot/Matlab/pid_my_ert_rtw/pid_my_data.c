/*
 * File: pid_my_data.c
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

#include "pid_my.h"
#include "pid_my_private.h"

/* Block parameters (auto storage) */
Parameters_pid_my pid_my_P = {
  1500.0,                              /* Expression: 1500
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  800.0,                               /* Computed Parameter: PulseGenerator_Period
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  400.0,                               /* Computed Parameter: PulseGenerator_Duty
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  0.005,                               /* Expression: P
                                        * Referenced by: '<S1>/Proportional Gain'
                                        */
  0.005,                               /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S1>/Integrator'
                                        */
  0.0,                                 /* Expression: InitialConditionForIntegrator
                                        * Referenced by: '<S1>/Integrator'
                                        */
  0.0,                                 /* Expression: D
                                        * Referenced by: '<S1>/Derivative Gain'
                                        */
  0.005,                               /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S1>/Filter'
                                        */
  0.0,                                 /* Expression: InitialConditionForFilter
                                        * Referenced by: '<S1>/Filter'
                                        */
  100.0,                               /* Expression: N
                                        * Referenced by: '<S1>/Filter Coefficient'
                                        */
  10.0,                                /* Expression: 10
                                        * Referenced by: '<Root>/Saturation'
                                        */
  -10.0,                               /* Expression: -10
                                        * Referenced by: '<Root>/Saturation'
                                        */
  0.0001                               /* Expression: I
                                        * Referenced by: '<S1>/Integral Gain'
                                        */
};

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
