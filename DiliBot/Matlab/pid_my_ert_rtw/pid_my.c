/*
 * File: pid_my.c
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

/* Exported block signals */
real_T In2;                            /* '<Root>/In2' */
real_T Out2;                           /* '<Root>/Subtract' */
real_T Out1;                           /* '<Root>/Saturation' */

/* Block states (auto storage) */
D_Work_pid_my pid_my_DWork;

/* Real-time model */
RT_MODEL_pid_my pid_my_M_;
RT_MODEL_pid_my *pid_my_M = &pid_my_M_;

/* Model step function */
void pid_my_step(void)
{
  real_T rtb_Sum;
  real_T rtb_FilterCoefficient;

  /* DiscretePulseGenerator: '<Root>/Pulse Generator' */
  rtb_Sum = ((real_T)pid_my_DWork.clockTickCounter <
             pid_my_P.PulseGenerator_Duty) && (pid_my_DWork.clockTickCounter >=
    0) ? pid_my_P.PulseGenerator_Amp : 0.0;
  if ((real_T)pid_my_DWork.clockTickCounter >= pid_my_P.PulseGenerator_Period -
      1.0) {
    pid_my_DWork.clockTickCounter = 0;
  } else {
    pid_my_DWork.clockTickCounter = pid_my_DWork.clockTickCounter + 1;
  }

  /* Sum: '<Root>/Subtract' incorporates:
   *  Inport: '<Root>/In2'
   */
  Out2 = rtb_Sum - In2;

  /* Gain: '<S1>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S1>/Filter'
   *  Gain: '<S1>/Derivative Gain'
   *  Sum: '<S1>/SumD'
   */
  rtb_FilterCoefficient = (pid_my_P.DerivativeGain_Gain * Out2 -
    pid_my_DWork.Filter_DSTATE) * pid_my_P.FilterCoefficient_Gain;

  /* Sum: '<S1>/Sum' incorporates:
   *  DiscreteIntegrator: '<S1>/Integrator'
   *  Gain: '<S1>/Proportional Gain'
   */
  rtb_Sum = (pid_my_P.ProportionalGain_Gain * Out2 +
             pid_my_DWork.Integrator_DSTATE) + rtb_FilterCoefficient;

  /* Saturate: '<Root>/Saturation' */
  Out1 = rt_SATURATE(rtb_Sum, pid_my_P.Saturation_LowerSat,
                     pid_my_P.Saturation_UpperSat);

  /* Update for DiscreteIntegrator: '<S1>/Integrator' incorporates:
   *  Gain: '<S1>/Integral Gain'
   */
  pid_my_DWork.Integrator_DSTATE = pid_my_P.IntegralGain_Gain * Out2 *
    pid_my_P.Integrator_gainval + pid_my_DWork.Integrator_DSTATE;

  /* Update for DiscreteIntegrator: '<S1>/Filter' */
  pid_my_DWork.Filter_DSTATE = pid_my_P.Filter_gainval * rtb_FilterCoefficient +
    pid_my_DWork.Filter_DSTATE;
}

/* Model initialize function */
void pid_my_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(pid_my_M, (NULL));

  /* block I/O */

  /* exported global signals */
  Out2 = 0.0;
  Out1 = 0.0;

  /* states (dwork) */
  (void) memset((void *)&pid_my_DWork, 0,
                sizeof(D_Work_pid_my));

  /* external inputs */
  In2 = 0.0;

  /* Start for DiscretePulseGenerator: '<Root>/Pulse Generator' */
  pid_my_DWork.clockTickCounter = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Integrator' */
  pid_my_DWork.Integrator_DSTATE = pid_my_P.Integrator_IC;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Filter' */
  pid_my_DWork.Filter_DSTATE = pid_my_P.Filter_IC;
}

/* Model terminate function */
void pid_my_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
