/*
 * File: paparazzi1.h
 *
 * Code generated for Simulink model 'paparazzi1'.
 *
 * Model version                  : 1.28
 * Simulink Coder version         : 8.4 (R2013a) 13-Feb-2013
 * TLC version                    : 8.4 (Jan 19 2013)
 * C/C++ source code generated on : Tue Dec 17 16:42:14 2013
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM 9
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_paparazzi1_h_
#define RTW_HEADER_paparazzi1_h_
#ifndef paparazzi1_COMMON_INCLUDES_
# define paparazzi1_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* paparazzi1_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */

/* Model entry point functions */
extern void paparazzi1_initialize(void);

/* Customized model step function */
extern void paparazzi1_step(int32_T arg_Gyros[3], int32_T arg_Accelerometers[3],
  uint32_T *arg_Barometer, uint32_T *arg_Sonar, uint32_T arg_Commands[4]);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'paparazzi1'
 * '<S1>'   : 'paparazzi1/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_paparazzi1_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
