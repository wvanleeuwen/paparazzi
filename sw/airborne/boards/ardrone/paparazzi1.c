/*
 * File: paparazzi1.c
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

#include "paparazzi1.h"
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error "Code was generated for compiler with different sized uchar/char. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compiler's limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, which will disable the preprocessor word size checks."
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error "Code was generated for compiler with different sized ushort/short. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error "Code was generated for compiler with different sized uint/int. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error "Code was generated for compiler with different sized ulong/long. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

/*===========*
 * Constants *
 *===========*/
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
# if defined(__LCC__)
#   define UNUSED_PARAMETER(x)                                   /* do nothing */
# else

/*
 * This is the semi-ANSI standard way of indicating that an
 * unused function parameter is required.
 */
#   define UNUSED_PARAMETER(x)         (void) (x)
# endif
#endif

/* Model step function */
void paparazzi1_step(int32_T arg_Gyros[3], int32_T arg_Accelerometers[3],
                     uint32_T *arg_Barometer, uint32_T *arg_Sonar, uint32_T
                     arg_Commands[4])
{
  int32_T tmp;
  UNUSED_PARAMETER(arg_Gyros);

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Accelerometers'
   */
  /* MATLAB Function 'MATLAB Function': '<S1>:1' */
  /* '<S1>:1:4' a1 = accel(1); */
  /* '<S1>:1:5' a2 = accel(2); */
  /* '<S1>:1:6' a3 = baro; */
  /* '<S1>:1:7' a4 = sonar; */
  /* roll/pitch/yaw/throttle */
  /* '<S1>:1:10' y = uint32([a1,a2,a3,a4]'); */
  tmp = (((arg_Accelerometers[0] & 512) != 0) && (((arg_Accelerometers[0] & 511)
           != 0) || (arg_Accelerometers[0] > 0))) + (arg_Accelerometers[0] >> 10);
  if (tmp < 0) {
    tmp = 0;
  }

  /* Outport: '<Root>/Commands' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  arg_Commands[0] = (uint32_T)tmp;

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Accelerometers'
   */
  tmp = (((arg_Accelerometers[1] & 512) != 0) && (((arg_Accelerometers[1] & 511)
           != 0) || (arg_Accelerometers[1] > 0))) + (arg_Accelerometers[1] >> 10);
  if (tmp < 0) {
    tmp = 0;
  }

  /* Outport: '<Root>/Commands' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  arg_Commands[1] = (uint32_T)tmp;

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Barometer'
   */
  if (*arg_Barometer > 2097151U) {
    tmp = MAX_int32_T;
  } else {
    tmp = (int32_T)*arg_Barometer << 10;
  }

  /* Outport: '<Root>/Commands' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  arg_Commands[2] = (uint32_T)((((tmp & 512) != 0) && (((tmp & 511) != 0) ||
                                 (tmp > 0))) + (tmp >> 10));

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Sonar'
   */
  if (*arg_Sonar > 2097151U) {
    tmp = MAX_int32_T;
  } else {
    tmp = (int32_T)*arg_Sonar << 10;
  }

  /* Outport: '<Root>/Commands' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  arg_Commands[3] = (uint32_T)((((tmp & 512) != 0) && (((tmp & 511) != 0) ||
                                 (tmp > 0))) + (tmp >> 10));
}

/* Model initialize function */
void paparazzi1_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
