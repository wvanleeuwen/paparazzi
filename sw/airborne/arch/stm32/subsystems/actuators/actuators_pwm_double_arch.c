/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/stm32/subsystems/actuators/actuators_pwm_double_arch.c
 *  STM32 PWM servos handling.
 */

//VALID TIMERS ARE TIM1,2,3,4,5,8,9,12

#include "subsystems/actuators/actuators_pwm_double_arch.h"
#include "subsystems/actuators/actuators_pwm_double.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>


#if defined(STM32F1)
//#define PCLK 72000000
#define PCLK AHB_CLK
#elif defined(STM32F4)
//#define PCLK 84000000
#define PCLK AHB_CLK/2
#endif

#define ONE_MHZ_CLK 1000000

#ifdef STM32F1
/**
 * HCLK = 72MHz, Timer clock also 72MHz since
 * TIM1_CLK = APB2 = 72MHz
 * TIM2_CLK = 2 * APB1 = 2 * 32MHz
 */
#define TIMER_APB1_CLK AHB_CLK
#define TIMER_APB2_CLK AHB_CLK
#endif

#ifdef STM32F4
/* Since APB prescaler != 1 :
 * Timer clock frequency (before prescaling) is twice the frequency
 * of the APB domain to which the timer is connected.
 */
#define TIMER_APB1_CLK (rcc_ppre1_frequency * 2)
#define TIMER_APB2_CLK (rcc_ppre2_frequency * 2)
#endif


/** Default servo update rate in Hz */
#ifndef SERVO_HZ
#define SERVO_HZ 40
#endif

// Update rate can be adapted for each timer
#ifndef TIM1_SERVO_HZ
#define TIM1_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM2_SERVO_HZ
#define TIM2_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM3_SERVO_HZ
#define TIM3_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM4_SERVO_HZ
#define TIM4_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM5_SERVO_HZ
#define TIM5_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM9_SERVO_HZ
#define TIM9_SERVO_HZ SERVO_HZ
#endif
#ifndef TIM12_SERVO_HZ
#define TIM12_SERVO_HZ SERVO_HZ
#endif


/** @todo: these should go into libopencm3 */
#define TIM9				TIM9_BASE
#define TIM12				TIM12_BASE

int32_t actuators_pwm_double_values[ACTUATORS_PWM_DOUBLE_NB];


/** Set PWM channel configuration
 */
static inline void actuators_pwm_double_arch_channel_init(uint32_t timer_peripheral,
                                                   enum tim_oc_id oc_id) {

  timer_disable_oc_output(timer_peripheral, oc_id);
  //There is no such register in TIM9 and 12.
  if (timer_peripheral != TIM9 && timer_peripheral != TIM12)
    timer_disable_oc_clear(timer_peripheral, oc_id);
  timer_enable_oc_preload(timer_peripheral, oc_id);
  timer_set_oc_slow_mode(timer_peripheral, oc_id);
  timer_set_oc_mode(timer_peripheral, oc_id, TIM_OCM_PWM1);
  timer_set_oc_polarity_high(timer_peripheral, oc_id);
  timer_enable_oc_output(timer_peripheral, oc_id);
  // Used for TIM1 and TIM8, the function does nothing if other timer is specified.
  timer_enable_break_main_output(timer_peripheral);
}

/** Set GPIO configuration
 */
#if defined(STM32F4)
static inline void set_servo_gpio(uint32_t gpioport, uint16_t pin, uint8_t af_num, uint32_t en) {
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, en);
  gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, pin);
  gpio_set_af(gpioport, af_num, pin);
}
#elif defined(STM32F1)
static inline void set_servo_gpio(uint32_t gpioport, uint16_t pin, uint8_t none, uint32_t en) {
  rcc_peripheral_enable_clock(&RCC_APB2ENR, en | RCC_APB2ENR_AFIOEN);
  gpio_set_mode(gpioport, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, pin);
}
#endif

/** Set Timer configuration
 */
static inline void set_servo_timer(uint32_t timer, uint32_t period, uint8_t channels_mask) {
  timer_reset(timer);

  /* Timer global mode:
   * - No divider.
   * - Alignement edge.
   * - Direction up.
   */
  if ((timer == TIM9) || (timer == TIM12))
    //There are no EDGE and DIR settings in TIM9 and TIM12
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, 0, 0);
  else
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);


  // TIM1, 8 and 9 use APB2 clock, all others APB1
  if (timer != TIM1 && timer != TIM8 && timer != TIM9) {
    timer_set_prescaler(timer, (TIMER_APB1_CLK / ONE_MHZ_CLK) - 1); // 1uS
  } else {
    // TIM9, 1 and 8 use APB2 clock
    timer_set_prescaler(timer, (TIMER_APB2_CLK / ONE_MHZ_CLK) - 1);
  }

  timer_disable_preload(timer);

  timer_continuous_mode(timer);

  timer_set_period(timer, (ONE_MHZ_CLK / period) - 1);

  /* Disable outputs and configure channel if needed. */
  if (bit_is_set(channels_mask, 0)) {
    actuators_pwm_double_arch_channel_init(timer, TIM_OC1);
  }
  if (bit_is_set(channels_mask, 1)) {
    actuators_pwm_double_arch_channel_init(timer, TIM_OC2);
  }
  if (bit_is_set(channels_mask, 2)) {
    actuators_pwm_double_arch_channel_init(timer, TIM_OC3);
  }
  if (bit_is_set(channels_mask, 3)) {
    actuators_pwm_double_arch_channel_init(timer, TIM_OC4);
  }

  /*
   * Set initial output compare values.
   * Note: Maybe we should preload the compare registers with some sensible
   * values before we enable the timer?
   */
  //timer_set_oc_value(timer, TIM_OC1, 1000);
  //timer_set_oc_value(timer, TIM_OC2, 1000);
  //timer_set_oc_value(timer, TIM_OC3, 1000);
  //timer_set_oc_value(timer, TIM_OC4, 1000);

  /* -- Enable timer -- */
  /*
   * ARR reload enable.
   * Note: In our case it does not matter much if we do preload or not. As it
   * is unlikely we will want to change the frequency of the timer during
   * runtime anyways.
   */
  timer_enable_preload(timer);

  /* Counter enable. */
  timer_enable_counter(timer);

}

/** PWM arch init called by generic pwm driver
 */
void actuators_pwm_double_arch_init(void) {

  /*-----------------------------------
   * Configure timer peripheral clocks
   *-----------------------------------*/
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);

  /*----------------
   * Configure GPIO
   *----------------*/

  set_servo_gpio(PWM_DOUBLE_SERVO_3_GPIO, PWM_DOUBLE_SERVO_3_PIN, PWM_DOUBLE_SERVO_3_AF, PWM_DOUBLE_SERVO_3_RCC_IOP);
  set_servo_gpio(PWM_DOUBLE_SERVO_4_GPIO, PWM_DOUBLE_SERVO_4_PIN, PWM_DOUBLE_SERVO_4_AF, PWM_DOUBLE_SERVO_4_RCC_IOP);

  set_servo_timer(TIM4, TIM4_SERVO_HZ, PWM_TIM4_CHAN_MASK);


  /* Enable timer Interrupt(s). */
  nvic_set_priority(NVIC_TIM4_IRQ, 2);
  nvic_enable_irq(NVIC_TIM4_IRQ);

  /* Enable the Capture/Compare and Update interrupt requests. */
  timer_enable_irq(TIM4, TIM_DIER_CC3IE);

}

/** Set pulse widths from actuator values, assumed to be in us
 */
void actuators_pwm_double_commit(void) {
  timer_set_oc_value(PWM_DOUBLE_SERVO_3_TIMER, PWM_DOUBLE_SERVO_3_OC, actuators_pwm_double_values[PWM_DOUBLE_SERVO_3]);
  timer_set_oc_value(PWM_DOUBLE_SERVO_4_TIMER, PWM_DOUBLE_SERVO_4_OC, actuators_pwm_double_values[PWM_DOUBLE_SERVO_4]);
}

#define PWM_DOUBLE_CC_IF TIM_SR_CC3IF

void tim4_isr(void) {
  if((TIM1_SR & PWM_DOUBLE_CC_IF) != 0) {
    timer_clear_flag(TIM1, PWM_DOUBLE_CC_IF);

    uint32_t counter = timer_get_counter(PWM_DOUBLE_SERVO_3_TIMER);
    if (counter < actuators_pwm_double_values[PWM_DOUBLE_SERVO_3] + 4000) {
      counter += 4000; // 4ms pause
      timer_set_oc_mode(PWM_DOUBLE_SERVO_3_TIMER, PWM_DOUBLE_SERVO_3_OC, TIM_OCM_TOGGLE);
    }
    else if (counter < actuators_pwm_double_values[PWM_DOUBLE_SERVO_3] + 4000 + actuators_pwm_double_values[PWM_DOUBLE_SERVO_4]) {
      counter += actuators_pwm_double_values[PWM_DOUBLE_SERVO_4];
      timer_set_oc_mode(PWM_DOUBLE_SERVO_3_TIMER, PWM_DOUBLE_SERVO_3_OC, TIM_OCM_PWM1);
    }
    timer_set_oc_value(PWM_DOUBLE_SERVO_3_TIMER, PWM_DOUBLE_SERVO_3_OC, counter);

  }
}
