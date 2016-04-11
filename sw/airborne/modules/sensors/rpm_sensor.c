/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/sensors/rpm_sensor.c"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * RPM sensor based on time difference between pulses
 */

#include "modules/sensors/rpm_sensor.h"

#include "mcu_arch.h"
#include "mcu_periph/gpio.h"
#include "generated/airframe.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

static uint32_t timer_rollover_cnt;
static uint32_t rpm_last_pulse_time;
static uint16_t rpm = 0;
static int32_t current = 0;

#define ONE_MHZ_CLK 1000000
#define RC_RPM_TICKS_PER_USEC 6
#define RPM_IRQ_PRIO 2

/*
  <message name="MOTOR" id="34">
    <field name="rpm" type="uint16" unit="Hz"/>
    <field name="current" type="int32" unit="mA"/>
  </message>
*/

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void rpm_sensor_send_motor(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_MOTOR(trans, dev, AC_ID, &rpm, &current);
}
#endif

void rpm_sensor_init(void)
{
  rpm_last_pulse_time = 0;
  /* timer clock enable */
  rcc_periph_clock_enable(RCC_TIM_RPM);

  /* GPIO configuration as input capture for timer */
  gpio_setup_pin_af(RPM_GPIO_PORT, RPM_GPIO_PIN, RPM_GPIO_AF, FALSE);

  /* Time Base configuration */
  timer_reset(RPM_TIMER);
  timer_set_mode(RPM_TIMER, TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_period(RPM_TIMER, 0xFFFF);
  uint32_t timer_clk = timer_get_frequency(RPM_TIMER);
  timer_set_prescaler(RPM_TIMER, (timer_clk / (RC_RPM_TICKS_PER_USEC * ONE_MHZ_CLK)) - 1);

  /* TIM configuration: Input Capture mode ---------------------
      The Falling edge is used as active edge
   ------------------------------------------------------------ */
  timer_ic_set_polarity(RPM_TIMER, RPM_CHANNEL, TIM_IC_FALLING);
  timer_ic_set_input(RPM_TIMER, RPM_CHANNEL, RPM_TIMER_INPUT);
  timer_ic_set_prescaler(RPM_TIMER, RPM_CHANNEL, TIM_IC_PSC_OFF);
  timer_ic_set_filter(RPM_TIMER, RPM_CHANNEL, TIM_IC_OFF);

  /* Enable timer Interrupt(s). */
  nvic_set_priority(RPM_IRQ, RPM_IRQ_PRIO);
  nvic_enable_irq(RPM_IRQ);

  /* Enable the Capture/Compare and Update interrupt requests. */
  timer_enable_irq(RPM_TIMER, (RPM_CC_IE | TIM_DIER_UIE));

  /* Enable capture channel. */
  timer_ic_enable(RPM_TIMER, RPM_CHANNEL);

  /* TIM enable counter */
  timer_enable_counter(RPM_TIMER);

  timer_rollover_cnt = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MOTOR, rpm_sensor_send_motor);
#endif
}

void tim4_isr(void)
{
  if ((TIM4_SR & RPM_CC_IF) != 0) {
    timer_clear_flag(TIM4, RPM_CC_IF);

    uint32_t now = timer_get_counter(TIM4) + timer_rollover_cnt;

    // Calculate the RPM
    float diff_time = now - rpm_last_pulse_time;
    rpm_last_pulse_time = now;

    rpm = 1/(diff_time/RC_RPM_TICKS_PER_USEC)*1e-6 * 60;


  } else if ((TIM4_SR & TIM_SR_UIF) != 0) {
    timer_rollover_cnt += (1 << 16);
    timer_clear_flag(TIM4, TIM_SR_UIF);
  }
}
