/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifdef HW_TIMERS

#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "HAL.h"

#include "timers.h"

// ------------------------
// Local defines
// ------------------------

#define NUM_HARDWARE_TIMERS 2

// ------------------------
// Private Variables
// ------------------------

stm32_timer_t TimerHandle[NUM_HARDWARE_TIMERS];
uint32_t TimerRates[NUM_HARDWARE_TIMERS];

HAL_STEP_TIMER_ISR();
HAL_TEMP_TIMER_ISR();

// ------------------------
// Public functions
// ------------------------

static uint32_t HAL_stepper_timer_prescaler(
  const uint8_t timer_num, const uint32_t desired_freq) {
  return (TimerHandle[timer_num]->getTimerClkFreq() + desired_freq / 2) / desired_freq;
}

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  if (!TimerHandle[timer_num]) {
    switch (timer_num) {
      case STEP_TIMER_NUM:
        // STEPPER TIMER TIM5 - use a 32bit timer
        TimerHandle[timer_num] = new HardwareTimer(STEP_TIMER_DEV);
        TimerHandle[timer_num]->attachInterrupt(STEP_TIMER_CALLBACK);
        TimerHandle[timer_num]->setPrescaleFactor(HAL_stepper_timer_prescaler(STEP_TIMER_NUM, STEPPER_TIMER_RATE));
        HAL_NVIC_SetPriority(STEP_TIMER_IRQ_NAME, 1, 0);
        break;

      case TEMP_TIMER_NUM:
        // TEMP TIMER TIM7 - any available 16bit Timer (1 already used for PWM)
        TimerHandle[timer_num] = new HardwareTimer(TEMP_TIMER_DEV);
        TimerHandle[timer_num]->attachInterrupt(TEMP_TIMER_CALLBACK);
        TimerHandle[timer_num]->setPrescaleFactor(HAL_stepper_timer_prescaler(TEMP_TIMER_NUM, TEMP_TIMER_RATE));
        HAL_NVIC_SetPriority(TEMP_TIMER_IRQ_NAME, 2, 0);
        break;
    }

    TimerRates[timer_num] = TimerHandle[timer_num]->getTimerClkFreq()
      / TimerHandle[timer_num]->getPrescaleFactor();
    TimerHandle[timer_num]->setOverflow(
      TimerRates[timer_num] / frequency - 1, TICK_FORMAT);
    TimerHandle[timer_num]->resume();
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case STEP_TIMER_NUM: HAL_NVIC_EnableIRQ(STEP_TIMER_IRQ_NAME); break;
    case TEMP_TIMER_NUM: HAL_NVIC_EnableIRQ(TEMP_TIMER_IRQ_NAME); break;
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case STEP_TIMER_NUM: HAL_NVIC_DisableIRQ(STEP_TIMER_IRQ_NAME); break;
    case TEMP_TIMER_NUM: HAL_NVIC_DisableIRQ(TEMP_TIMER_IRQ_NAME); break;
  }

  // We NEED memory barriers to ensure Interrupts are actually disabled!
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  switch (timer_num) {
    case STEP_TIMER_NUM:
      return NVIC->ISER[(uint32_t)((int32_t)STEP_TIMER_IRQ_NAME) >> 5]
        & (uint32_t)(1 << ((uint32_t)((int32_t)STEP_TIMER_IRQ_NAME) & (uint32_t)0x1F));
    case TEMP_TIMER_NUM:
      return NVIC->ISER[(uint32_t)((int32_t)TEMP_TIMER_IRQ_NAME) >> 5]
        & (uint32_t)(1 << ((uint32_t)((int32_t)TEMP_TIMER_IRQ_NAME) & (uint32_t)0x1F));
  }

  return false;
}

#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC

#endif // !HW_TIMERS