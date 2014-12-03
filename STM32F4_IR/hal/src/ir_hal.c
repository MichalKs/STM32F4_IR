/**
 * @file: 	ir_hal.c
 * @brief:	
 * @date: 	3 gru 2014
 * @author: Michal Ksiezopolski
 *
 *
 * @verbatim
 * Copyright (c) 2014 Michal Ksiezopolski.
 * All rights reserved. This program and the
 * accompanying materials are made available
 * under the terms of the GNU Public License
 * v3.0 which accompanies this distribution,
 * and is available at
 * http://www.gnu.org/licenses/gpl.html
 * @endverbatim
 */


#include <stm32f4xx.h>

static void (*readDataCallback)(uint16_t pulseWidth, uint8_t edge); ///< Callback for sending received pulses to higher layer
static void (*resetFrameCallback)(void); ///< Callback for resetting frame if timeout occurs.

/**
 * @brief Initialize hardware for decoding IR codes.
 */
void IR_HAL_Init(
    void (*readDataCb)(uint16_t pulseWidth, uint8_t edge),
    void (*resetFrameCb)(void),
    uint32_t timeout) {

  readDataCallback = readDataCb;
  resetFrameCallback = resetFrameCb;

  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // TIM4 clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  // GPIOB clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  // TIM4 IC2 pin configuration (PB7)
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Connect IC2 pin to AF2
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

  // Time Base configuration
  // Timer clock is 84MHz
  // Update flag will cause timeout of frame
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 1 tick = 1 us
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = timeout;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  // Enable the TIM4 global Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM4 configuration: PWM Input mode for measuring IR signal pulses.
   * The external signal is connected to TIM4 CH2 pin (PB7).
   * The falling edge is used as active edge - IC counters will be reset at falling edge
   * The TIM4 CCR2 is used to compute the period (from which we derive high pulse value)
   * The TIM4 CCR1 is used to compute the low pulse value
   */
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Falling; // Falling edge triggers CC2
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // Standard connections
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // count every edge
  TIM_ICInitStructure.TIM_ICFilter    = 0x0;

  TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);

  // Select the TIM4 Input Trigger: TI2FP2
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);

  // Select the slave Mode: Reset Mode
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);
  TIM_UpdateRequestConfig(TIM4, TIM_UpdateSource_Regular);

  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); // update interrupt for frame timeout
  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE); // falling edge - period value
  TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE); // rising edge - low pulse value
  TIM_Cmd(TIM4, ENABLE);

}

/**
 * @brief Timer 4 Handler
 *
 * @details Used for decoding RC4 frames using PWMI (input capture) measurement.
 */
void TIM4_IRQHandler(void) {

  static uint16_t high = 0;  // high pulse duration
  static uint16_t low = 0;   // low pulse duration

  // IC1 Interrupt
  if((TIM_GetFlagStatus(TIM4, TIM_FLAG_CC1) != RESET)) {

    TIM_ClearFlag(TIM4, TIM_FLAG_CC1);
    // Get the Input Capture value
    low = TIM_GetCapture1(TIM4);

    readDataCallback(low, 0); // decode low pulse

  } else  if((TIM_GetFlagStatus(TIM4, TIM_FLAG_CC2) != RESET)) {

    TIM_ClearFlag(TIM4, TIM_FLAG_CC2);
    // Get the Input Capture value
    high = TIM_GetCapture2(TIM4);

    readDataCallback(high - low, 1); // decode high pulse

  } else if ((TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) != RESET)) {
    // Clears the IR_TIM's pending flags
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);

    // If timeout occurs, clear frame state.
    resetFrameCallback();
  }

}

