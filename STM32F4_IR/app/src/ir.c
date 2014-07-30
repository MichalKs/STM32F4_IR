/**
 * @file:   ir.c
 * @brief:  Infrared decoding functions
 * @date:   27 lip 2014
 * @author: Michal Ksiezopolski
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

#include <ir.h>
#include <stdio.h>
#include <stm32f4xx.h>

/**
 * @defgroup  IR IR
 * @brief     Infrared decoding functions
 */

/**
 * @addtogroup IR
 * @{
 */

#define RC5
//#define SIRC

#define RC5_TIMEOUT   3600 ///< Timeout value in us of RC5 frame
#define RC5_MAX_BIT   1900 ///< Max bit length in us
#define RC5_MIN_HALFBIT  700 ///< Min half bit value in us
#define RC5_MAX_HALFBIT  1000 ///< Max half bit value in us

static uint8_t address; ///< Remote address
static uint8_t command; ///< Remote command
static uint8_t toggle;  ///< Toggle bit
static uint8_t rxCount; ///< Received frames counter

static uint16_t frame;       ///< The whole received frame
static uint8_t  pulseCount;  ///< Counts the number of half bits
static uint8_t  bitCount;    ///< Counts the number of bits received

typedef enum {
  RC5_DIGIT0 = 0,
  RC5_DIGIT1,
  RC5_DIGIT2,
  RC5_DIGIT3,
  RC5_DIGIT4,
  RC5_DIGIT5,
  RC5_DIGIT6,
  RC5_DIGIT7,
  RC5_DIGIT8,
  RC5_DIGIT9,
  RC5_STANDBY     = 12,
  RC5_MUTE        = 13,
  RC5_VOL_UP      = 16,
  RC5_VOL_DOWN    = 17,
  RC5_BRIGHT_UP   = 18,
  RC5_BRIGHT_DOWN = 19,
  RC5_CHAN_UP     = 32,
  RC5_CHAN_DOWN   = 33,
  RC5_SLEEP       = 38,
  RC5_TXT         = 60,

} IR_RC5_Commands;

/**
 * @brief Initialize timers and pins for decoding IR signals.
 */
void IR_Init(void) {

  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // TIM4 clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  // GPIOB clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  // TIM4 IC2 configuration (PB7)
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Connect TIM pin to AF2
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

  // Time Base configuration
  // Timer clock is 84MHz
  // Update flag will cause timeout of frame
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 1 tick = 1 us
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = RC5_TIMEOUT;
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
  The external signal is connected to TIM4 CH2 pin (PB7).
  The falling edge is used as active edge - IC counters will be reset at falling edge
  The TIM4 CCR2 is used to compute the period (from which we derive high pulse value)
  The TIM4 CCR1 is used to compute the low pulse value */

  TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Falling; // Falling edge triggers CC2
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
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
 * @brief Decode RC5 data
 * @param pulseWidth Width of the received pulse
 * @param edge 0 - low pulse (rising edge), 1 - period (falling edge)
 */
void IR_RxData(uint16_t pulseWidth, uint8_t edge) {

  // frame starts with falling edge
  if (pulseCount == 0 && edge == 1) {
    pulseCount++;
    bitCount = 13;
    frame = 0; // new frame
    return;
  } else if (pulseCount == 0 && edge == 0) {
    // frame should start with falling edge
    printf("Frame error - wrong edge\r\n");
    pulseCount = 0;
    bitCount = 13;
    return;
  }

  // Pulse width was irrelevant at first edge.
  // It is relevant for all following edges
  if ((pulseWidth > RC5_MAX_BIT) || (pulseWidth < RC5_MIN_HALFBIT)) {
    printf("Frame error - wrong pulse width\r\n");
    pulseCount = 0;
    bitCount = 13;
    return;
  }

  if (pulseCount == 1) {
    // pulseWidth has to be 800 us - first two bits are a one
    if (pulseWidth > RC5_MAX_HALFBIT) {
      printf("Frame error - wrong start bits, probably not RC5\r\n");
      pulseCount = 0;
      bitCount = 13;
      return;
    }
    frame |= (1<<bitCount--); // First bit is a one
    pulseCount++;
  } else if (pulseCount == 2) {
    // pulseWidth has to be 800 us - first two bits are a one
    if (pulseWidth > RC5_MAX_HALFBIT) {
      printf("Frame error - wrong start bits, probably not RC5\r\n");
      pulseCount = 0;
      bitCount = 13;
      return;
    }
    frame |= (1<<bitCount--); // Second bit is a one
    pulseCount++;
  } else if ((bitCount >= 0) && (bitCount < 12)) { // for bits 11 to 0
    // if pulseWidth is about 1700us then two half bits were transmitted
    // so increment pulseCount
    if (pulseWidth > RC5_MAX_HALFBIT) {
      pulseCount++;
    }
    // even pulseCounts mean middle of bit
    // rising edge in the middle of a bit means a zero
    // falling edge is a one
    if (pulseCount % 2 == 0) {
      if (edge == 1) { // falling edge
        frame |= (1<<bitCount--);
      } else { // rising edge
        bitCount--;
      }
    }
    // when bit zero is written, bitCount underflows to 0xff (8 bits)
    if (bitCount == 0xff) {

      bitCount = 13; // reset frame
      pulseCount = 0;

      rxCount++; // add received frame
      toggle = (frame>>11) & (0x0001); // toggle bit is 3rd bit
      address = (frame>>6) & (0x001f); // address is 5 bits
      command = (frame>>0) & (0x003f); // command is 6 bits
      printf("Frame received: %04x. Toggle = %d Command = %d Address = %d\r\n",
          frame, toggle, command, address);
      return;
    }
    pulseCount++; // increment pulse count value
  }

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

    IR_RxData(low, 0); // decode low pulse

  } else  if((TIM_GetFlagStatus(TIM4, TIM_FLAG_CC2) != RESET)) {

    TIM_ClearFlag(TIM4, TIM_FLAG_CC2);
    // Get the Input Capture value
    high = TIM_GetCapture2(TIM4);

    IR_RxData(high - low, 1); // decode high pulse

  } else if ((TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) != RESET)) {
    // Clears the IR_TIM's pending flags
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);

    // If timeout occurs, clear frame state.
    bitCount = 13;
    pulseCount = 0;
  }

}

/**
 * @}
 */
