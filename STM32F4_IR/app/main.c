/**
 * @file: 	main.c
 * @brief:	SD Card test
 * @date: 	9 kwi 2014
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
#include <stdio.h>
#include <stdlib.h>

#include <timers.h>
#include <led.h>
#include <uart2.h>
#include <utils.h>
#include <ir.h>

void PWM_Config(uint8_t deadTime);

#define SYSTICK_FREQ 1000 ///< Frequency of the SysTick.

void softTimerCallback(void);


int main(void) {
	
	UART2_Init(); // Initialize USART2 (for printf)
	TIMER_Init(SYSTICK_FREQ); // Initialize timer

	int8_t timerID = TIMER_AddSoftTimer(1000,softTimerCallback);
	TIMER_StartSoftTimer(timerID);

	LED_TypeDef led;
	led.nr    = LED0;
	led.gpio  = GPIOD;
	led.pin   = 12;
	led.clk   = RCC_AHB1Periph_GPIOD;

	LED_Add(&led); // Add an LED

	printf("Starting program\r\n"); // Print a string to UART2

	PWM_Config(0);
	IR_Init();

	while (1) {
		TIMER_SoftTimersUpdate();
	}
}

extern unsigned int pulses[30];
extern uint8_t i;
void softTimerCallback(void) {

	LED_Toggle(LED0); // Toggle LED

	//printf("Test string sent from STM32F4!!!\r\n"); // Print test string
	uint8_t j;

//	for (j = 0; j < 30; j++) {
//	  printf("%u ", pulses[j]);
//	}
//	printf("\r\n");
//	i=0;

}

/**
* Initialize PWM (based on peripheral example for STM32F4 discovery board)
* @param deadTime
*/
void PWM_Config(uint8_t deadTime) {

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
  uint16_t TimerPeriod = 0;
  uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0;

  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA and GPIOB clocks enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE, ENABLE);

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* GPIOA Configuration: Channel 1 and 3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;//| GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOA Configuration: Channel 2 as alternate function push-pull */
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  //GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* GPIOB Configuration: BKIN, Channel 1N, 2N and 3N as alternate function push-pull */
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  //GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect TIM pins to AF1 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_TIM1);
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_TIM1);
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM1);
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM1);

  //GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, DISABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, DISABLE);

  TIM_DeInit(TIM1);

  /* -----------------------------------------------------------------------
  1/ Generate 3 complementary PWM signals with 3 different duty cycles:

  In this example TIM1 input clock (TIM1CLK) is set to 2 * APB2 clock (PCLK2),
  since APB2 prescaler is different from 1 (APB2 Prescaler = 2, see system_stm32f4xx.c file).
  TIM1CLK = 2 * PCLK2
  PCLK2 = HCLK / 2
  => TIM1CLK = 2*(HCLK / 2) = HCLK = SystemCoreClock

  To get TIM1 counter clock at 168 MHz, the prescaler is computed as follows:
  Prescaler = (TIM1CLK / TIM1 counter clock) - 1
  Prescaler = (SystemCoreClock / 168 MHz) - 1 = 0

  The objective is to generate PWM signal at 17.57 KHz:
  - TIM1_Period = (SystemCoreClock / 17570) - 1

  To get TIM1 output clock at 17.57 KHz, the period (ARR) is computed as follows:
  ARR = (TIM1 counter clock / TIM1 output clock) - 1
  = 9561

  The Three Duty cycles are computed as the following description:

  TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR)* 100 = 50%
  TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR)* 100 = 25%
  TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR)* 100 = 12.5%

  The Timer pulse is calculated as follows:
  - TIM1_CCRx = (DutyCycle * TIM1_ARR)/ 100

  2/ Insert a dead time equal to (11/SystemCoreClock) ns

  3/ Configure the break feature, active at High level, and using the automatic
  output enable feature

  4/ Use the Locking parameters level1.

  Note:
  SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
  Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
  function to update SystemCoreClock variable value. Otherwise, any configuration
  based on this variable will be incorrect.
  ----------------------------------------------------------------------- */

  /* Compute the value to be set in ARR register to generate signal frequency at 17.57 Khz */
  TimerPeriod = (SystemCoreClock / 100000) - 1;

  /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
  Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);

  /* Compute CCR2 value to generate a duty cycle at 25% for channel 2 */
  Channel2Pulse = (uint16_t) (((uint32_t) 25 * (TimerPeriod - 1)) / 100);

  /* Compute CCR3 value to generate a duty cycle at 12.5% for channel 3 */
  Channel3Pulse = (uint16_t) (((uint32_t) 125 * (TimerPeriod - 1)) / 1000);

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 9;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2 and 3 Configuration in PWM mode */
  // TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  // TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  // TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  // TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
  // TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  // TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  // TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  // TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  //TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
  //TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  //
  //TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  //TIM_OC3Init(TIM1, &TIM_OCInitStructure);

  /* Automatic Output enable, Break, dead time and lock configuration*/
  //TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  //TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  //TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
  //TIM_BDTRInitStructure.TIM_DeadTime = deadTime;
  //TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
  //TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  //TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  //
  //TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

}


