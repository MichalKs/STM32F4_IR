/**
 * @file: 	ir.c
 * @brief:	   
 * @date: 	27 lip 2014
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


__IO uint16_t IC1Value = 0;
__IO uint16_t IC2Value = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;

unsigned int pulses[30];
uint8_t i;

void IR_Init(void) {

  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* TIM4 chennel2 configuration : PB.07 */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect TIM pin to AF2 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

  /* Time Base configuration */
  // Timer clock is 84MHz
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 1 tick = 1 us
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 30000;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM4 configuration: PWM Input mode ------------------------
  The external signal is connected to TIM4 CH2 pin (PB.07),
  The Rising edge is used as active edge,
  The TIM4 CCR2 is used to compute the frequency value
  The TIM4 CCR1 is used to compute the duty cycle value
  ------------------------------------------------------------ */
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);

  /* Select the TIM4 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);

  //  TIM4->CCR1 = 0;
  //  TIM4->CCR2 = 0;
  /* TIM enable counter */
  TIM_Cmd(TIM4, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE); // falling edge - high pulse value
  TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE); // rising edge - period value
  // Reset to 0 is on every rising edge
}

void TIM4_IRQHandler(void)
{


  /* IC1 Interrupt*/
  if((TIM_GetFlagStatus(TIM4, TIM_FLAG_CC1) != RESET))
  {
    TIM_ClearFlag(TIM4, TIM_FLAG_CC1);
    /* Get the Input Capture value */
    IC1Value = TIM_GetCapture1(TIM4);
    pulses[i++]= IC1Value;

    /* RC5 */
//    IR_RC5_DataSampling(ICValue2 - ICValue1, 0);
  }  /* IC2 Interrupt */
  else  if((TIM_GetFlagStatus(TIM4, TIM_FLAG_CC2) != RESET))
  {
    TIM_ClearFlag(TIM4, TIM_FLAG_CC2);
    /* Get the Input Capture value */
    IC2Value = TIM_GetCapture2(TIM4);
    pulses[i++]= IC2Value;
//    IR_RC5_DataSampling(ICValue1 , 1);
  }
  /* Checks whether the IR_TIM flag is set or not.*/
  else if ((TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) != RESET))
  {
    /* Clears the IR_TIM's pending flags*/
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);

//    IR_RC5_ResetPacket();
  }
  if (i == 30) {
    i = 0;
  }

//  GPIO_ToggleBits(GPIOB, GPIO_Pin_6);
}

//void TIM4_IRQHandler(void)
//{
//
//
//  RCC_ClocksTypeDef RCC_Clocks;
//  RCC_GetClocksFreq(&RCC_Clocks);
//
//  /* Clear TIM4 Capture compare interrupt pending bit */
//  TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
//
//  /* Get the Input Capture value */
//  IC2Value = TIM_GetCapture2(TIM4); // period
//  IC1Value = TIM_GetCapture1(TIM4); // low pulse
//
//  pulses[i++]= IC1Value;
//  if (i == 14) {
//    i = 0;
//  }

//  if (IC2Value != 0)
//  {
//    /* Duty cycle computation */
//    DutyCycle = ( IC1Value* 100) / IC2Value;
//
//    /* Frequency computation
//       TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */
//
//    Frequency = (RCC_Clocks.HCLK_Frequency)/2 / IC2Value;
//  }
//  else
//  {
//    DutyCycle = 0;
//    Frequency = 0;
//  }
//}



