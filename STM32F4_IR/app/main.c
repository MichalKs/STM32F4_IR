/**
 * @file: 	main.c
 * @brief:	IR test
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

#define SYSTICK_FREQ 1000 ///< Frequency of the SysTick.

void softTimerCallback(void);

int main(void) {
	
	UART2_Init(); // Initialize USART2 (for printf)
	TIMER_Init(SYSTICK_FREQ); // Initialize timer

	// Add a soft timer
	int8_t timerID = TIMER_AddSoftTimer(1000,softTimerCallback);
	TIMER_StartSoftTimer(timerID);

	LED_TypeDef led;
	led.nr    = LED0;
	led.gpio  = GPIOD;
	led.pin   = 12;
	led.clk   = RCC_AHB1Periph_GPIOD;

	LED_Add(&led); // Add an LED

	printf("Starting program\r\n"); // Print a string to UART2

	IR_Init(); // Initialize IR signals decoding

	while (1) {
		TIMER_SoftTimersUpdate();
	}
}
/**
 * @brief Callback function called on every soft timer overflow
 */
void softTimerCallback(void) {

	LED_Toggle(LED0); // Toggle LED

	//printf("Test string sent from STM32F4!!!\r\n"); // Print test string
}
