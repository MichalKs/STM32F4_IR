/**
 * @file:   ir.c
 * @brief:  Infrared decoding functions
 * @date:   27 lip 2014
 * @author: Michal Ksiezopolski
 * 
 * @details For now this function only supports RC5 decoding.
 *
 * TODO Add other IR formats.
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
#include <ir_hal.h>

#ifndef DEBUG
  #define DEBUG
#endif

#ifdef DEBUG
  #define print(str, args...) printf(""str"%s",##args,"")
  #define println(str, args...) printf("IR--> "str"%s",##args,"\r\n")
#else
  #define print(str, args...) (void)0
  #define println(str, args...) (void)0
#endif

/**
 * @defgroup  IR IR
 * @brief     Infrared decoding functions
 */

/**
 * @addtogroup IR
 * @{
 */

void IR_ResetFrame(void);
void IR_RxData(uint16_t pulseWidth, uint8_t edge);


#define RC5   0 ///< RC5 coding
#define SIRC  1 ///< SIRC coding

#define IR_CODING RC5 ///< Default coding for library

#define RC5_TIMEOUT       3600 ///< Timeout value in us of RC5 frame
#define RC5_MAX_BIT       1900 ///< Max bit length in us
#define RC5_MIN_HALFBIT   700  ///< Min half bit value in us
#define RC5_MAX_HALFBIT   1000 ///< Max half bit value in us

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
 * @brief Initialize decoding IR signals.
 *
 * @details Sets up callback functions for lower layers
 * and the frame timeout time.
 * After the timeout is over the frame is reset.
 * If an edge is detected on the line then information about
 * it is transported to the higher layer via the read data callback.
 */
void IR_Init(void) {

  IR_HAL_Init(IR_RxData, IR_ResetFrame, RC5_TIMEOUT);
}


/**
 * @brief Decode RC5 data
 *
 * @details This function is called by the lower layer every time
 * a transition on the IR data line occurs (rising or falling edge).
 * The period is the time between two falling edges (the IR data
 * line is pulled up in idle state).
 *
 * @param pulseWidth Width of the received pulse in us.
 * @param edge 0 - low pulse (rising edge), 1 - high pulse (falling edge)
 */
void IR_RxData(uint16_t pulseWidth, uint8_t edge) {

  // frame starts with falling edge
  // since the data line is normally high (pullup)
  // frame visualization: -----|_
  if (pulseCount == 0 && edge == 1) {
    pulseCount++;
    bitCount = 13;
    frame = 0; // new frame
    return;
  } else if (pulseCount == 0 && edge == 0) {
    // frame should start with falling edge
    println("Frame error - wrong edge");
    IR_ResetFrame();
    return;
  }

  // Pulse width was irrelevant at first edge.
  // It is relevant for all following edges
  if ((pulseWidth > RC5_MAX_BIT) || (pulseWidth < RC5_MIN_HALFBIT)) {
    println("Frame error - wrong pulse width");
    IR_ResetFrame();
    return;
  }

  // frame visualization: -----|_|-
  if (pulseCount == 1) {
    // pulseWidth has to be 800 us - first two bits are a one
    if (pulseWidth > RC5_MAX_HALFBIT) {
      println("Frame error - wrong start bits, probably not RC5");
      IR_ResetFrame();
      return;
    }

    frame |= (1<<bitCount--); // First bit (put as MSB) is a one
    pulseCount++;

  // frame visualization: -----|_|-|_
  } else if (pulseCount == 2) {
    // pulseWidth has to be 800 us - first two bits are a one
    if (pulseWidth > RC5_MAX_HALFBIT) {
      println("Frame error - wrong start bits, probably not RC5");
      IR_ResetFrame();
      return;
    }
    frame |= (1<<bitCount--); // Second bit is a one
    pulseCount++;
  } else if ((bitCount >= 0) && (bitCount < 12)) { // for bits 11 to 0
    // if pulseWidth is about 1700us then two half bits were transmitted
    // so increment pulseCount one more time
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

      IR_ResetFrame(); // reset frame

      rxCount++; // add received frame
      toggle = (frame>>11) & (0x0001); // toggle bit is 3rd bit
      address = (frame>>6) & (0x001f); // address is 5 bits
      command = (frame>>0) & (0x003f); // command is 6 bits
      println("Frame received: %04x. Toggle = %d Command = %d Address = %d",
          frame, toggle, command, address);
      return;
    }

    pulseCount++; // increment pulse count value
  }

}
/**
 * @brief Resets frame after timeout.
 */
void IR_ResetFrame(void) {
  bitCount = 13;
  pulseCount = 0;
}


/**
 * @}
 */
