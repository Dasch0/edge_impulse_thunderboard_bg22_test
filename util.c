/***************************************************************************//**
 * @file
 * @brief Utility Functions for the Thunderboard Sense
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include "util.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

/**************************************************************************//**
* @addtogroup TBSense_BSP
* @{
******************************************************************************/

/***************************************************************************//**
 * @defgroup Util Utility Functions
 * @{
 * @brief Utility functions
 ******************************************************************************/

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/* Local variables */
volatile uint64_t msTickCount;       /**< Counts 1ms time ticks               */

/** @endcond */

/***************************************************************************//**
 * @brief
 *    Sets up the SysTick timer for 1ms interrupts and initializes and starts
 *    the RTC timer
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t UTIL_init(void)
{
  uint32_t stat;
  uint32_t ticks;

  /* Setup SysTick Timer for 1 msec interrupts  */
  ticks = CMU_ClockFreqGet(cmuClock_CORE) / 1000;
  stat = SysTick_Config(ticks);

  return stat;
}

/***************************************************************************//**
 * @brief
 *    Interrupt Service Routine for system tick counter
 *
 * @return
 *    None
 ******************************************************************************/
void SysTick_Handler(void)
{
  msTickCount++;

  return;
}

/***************************************************************************//**
 * @brief
 *    Delays number of msTick Systicks (1 ms)
 *
 * @param[in] ms
 *    Number of ticks (ms) to delay
 *
 * @return
 *    None
 ******************************************************************************/
void UTIL_delay(uint32_t ms)
{
  uint64_t curTicks;

  curTicks = msTickCount;
  while ( (msTickCount - curTicks) < ms ) {
    EMU_EnterEM1();
  }

  return;
}

/**
 * @brief      Return current tick value
 *
 * @return     tick count value
 */
uint64_t UTIL_getTick(void)
{
  return msTickCount;
}
/** @} {end defgroup Util} */
/** @} {end addtogroup TBSense_BSP} */
