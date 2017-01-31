/*****************************************************************************
 * @file
 * @brief LETIMER demo application, PWM and pulse output
 * @author Pavan Dhareshwar
 * @version 1.05
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_gpio.h"

/* Maximum value for compare registers */
/* #define COMPMAX 500 */

/* COMP1 is changed throughout the code to vary the PWM duty cycle
   but starts with the maximum value (100% duty cycle) */
/* uint16_t comp1 = COMPMAX; */

/* Max Energy Mode */
#define MAX_ENERGY_MODE (4)

/* Energy modes */
#define EM0 (0)
#define EM1 (1)
#define EM2 (2)
#define EM3 (3)
#define EM4 (4)

/* Max frequency */
#define MAX_FREQUECNY (32768)

#define ONE_SEC (1)
#define ONE_MS (1/1000)

/* Peripheral Period */
#define PERIPHERAL_PERIOD 1.75*ONE_SEC /* LETimer Peripheral has a period of 1.75s */

/* LED On Time */
#define LED_ON_TIME 30*ONE_MS /* LED on for 30ms */

/* Sleep Block Counter */
uint32_t sleep_block_counter[MAX_ENERGY_MODE];

/**************************************************************************//**
 * @brief LETIMER0_IRQHandler
 * Interrupt Service Routine for LETIMER
 *****************************************************************************/
void LETIMER0_IRQHandler(void)
{
	/* Underflow Interrupt */
	/*if (LETIMER0->IF & LETIMER_IF_UF)
	{
		 Clear LETIMER0 underflow interrupt flag
		LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);

		 If the value of comp1 is over 0, decrement it
		   or bring comp1 back to COMPMAX (500)
		if(comp1 != 0)
			comp1--;
		else
			comp1 = COMPMAX;

		 Write the new compare value to COMP1
		LETIMER_CompareSet(LETIMER0, 1, comp1);
	}*/

	/* COMP0 Interrupt */
	if (LETIMER0->IF & LETIMER_IF_COMP0)
	{
		/* Clear LETIMER0 comp0 interrupt flag */
		LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);

		GPIO_PinOutSet(gpioPortE, 2);
	}

	/* COMP1 Interrupt */
	if (LETIMER0->IF & LETIMER_IF_COMP1)
	{
		/* Clear LETIMER0 comp1 interrupt flag */
		LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);

		GPIO_PinOutClear(gpioPortE, 2);
	}
}

/**************************************************************************//**
 * @brief  LETIMER_setup
 * Configures and starts the LETIMER0
 *****************************************************************************/
void LETIMER_setup(void)
{
	uint32_t sync_busy;
	uint32_t comp0_val = 0;
	uint32_t comp1_val = 0;

	/* Enable necessary clocks */
	CMU_OscillatorEnable(cmuOsc_LFXO, true, false); /* To enable the LFXO */
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO); /* To support energy modes EM0-EM2 */
	CMU_ClockEnable(cmuClock_CORELE, true); /* To enable the low frequency clock tree */
	CMU_ClockEnable(cmuClock_LETIMER0, true); /* To enable the LFA clock tree to LETimer0 */
	CMU_ClockEnable(cmuClock_GPIO, true);

	/* Configure PD6 and PD7 as push pull so the
     LETIMER can override them */
	GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 0); /* LED0 */
	/* GPIO_PinModeSet(gpioPortE, 3, gpioModePushPull, 0); */

	/* Set initial compare values for COMP0 and COMP1
     COMP1 keeps it's value and is used as TOP value
     for the LETIMER.
     COMP1 gets decremented through the program execution
     to generate a different PWM duty cycle */
	/* LETIMER_CompareSet(LETIMER0, 0, COMPMAX);
	LETIMER_CompareSet(LETIMER0, 1, COMPMAX); */

	/* Repetition values must be nonzero so that the outputs
     return switch between idle and active state */
	LETIMER_RepeatSet(LETIMER0, 0, 0x01);
	LETIMER_RepeatSet(LETIMER0, 1, 0x01);

	/* Route LETIMER to location 0 (PD6 and PD7) and enable outputs */
	LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_OUT1PEN | LETIMER_ROUTE_LOCATION_LOC0;

	/* Set configurations for LETIMER 0 */
	const LETIMER_Init_TypeDef letimerInit =
	{
	  .enable         = true,                   /* Start counting when init completed. */
	  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
	  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
	  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
	  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
	  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
	  .out0Pol        = 0,                      /* Idle value for output 0. */
	  .out1Pol        = 0,                      /* Idle value for output 1. */
	  .ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
	  .ufoa1          = letimerUFOAPulse,       /* Pulse output on output 1*/
	  .repMode        = letimerRepeatFree       /* Count until stopped */
	};

	/* Initialize LETIMER */
	LETIMER_Init(LETIMER0, &letimerInit);

	/* Set initial compare values for COMP0 and COMP1
	COMP1 keeps it's value and is used as TOP value
	for the LETIMER.
	COMP1 gets decremented through the program execution
	to generate a different PWM duty cycle */

	comp0_val = MAX_FREQUECNY*PERIPHERAL_PERIOD;
	comp1_val = comp0_val - MAX_FREQUECNY*LED_ON_TIME;

	LETIMER_CompareSet(LETIMER0, 0, comp0_val);
	LETIMER_CompareSet(LETIMER0, 1, comp1_val);

	/* Waiting for setting of a compare register 0 to synchronize into low frequency domain */
	while ((sync_busy = LETIMER_SYNCBUSY_COMP0) == 0);
	/* Waiting for setting of a compare register 1 to synchronize into low frequency domain */
	while ((sync_busy = LETIMER_SYNCBUSY_COMP1) == 0);
}

/**************************************************************************//**
 * @brief  LETIMER_InterruptEnable
 * Enables the necessary interrupts for LETIMER0
 *****************************************************************************/
void LETIMER_InterruptEnable(void)
{
	/* Clearing all the interrupts that may have been set-up inadvertently */
	LETIMER0->IFC |= (LETIMER_IF_COMP0 | LETIMER_IF_COMP1 | LETIMER_IF_UF | LETIMER_IF_REP0 | LETIMER_IF_REP1);

	//-----------------------------------------------------------------------------------
	// THE INTERRUPT IS SIMPLY TO DECREASE THE VALUE OF COMP1 TO VARY THE PWM DUTY CYCLE
	//-----------------------------------------------------------------------------------

	/* Enable comp0 interrupt */
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0);

	/* Enable comp1 interrupt */
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP1);

	/* Enable underflow interrupt */
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
}

void Sleep(void)
{
	if (sleep_block_counter[EM0] > 0 )
	{
		return; /* Block everything below EM0, just return */
	}
	else if (sleep_block_counter[EM1] > 0)
	{
		EMU_EnterEM1(); /* Block everything below EM1, enter EM1 */
	}
	else if (sleep_block_counter[EM2] > 0)
	{
		EMU_EnterEM2(true); /* Block everything below EM2, enter EM2 */
	}
	else if (sleep_block_counter[EM3] > 0)
	{
		EMU_EnterEM3(true); /* Block everything below EM3, enter EM3 */
	}
	else
	{
		EMU_EnterEM3(true); /* Nothing is blocked, enter EM3 */
	}
}

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
	/* Align different chip revisions */
	CHIP_Init();

	/* LETimer0 Setup
  	  1. LETimer0 Clock Tree Setup
  	  2. LETimer0 Setup
  	  3. LETimer0 Interrupts Setup
  	  4. LETimer0 Interrupt Handler Setup
  	  5. LETimer0 Enable
	*/

	/* Initialize LETIMER */
	LETIMER_setup();

	/* Enable the LETIMER Interrupts */
	LETIMER_InterruptEnable();

	/* Enable LETIMER0 interrupt vector in NVIC*/
	NVIC_EnableIRQ(LETIMER0_IRQn);

	while(1)
	{
		/* Go to EM2 */
		/* EMU_EnterEM2(false); */
		Sleep();
	}
}
