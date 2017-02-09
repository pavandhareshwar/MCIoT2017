/*****************************************************************************
 * @file MCIoT_assg3.c
 * @brief This file describes the methodology to self calibrate ULFRCO for
 * 		  Leopard Gecko and describes the usage of ACMP, ambient light sensor
 * 		  and controlling light using ambient light sensor.
 * @author Pavan Dhareshwar
 * @version 1.0
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
 ************************************************************************************/

/************************************ INCLUDES **************************************/
#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "em_acmp.h"
#include "MCIoT_assg3.h"

/************************************ INCLUDES **************************************/

/****************************** FUNCTION DEFINITIONS ********************************/

/************************************************************************************
 * @function 	LED_On
 * @params 		[in] port - The GPIO port to access
 * 				[in] pin  - The pin to set
 * @brief 		Routine to turn on the LED.
 ************************************************************************************/
void LED_On(GPIO_Port_TypeDef port, unsigned int pin)
{
	/* Turning on LED */
	GPIO_PinOutSet(port, pin);
}

/************************************************************************************
 * @function 	LED_Off
 * @params 		[in] port - The GPIO port to access
 * 				[in] pin  - The pin to clear
 * @brief 		Routine to turn on the LED.
 ************************************************************************************/
void LED_Off(GPIO_Port_TypeDef port, unsigned int pin)
{
	/* Turning off LED */
	GPIO_PinOutClear(port, pin);
}

#if 0
/************************************************************************************
 * @function 	LETIMER0_IRQHandler
 * @params 		None
 * @brief 		Interrupt Service Routine for LETIMER.
 ************************************************************************************/
void LETIMER0_IRQHandler(void)
{
	/* COMP0 Interrupt */
	if ((LETIMER0->IF & LETIMER_IF_COMP0) == LETIMER_IF_COMP0)
	{
		/* Clearing the source of interrupt: LETIMER0 comp0 interrupt flag */
		LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);

		/* Turning on LED0 */
		LED_On(LED0_GPIO_PORT, LED0_GPIO_PIN);
	}

	/* COMP1 Interrupt */
	if ((LETIMER0->IF & LETIMER_IF_COMP1) == LETIMER_IF_COMP1)
	{
		/* Clearing the source of interrupt: LETIMER0 comp1 interrupt flag */
		LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);

		/* Turning off LED0 */
		LED_Off(LED0_GPIO_PORT, LED0_GPIO_PIN);
	}
}
#else
/************************************************************************************
 * @function 	LETIMER0_IRQHandler
 * @params 		None
 * @brief 		Interrupt Service Routine for LETIMER.
 ************************************************************************************/
void LETIMER0_IRQHandler(void)
{
	uint32_t acmp_out_val = 0;

	INT_Disable();

	/* COMP0 Interrupt */
	if ((LETIMER0->IF & LETIMER_IF_COMP0) == LETIMER_IF_COMP0)
	{
		/* Clearing the source of interrupt: LETIMER0 comp0 interrupt flag */
		LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);

		/* Turning on LED0 */
		//LED_On(LED0_1_GPIO_PORT, LED0_GPIO_PIN);

		CMU_ClockEnable(cmuClock_ACMP0, true); /* To enable clock to ACMP0 */

		/* Turn on ACMP */
		ACMP0->CTRL |= ACMP_CTRL_EN;

		GPIO_PinModeSet(ALS_GPIO_PORT, ALS_SENSE_GPIO_PIN, gpioModePushPull, 1);

		/* Waiting for the ACMPACT bit to be set in ACMP0_STATUS register
		 * indicating that the ACMP is warmed up */
		while ((ACMP0->STATUS & ACMP_STATUS_ACMPACT) != ACMP_STATUS_ACMPACT);
	}

	/* COMP1 Interrupt */
	if ((LETIMER0->IF & LETIMER_IF_COMP1) == LETIMER_IF_COMP1)
	{
		/* Clearing the source of interrupt: LETIMER0 comp1 interrupt flag */
		LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);

		/* Turning off LED0 */
		//LED_Off(LED0_1_GPIO_PORT, LED0_GPIO_PIN);

		/* Check the ACMP out value and decide the action and switch off ACMP */
		acmp_out_val = (ACMP0->STATUS & ACMP_STATUS_ACMPOUT) >> 1;

		if (!acmp_out_val)
		{
			/* Turning on LED1 */
			LED_On(LED0_1_GPIO_PORT, LED1_GPIO_PIN);

			/* Changing the ACMP reference to check the state of ALS GPIO
			 * for brightness on next iteration */
			ACMP0->INPUTSEL &= ~(DARK_REFERENCE_VDDLEVEL << _ACMP_INPUTSEL_VDDLEVEL_SHIFT);
			ACMP0->INPUTSEL |= (LIGHT_REFERENCE_VDDLEVEL << _ACMP_INPUTSEL_VDDLEVEL_SHIFT);
		}
		else
		{
			/* Turning off LED1 */
			LED_Off(LED0_1_GPIO_PORT, LED1_GPIO_PIN);

			/* Changing the ACMP reference to check the state of ALS GPIO
			 * for darkness on next iteration */
			ACMP0->INPUTSEL &= ~(LIGHT_REFERENCE_VDDLEVEL << _ACMP_INPUTSEL_VDDLEVEL_SHIFT);
			ACMP0->INPUTSEL |= (DARK_REFERENCE_VDDLEVEL << _ACMP_INPUTSEL_VDDLEVEL_SHIFT);
		}

		/* Turn off the ACMP */
		ACMP0->CTRL &= ~ACMP_CTRL_EN;

		CMU_ClockEnable(cmuClock_ACMP0, false); /* To disable clock to ACMP0 */

		GPIO_PinModeSet(ALS_GPIO_PORT, ALS_SENSE_GPIO_PIN, gpioModeDisabled, 0);
	}

	INT_Enable();
}
#endif

/************************************************************************************
 * @function 	blockSleepMode
 * @params 		[in] e_letimer_energy_modes (LETimer energy mode enum)
 * @brief 		Blocks the MCU from sleeping below a certain mode.
 ************************************************************************************/
void blockSleepMode(ENERGY_MODES e_letimer_energy_modes)
{
	INT_Disable();
	sleep_block_counter[e_letimer_energy_modes]++;
	INT_Enable();
}

/************************************************************************************
 * @function 	unblockSleepMode
 * @params 		[in] e_letimer_energy_modes - LETimer energy mode enum
 * @brief 		Unblocks the MCU from sleeping below a certain mode.
 ************************************************************************************/
void unblockSleepMode(ENERGY_MODES e_letimer_energy_modes)
{
	INT_Disable();
	if (sleep_block_counter[e_letimer_energy_modes] > 0)
	{
		sleep_block_counter[e_letimer_energy_modes]--;
	}
	INT_Enable();
}

/************************************************************************************
 * @function 	CMU_SetUp
 * @params 		None
 * @brief 		Configures and starts necessary clocks.
 ************************************************************************************/
void CMU_SetUp(void)
{
	CMU_ClockEnable(cmuClock_CORELE, true); /* To enable the low frequency clock tree */

	if (e_letimer_energy_modes == ENERGY_MODE_EM3)
	{
		/* Enable necessary clocks */
		CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true); /* To enable the LFXO */
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO); /* To support energy modes EM0-EM2 */
		CMU_ClockEnable(cmuClock_LETIMER0, true); /* To enable the LFA clock tree to LETimer0 */

#ifdef ULFRCO_SELF_CALIBRATE

		/* Perform self calibration of ULFRCO */
		CMU_ClockEnable(cmuClock_TIMER0, true); /* To enable HFPER clock tree for Timer0 peripheral */
		CMU_ClockEnable(cmuClock_TIMER1, true); /* To enable HFPER clock tree for Timer1 peripheral */

		/* Routine to self calibrate ULFRCO */
		Calibrate_ULFRCO();

#endif

	}
	else
	{
		/* Enable necessary clocks */
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true); /* To enable the LFXO */
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO); /* To support energy modes EM0-EM2 */
		CMU_ClockEnable(cmuClock_LETIMER0, true); /* To enable the LFA clock tree to LETimer0 */
	}

	CMU_ClockEnable(cmuClock_ACMP0, true); /* To enable clock to ACMP0 */
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_HFPER, true); /* To enable the clock tree for Timer peripherals */
}

/************************************************************************************
 * @function 	GPIO_SetUp
 * @params 		None
 * @brief 		Configures LED GPIO's.
 ************************************************************************************/
void GPIO_SetUp(void)
{
	/* Set GPIO PE2 and PE3 as pushpull */
	GPIO_PinModeSet(LED0_1_GPIO_PORT, LED0_GPIO_PIN, gpioModePushPullDrive, 0); /* Configure LED0 pin as digital output (push-pull) */
	GPIO_PinModeSet(LED0_1_GPIO_PORT, LED1_GPIO_PIN, gpioModePushPullDrive, 0); /* Configure LED1 pin as digital output (push-pull) */

	GPIO_DriveModeSet(LED0_1_GPIO_PORT, gpioDriveModeLowest); /* Set DRIVEMODE to lowest setting (0.5 mA) for all LEDs configured with alternate drive strength */
}

/************************************************************************************
 * @function 	LETIMER_Calibration_Setup
 * @params 		None
 * @brief 		Setup LETIMER for self calibration of ULFRCO
 ************************************************************************************/
void LETIMER_Calibration_Setup(void)
{
	uint32_t letimer_sync_busy = 0;

	/* Set configurations for LETIMER 0 */
	const LETIMER_Init_TypeDef letimerInit =
	{
	  .enable         = false,                  /* Don't start counting when init completed. */
	  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
	  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
	  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
	  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
	  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
	  .out0Pol        = 0,                      /* Idle value for output 0. */
	  .out1Pol        = 0,                      /* Idle value for output 1. */
	  .ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
	  .ufoa1          = letimerUFOAPulse,       /* Pulse output on output 1*/
	  .repMode        = letimerRepeatOneshot    /* Count once */
	};

	/* Initialize LETIMER */
	LETIMER_Init(LETIMER0, &letimerInit);

	/* Wait for setting of a cmd register to synchronize into low frequency domain */
	while ((letimer_sync_busy = LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_CMD) == 1);

	letimer_sync_busy = 1;

	/* Wait for setting of a ctrl register to synchronize into low frequency domain */
	while ((letimer_sync_busy = LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_CTRL) == 1);
}

/************************************************************************************
 * @function 	LETIMER_Setup
 * @params 		None
 * @brief 		Configures and starts the LETIMER0.
 ************************************************************************************/
void LETIMER_Setup(void)
{
	uint32_t letimer_sync_busy;

	//blockSleepMode(e_letimer_energy_modes);

	/* Set initial compare values for COMP0 and COMP1
     COMP1 keeps it's value and is used as TOP value
     for the LETIMER.
     COMP1 gets decremented through the program execution
     to generate a different PWM duty cycle */
	/* LETIMER_CompareSet(LETIMER0, 0, COMPMAX);
	LETIMER_CompareSet(LETIMER0, 1, COMPMAX); */

	/* Repetition values must be nonzero so that the outputs
     return switch between idle and active state */
	/* LETIMER_RepeatSet(LETIMER0, 0, 0x01);
	LETIMER_RepeatSet(LETIMER0, 1, 0x01); */

	/* Route LETIMER to location 0 (PD6 and PD7) and enable outputs */
	/* LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_OUT1PEN | LETIMER_ROUTE_LOCATION_LOC0; */

	/* Set configurations for LETIMER 0 */
	const LETIMER_Init_TypeDef letimerInit =
	{
	  .enable         = false,                  /* Don't start counting when init completed. */
	  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
	  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
	  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
	  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
	  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
	  .out0Pol        = 0,                      /* Idle value for output 0. */
	  .out1Pol        = 0,                      /* Idle value for output 1. */
	  .ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
	  .ufoa1          = letimerUFOAPulse,       /* Pulse output on output 1*/
	  .repMode        = letimerRepeatFree    /* Count until stopped */
	};

	/* Initialize LETIMER */
	LETIMER_Init(LETIMER0, &letimerInit);

	/* Wait for setting of a cmd register to synchronize into low frequency domain */
	while ((letimer_sync_busy = LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_CMD) == 1);

	letimer_sync_busy = 1;

	/* Wait for setting of a ctrl register to synchronize into low frequency domain */
	while ((letimer_sync_busy = LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_CTRL) == 1);
}

/************************************************************************************
 * @function 	LETIMER_Interrupt_Enable
 * @params 		None
 * @brief 		Enables the necessary interrupts for LETIMER0.
 ************************************************************************************/
void LETIMER_Interrupt_Enable(void)
{
	/* Clear all the interrupts that may have been set-up inadvertently */
	LETIMER0->IFC |= (LETIMER_IF_COMP0 | LETIMER_IF_COMP1 | LETIMER_IF_UF | LETIMER_IF_REP0 | LETIMER_IF_REP1);

	//-----------------------------------------------------------------------------------
	// THE INTERRUPT IS SIMPLY TO DECREASE THE VALUE OF COMP1 TO VARY THE PWM DUTY CYCLE
	//-----------------------------------------------------------------------------------

	/* Enable comp0 interrupt */
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0);

	/* Enable comp1 interrupt */
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP1);
}

/************************************************************************************
 * @function 	Sleep
 * @params 		None
 * @brief 		Sleep routine for LETIMER0.
 *				Credits to Silicon Labs for the following sleep routine.
 * @section 	License (C) Copyright 2015 Silicon Labs, http://www.silabs.com/
 ************************************************************************************/
void Sleep(void)
{
	if (sleep_block_counter[EM0] > 0)
	{
		return; 				/* Block everything below EM0, just return */
	}
	else if (sleep_block_counter[EM1] > 0)
	{
		EMU_EnterEM1(); 	/* Block everything below EM2, enter EM2 */
	}
	else if (sleep_block_counter[EM2] > 0)
	{
		EMU_EnterEM2(true); 	/* Block everything below EM2, enter EM2 */
	}
	else if (sleep_block_counter[EM3] > 0)
	{
		EMU_EnterEM3(true); 	/* Block everything below EM3, enter EM3 */
	}
	else
	{
		EMU_EnterEM3(true);  	/* Nothing is blocked, enter EM3 */
	}
}

/************************************************************************************
 * @function 	TIMER_Setup
 * @params 		None
 * @brief 		Configures and starts the TIMER peripherals.
 ************************************************************************************/
void TIMER_Setup(void)
{
	/* Set configurations for TIMER0 and TIMER1 */
	const TIMER_Init_TypeDef timer0Init =
	{
	  .enable         = false,                  /* Don't start counting when init completed. */
	  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
	  .prescale       = 0,						/* Use a prescale of 0 */
	  .clkSel		  = timerClkSelHFPerClk, 	/* Using HFPER clock */
	  .fallAction 	  = timerInputActionNone,	/* Don't start/stop/reload counter on falling edge */
	  .riseAction 	  = timerInputActionNone,	/* Don't start/stop/reload counter on rising edge */
	  .mode			  = timerModeUp,			/* Count upwards */
	  .dmaClrAct      = false,					/* Don't clear DMA request on active */
	  .quadModeX4	  = false,					/* Don't use quadrature decode mode */
	  .oneShot 		  = false,					/* Not counting up just once */
	  .sync			  = false					/* Timer0 start not in sync with anything */
	};

	const TIMER_Init_TypeDef timer1Init =
	{
	  .enable         = false,                  /* Don't start counting when init completed. */
	  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
	  .prescale       = 0,						/* Use a prescale of 0 */
	  .clkSel		  = timerClkSelCascade, 	/* Cascaded, clocked by overflow of timer0 */
	  .fallAction 	  = timerInputActionNone,	/* Don't start/stop/reload counter on falling edge */
	  .riseAction 	  = timerInputActionNone,	/* Don't start/stop/reload counter on rising edge */
	  .mode			  = timerModeUp,			/* Count upwards */
	  .dmaClrAct      = false,					/* Don't clear DMA request on active */
	  .quadModeX4	  = false,					/* Don't use quadrature decode mode */
	  .oneShot 		  = false,					/* Not counting up just once */
	  .sync			  = true					/* Timer1 start in sync with timer0 */
	};

	/* Initialize TIMER0 */
	TIMER_Init(TIMER0, &timer0Init);

	/* Initialize TIMER1 */
	TIMER_Init(TIMER1, &timer1Init);
}

/************************************************************************************
 * @function 	Calibrate_ULFRCO
 * @params 		None
 * @brief 		Routine to self calibrate ULFRCO.
 ************************************************************************************/
void Calibrate_ULFRCO(void)
{
#if 0
	uint32_t comp0_val = 0;
	uint32_t i = 0;
#endif
	uint32_t sync_busy = 0;
	uint32_t lfxo_count = 0;
	uint32_t ulfrco_count = 0;

	/* Initialize LETIMER for ULFRCO calibration */
	LETIMER_Calibration_Setup();

	/* Initialize Timer peripherals */
	TIMER_Setup();

	/* Obtain ULFRCO count */
#if 1
	LETIMER0->CNT = ULFRCO_FREQUENCY*CALIBRATION_PERIOD;
#else
	comp0_val = ULFRCO_FREQUENCY*CALIBRATION_PERIOD;

	LETIMER_CompareSet(LETIMER0, 0, comp0_val);

	/* Waiting for setting of a compare register 0 to synchronize into low frequency domain */
	while ((sync_busy = LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_COMP0) == 1);
#endif

	/* Start LETIMER, TIMER0 and TIMER1 */
	LETIMER0->CMD |= LETIMER_CMD_START;
	TIMER1->CMD |= TIMER_CMD_START;
	TIMER0->CMD |= TIMER_CMD_START;

	while(LETIMER0->CNT != 0);

	LETIMER0->CMD |= LETIMER_CMD_STOP;

	TIMER1->CMD |= TIMER_CMD_STOP;
	TIMER0->CMD |= TIMER_CMD_STOP;

	ulfrco_count = (TIMER1->CNT << 16) | TIMER0->CNT;

	TIMER1->CNT = 0;
	TIMER0->CNT = 0;

	CMU_OscillatorEnable(cmuOsc_ULFRCO, false, false); /* To disable the ULFRCO */

	CMU_OscillatorEnable(cmuOsc_LFXO, true, true); /* To enable the LFXO */
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO); /* To support energy modes EM0-EM2 */
	CMU_ClockEnable(cmuClock_LETIMER0, true); /* To enable the LFA clock tree to LETimer0 */

	/* Obtain LFXO count */
#if 1
	LETIMER0->CNT = LFXO_FREQUECNY*CALIBRATION_PERIOD;
#else
	comp0_val = LFXO_FREQUECNY*CALIBRATION_PERIOD;

	LETIMER_CompareSet(LETIMER0, 0, comp0_val);

	/* Waiting for setting of a compare register 0 to synchronize into low frequency domain */
	while ((sync_busy = LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_COMP0) == 1);
#endif

	LETIMER0->CMD |= LETIMER_CMD_START;
	while ((sync_busy = LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_CMD) == 1);

	TIMER0->CMD |= TIMER_CMD_START;
	TIMER1->CMD |= TIMER_CMD_START;

	while(LETIMER0->CNT != 0);

	LETIMER0->CMD |= LETIMER_CMD_STOP;

	TIMER0->CMD |= TIMER_CMD_STOP;
	TIMER1->CMD |= TIMER_CMD_STOP;

	lfxo_count = (TIMER1->CNT << 16) | TIMER0->CNT;

	osc_ratio = (float)lfxo_count/(float)ulfrco_count;

	CMU_OscillatorEnable(cmuOsc_LFXO, false, false); /* To disable the LFXO */

	CMU_ClockEnable(cmuClock_TIMER0, false); /* To disable HFPER clock tree for Timer0 peripheral */
	CMU_ClockEnable(cmuClock_TIMER1, false); /* To disable HFPER clock tree for Timer1 peripheral */

}

/************************************************************************************
 * @function 	ACMP_setup
 * @params 		None
 * @brief 		Configures the ACMP.
 ************************************************************************************/
void ACMP_SetUp(void)
{

	/* Set configurations for ACMP 0 */
	const ACMP_Init_TypeDef acmpInit =
	{
	  .enable         			= false,             		/* Enable ACMP */
	  .fullBias       			= false,                  	/* Full Bias Current */
	  .halfBias 	  			= true,                   	/* Half Bias Current */
	  .biasProg 	 			= 0,                  		/* BiasProg Current Configuration */
	  .interruptOnFallingEdge 	= false,                   	/* Enable interrupt on falling edge */
	  .interruptOnRisingEdge  	= false,                  	/* Enable interrupt on rising edge */
	  .warmTime        			= acmpWarmTime256,        	/* Warm-Up Time for ACMP, should be atleast 10us */
	  .hysteresisLevel        	= _ACMP_CTRL_HYSTSEL_HYST1, /* Hysteresis Configuration */
	  .inactiveValue          	= false,         			/* Inactive comparator active value */
	  .lowPowerReferenceEnabled	= false,       				/* Enable low power mode */
	  .vddLevel        			= DARK_REFERENCE_VDDLEVEL  	/* Vdd reference scaling */
	};

	/* Initialize LETIMER */
	ACMP_Init(ACMP0, &acmpInit);

	/* Configure the ACMP inputs - positive and negative */
	ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel6);

	/* Set GPIO PD6 as pushpull */
	GPIO_PinModeSet(ALS_GPIO_PORT, ALS_SENSE_GPIO_PIN, gpioModePushPull, 0);
}

/************************************************************************************
 * @function 	ACMP_Interrupt_Enable
 * @params 		None
 * @brief 		Enables the necessary interrupts for ACMP0.
 ************************************************************************************/
void ACMP_Interrupt_Enable(void)
{
	/* Clearing all the interrupts that may have been set-up inadvertently */
	ACMP0->IFC |= (ACMP_IF_EDGE | ACMP_IF_WARMUP);

	/* Enable edge interrupt */
	ACMP_IntEnable(ACMP0, ACMP_IF_EDGE);
}

/************************************************************************************
 * @function 	LETimer_Config
 * @params 		None
 * @brief 		Configures the LETIMER0.
 ************************************************************************************/
void LETimer_Config(void)
{
	uint32_t letimer_sync_busy;
	uint32_t comp0_val = 0;
	uint32_t comp1_val = 0;

	/* Enable necessary clocks */
	if (e_letimer_energy_modes == ENERGY_MODE_EM3)
	{
		CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true); /* To enable the ULFRCO */
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO); /* To support energy modes EM3 */
	}
	else
	{
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true); /* To enable the LFXO */
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO); /* To support energy modes EM0-EM2 */
	}

	LETIMER_Setup();

	/*  Set initial compare values for COMP0 and COMP1
	    COMP1 keeps it's value and is used as TOP value for the LETIMER.
		COMP1 gets decremented through the program execution
		to generate a different PWM duty cycle */

	if (e_letimer_energy_modes == ENERGY_MODE_EM3)
	{
		comp0_val = ULFRCO_FREQUENCY*osc_ratio*ALS_EXCITE_PERIOD;
		comp1_val = comp0_val - ULFRCO_FREQUENCY*osc_ratio*ALS_MIN_EXCITE_PERIOD;
	}
	else
	{
		comp0_val = (LFXO_FREQUECNY/prescaled_two_power)*ALS_EXCITE_PERIOD;
		comp1_val = comp0_val - (LFXO_FREQUECNY/prescaled_two_power)*ALS_MIN_EXCITE_PERIOD;
	}

	/* Setting the LETimer compare registers comp0 and comp1 */
	LETIMER_CompareSet(LETIMER0, 0, comp0_val);
	LETIMER_CompareSet(LETIMER0, 1, comp1_val);

	/* Waiting for setting of a compare register 0 to synchronize into low frequency domain */
	while ((letimer_sync_busy = LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_COMP0) == 1);

	letimer_sync_busy = 1;

	/* Waiting for setting of a compare register 1 to synchronize into low frequency domain */
	while ((letimer_sync_busy = LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_COMP1) == 1);
}

/************************************************************************************
 * @function 	cal_prescaler_value
 * @params 		[in] period - duration that has to be achieved (greater than 2 secs)
 * 				[in] osc_cnt - Oscillator count value
 * @brief 		Calculates the prescaler value that will divide the clock to achieve
 *				time duration greater than 2 seconds.
 * @credits		Slides from Professor Keith Graham
 ************************************************************************************/
void cal_prescaler_value(float period, uint32_t osc_cnt)
{
	uint32_t desired_period = 0;
	uint32_t temp = 0;

	desired_period = period * osc_cnt;
	temp = desired_period/1;
	prescaled_two_power = 1;

	while(temp > LETIMER0_MAX_COUNT)
	{
		letimer0_prescaler++;
		prescaled_two_power = prescaled_two_power * 2;
		temp = desired_period/prescaled_two_power;
	}
}

/************************************************************************************
 * @function 	main
 * @params 		None
 * @brief 		Main Function
 *				Main is called from __iar_program_start, see assembly startup file
 ************************************************************************************/
int main(void)
{
	uint32_t letimer_sync_busy;
	uint32_t cmu_sync_busy;

	/* Align different chip revisions */
	CHIP_Init();

	/* LETimer0 Setup
  	  1. LETimer0 Clock Tree Setup
  	  2. LETimer0 Setup
  	  3. LETimer0 Interrupts Setup
  	  4. LETimer0 Interrupt Handler Setup
  	  5. LETimer0 Enable
	*/

	if (ALS_EXCITE_PERIOD > MAX_PERIOD_32kHZ)
	{
		cal_prescaler_value(ALS_EXCITE_PERIOD, LFXO_FREQUECNY);
	}
	else
	{
		letimer0_prescaler = 0;
		prescaled_two_power = 1;
	}

	CMU->LFAPRESC0 |= letimer0_prescaler << _CMU_LFAPRESC0_LETIMER0_SHIFT;

	while ((cmu_sync_busy = CMU->SYNCBUSY & CMU_SYNCBUSY_LFAPRESC0) == 1);

	CMU_SetUp();

	blockSleepMode(e_letimer_energy_modes);

	LETimer_Config();

	GPIO_SetUp();

	/* Enable the LETIMER Interrupts */
	LETIMER_Interrupt_Enable();

	/* Start LETIMER */
	LETIMER0->CMD |= LETIMER_CMD_START;
	while ((letimer_sync_busy = LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_CMD) == 1);

	/* Enable LETIMER0 interrupt vector in NVIC*/
	NVIC_EnableIRQ(LETIMER0_IRQn);

	LETIMER0->CMD = LETIMER_CMD_START;

	/* Configure ACMP */
	ACMP_SetUp();

	while(1)
	{
		Sleep();
	}
}

/* This code is originally copy righted by Silicon Labs and it grants permission
 * to anyone to use this software for any purpose, including commercial applications,
 * and to alter it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Unused part of the code is commented to avoid confusion.
 *
 * Name/argument list of routines may have been changed to confirm to the
 * naming convention of the developer.
 *
 * Routines include:
 *
 * void LETIMER0_IRQHandler(void);
 * void blockSleepMode(ENERGY_MODES e_letimer_energy_modes);
 * void LETIMER_Setup(void);
 * void LETIMER_Interrupt_Enable(void);
 * void Sleep(void);
 */
