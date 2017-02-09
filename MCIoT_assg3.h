
#ifndef _MCIOT_ASSG3_H_
#define _MCIOT_ASSG3_H_
#endif
/************************************* MACROS ***************************************/

/* Maximum value for compare registers */
/* #define COMPMAX 500 */

/* Max Energy Mode */
#define MAX_ENERGY_MODE 4

/* Energy Modes */
#define EM0 0
#define EM1 1
#define EM2 2
#define EM3 3
#define EM4 4

/* Max frequencies */
#define LFXO_FREQUECNY 				32768		/* 32.768 kHz */
#define ULFRCO_FREQUENCY 			1000		/* 1 kHz */
#define ULFRCO_SELF_CALIBRATE 		1			/* Self Calibrate ULFRCO */

/* One Second and One millisecond definitions */
#define ONE_SEC 					1
#define ONE_MS 						1/1000

/* Calibration Period */
#define CALIBRATION_PERIOD 			1*ONE_SEC 	/* ULFRCO calibration period*/
#define ALS_EXCITE_PERIOD  			2.5*ONE_SEC /* Ambient Light Sensor Excite Time */
#define MAX_PERIOD_32kHZ   			2*ONE_SEC 	/* Max time duration that can be generated using 32kHz clock */

/* LED On Time */
#define ALS_MIN_EXCITE_PERIOD 		4*ONE_MS /* Keep the ambient light sensor excited for a minimum of 4 ms */

/* LED GPIO port name and pin */
#define LED0_1_GPIO_PORT 			gpioPortE
#define LED0_GPIO_PIN				2
#define LED1_GPIO_PIN				3

#define ALS_GPIO_PORT 				gpioPortD
#define ALS_SENSE_GPIO_PIN 			6

#define DARK_REFERENCE_VDDLEVEL 	2
#define LIGHT_REFERENCE_VDDLEVEL 	61
#define LETIMER0_MAX_COUNT			65536

/************************************* MACROS ***************************************/

/********************************** ENUMERATIONS ************************************/

/* Enumeration for LETimer Energy Modes */
typedef enum _ENERGY_MODES
{
	ENERGY_MODE_INVALID = -1,
	ENERGY_MODE_EM0,
	ENERGY_MODE_EM1,
	ENERGY_MODE_EM2,
	ENERGY_MODE_EM3,
	ENERGY_MODE_EM4,
	LETIMER_ENERGY_MODE_MAX
} ENERGY_MODES;

/********************************** ENUMERATIONS ************************************/

/************************************ GLOBALS ***************************************/

/* COMP1 is changed throughout the code to vary the PWM duty cycle
   but starts with the maximum value (100% duty cycle) */
/* uint16_t comp1 = COMPMAX; */

/* Sleep Block Counter */
uint32_t sleep_block_counter[MAX_ENERGY_MODE+1];

/* Initializing LETimer in energy mode EM2 */
ENERGY_MODES e_letimer_energy_modes = ENERGY_MODE_EM3;

#ifdef ULFRCO_SELF_CALIBRATE
	/* Oscillator Ratio (used for self-calibration) */
	float osc_ratio = 0;
#else
	float osc_ratio = 1;
#endif

uint32_t prescaled_two_power = 0;
uint32_t letimer0_prescaler = 0;

/************************************ GLOBALS ***************************************/

/****************************** FUNCTION PROTOTYPES *********************************/

void LETIMER0_IRQHandler(void);

void blockSleepMode(ENERGY_MODES e_letimer_energy_modes);

void unblockSleepMode(ENERGY_MODES e_letimer_energy_modes);

void LETIMER_Setup(void);

void Calibrate_ULFRCO(void);

void CMU_SetUp(void);

void GPIO_SetUp(void);

void LETIMER_Interrupt_Enable(void);

void LETimer_Config(void);

void Sleep(void);

void TIMER_Setup(void);

void LED_On(GPIO_Port_TypeDef port, unsigned int pin);

void LED_Off(GPIO_Port_TypeDef port, unsigned int pin);

void ACMP_setup(void);

void ACMP_Interrupt_Enable(void);

void cal_prescaler_value(float period, uint32_t osc_cnt);

/****************************** FUNCTION PROTOTYPES *********************************/
