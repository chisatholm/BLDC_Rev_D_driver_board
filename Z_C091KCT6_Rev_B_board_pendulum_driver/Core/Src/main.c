/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <functions_structs.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PENDULUM_NO               0
//#define all_tests                 false // if false, no tests occur ('production'), if true, falls into while(all_tests) loop (testing)
//#define neutral_mag               true  // if ((all_tests) && neutral_mag) then print out the reading from the MA780 magnetic sensor
//#define current_sink              false // if ((all_tests) && current_sink) then cycle through all possible current levels, including reversing the current
//#define READ_DIRECTION_REGISTER   false  // if ((all_tests) && direction_register) then read the direction register every 5 seconds (not that it is likely to change!)
//#define WRITE_DIRECTION_REGISTER  false

#if (PENDULUM_NO == 0)

#define DESIRED_PERIOD            (1960.0)
#define PERIOD_us				  (DESIRED_PERIOD * 1000)
#define DESIRED_AMPLITUDE         (1000)
#define PERIODS_BETWEEN_METAPINGS (4)
#define CTRL_PULSE_MODE           (OUTPUT)    // Board #0 produces the metapings and needs this to be OUTPUT. All other boards need INPUT
#define attach_the_interrupt      (0)         // Board #0 needs NOT to have the interrupt attached, all other boards need interrupt attached
#define OFFSET					  (0)		// 43 for #0 & #1, 15 for #2, 19 for #3
#endif

#if (PENDULUM_NO == 1)

#define PERIOD                    (954.0)
#define PERIOD_us				  (954092)
#define DESIRED_AMPLITUDE         (5000)
#define PERIODS_BETWEEN_METAPINGS (65)
#define CTRL_PULSE_MODE           (INPUT)    // Board #0 produces the metapings and needs this to be OUTPUT. All other boards need INPUT
#define attach_the_interrupt      (1)         // Board #0 needs NOT to have the interrupt attached, all other boards need interrupt attached
#define OFFSET					  (43)		// 43 for #0 & #1, 15 for #2, 19 for #3
#endif

#if (PENDULUM_NO == 2)

#define PERIOD                    (940.0)
#define PERIOD_us				  (939636)
#define DESIRED_AMPLITUDE         (5000)
#define PERIODS_BETWEEN_METAPINGS (66)
#define CTRL_PULSE_MODE           (INPUT)    // Board #0 produces the metapings and needs this to be OUTPUT. All other boards need INPUT
#define attach_the_interrupt      (1)         // Board #0 needs NOT to have the interrupt attached, all other boards need interrupt attached
#define OFFSET					  (15)		// 43 for #0 & #1, 15 for #2, 19 for #3
#endif

#if (PENDULUM_NO == 3)

#define PERIOD                    (1500.0) // 926.0
#define PERIOD_us				  (1500000)
#define DESIRED_AMPLITUDE         (5000)  // 5000
#define PERIODS_BETWEEN_METAPINGS (67)    // 67
#define CTRL_PULSE_MODE           (INPUT)    // Board #0 produces the metapings and needs this to be OUTPUT. All other boards need INPUT
#define attach_the_interrupt      (1)         // Board #0 needs NOT to have the interrupt attached, all other boards need interrupt attached
#define OFFSET					  (20)		// 43 for #0 & #1, 15 for #2, 19 for #3
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
long AS5048A_read();
void AS5048A_update_pos_array(void);

void apply_current_vector(struct pendulum_t *p_pendulum);
void apply_specified_current_vector(struct pendulum_t *p_pendulum, int a);

long measure_neutral(void);
int measure_offset(struct pendulum_t *p_pendulum);
//int measure_offset(void);
void read_sensor();

long elec_pos(long sensor_reading);

void update_max_min(struct pendulum_t *p_pendulum);

void ping_due(struct pendulum_t *p_pendulum,
		struct time_tracker_t *p_time_tracker);
void duly_ping(void);

void configure_as_controller(void);

void offset_experiment(int brian);

#//define PRINTF2UART1 int __io_putchar(int data)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t UART1_rxBuffer[12] = { 0 };
UART_HandleTypeDef huart1;

float torqueB = 30; // 0 - 100
float torque_split = 0.5; // proportion of overall drive used to advance the phase of the pendulum
long errorB = 0;
long directnB = 0;
bool central_pos_flag = true; // measure the central position one time only at the start of the loop when flag = true
long neutral = -1; // variable to hold reading of AS5048A (use -1 as a warning that it has not been set)
bool amp_exceeds_target = false;
float X = 68, Y = 68, Z = 242; // 66 < Z < 68
long a = 39;
// long cumulative_lateness = 0;
float power = 38;
bool motor_update_flag = false;
long ping_count_current = 0; // counter to keep track of the number of metapings
long ping_count_previous = 0;	//
long ping_count = 0;


long offset_results[48];

bool if_flag = false;

struct time_tracker_t time_track;
struct pendulum_t pendulum_A;

const int A_offset = 0;
const int B_offset = 16;
const int C_offset = 32;

int currentStepA;
int currentStepB; //
int currentStepC;

int pwmSin[] = { 127, 110, 94, 78, 64, 50, 37, 26, 17, 10, 4, 1, 0, 1, 4, 10,
		17, 26, 37, 50, 64, 79, 95, 111, 128, 144, 160, 176, 191, 205, 218, 229,
		238, 245, 251, 254, 255, 254, 251, 245, 238, 229, 218, 205, 191, 176,
		160, 144 };
long count = 0;
long swing_counter = 0;
long swing_counter_ii = 0;
//int  offset = 12;

volatile bool positive = false;
volatile bool pos_going = false;
volatile bool outbound = false;
volatile uint16_t electrical_pos = 0;
volatile bool early = false; // if true, pendulum is arriving early at zero crossing
volatile bool longswing = false; // if true, pendulum is swinging too far

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	pendulum_A.pendulum_number = PENDULUM_NO;
	pendulum_A.periods_in_meta_period = PERIODS_BETWEEN_METAPINGS;
	pendulum_A.desired_period = DESIRED_PERIOD;      //
	pendulum_A.desired_period_us = PERIOD_us;
	pendulum_A.desired_amplitude = DESIRED_AMPLITUDE;   //
	pendulum_A.pgzc_flag = true; // one of the 2 flags needs to be true otherwise no zero crossings reported
	pendulum_A.ngzc_flag = true; // probably the other flag could also be true, TLDT
	pendulum_A.period = 1234L;              // period in milliseconds
	pendulum_A.meta_period = (PERIODS_BETWEEN_METAPINGS * DESIRED_PERIOD);
	pendulum_A.time_tolerance = 20; // if +/- tolerance from desired pgzc time, treat as correct time
	pendulum_A.amplitude_tolerance = 100; // if +/- tolerance from desired amplitude, treat as correct amplitude
	pendulum_A.period_tolerance = 5;
	pendulum_A.cumulative_pgzc_error = 0;
	pendulum_A.offset = 37;

	time_track.measurement_interval = 50L; // time between position measurements
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	if (PENDULUM_NO == 0) {
		configure_as_controller();
	}

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // necessary to start the
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	HAL_GPIO_WritePin(GPIOA, CSn_Pin, GPIO_PIN_SET); // Set CSn_Pin / PB0 high to disable SPI sensor
	HAL_GPIO_WritePin(GPIOB, DRV8313_en_Pin, GPIO_PIN_RESET); // Set En1_Pin  low to disable DRV8313

	while (1) {

		if (central_pos_flag) {
			central_pos_flag = false;
			pendulum_A.neutral_position = measure_neutral(); // execute once only to determine neutral position
			// pendulum_A.offset = measure_offset(&pendulum_A); // this function is time-consuming, use it in development but put the offset value in by hand for production
		}

		update_time(&time_track, HAL_GetTick());
		time_track.pgzc_time_last_desired = desired_pgzc(HAL_GetTick(),
				&pendulum_A, &time_track);
		ping_due(&pendulum_A, &time_track);

		if (measure_due(&time_track,
				pendulum_A.pos_array[(pendulum_A.pos_array[10][0])][1])) {
			AS5048A_update_pos_array(); // puts a new magnetic reading into position pos_array[10][0] of pos_array[][]
			motor_update_flag = true;
			if_flag = !if_flag;
		}
//
		update_max_min(&pendulum_A); // updates provisional max & min
		long dummy_2 = pgzc(&pendulum_A); // TODO can you make these functions 'void fn()' to avoid Wunused variable warning
		long dummy_3 = ngzc(&pendulum_A); // need to call both pgzc & ngzc to reset flags
		determine_late_early(&pendulum_A, &time_track);

		positive = (pendulum_A.pos_array[pendulum_A.pos_array[10][0]][0] > 0); // pendulum is in positive territory
		pos_going =
				(pendulum_A.pos_array[pendulum_A.pos_array[10][0]][0]
						- pendulum_A.pos_array[(9 + pendulum_A.pos_array[10][0])
								% 10][0] > 0); // pendulum is moving in positive direction
		outbound = ((positive && pos_going) || (!positive && !pos_going)); // pendulum is moving away from central rest position

		float multiplier = 1.8; // 2:- strongest period correction, 1:- weakest period correction
		// see extensive comment at line ~900

		if (outbound && !pendulum_A.early) { // driving here tends to make pendulum later, antagonistic to
			multiplier = 2 - multiplier;
		}

		if (!outbound && pendulum_A.early) {
			multiplier = 2 - multiplier;
		}

		if (pos_going) {
			directnB = 12;
		} else {
			directnB = 36;
		}

		if (pendulum_A.amplitude < pendulum_A.desired_amplitude) {
			torqueB = 120; // 100
		} else
			torqueB = 85;

		electrical_pos = (elec_pos(
				pendulum_A.pos_array[pendulum_A.pos_array[10][0]][0]) % 48);

		currentStepA = electrical_pos + directnB + pendulum_A.offset;
		currentStepB = currentStepA + B_offset;
		currentStepC = currentStepA + C_offset;

		currentStepA %= 48;
		currentStepB %= 48;
		currentStepC %= 48;

		if (motor_update_flag == true) {
			motor_update_flag = false;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
					pwmSin[currentStepA] * multiplier * torqueB / 100.0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
					pwmSin[currentStepB] * multiplier * torqueB / 100.0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
					pwmSin[currentStepC] * multiplier * torqueB / 100.0);
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	} // closing brace of principal while() loop
	/* USER CODE END 3 */
} // closing brace of main()

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 4000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 123;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 456;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 789;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DRV8313_en_GPIO_Port, DRV8313_en_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CSn_Pin */
	GPIO_InitStruct.Pin = CSn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CSn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DRV8313_en_Pin */
	GPIO_InitStruct.Pin = DRV8313_en_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DRV8313_en_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

long AS5048A_read() // use this function to obtain neutral reading, do not apply neutral offset
{
	uint8_t addr[] = { 0xFF, 0xFF };
	uint8_t buff[] = { 0, 0 };
	HAL_GPIO_WritePin(GPIOA, CSn_Pin, GPIO_PIN_RESET); // pull the cs pin low to enable the chip
	HAL_SPI_Transmit(&hspi1, addr, 1, 100); // send the address from where you want to read data
	HAL_GPIO_WritePin(GPIOA, CSn_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, CSn_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi1, buff, 1, 100);  // read 2 BYTES of data
	HAL_GPIO_WritePin(GPIOA, CSn_Pin, GPIO_PIN_SET); // pull the cs pin high to disable the chip
	long ret_val = (buff[0] | (buff[1] << 8));
	ret_val = ret_val & 0x3FFF;
	//ret_val = apply_neutral_offset(ret_val, &pendulum_A);
	return ret_val;
}

long measure_neutral(void) // remove power from motor, take magnetic readings
{
	HAL_GPIO_WritePin(GPIOB, DRV8313_en_Pin, GPIO_PIN_RESET); // Set pin  low to disable DRV8313

	HAL_Delay(4000); // allow motion to decay to zero
	long total = 0;
	for (int i = 0; i < 20; i++) {
		total += AS5048A_read();
		HAL_Delay(100);
	}
	HAL_GPIO_WritePin(GPIOB, DRV8313_en_Pin, GPIO_PIN_SET); // Set En1_Pin  high to enable DRV8313

	return (total / 20.0);
}

int measure_offset(struct pendulum_t *p_pendulum) // remove power from motor, take magnetic readings
{
	//if (PENDULUM_NO == 0){return 25;}
	//if (PENDULUM_NO == 1){return 23;}
	//if (PENDULUM_NO == 2){return 0;}
	//if (PENDULUM_NO == 3){return 3;} // check 0 number, 0 was 'made up'

	// TODO
	// 1) needs to find two local minima, expected to be 24 clicks of i apart from one another
	// 2) the correct local minimum is the one where the AS5048A reading increases with increasing i
	// eg
	// i = 11, reading = -20;
	// i = 12, reading = 2;
	// i = 13, reading = 24;
	// 3) return the correct local minimum

	HAL_GPIO_WritePin(GPIOB, DRV8313_en_Pin, GPIO_PIN_RESET); // Set pin low to disable DRV8313
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOB, DRV8313_en_Pin, GPIO_PIN_SET); // Set pin high to enable DRV8313

	AS5048A_update_pos_array();
	const long electrical_pos = elec_pos(
			pendulum_A.pos_array[pendulum_A.pos_array[10][0]][0]);
	//long offset_array[48];
	for (int i = 0; i < 48; i++) {

		currentStepA = electrical_pos + i; // 43 for #0 & #1, 15 for #2, 19 for #3
		currentStepB = currentStepA + B_offset;
		currentStepC = currentStepA + C_offset;

		currentStepA %= 48;
		currentStepB %= 48;
		currentStepC %= 48;
		torqueB = 500;

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
				pwmSin[currentStepA] * torqueB / 100.0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
				pwmSin[currentStepB] * torqueB / 100.0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
				pwmSin[currentStepC] * torqueB / 100.0);

		HAL_Delay(3500);
		AS5048A_update_pos_array();
		offset_results[i] =
				pendulum_A.pos_array[pendulum_A.pos_array[10][0]][0];
	}
	int j = 0;
	//int k[2];
	//long min_pos = 1000000;
	for (int i = 0; i < 48; i++) {
		if ( /* successive results are higher && cross zero */
		((offset_results[(i + 47) % 48]) < (offset_results[i]))
				&& ((offset_results[i]) < (offset_results[(i + 1) % 48]))
				&& (offset_results[(i + 47) % 48]) * (offset_results[i]) <= 0) {
			j = i;
		}
	}
	//for (int i = 0; i < 48; i++)
	//    {
	//	  if (abs(offset_results[i]) < min_pos){j = i; min_pos = abs(offset_results[i]);}
	//    }
	return j;

}

void offset_experiment(int brian) {
	AS5048A_update_pos_array();
	long electrical_pos = elec_pos(
			pendulum_A.pos_array[pendulum_A.pos_array[10][0]][0]);

	printf("electrical position %li\n\r", electrical_pos);

	currentStepA = electrical_pos + pendulum_A.offset + brian; // 43 for #0 & #1, 15 for #2, 19 for #3
	currentStepB = currentStepA + B_offset;
	currentStepC = currentStepA + C_offset;

	currentStepA %= 48;
	currentStepB %= 48;
	currentStepC %= 48;
	torqueB = 0;
	HAL_GPIO_WritePin(GPIOB, DRV8313_en_Pin, GPIO_PIN_SET); // Set pin high to enable DRV8313

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
			pwmSin[currentStepA] * torqueB / 100.0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
			pwmSin[currentStepB] * torqueB / 100.0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
			pwmSin[currentStepC] * torqueB / 100.0);
	HAL_GPIO_WritePin(GPIOB, DRV8313_en_Pin, GPIO_PIN_SET); // Set pin high to enable DRV8313

	HAL_Delay(50);

}

// use clock low, 2 edge in SPI configurations
void AS5048A_update_pos_array(void) // use this function to update the reading array, apply neutral offset
{
	uint8_t addr[] = { 0xFF, 0xFF };
	uint8_t buff[] = { 0, 0 };
	long write_location = (pendulum_A.pos_array[10][0] + 1) % 10;
	HAL_GPIO_WritePin(GPIOA, CSn_Pin, GPIO_PIN_RESET); // pull the cs pin low to enable the chip
	HAL_SPI_Transmit(&hspi1, addr, 1, 100); // send the address from where you want to read data
	HAL_GPIO_WritePin(GPIOA, CSn_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, CSn_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi1, buff, 1, 100);  // read 2 BYTES of data
	HAL_GPIO_WritePin(GPIOA, CSn_Pin, GPIO_PIN_SET); // pull the cs pin high to disable the chip
	long ret_val = 0;
	ret_val = (buff[0] | (buff[1] << 8));
	ret_val = ret_val & 0x3FFF; // the AS5048A is 'only' 14 bit, so discard 15:14 of 15:0
	ret_val = apply_neutral_offset(ret_val, &pendulum_A);
	pendulum_A.pos_array[10][0] = write_location;
	pendulum_A.pos_array[write_location][0] = ret_val;
	pendulum_A.pos_array[write_location][1] = HAL_GetTick();
}

void update_max_min(struct pendulum_t *p_pendulum) {
	long dummy_var = apply_neutral_offset(AS5048A_read(), &pendulum_A);
	//ret_val = apply_neutral_offset(ret_val, &pendulum_A);

	p_pendulum->pos_reading_provisional_max = fmax(
			p_pendulum->pos_reading_provisional_max, dummy_var);
	p_pendulum->pos_reading_provisional_min = fmin(
			p_pendulum->pos_reading_provisional_min, dummy_var);
}

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

//PRINTF2UART1
//	{
//		HAL_UART_Transmit(&huart1, (uint8_t *) &data, 1, 0xFFFF);
//		return data;
//	}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);
	time_track.meta_period_start_time = HAL_GetTick();
	ping_count++;

	/* NOTE: This function should not be modified, when the callback is needed,
	 the HAL_GPIO_EXTI_Falling_Callback could be implemented in the user file
	 */
	// printf("Interrupt received\n\r");
}

void ping_due(struct pendulum_t *p_pendulum,
		struct time_tracker_t *p_time_tracker) {
	if (PENDULUM_NO != 0) {
		return;
	}
	ping_count_current = (p_time_tracker->loop_time_current
			/ p_pendulum->meta_period);
	if (ping_count_current > ping_count_previous) {
		ping_count_previous = ping_count_current;
		duly_ping();
		ping_count++;
	}
}

void duly_ping(void) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // write this as explicit pin to avoid error when pendulum number != 0
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	time_track.meta_period_start_time = HAL_GetTick();
}

void configure_as_controller(void) {
	uint32_t *p_EXTI_FTSR1 = (uint32_t*) 0x40021804;
	uint32_t *p_EXTI_FPR1 = (uint32_t*) 0x40021810;
	uint32_t *p_EXTI_EXTICR2 = (uint32_t*) 0x40021864;
	uint32_t *p_EXTI_IMR1 = (uint32_t*) 0x40021880;

	uint32_t *p_GPIOB_MODER = (uint32_t*) 0x50000400;
	//uint32_t *p_GPIOB_IDR     = (uint32_t*) 0x50000410;
	uint32_t *p_GPIOB_ODR = (uint32_t*) 0x50000414;

	*p_EXTI_FTSR1 &= (0xFFFFFFFF - (1 << 7));
	*p_EXTI_FPR1 &= (0xFFFFFFFF - (1 << 7));
	*p_EXTI_EXTICR2 &= (0xFFFFFFFF - (1 << 28));
	*p_EXTI_IMR1 &= (0xFFFFFFFF - (1 << 7));

	*p_GPIOB_MODER |= (1 << 14);
	*p_GPIOB_ODR |= (1 << 7);

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

// comment from line ~292
// use multiplier to moderate the vigour with which the code 'tries' to return
// to ideal pgzc

// to modulate the period, the code determines whether the pendulum is
// 1) late or early and
// 2) inbound (moving towards the centre of the swing, also called the zero crossing point) or outbound

// the strongest correction to period is achieved as follows:
//		inbound  & early  : 	no power		-> no effect (if power were applied, it would cause the {early} pendulum to become even earlier)
//		inbound  & late   :		apply power		-> causes pendulum to be less late
//		outbound & early  : 	apply power		-> causes pendulum to be less early
//		outbound & late   :		no power		-> no effect (if power were applied, it would cause the {late} pendulum to become even later)

// however, this can be excessively effective, changing the PGZC by ~100ms or so, which is a visible misalignment

// if multiplier == 2, then all of the drive is directed towards
// changing the period to conform to ideal pgzc

// if multiplier == 1, then as much of the drive goes into conforming to pgzc as goes into making it worse

