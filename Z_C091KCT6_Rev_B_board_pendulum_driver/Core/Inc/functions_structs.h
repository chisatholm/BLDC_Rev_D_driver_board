/*
 * functions_structs.h
 *
 *  Created on: Apr 10, 2025
 *      Author: andrewchisholm
 */

#ifndef INC_FUNCTIONS_STRUCTS_H_
#define INC_FUNCTIONS_STRUCTS_H_

#include <stdbool.h>

struct time_tracker_t // excludes all magnetic measurement times, these are held in the array pos_array[][]
{
	unsigned long long loop_time_current; // uint16_t
	unsigned long long loop_time_previous;

	unsigned long long pgzc_time_current;
	unsigned long long pgzc_time_previous;

	unsigned long long ngzc_time_current;
	unsigned long long ngzc_time_previous;

	unsigned long long pgzc_time_last_desired;
	unsigned long long pgzc_time_next_desired;

	long measurement_interval; // milliseconds between position measurements
	long motor_update_interval; // milliseconds between motor instructions TODO is this variable AND measurement_interval needed?

	long meta_period_start_time;
};

struct pendulum_t {
	long neutral_position;// value returned by encoder when motor is turned off
	int offset;					// empirically found - positional motor offset
	long pos_reading_provisional_max; // hold the max position reading as it is collected in this variable
	long pos_reading_max; // receive the the max reading at ngzc into this variable
	long pos_reading_provisional_min; // hold the min position reading as it is collected in this variable
	long pos_reading_min;  // receive the min reading at pgzc into this variable
	int current_vector[3];           	// not used
	int pendulum_number;// number 0 - 7, 0 for controller, 1-7 for peripherals
	int periods_in_meta_period;	// the number of periods of the slowest pendulum in one meta period
	long meta_period;				// length of a meta period in milliseconds
	long desired_period;// the period of the pendulum required to form the pattern, milliseconds
	long desired_period_us;	// the period of the pendulum required to form the pattern, microseconds
	long period;						// the actual period of the pendulum
	int time_tolerance;	// PGZC acceptable error - used for category five swing, in which period and amplitude were OK / required no change
	long period_tolerance;				// period acceptalbe error
	long period_error;					// period actual error
	long pgzc_error;					// PGZC actual error
	long cumulative_pgzc_error;
	long desired_amplitude;				// desired amplitude of swing
	long amplitude;						// actual amplitude of swing
	int amplitude_tolerance;			// acceptable error in amplitude
	long pos_array[11][2];// circular array to hold position and time data, see below this struct definition for further detail
	bool pgzc_flag;
	bool ngzc_flag;
	unsigned long long pgzc_time;				// Positive Going Zero Crossing
	unsigned long long ngzc_time;						// Negative G     Z    C
	bool early;					// if true, pendulum arrived at last pgzc early

};

// pos_array[][] ("position array") is a circular array:
// locations [0:9][0:1] store data
// eg pos_array[5][0] is a position measurement, an integer between 0 & (2^14-1) ...
// ... taken at time pos_array[5][1]
// time is measured in milliseconds from the boot time of the MCU
// location [10][0] stores the location of last datum added, eg if pos_array[10][0] == 3, then the last data were written into pos_array[3][0:1]
// location [10][1] is garbage, pos_array[][] was originally called my_array, there are still some legacy references to my_array

struct admin_t {
};

long elec_pos(long sensor_reading);
long mapping(long x, long in_min, long in_max, long out_min, long out_max);
bool measure_due(struct time_tracker_t *time_track_var, long last_measure_time);
bool motor_update_due(struct time_tracker_t *time_track_var,
		long last_measure_time);
long ngzc(struct pendulum_t *p_pendulum);
long pgzc(struct pendulum_t *p_pendulum);

void set_neutral_position(struct pendulum_t *p_pendulum, long position);
void update_time(struct time_tracker_t *time_track_var, long time);
void update_max_min(struct pendulum_t *p_pendulum);
void calculate_current_vector(struct pendulum_t *p_pendulum, long position,
		int magnitude); // current as in Amperes, not 'present' or dried grape
//void apply_current_vector(struct pendulum_t * p_pendulum);                                   // current as in Amperes, not 'present' or dried grape
void apply_specified_current_vector(struct pendulum_t *p_pendulum, int a);
long desired_pgzc(long time, struct pendulum_t *p_pendulum,
		struct time_tracker_t *p_time_track_var); // returns the time at which the most recent pgzc should have been seen
long apply_neutral_offset(long reading, struct pendulum_t *p_pendulum);
void determine_late_early(struct pendulum_t *p_pendulum,
		struct time_tracker_t *p_time_track_var);

#endif /* INC_FUNCTIONS_STRUCTS_H_ */

//
//  static void MX_GPIO_Init(void)
//  {
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//  /* USER CODE BEGIN MX_GPIO_Init_1 */
//  /* USER CODE END MX_GPIO_Init_1 */
//
//    /* GPIO Ports Clock Enable */
//    __HAL_RCC_GPIOC_CLK_ENABLE();
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    __HAL_RCC_GPIOB_CLK_ENABLE();
//
//    /*Configure GPIO pin Output Level */
//    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
//
//    /*Configure GPIO pin Output Level */
//    HAL_GPIO_WritePin(DRV8313_en_GPIO_Port, DRV8313_en_Pin, GPIO_PIN_RESET);
//
//    /*Configure GPIO pin : CSn_Pin */
//    GPIO_InitStruct.Pin = CSn_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(CSn_GPIO_Port, &GPIO_InitStruct);
//
//    /*Configure GPIO pin : DRV8313_en_Pin */
//    GPIO_InitStruct.Pin = DRV8313_en_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(DRV8313_en_GPIO_Port, &GPIO_InitStruct);
//
//    /*Configure GPIO pin : PB7 */
//    GPIO_InitStruct.Pin = GPIO_PIN_7;
//    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//    /* EXTI interrupt init*/
//    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
//
//  /* USER CODE BEGIN MX_GPIO_Init_2 */
//  /* USER CODE END MX_GPIO_Init_2 */
//  }

//  static void MX_GPIO_Init(void)
//  {
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//  /* USER CODE BEGIN MX_GPIO_Init_1 */
//  /* USER CODE END MX_GPIO_Init_1 */
//
//    /* GPIO Ports Clock Enable */
//    __HAL_RCC_GPIOC_CLK_ENABLE();
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    __HAL_RCC_GPIOB_CLK_ENABLE();
//
//    /*Configure GPIO pin Output Level */
//    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
//
//    /*Configure GPIO pin Output Level */
//    HAL_GPIO_WritePin(GPIOB, DRV8313_en_Pin|Ping_Pin, GPIO_PIN_RESET);
//
//    /*Configure GPIO pin : CSn_Pin */
//    GPIO_InitStruct.Pin = CSn_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(CSn_GPIO_Port, &GPIO_InitStruct);
//
//    /*Configure GPIO pins : DRV8313_en_Pin Ping_Pin */
//    GPIO_InitStruct.Pin = DRV8313_en_Pin|Ping_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN MX_GPIO_Init_2 */
//  /* USER CODE END MX_GPIO_Init_2 */
//  }

