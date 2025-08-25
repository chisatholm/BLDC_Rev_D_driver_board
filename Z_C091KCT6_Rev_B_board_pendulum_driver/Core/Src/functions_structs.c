/*
 * functions_structs.c
 *
 *  Created on: Apr 17, 2025
 *      Author: andrewchisholm
 */

/*
 * functions_structs.c
 *
 *  Created on: Apr 10, 2025
 *      Author: andrewchisholm
 */

#include "functions_structs.h"

#include <stdlib.h>

extern int pwmSin[];

#ifndef FUNCTIONS_STRUCTS
#define FUNCTIONS_STRUCTS

#define TIM_CHANNEL_1                      0x00000000U                          /*!< Capture/compare channel 1 identifier      */
#define TIM_CHANNEL_2                      0x00000004U                          /*!< Capture/compare channel 2 identifier      */
#define TIM_CHANNEL_3                      0x00000008U

#define __HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
  (((__CHANNEL__) == TIM_CHANNEL_1) ? ((__HANDLE__)->Instance->CCR1 = (__COMPARE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_2) ? ((__HANDLE__)->Instance->CCR2 = (__COMPARE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_3) ? ((__HANDLE__)->Instance->CCR3 = (__COMPARE__)) :\
  /* ((__CHANNEL__) == TIM_CHANNEL_4) ? ((__HANDLE__)->Instance->CCR4 = (__COMPARE__)) :\ */
/* ((__CHANNEL__) == TIM_CHANNEL_5) ? ((__HANDLE__)->Instance->CCR5 = (__COMPARE__)) :\ */
/* ((__HANDLE__)->Instance->CCR6 = (__COMPARE__))) */

#endif
long mapping(long x, long in_min, long in_max, long out_min, long out_max) {
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

long apply_neutral_offset(long reading, struct pendulum_t *p_pendulum) {
	long dummy = reading - p_pendulum->neutral_position;
	long ret_val = 98765;
	if (dummy > 8191) {
		ret_val = dummy - 16384;
	} else if (dummy < -8192) {
		ret_val = dummy + 16384;
	} else {
		ret_val = dummy;
	}

	return ret_val;
}

void calculate_current_vector(struct pendulum_t *p_pendulum, long position,
		int magnitude) {
	int dir;
	if (magnitude <= 0) {
		dir = 36;
	} else
		dir = 12;
	int dummyA = ((elec_pos(position) + dir) % 48); // direction = 12 or 36, dummy is the value in array pwmSin to select
	int dummyB = (dummyA + 16) % 48;
	int dummyC = (dummyA + 32) % 48;

	if (abs(magnitude) > 100) {
		magnitude = 100;
	} // don't care about the sign of magnitude any more.
	// TODO magnitude is a bad word for this job, replace with eg torque

	p_pendulum->current_vector[0] = (int) (abs(magnitude) * pwmSin[dummyA]
			/ 100.0);
	p_pendulum->current_vector[1] = (int) (abs(magnitude) * pwmSin[dummyB]
			/ 100.0);
	p_pendulum->current_vector[2] = (int) (abs(magnitude) * pwmSin[dummyC]
			/ 100.0);
}

long elec_pos(long sensor_reading) {
	long dummy = mapping(sensor_reading, -8192, 8191, 0, 48 * 7); // 'electrical position'
	return dummy;
}

bool measure_due(struct time_tracker_t *time_track_var, long last_measure_time) {
	long dummy = time_track_var->loop_time_current;
	if (dummy - last_measure_time >= time_track_var->measurement_interval) {
		return true;
	} else
		return false;
}

void set_neutral_position(struct pendulum_t *p_pendulum, long position) {
	p_pendulum->neutral_position = position;
}

void update_time(struct time_tracker_t *time_track_var, long time) {
	time_track_var->loop_time_previous = time_track_var->loop_time_current;
	time_track_var->loop_time_current = time;
}

long pgzc(struct pendulum_t *p_pendulum) // returns the est. time at which the pendulum crosses zero going +ve, updates pendulum struct
{
	if (p_pendulum->neutral_position < 0) {
		return -1;
	} // neutral position has not been measured (so all readings are meaningless)
	long positionB = p_pendulum->pos_array[p_pendulum->pos_array[10][0]][0]; // newest position taken into account
	long positionA = p_pendulum->pos_array[(p_pendulum->pos_array[10][0] + 9)
			% 10][0]; // oldest position taken into account
	if (positionB >= 0 && positionA < 0 && p_pendulum->pgzc_flag == true) {
		p_pendulum->pos_reading_min = p_pendulum->pos_reading_provisional_min;
		p_pendulum->pos_reading_provisional_min = 20123;
		//p_pendulum->pos_reading_max = p_pendulum->pos_reading_provisional_max;
		//p_pendulum->pos_reading_provisional_max = 0;
		p_pendulum->amplitude = p_pendulum->pos_reading_max
				- p_pendulum->pos_reading_min;
		float numerator =
				positionB
						* p_pendulum->pos_array[(p_pendulum->pos_array[10][0]
								+ 9) % 10][1]
						- positionA
								* p_pendulum->pos_array[p_pendulum->pos_array[10][0]][1];
		float denominator = positionB - positionA;
		long crossing_time = (long) (numerator / denominator);
		p_pendulum->ngzc_flag = true;
		p_pendulum->pgzc_flag = false; //
		p_pendulum->period = (crossing_time - p_pendulum->pgzc_time);
		p_pendulum->pgzc_time = crossing_time;
		p_pendulum->pgzc_count++;
		return crossing_time;
	} else {
		return -2;
	} // benign, but no zero crossing
}

long ngzc(struct pendulum_t *p_pendulum) // TODO does not cope with neutral position ~= 0
{
	if (p_pendulum->neutral_position < 0) {
		return -1;
	} // neutral position has not been measured (so all readings are meaningless)
	long positionB = p_pendulum->pos_array[p_pendulum->pos_array[10][0]][0]; // newest position taken into account
	long positionA = p_pendulum->pos_array[(p_pendulum->pos_array[10][0] + 9)
			% 10][0]; // oldest position taken into account
	if (positionB <= 0 && positionA > 0 && p_pendulum->ngzc_flag == true) // condition for position to have crossed from +ve to -ve position
	{
		p_pendulum->pos_reading_max = p_pendulum->pos_reading_provisional_max;
		p_pendulum->pos_reading_provisional_max = 0;
		//p_pendulum->pos_reading_min = p_pendulum->pos_reading_provisional_min;
		//p_pendulum->pos_reading_provisional_min = 20123;
		p_pendulum->amplitude = p_pendulum->pos_reading_max
				- p_pendulum->pos_reading_min;
		float numerator =
				positionB
						* p_pendulum->pos_array[(p_pendulum->pos_array[10][0]
								+ 9) % 10][1]
						- positionA
								* p_pendulum->pos_array[p_pendulum->pos_array[10][0]][1];
		float denominator = positionB - positionA;
		long crossing_time = (long) (numerator / denominator); // linear interpolation of crossing time
		p_pendulum->ngzc_flag = false;
		p_pendulum->pgzc_flag = true; //
		p_pendulum->ngzc_time = crossing_time;
		return crossing_time;
	} else {
		return -2;
	} // benign, but no zero crossing
}

long desired_pgzc(long time, struct pendulum_t *p_pendulum,
		struct time_tracker_t *p_time_track_var) // returns time of most recent desired pgzc
{
	// TODO refine this to use microseconds

	long long A = p_time_track_var->meta_period_start_time; // meta period start time, ms
	long long B = (time * 1000 - 1000 * p_time_track_var->meta_period_start_time); // time since start of meta period, us (MICRO seconds)
	long long C = p_pendulum->desired_period_us; // desired period of pendulum, us (MICRO seconds)
	long long D = 0;

	/*
	return (p_time_track_var->meta_period_start_time
			+ ((time * 1000 - 1000 * p_time_track_var->meta_period_start_time)
					/ p_pendulum->desired_period_us) //* p_pendulum->desired_period_us/1000));*/
	D = A + (B / C) * C / 1000; //
	return D;
	//  A 					: metaperiod start time, ms
	// (B / C) 				: no of complete periods since start of meta period
	// (B / C) * C 			: elapsed time from start of meta period to desired PGZC, us (MICRO seconds)
	// (B / C) * C / 1000 	: as above, ms
}

void determine_late_early(struct pendulum_t *p_pendulum,
		struct time_tracker_t *p_time_track_var) {
	unsigned long long A = p_pendulum->pgzc_time;
	unsigned long long D = p_time_track_var->pgzc_time_last_desired;
	long long AlessD = A - D;

	if ((AlessD) > 0) {
		if (2 * abs(AlessD) > (p_pendulum->desired_period)) {
			p_pendulum->early = true;
		} else {
			p_pendulum->early = false;
		}
	}

	if ((AlessD) <= 0) {
		if (2 * abs(AlessD) > (p_pendulum->desired_period)) {
			p_pendulum->early = false;
		} else {
			p_pendulum->early = true;
		}
	}
}

