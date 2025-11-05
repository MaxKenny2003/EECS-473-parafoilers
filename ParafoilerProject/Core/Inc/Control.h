/*
 * Control.h
 *
 *  Created on: Oct 29, 2025
 *
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "stm32h7xx_hal.h" // Required for TIM_HandleTypeDef


float calc_angle(float target_long, float target_lat, float current_long, float current_lat);

float normalize_angle(float angle);

#endif /* INC_CONTROL_H_ */
