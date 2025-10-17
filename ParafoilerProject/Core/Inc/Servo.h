#ifndef SERVO_H_
#define SERVO_H_

#include "stm32h7xx_hal.h" // Required for TIM_HandleTypeDef

// Function to initialize the servo driver with its timer
void servo_init(TIM_HandleTypeDef *htim);

// speed 100(fast) -> 149 (slow)
// clockwise is based on the servo label being right side up
void spin_servo_clockwise();

// speed 151(slow) -> 200 (fast)
// counterclockwise is based on the servo label being right side up
void spin_servo_counterclockwise();

// speed 100(fast) -> 149 (slow) clockwise
// speed 151(slow) -> 200 (fast) counter-clockwise
void spin_servo(int speed);

void stop_servo();

#endif /* SERVO_H_ */
