#ifndef SERVO_H_
#define SERVO_H_

#include "stm32h7xx_hal.h" // Required for TIM_HandleTypeDef

typedef enum {
	SERVO_L = TIM_CHANNEL_1,
	SERVO_R = TIM_CHANNEL_2
} SERVO_CHANNELS;


// Function to initialize the servo driver with its timer
void servo_init(TIM_HandleTypeDef *htim);

// speed 100(fast) -> 149 (slow)
// clockwise is based on the servo label being right side up
void spin_servo_clockwise(uint8_t servoNum);

// speed 151(slow) -> 200 (fast)
// counterclockwise is based on the servo label being right side up
void spin_servo_counterclockwise(uint8_t servoNum);

// speed 100(fast) -> 149 (slow) clockwise
// speed 151(slow) -> 200 (fast) counter-clockwise
void spin_servo(int speed, uint8_t servoNum);

void stop_servo(uint8_t servoNum);

#endif /* SERVO_H_ */
