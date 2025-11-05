#ifndef SERVO_H_
#define SERVO_H_

#include "stm32h7xx_hal.h" // Required for TIM_HandleTypeDef
#include <stdint.h>

typedef enum {
	SERVO_L = TIM_CHANNEL_1,
	SERVO_R = TIM_CHANNEL_2
} SERVO_CHANNELS;

// define the servo speeds here so freertos.c can see them
// 1.5ms pulse = 0 speed. Ticks = 288 (based on 120MHz / 625 prescaler)
// 1.0ms pulse = full reverse. Ticks = 192
// 2.0ms pulse = full forward. Ticks = 384
#define SERVO_REVERSE_FAST 192
#define SERVO_STOP_PULSE 288
#define SERVO_FORWARD_FAST 384

// define the Servo Instance structure
typedef struct {
    TIM_HandleTypeDef* htim; // Pointer to the timer
    uint32_t channel;        // TIM_CHANNEL_1 or TIM_CHANNEL_2
    int servo_num;           // The servo number (1 or 2) for logging
    int32_t  current_run_time; // Tracks how far it has spun (in ms)
    uint32_t last_update_tick; // Last time we updated its time
    int8_t   dir;              // -1 (reverse), 0 (stopped), 1 (forward)
} Servo_Instance_t;

// 3000ms (3 seconds) of continuous pull is the limit
// ADJUST THIS VALUE
#define MAX_SERVO_RUN_TIME 3000

extern Servo_Instance_t ServoL;
extern Servo_Instance_t ServoR;

// Function to initialize the servo driver with its timer
void servo_init(TIM_HandleTypeDef *htim);

// spin_servo now returns 1 if it moved, 0 if safety stopped it.
int spin_servo(Servo_Instance_t* servo, int speed);
void stop_servo(Servo_Instance_t* servo);
void turnLeft(float turn_angle);

/**
 * @brief Turns right by pulling the right brake.
 * @param turn_angle: The error angle, from 0 to +180.
 */
void turnRight(float turn_angle);

/**
 * @brief Updates the run-time counter for a servo.
 * @param servo: Pointer to the servo instance
 */
void update_servo_runtime(Servo_Instance_t* servo);

#endif /* SERVO_H_ */


