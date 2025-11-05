#include "Servo.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>


Servo_Instance_t ServoL;
Servo_Instance_t ServoR;


/**
 * @brief Initializes the servo driver and starts the PWM signal.
 * @param htim: Pointer to the TIM_HandleTypeDef for the servo's timer.
 */
void servo_init(TIM_HandleTypeDef *htim) {
    // Initialize Left Servo
    ServoL.htim = htim;
    ServoL.channel = TIM_CHANNEL_1;
    ServoL.servo_num = 1; //added for logging
    ServoL.current_run_time = 0;
    ServoL.last_update_tick = osKernelGetTickCount();
    ServoL.dir = 0;

    // Initialize Right Servo
    ServoR.htim = htim;
    ServoR.channel = TIM_CHANNEL_2;
    ServoR.servo_num = 2; // added for logging
    ServoR.current_run_time = 0;
    ServoR.last_update_tick = osKernelGetTickCount();
    ServoR.dir = 0;

    // Start the PWM channels
    HAL_TIM_PWM_Start(htim, ServoL.channel);
    HAL_TIM_PWM_Start(htim, ServoR.channel);

    // Stop both servos on startup
    stop_servo(&ServoL);
    stop_servo(&ServoR);
}

/**
 * @brief Updates the run-time counter for a servo.
 * This should be called in the main servo loop (e.g., updateServos task)
 * @param servo: Pointer to the servo instance
 */
void update_servo_runtime(Servo_Instance_t* servo) {
    uint32_t current_tick = osKernelGetTickCount();

    if (servo->dir != 0) {
        uint32_t elapsed = current_tick - servo->last_update_tick;
        servo->current_run_time += (int32_t)elapsed * servo->dir;
    }

    servo->last_update_tick = current_tick;

    // Anti-coiling safety net
    // If servo runs too far in one direction, stop it.
    if (servo->current_run_time > MAX_SERVO_RUN_TIME) {
        stop_servo(servo);
        // print servo_num (1 or 2) instead of HAL channel value
        printf("SERVO SAFETY STOP: Max pull on channel %d\r\n", servo->servo_num);
    } else if (servo->current_run_time < -MAX_SERVO_RUN_TIME) {
        stop_servo(servo);
        // print servo_num (1 or 2) instead of HAL channel value
        printf("SERVO SAFETY STOP: Max release on channel %d\r\n", servo->servo_num);
    }
}


/**
 * @brief Spin the servo at a specific speed.
 * @param servo: Pointer to the servo instance
 * @param speed: The PWM pulse width (e.g., 192-384)
 * @return: 1 if servo was commanded to move, 0 if safety stop prevented it.
 */
int spin_servo(Servo_Instance_t* servo, int speed) {
    if (servo->htim == NULL) return 0;

    // Update direction based on speed
    if (speed > (SERVO_STOP_PULSE + 5)) { // +5 for deadband
        servo->dir = 1;  // Forward
    } else if (speed < (SERVO_STOP_PULSE - 5)) { // -5 for deadband
        servo->dir = -1; // Reverse
    } else {
        servo->dir = 0;
        speed = SERVO_STOP_PULSE; // Force stop
    }

    // Anti-coiling safety check
    // If we are at the limit, don't allow spinning further in that direction.
    if ((servo->dir == 1) && (servo->current_run_time >= MAX_SERVO_RUN_TIME)) {
        speed = SERVO_STOP_PULSE;
        servo->dir = 0;
        __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, speed);
        return 0; //return 0 no movement
    } else if ((servo->dir == -1) && (servo->current_run_time <= -MAX_SERVO_RUN_TIME)) {
        speed = SERVO_STOP_PULSE;
        servo->dir = 0;
        __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, speed);
        return 0; //return 0 no movement
    }

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, speed);
    return 1; //return 1 show movement
}

/**
 * @brief Stop the servo (sends 1.5ms pulse).
 * @param servo: Pointer to the servo instance
 */
void stop_servo(Servo_Instance_t* servo) {
    spin_servo(servo, SERVO_STOP_PULSE);
}

/**
 * @brief Turns left by pulling the left brake.
 * @param turn_angle: The error angle, from -180 to 0.
 */
void turnLeft(float turn_angle) {
    // This assumes:
    // Left Servo: FORWARD (384) pulls brake, REVERSE (192) releases
    // Right Servo: FORWARD (384) pulls brake, REVERSE (192) releases

    // Map turn_angle [-180, 0] to speed [SERVO_FORWARD_FAST, SERVO_STOP_PULSE]
    float gain = (float)(SERVO_FORWARD_FAST - SERVO_STOP_PULSE) / 90.0f; // Range/90deg
    float speed_float = SERVO_STOP_PULSE + fabs(turn_angle) * gain;

    if (speed_float > SERVO_FORWARD_FAST) speed_float = SERVO_FORWARD_FAST;

    // Check if the left servo actually moved (wasn't stopped by safety)
    int did_move = spin_servo(&ServoL, (int)speed_float); // Pull Left

    if (did_move) {
        spin_servo(&ServoR, SERVO_REVERSE_FAST); // Actively release Right
    } else {
        stop_servo(&ServoR); // If L is at limit, stop R too
    }
}

/**
 * @brief Turns right by pulling the right brake.
 * @param turn_angle: The error angle, from 0 to +180.
 */
void turnRight(float turn_angle) {
    // Map turn_angle [0, 180] to speed [SERVO_STOP_PULSE, SERVO_FORWARD_FAST]
    float gain = (float)(SERVO_FORWARD_FAST - SERVO_STOP_PULSE) / 90.0f; // Range/90deg
    float speed_float = SERVO_STOP_PULSE + fabs(turn_angle) * gain;

    if (speed_float > SERVO_FORWARD_FAST) speed_float = SERVO_FORWARD_FAST;

    // Check if the right servo actually moved (wasn't stopped by safety)
    int did_move = spin_servo(&ServoR, (int)speed_float); // Pull Right

    if (did_move) {
        spin_servo(&ServoL, SERVO_REVERSE_FAST); // Actively release Left
    } else {
        stop_servo(&ServoL); // If R is at limit, stop L too
    }
}



