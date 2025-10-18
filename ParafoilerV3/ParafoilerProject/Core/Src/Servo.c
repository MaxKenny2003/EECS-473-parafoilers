#include "Servo.h"
#include "stm32h7xx_hal.h" // Include HAL definitions

// Static pointer to store the timer handle
static TIM_HandleTypeDef *servo_htim;

/**
 * @brief Initializes the servo driver and starts the PWM signal.
 * @param htim: Pointer to the TIM_HandleTypeDef for the servo's timer.
 */
void servo_init(TIM_HandleTypeDef *htim) {
    servo_htim = htim;
    // Start the PWM channel required for the servo
    HAL_TIM_PWM_Start(servo_htim, TIM_CHANNEL_2);
}

// speed 100(fast) -> 149 (slow)
// clockwise is based on the servo label being right side up
void spin_servo_clockwise()
{
    if (servo_htim != NULL) {
        // Use the safe HAL function to set the PWM pulse width
        __HAL_TIM_SET_COMPARE(servo_htim, TIM_CHANNEL_2, 200);
    }
}

// speed 151(slow) -> 200 (fast)
// counterclockwise is based on the servo label being right side up
void spin_servo_counterclockwise()
{
    if (servo_htim != NULL) {
        __HAL_TIM_SET_COMPARE(servo_htim, TIM_CHANNEL_2, 350);
    }
}

// speed 100(fast) -> 149 (slow) clockwise
// speed 151(slow) -> 200 (fast) counter-clockwise
void spin_servo(int speed)
{
    if (servo_htim != NULL) {
        __HAL_TIM_SET_COMPARE(servo_htim, TIM_CHANNEL_2, speed);
    }
}

void stop_servo()
{
    if (servo_htim != NULL) {
        // Standard 1.5ms pulse for stopping a continuous rotation servo
        __HAL_TIM_SET_COMPARE(servo_htim, TIM_CHANNEL_2, 288);
    }
}
