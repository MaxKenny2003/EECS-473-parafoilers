// Servo interface
/*
Servo Purpose:
    Control a servo motor for positioning/guidance of parafoil payload

Servo Requirements:
    - PWM signal generation
    - Duty cycle controls servo angle 
*/

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Function pointer type for writing a PWM duty cycle value.
 *
 * The driver will call this function to update the PWM signal
 * controlling the servo.
 *
 * @param duty_us Pulse width in microseconds
 * @return true if the PWM update succeeded
 */
typedef bool (*servo_write)(uint16_t duty_us);

/**
 * @brief Servo driver context for one device.
 */
typedef struct {
    servo_write write;     // Function to set PWM duty cycle
    uint16_t min_us;       // Minimum pulse width
    uint16_t max_us;       // Maximum pulse width
} servo_t;

/**
 * @brief Initialize a servo driver context.
 *
 * @param ctx      Pointer to servo struct
 * @param write_fn PWM write function
 * @param min_us   Minimum pulse width in microseconds
 * @param max_us   Maximum pulse width in microseconds
 */
void servo_init(servo_t *ctx,
                servo_write write_fn,
                uint16_t min_us,
                uint16_t max_us);

/**
 * @brief Control servo by setting a specific angle.
 *
 * @param ctx   Servo struct
 * @param angle Desired angle in degrees (0–180)
 * @return true if the PWM update succeeded
 */
bool servo_set_angle(servo_t *ctx, uint8_t angle);

/**
 * @brief Contorl servo with PWM signal.
 *
 * @param ctx     Servo struct
 * @param duty_us Pulse width in microseconds
 * @return true if the PWM update succeeded
 */
bool servo_set_pulse(servo_t *ctx, uint16_t duty_us);
