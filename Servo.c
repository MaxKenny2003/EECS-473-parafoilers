#include "Servo.h"


// speed 100(fast) -> 149 (slow)
// clockwise is based on the servo label being right side up
void spin_servo_clockwise()
{
    uint32_t * tim4_ccr2 = (uint32_t *)(TIM4_ADDR + TIM_CCR2_OFFSET);

    *tim4_ccr2 &= ~CCR_MASK;
    *tim4_ccr2 |= 125;
}

// speed 151(slow) -> 200 (fast)
// counterclockwise is based on the servo label being right side up
void spin_servo_counterclockwise()
{
    uint32_t *tim4_ccr2 = (uint32_t *)(TIM4_ADDR + TIM_CCR2_OFFSET);

    *tim4_ccr2 &= ~CCR_MASK;
    *tim4_ccr2 |= 175;
}

// speed 100(fast) -> 149 (slow) clockwise
// speed 151(slow) -> 200 (fast) counter-clockwise
void spin_servo(int speed)
{
    uint32_t *tim4_ccr2 = (uint32_t *)(TIM4_ADDR + TIM_CCR2_OFFSET);

    *tim4_ccr2 &= ~CCR_MASK;
    *tim4_ccr2 |= speed;
}

void stop_servo()
{
    uint32_t *tim4_ccr2 = (uint32_t *)(TIM4_ADDR + TIM_CCR2_OFFSET);

    *tim4_ccr2 &= ~CCR_MASK;
    *tim4_ccr2 |= 150;
}
