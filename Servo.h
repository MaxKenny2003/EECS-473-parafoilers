#include <stdint.h>


#define TIM4_ADDR 0x40000800 //timer 4 base register
#define TIM_CCR2_OFFSET 0x38//capture/compare register 2
#define CCR_MASK 0xFFFF


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
