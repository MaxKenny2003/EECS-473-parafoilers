/**************************************************************************************************
 * @file system_core.h
 * @author Autonomous Parafoil Glider Team
 * @brief Core system definitions, pin mappings, and hardware configurations
 * @version 0.1
 * @date 2025-09-25
 *
 * This header file centralizes all hardware-specific definitions for the Airborne Guidance Unit
 * (AGU). It defines GPIO pin assignments for all peripherals connected to the STM32H7 MCU,
 * ensuring a single source of truth for hardware connections.
 *
 *************************************************************************************************/

#ifndef SYSTEM_CORE_H
#define SYSTEM_CORE_H

#include "stm32h7xx_hal.h"

// MCU Core and Clock Configuration
#define SYSTEM_CLOCK_FREQ_HZ 480000000UL // System core clock frequency in Hz [18, 34]

// Peripheral Interface Definitions
// --- UART for GPS Communication ---
#define GPS_UART_INSTANCE USART3
#define GPS_UART_BAUDRATE 115200
#define GPS_UART_TX_PORT GPIOD
#define GPS_UART_TX_PIN GPIO_PIN_8
#define GPS_UART_RX_PORT GPIOD
#define GPS_UART_RX_PIN GPIO_PIN_9

// --- UART for XBee Communication ---
#define XBEE_UART_INSTANCE UART5
#define XBEE_UART_BAUDRATE 115200
#define XBEE_UART_TX_PORT GPIOB
#define XBEE_UART_TX_PIN GPIO_PIN_13
#define XBEE_UART_RX_PORT GPIOB
#define XBEE_UART_RX_PIN GPIO_PIN_12
#define XBEE_RESET_PORT GPIOG
#define XBEE_RESET_PIN GPIO_PIN_10

// --- I2C for IMU and Altimeter ---
#define SENSORS_I2C_INSTANCE I2C1
#define SENSORS_I2C_SCL_PORT GPIOB
#define SENSORS_I2C_SCL_PIN GPIO_PIN_8
#define SENSORS_I2C_SDA_PORT GPIOB
#define SENSORS_I2C_SDA_PIN GPIO_PIN_9

// --- PWM for Servo Actuators ---
// Using Timer 1 (TIM1) for servos [34]
#define SERVO_PWM_TIMER TIM1
#define SERVO_L_PWM_CHANNEL TIM_CHANNEL_1 // Left servo
#define SERVO_R_PWM_CHANNEL TIM_CHANNEL_2 // Right servo
#define SERVO_L_PWM_PORT GPIOA
#define SERVO_L_PWM_PIN GPIO_PIN_8
#define SERVO_R_PWM_PORT GPIOA
#define SERVO_R_PWM_PIN GPIO_PIN_9

//More servos to be added

// --- Onboard LED for Status Indication ---
#define LED_STATUS_PORT GPIOB
#define LED_STATUS_PIN GPIO_PIN_0 // Corresponds to LD1 on Nucleo board [35]

// initialize system core components
void system_core_init(void);

#endif // SYSTEM_CORE_H