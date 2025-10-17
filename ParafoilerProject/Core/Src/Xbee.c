#include "Xbee.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "Servo.h"
#include "main.h"
#include "NEO_M9_UART.h" // Include for M9N_Start function

#define XBEE_RX_BUFFER_SIZE 9

UART_HandleTypeDef* xbee_uart_handle; // Renamed for clarity

uint8_t rx_buf[XBEE_RX_BUFFER_SIZE];
uint8_t NEW_COORD_BUF[8];

volatile uint8_t new_coord_flag = 0;
volatile uint32_t interrupt_count = 0;
volatile uint32_t coord_received_count = 0;
volatile uint32_t error_count = 0;
volatile uint32_t restart_count = 0;
volatile uint8_t gps_error_recovery_in_progress = 0;
uint8_t MODE = 0;
extern volatile bool manual_control_mode;
extern volatile uint8_t manual_servo_command;
extern volatile uint32_t last_manual_command_time;

void xbee_init(UART_HandleTypeDef *huart) {
    xbee_uart_handle = huart; // Store the handle

    memset(rx_buf, 0, XBEE_RX_BUFFER_SIZE);
    memset(NEW_COORD_BUF, 0, 8);

    printf("XBee Init - UART State: %u\r\n", xbee_uart_handle->RxState);
    printf("XBee Init - Error Code: 0x%08lX\r\n", xbee_uart_handle->ErrorCode);

    HAL_StatusTypeDef status = HAL_UART_Receive_IT(xbee_uart_handle, rx_buf, XBEE_RX_BUFFER_SIZE);

    if(status != HAL_OK) {
        printf("ERROR: Xbee HAL_UART_Receive_IT failed! Status: %d\r\n", status);
        printf("UART State after fail: %u\r\n", xbee_uart_handle->RxState);
        Error_Handler();
    } else {
        printf("XBee UART RX interrupt started OK\r\n");
    }

    if(NVIC_GetEnableIRQ(USART2_IRQn)) {
        printf("USART2 interrupt is ENABLED in NVIC\r\n");
    } else {
        printf("ERROR: USART2 interrupt NOT enabled!\r\n");
    }
}

bool xbee_send_data(uint8_t *payload, uint16_t length) {
    HAL_UART_Transmit(xbee_uart_handle, payload, length, 1000);
    return true;
}

bool xbee_receive_data(uint8_t *buffer, uint16_t max_len) {
    HAL_UART_Receive(xbee_uart_handle, buffer, max_len, HAL_MAX_DELAY);
    return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART2) { // Check if it's the XBee UART
        interrupt_count++;

        // Toggle LED1 to show interrupt firing
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

        handle_cmd(rx_buf);

        // CRITICAL: Restart the interrupt
        HAL_StatusTypeDef status = HAL_UART_Receive_IT(huart, rx_buf, XBEE_RX_BUFFER_SIZE);
        if(status != HAL_OK) {
            error_count++;
            printf("ERROR: Failed to restart Xbee UART_Receive_IT. Status: %d\r\n", status);
        }
    }
}

/**
  * @brief  Central UART error handler for the application.
  * @param  huart: UART handle pointer
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) { // GPS UART
        uint32_t error_code = HAL_UART_GetError(huart);

        printf("\r\n!!! GPS UART Error Callback !!!\r\n");
        printf("  Error Code: 0x%08lX\r\n", error_code);
        printf("  RxState: %d\r\n", huart->RxState);

        if (error_code & HAL_UART_ERROR_ORE)
        {
             printf("  Type: Overrun Error (ORE)\r\n");
             printf("  Setting gps_error_recovery_in_progress flag\r\n");

             gps_error_recovery_in_progress = 1;

             printf("  Calling HAL_UART_AbortReceive_IT...\r\n");
             HAL_StatusTypeDef abort_status = HAL_UART_AbortReceive_IT(huart);
             printf("  Abort returned: %d\r\n", abort_status);

             if(abort_status != HAL_OK)
             {
                printf("  CRITICAL: HAL_UART_AbortReceive_IT failed!\r\n");
                gps_error_recovery_in_progress = 0;
             }
        } else {
            printf("  Type: Other UART error\r\n");
        }
        printf("!!! GPS Error Callback complete !!!\r\n\r\n");
    }
    else if(huart->Instance == USART2) { // Xbee UART
        error_count++;
        uint32_t error_code = HAL_UART_GetError(huart);
        printf("ERROR: UART Error on Xbee (USART2). Code: 0x%lX. Restarting IT.\r\n", error_code);

        HAL_UART_AbortReceive_IT(huart);
        HAL_UART_Receive_IT(huart, rx_buf, XBEE_RX_BUFFER_SIZE);
    }
}

/**
  * @brief  Callback executed when a UART Abort Receive operation is complete.
  * @param  huart: UART handle.
  * @retval None
  */
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    printf("GPS Abort Complete - restarting DMA\r\n");

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_READY;

    // Just restart DMA
    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)gpsRxBuffer, RX_BUFFER_LEN);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    gps_error_recovery_in_progress = 0;
  }
}

void handle_cmd(uint8_t *cmd) {
    char cmd_type = cmd[0];

    if(cmd_type == 'C') {
        memcpy(NEW_COORD_BUF, cmd + 1, 8);
        new_coord_flag = 1;
        coord_received_count++;
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        manual_control_mode = false;

    } else if(cmd_type == 'L') {
        manual_control_mode = true;
        manual_servo_command = 1;
        last_manual_command_time = osKernelGetTickCount();

    } else if(cmd_type == 'R') {
        manual_control_mode = true;
        manual_servo_command = 2;
        last_manual_command_time = osKernelGetTickCount();

    } else if(cmd_type == 'S') {
        manual_control_mode = true;
        manual_servo_command = 0;
        last_manual_command_time = osKernelGetTickCount();

    } else if(cmd_type == 'M') {
        manual_control_mode = !manual_control_mode;
        if(!manual_control_mode) {
            manual_servo_command = 0;
        }
    }
}

uint32_t xbee_get_interrupt_count(void) {
    return interrupt_count;
}

uint32_t xbee_get_coord_count(void) {
    return coord_received_count;
}

uint32_t xbee_get_error_count(void) {
    return error_count;
}

uint32_t xbee_get_restart_count(void) {
    return restart_count;
}

