#pragma once

#include "driver/uart.h"
#include "esp_err.h"

#define UART_BUFFER_SIZE            2048
#define UART_PRINT_PORT             0

esp_err_t init_UART(void);
char uart_getchar(uart_port_t uart_num);
void uart_putchar(uart_port_t uart_num, char c);
void uart_puts(uart_port_t uart_num, char *str);
void uart_gets(uart_port_t uart_num, char *str, uint8_t max_len);
void uart_getNum(uart_port_t uart_num, char *str, uint8_t max_len);
void uart_getAlpha(uart_port_t uart_num, char *str, uint8_t max_len);