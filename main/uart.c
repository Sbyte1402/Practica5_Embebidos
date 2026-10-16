#include "uart.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "hal/uart_types.h"
#include "portmacro.h"
#include "soc/clk_tree_defs.h"
#include <string.h>

esp_err_t init_UART(void){
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PRINT_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PRINT_PORT,
                                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return uart_driver_install(UART_PRINT_PORT, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0);
}

void uart_putchar(uart_port_t uart_num, char c){
    uart_write_bytes(uart_num, (const void*)&c, sizeof(char));
}

char uart_getchar(uart_port_t uart_num){
    char c;
    uart_read_bytes(uart_num, &c, sizeof(char), portMAX_DELAY);
    
    return c;
}

void uart_puts(uart_port_t uart_num, char *str){
    uart_write_bytes(uart_num, str, strlen(str));
}

void uart_gets(uart_port_t uart_num, char *str, uint8_t max_len){
    int i = 0;
    while(1){
        char c = uart_getchar(uart_num);
        
        if(c == '\r' || c == '\n'){
            uart_write_bytes(uart_num, "\r\n", 2);
            break;
        }

        if(c == '\b'){
            if(i > 0){
                i--;
                uart_write_bytes(uart_num, "\b \b", 3);
            }
            continue;
        }

        if(i < max_len - 1){
            str[i++] = c;
            uart_putchar(uart_num, c);
        }    
    }
    str[i] = '\0';
}

void uart_getNum(uart_port_t uart_num, char *str, uint8_t max_len){
    int i = 0;
    while(i < max_len - 1){
        char c = uart_getchar(uart_num);
        
        if(c == '\r' || c == '\n'){
            uart_write_bytes(uart_num, "\r\n", 2);
            break;
        }

        if(c == '\b'){
            if(i > 0){
                i--;
                uart_write_bytes(uart_num, "\b \b", 3);
            }
            continue;
        }

        if(c >= '0' && c <= '9'){
            str[i++] = c;
            uart_putchar(uart_num, c);
        }
    }
    str[i] = '\0';
}

void uart_getAlpha(uart_port_t uart_num, char *str, uint8_t max_len){
    int i = 0;
    while(i < max_len - 1){
        char c = uart_getchar(uart_num);
        
        if(c == '\r' || c == '\n'){
            uart_write_bytes(uart_num, "\r\n", 2);
            break;
        }

        if(c == '\b'){
            if(i > 0){
                i--;
                uart_write_bytes(uart_num, "\b \b", 3);
            }
            continue;
        }

        if((c >= 'A' && c <= 'Z') ||
           (c >= 'a' && c <= 'z')){
            str[i++] = c;
            uart_putchar(uart_num, c);
        }
    }
    str[i] = '\0';
}