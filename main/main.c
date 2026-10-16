#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "uart.h"
#include "bmp280_api.h"

#define BUTTON_PIN 21

typedef struct {
    float temp;
    float press;
} Measurement_t;

Measurement_t history[10];
uint8_t hist_idx = 0;
uint8_t hist_count = 0;
SemaphoreHandle_t history_mutex;

QueueHandle_t button_queue;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t time_ms = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    xQueueSendFromISR(button_queue, &time_ms, NULL);
}

void float_to_str(float val, char* buffer) {
    int enter = (int)val;
    int dec = (int)((val - enter) * 100);
    if(dec < 0) dec = -dec;
    snprintf(buffer, 32, "%d.%02d", enter, dec);
}

int compare_measurements_temp(const void *a, const void *b) {
    float fa = ((Measurement_t*)a)->temp;
    float fb = ((Measurement_t*)b)->temp;
    return (fa > fb) - (fa < fb);
}

void bmp280_task(void *pvParameters) {
    Bmp280_Calibration_t calib = bmp280_get_calibration_params();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        int32_t raw_t, raw_p;
        int32_t t_fine;
        
        if (bmp280_read_raw(&raw_t, &raw_p) == ESP_OK) {
            float temp = bmp280_compensate_T(raw_t, &calib, &t_fine);
            float press = bmp280_compensate_P(raw_p, &calib, t_fine);
            
            xSemaphoreTake(history_mutex, portMAX_DELAY);
            history[hist_idx].temp = temp;
            history[hist_idx].press = press;
            
            hist_idx = (hist_idx + 1) % 10;
            if (hist_count < 10) {
                hist_count++;
            }
            xSemaphoreGive(history_mutex);
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
    }
}

void button_task(void *pvParameters) {
    uint32_t press_time;
    uint32_t last_press = 0;
    uint8_t presses = 0;
    char buffer[64];

    while(1) {
        if(xQueueReceive(button_queue, &press_time, pdMS_TO_TICKS(100))) {
            if(press_time - last_press > 200) { // Antirrebote de 200ms
                if(presses == 0 || (press_time - last_press <= 1500)) {
                    presses++;
                } else {
                    presses = 1; 
                }
                last_press = press_time;
            }
        } else {
            if(presses > 0 && (xTaskGetTickCount() * portTICK_PERIOD_MS - last_press > 1500)) {
                xSemaphoreTake(history_mutex, portMAX_DELAY);
                if(presses == 1) {
                    uart_puts(UART_PRINT_PORT, "\r\n--- Ultimas mediciones recibidas ---\r\n");
                    for(int i = 0; i < hist_count; i++) {
                        int read_idx = (hist_count == 10) ? ((hist_idx + i) % 10) : i;
                        char tmp_str[16];
                        float_to_str(history[read_idx].temp, tmp_str);
                        snprintf(buffer, sizeof(buffer), "[%d] Temp: %s C\r\n", i + 1, tmp_str);
                        uart_puts(UART_PRINT_PORT, buffer);
                    }
                } else if(presses >= 2) {
                    uart_puts(UART_PRINT_PORT, "\r\n--- Media y Mediana ---\r\n");
                    if(hist_count > 0) {
                        float sum = 0;
                        Measurement_t sorted[10];
                        for(int i = 0; i < hist_count; i++) {
                            sum += history[i].temp;
                            sorted[i] = history[i];
                        }
                        
                        float mean = sum / hist_count;
                        char mean_str[16];
                        float_to_str(mean, mean_str);
                        snprintf(buffer, sizeof(buffer), "Media Temp: %s C\r\n", mean_str);
                        uart_puts(UART_PRINT_PORT, buffer);
                        
                        qsort(sorted, hist_count, sizeof(Measurement_t), compare_measurements_temp);
                        float median = (hist_count % 2 == 0) 
                            ? (sorted[hist_count/2 - 1].temp + sorted[hist_count/2].temp) / 2.0f 
                            : sorted[hist_count/2].temp;
                        char med_str[16];
                        float_to_str(median, med_str);
                        snprintf(buffer, sizeof(buffer), "Mediana Temp: %s C\r\n", med_str);
                        uart_puts(UART_PRINT_PORT, buffer);
                    } else {
                        uart_puts(UART_PRINT_PORT, "No hay datos suficientes.\r\n");
                    }
                }
                xSemaphoreGive(history_mutex);
                presses = 0;
            }
        }
    }
}

void app_main(void) {
    init_UART();
    init_bmp280_spi();

    history_mutex = xSemaphoreCreateMutex();
    button_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_config_t btn_config = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .pull_up_en = 1,
        .pull_down_en = 0
    };
    gpio_config(&btn_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, NULL);

    xTaskCreate(bmp280_task, "BMP280 Task", 4096, NULL, 5, NULL);
    xTaskCreate(button_task, "Button Task", 4096, NULL, 6, NULL);
}