#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#define SPI_HOST_NUM            SPI2_HOST
#define BMP280_SPI_CLOCK_SPEED  1000000

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} Bmp280_Calibration_t;

esp_err_t init_bmp280_spi(void);
Bmp280_Calibration_t bmp280_get_calibration_params(void);
esp_err_t bmp280_read_raw(int32_t *raw_temp, int32_t *raw_pressure);
float bmp280_compensate_T(int32_t adc_T, Bmp280_Calibration_t *calib, int32_t *t_fine);
float bmp280_compensate_P(int32_t adc_P, Bmp280_Calibration_t *calib, int32_t t_fine);