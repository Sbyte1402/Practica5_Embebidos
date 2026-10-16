#include "bmp280_api.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "BMP280_API";
static spi_device_handle_t spi_handle;

// Escritura de registro SPI (MSB = 0 operación de escritura)
static void bmp280_write_register(uint8_t reg, uint8_t value) {
    uint8_t tx_buf[2] = { reg & 0x7F, value };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_buf
    };
    spi_device_transmit(spi_handle, &t);
}

static void bmp280_read_registers(uint8_t reg, uint8_t *data, size_t len) {
    uint8_t tx_buf[32] = {0};
    uint8_t rx_buf[32] = {0};

    tx_buf[0] = reg | 0x80;

    spi_transaction_t t = {
        .length = (len + 1) * 8, // 1 byte de dirección + len bytes de datos
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf
    };

    if (spi_device_transmit(spi_handle, &t) == ESP_OK) {
        // Se descarta rx_buf[0] correspondiente al byte dummy de dirección
        memcpy(data, &rx_buf[1], len);
    }
}

esp_err_t init_bmp280_spi(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 500000, // Ajustado a 500 kHz para estabilidad de señal
        .mode = 3,                // SPI Modo 3 (CPOL=1, CPHA=1) para alineación correcta de datos
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };

    esp_err_t ret = spi_bus_initialize(SPI_HOST_NUM, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return ret;

    ret = spi_bus_add_device(SPI_HOST_NUM, &devcfg, &spi_handle);
    if (ret != ESP_OK) return ret;

    // Configuración ctrl_meas (0xF4): Modo Normal, Oversampling Temp x2, Press x16    
    bmp280_write_register(0xE0, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configurar control de mediciones (ctrl_meas = 0xF4)
    // osrs_t x2 (010), osrs_p x16 (101), normal mode (11) -> 0x57
    bmp280_write_register(0xF4, 0x57);
    vTaskDelay(pdMS_TO_TICKS(20));

    return ESP_OK;
}

Bmp280_Calibration_t bmp280_get_calibration_params(void) {
    Bmp280_Calibration_t calib = {0};
    uint8_t data[24] = {0};

    bmp280_read_registers(0x88, data, 24);

    // Reconstrucción Little-Endian
    calib.dig_T1 = (uint16_t)((data[1] << 8) | data[0]);
    calib.dig_T2 = (int16_t)((data[3] << 8) | data[2]);
    calib.dig_T3 = (int16_t)((data[5] << 8) | data[4]);

    calib.dig_P1 = (uint16_t)((data[7] << 8) | data[6]);
    calib.dig_P2 = (int16_t)((data[9] << 8) | data[8]);
    calib.dig_P3 = (int16_t)((data[11] << 8) | data[10]);
    calib.dig_P4 = (int16_t)((data[13] << 8) | data[12]);
    calib.dig_P5 = (int16_t)((data[15] << 8) | data[14]);
    calib.dig_P6 = (int16_t)((data[17] << 8) | data[16]);
    calib.dig_P7 = (int16_t)((data[19] << 8) | data[18]);
    calib.dig_P8 = (int16_t)((data[21] << 8) | data[20]);
    calib.dig_P9 = (int16_t)((data[23] << 8) | data[22]);

    ESP_LOGI(TAG, "Calibración T1: %u | T2: %d | T3: %d", calib.dig_T1, calib.dig_T2, calib.dig_T3);

    return calib;
}

esp_err_t bmp280_read_raw(int32_t *raw_temp, int32_t *raw_pressure) {
    uint8_t status = 0;
    uint8_t data[6] = {0};

    for (int attempts = 0; attempts < 3; attempts++) {
        // Verificar si hay conversión activa (registro status 0xF3, bit 3)
        bmp280_read_registers(0xF3, &status, 1);
        if (status & 0x08) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        bmp280_read_registers(0xF7, data, 6);

        int32_t press = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4));
        int32_t temp  = (int32_t)(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4));

        if (temp > 300000 && temp < 800000) {
            *raw_pressure = press;
            *raw_temp = temp;
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return ESP_FAIL;
}

float bmp280_compensate_T(int32_t adc_T, Bmp280_Calibration_t *calib, int32_t *t_fine) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib->dig_T1 << 1))) * ((int32_t)calib->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib->dig_T1)) * ((adc_T >> 4) - ((int32_t)calib->dig_T1))) >> 12) * ((int32_t)calib->dig_T3)) >> 14;
    *t_fine = var1 + var2;
    T = (*t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

float bmp280_compensate_P(int32_t adc_P, Bmp280_Calibration_t *calib, int32_t t_fine) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib->dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib->dig_P5) << 17);
    var2 = var2 + (((int64_t)calib->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib->dig_P3) >> 8) + ((var1 * (int64_t)calib->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib->dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib->dig_P7) << 4);
    return (float)p / 256.0f;
}