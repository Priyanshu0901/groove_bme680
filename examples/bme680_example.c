/**
 * @file bme680_example.c
 * @brief Example application for BME680 sensor
 * 
 * This example demonstrates how to use the BME680 sensor component.
 * It reads sensor data periodically and prints it to the console.
 * 
 * @copyright Copyright (c) 2024
 * @license BSD-3-Clause
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bme680.h"

static const char *TAG = "BME680_EXAMPLE";

void app_main(void)
{
    ESP_LOGI(TAG, "BME680 sensor example started");
    
    /* Initialize the BME680 sensor */
    ESP_LOGI(TAG, "Initializing BME680 sensor...");
    esp_err_t ret = bme680_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BME680 sensor: %s", esp_err_to_name(ret));
        return;
    }
    
    /* Add a delay after initialization to allow the sensor to stabilize */
    ESP_LOGI(TAG, "BME680 initialized, waiting for sensor to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    /* Read and print sensor data in a loop */
    while (1) {
        bme680_data_t data;
        ret = bme680_read_data(&data);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Sensor Data:");
            ESP_LOGI(TAG, "  Temperature: %.2f °C", data.temperature);
            ESP_LOGI(TAG, "  Pressure: %.2f hPa", data.pressure);
            ESP_LOGI(TAG, "  Humidity: %.2f %%", data.humidity);
            ESP_LOGI(TAG, "  Gas Resistance: %.2f kOhm", data.gas_resistance / 1000.0f);
            ESP_LOGI(TAG, "  Gas Valid: %s", data.gas_valid ? "Yes" : "No");
            ESP_LOGI(TAG, "  Data Valid: %s", data.meas_valid ? "Yes" : "No");
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
        }
        
        /* Wait 2 seconds before next reading */
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
} 