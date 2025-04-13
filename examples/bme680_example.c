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

/* Maximum number of consecutive read failures before taking action */
#define MAX_CONSECUTIVE_FAILURES 3

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
    vTaskDelay(pdMS_TO_TICKS(1000)); /* Increased to 1 second */
    
    /* Make special first read attempt with extra retries if needed */
    ESP_LOGI(TAG, "Performing first sensor reading...");
    bme680_data_t data;
    bool first_read_success = false;
    
    /* Try up to 3 times for the first reading with increasing delays */
    for (int i = 0; i < 3 && !first_read_success; i++) {
        ret = bme680_read_data(&data);
        if (ret == ESP_OK) {
            first_read_success = true;
            ESP_LOGI(TAG, "First sensor reading successful on attempt %d", i + 1);
            ESP_LOGI(TAG, "Sensor Data:");
            ESP_LOGI(TAG, "  Temperature: %.2f °C", data.temperature);
            ESP_LOGI(TAG, "  Pressure: %.2f hPa", data.pressure);
            ESP_LOGI(TAG, "  Humidity: %.2f %%", data.humidity);
            ESP_LOGI(TAG, "  Gas Resistance: %.2f kOhm", data.gas_resistance / 1000.0f);
            ESP_LOGI(TAG, "  Gas Valid: %s", data.gas_valid ? "Yes" : "No");
            ESP_LOGI(TAG, "  Data Valid: %s", data.meas_valid ? "Yes" : "No");
        } else {
            ESP_LOGW(TAG, "First reading attempt %d failed: %s", i + 1, esp_err_to_name(ret));
            /* Use increasing delays between retries */
            vTaskDelay(pdMS_TO_TICKS(500 * (i + 1)));
        }
    }
    
    if (!first_read_success) {
        ESP_LOGW(TAG, "First reading failed after multiple attempts, continuing with regular readings");
    }
    
    /* Read and print sensor data in a loop */
    int consecutive_failures = 0;
    
    while (1) {
        ret = bme680_read_data(&data);
        
        if (ret == ESP_OK) {
            consecutive_failures = 0; /* Reset failure counter on success */
            
            ESP_LOGI(TAG, "Sensor Data:");
            ESP_LOGI(TAG, "  Temperature: %.2f °C", data.temperature);
            ESP_LOGI(TAG, "  Pressure: %.2f hPa", data.pressure);
            ESP_LOGI(TAG, "  Humidity: %.2f %%", data.humidity);
            ESP_LOGI(TAG, "  Gas Resistance: %.2f kOhm", data.gas_resistance / 1000.0f);
            ESP_LOGI(TAG, "  Gas Valid: %s", data.gas_valid ? "Yes" : "No");
            ESP_LOGI(TAG, "  Data Valid: %s", data.meas_valid ? "Yes" : "No");
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
            consecutive_failures++;
            
            /* If we have too many consecutive failures, try to recover */
            if (consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
                ESP_LOGW(TAG, "Multiple consecutive read failures, attempting recovery...");
                
                /* Re-initialize the sensor */
                bme680_deinit();
                vTaskDelay(pdMS_TO_TICKS(500));
                
                ret = bme680_init();
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Sensor recovery failed: %s", esp_err_to_name(ret));
                } else {
                    ESP_LOGI(TAG, "Sensor recovery successful");
                    consecutive_failures = 0;
                }
                
                /* Additional stabilization time after recovery */
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        
        /* Wait before next reading */
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
} 