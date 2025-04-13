/**
 * @file bme680.c
 * @brief BME680 sensor driver implementation for ESP-IDF
 *
 * This file contains the ESP-IDF component implementation for the BME680 sensor.
 *
 * @copyright Copyright (c) 2024
 * @license MIT
 */

#include "bme680.h"
#include "bme680_interface.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>

/* Logging tag */
static const char* TAG = "BME680";

/* Global device structure */
static struct bme68x_dev dev;
static bool is_initialized = false;

esp_err_t bme680_init(void)
{
    esp_err_t ret;

    /* If already initialized, just return success */
    if (is_initialized) {
        ESP_LOGW(TAG, "BME680 already initialized");
        return ESP_OK;
    }

    /* Initialize device structure */
    memset(&dev, 0, sizeof(struct bme68x_dev));

    /* Initialize hardware interface */
    ret = bme680_interface_init(&dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize interface: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize the BME68x sensor */
    int8_t rslt = bme68x_init(&dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to initialize BME680 sensor: %d", rslt);
        bme680_interface_deinit();
        return ESP_FAIL;
    }
    
    /* Set initialized flag before configuring to avoid invalid state error */
    is_initialized = true;
    
    /* Configure the sensor with default settings */
    ret = bme680_configure();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BME680 sensor: %s", esp_err_to_name(ret));
        is_initialized = false;
        bme680_interface_deinit();
        return ret;
    }

    ESP_LOGI(TAG, "BME680 initialized successfully");
    return ESP_OK;
}

struct bme68x_dev* bme680_get_device(void)
{
    if (!is_initialized) {
        return NULL;
    }
    
    return &dev;
}

esp_err_t bme680_configure(void)
{
    if (!is_initialized) {
        ESP_LOGE(TAG, "BME680 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Configure the sensor settings */
    struct bme68x_conf conf;
    int8_t rslt = bme68x_get_conf(&conf, &dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to get sensor configuration: %d", rslt);
        return ESP_FAIL;
    }

    /* Set temperature oversampling */
#if defined(CONFIG_BME680_TEMPERATURE_OVERSAMPLING_NONE)
    conf.os_temp = BME68X_OS_NONE;
#elif defined(CONFIG_BME680_TEMPERATURE_OVERSAMPLING_1X)
    conf.os_temp = BME68X_OS_1X;
#elif defined(CONFIG_BME680_TEMPERATURE_OVERSAMPLING_2X)
    conf.os_temp = BME68X_OS_2X;
#elif defined(CONFIG_BME680_TEMPERATURE_OVERSAMPLING_4X)
    conf.os_temp = BME68X_OS_4X;
#elif defined(CONFIG_BME680_TEMPERATURE_OVERSAMPLING_8X)
    conf.os_temp = BME68X_OS_8X;
#elif defined(CONFIG_BME680_TEMPERATURE_OVERSAMPLING_16X)
    conf.os_temp = BME68X_OS_16X;
#else
    conf.os_temp = BME68X_OS_2X;
#endif

    /* Set pressure oversampling */
#if defined(CONFIG_BME680_PRESSURE_OVERSAMPLING_NONE)
    conf.os_pres = BME68X_OS_NONE;
#elif defined(CONFIG_BME680_PRESSURE_OVERSAMPLING_1X)
    conf.os_pres = BME68X_OS_1X;
#elif defined(CONFIG_BME680_PRESSURE_OVERSAMPLING_2X)
    conf.os_pres = BME68X_OS_2X;
#elif defined(CONFIG_BME680_PRESSURE_OVERSAMPLING_4X)
    conf.os_pres = BME68X_OS_4X;
#elif defined(CONFIG_BME680_PRESSURE_OVERSAMPLING_8X)
    conf.os_pres = BME68X_OS_8X;
#elif defined(CONFIG_BME680_PRESSURE_OVERSAMPLING_16X)
    conf.os_pres = BME68X_OS_16X;
#else
    conf.os_pres = BME68X_OS_4X;
#endif

    /* Set humidity oversampling */
#if defined(CONFIG_BME680_HUMIDITY_OVERSAMPLING_NONE)
    conf.os_hum = BME68X_OS_NONE;
#elif defined(CONFIG_BME680_HUMIDITY_OVERSAMPLING_1X)
    conf.os_hum = BME68X_OS_1X;
#elif defined(CONFIG_BME680_HUMIDITY_OVERSAMPLING_2X)
    conf.os_hum = BME68X_OS_2X;
#elif defined(CONFIG_BME680_HUMIDITY_OVERSAMPLING_4X)
    conf.os_hum = BME68X_OS_4X;
#elif defined(CONFIG_BME680_HUMIDITY_OVERSAMPLING_8X)
    conf.os_hum = BME68X_OS_8X;
#elif defined(CONFIG_BME680_HUMIDITY_OVERSAMPLING_16X)
    conf.os_hum = BME68X_OS_16X;
#else
    conf.os_hum = BME68X_OS_1X;
#endif

    /* Set IIR filter size */
    conf.filter = CONFIG_BME680_FILTER_SIZE;
    
    /* Set ODR to none (no standby time in forced mode) */
    conf.odr = BME68X_ODR_NONE;

    /* Set the configuration */
    rslt = bme68x_set_conf(&conf, &dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set sensor configuration: %d", rslt);
        return ESP_FAIL;
    }

    /* Configure gas heater */
    struct bme68x_heatr_conf heatr_conf;
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = CONFIG_BME680_GAS_HEATER_TEMP;
    heatr_conf.heatr_dur = CONFIG_BME680_GAS_HEATER_DURATION;

    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set heater configuration: %d", rslt);
        return ESP_FAIL;
    }

    /* Set to sleep mode by default */
    rslt = bme68x_set_op_mode(BME68X_SLEEP_MODE, &dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set sleep mode: %d", rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BME680 configured successfully");
    return ESP_OK;
}

esp_err_t bme680_set_mode(uint8_t mode)
{
    if (!is_initialized) {
        ESP_LOGE(TAG, "BME680 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int8_t rslt = bme68x_set_op_mode(mode, &dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set operation mode: %d", rslt);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bme680_get_measurement_duration(uint32_t *duration_ms)
{
    if (!is_initialized || duration_ms == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    struct bme68x_conf conf;
    int8_t rslt = bme68x_get_conf(&conf, &dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to get sensor configuration: %d", rslt);
        return ESP_FAIL;
    }

    /* Get measurement duration in microseconds */
    uint32_t duration_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &dev);
    
    /* Add 10ms for gas measurement */
    *duration_ms = (duration_us / 1000) + 10;

    return ESP_OK;
}

esp_err_t bme680_read_data(bme680_data_t *data)
{
    if (!is_initialized || data == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    /* Set to forced mode to trigger a measurement */
    int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set forced mode: %d", rslt);
        return ESP_FAIL;
    }

    /* Get measurement duration */
    uint32_t duration_ms;
    esp_err_t ret = bme680_get_measurement_duration(&duration_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get measurement duration: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Increase wait time to 3x the calculated duration plus a fixed offset */
    duration_ms = (duration_ms * 3) + 30;
    
    /* Wait for measurement to complete */
    dev.delay_us(duration_ms * 1000, NULL);

    /* Get data with retry mechanism */
    struct bme68x_data sensor_data[3];
    uint8_t n_data = 0;
    int max_retries = 3;
    int retry_count = 0;
    
    do {
        rslt = bme68x_get_data(BME68X_FORCED_MODE, sensor_data, &n_data, &dev);
        
        /* If we got no new data and haven't exceeded retries, wait and try again */
        if (rslt == BME68X_W_NO_NEW_DATA && retry_count < max_retries) {
            ESP_LOGW(TAG, "No new data available (retry %d/%d), waiting...", 
                     retry_count + 1, max_retries);
            dev.delay_us(50000, NULL); /* Wait 50ms before retry */
            retry_count++;
        } else {
            break; /* Either success or different error */
        }
    } while (retry_count <= max_retries);
    
    /* Handle errors after retry attempts */
    if (rslt != BME68X_OK && rslt != BME68X_W_NO_NEW_DATA) {
        ESP_LOGE(TAG, "Failed to get sensor data: %d", rslt);
        return ESP_FAIL;
    }
    
    /* If we still have no data after retries, return error */
    if (n_data == 0) {
        if (rslt == BME68X_W_NO_NEW_DATA) {
            ESP_LOGE(TAG, "No new data available after retries");
        } else {
            ESP_LOGE(TAG, "No sensor data available");
        }
        return ESP_ERR_NOT_FOUND;
    }

    /* Fill out the data structure */
    data->temperature = sensor_data[0].temperature;
    data->pressure = sensor_data[0].pressure / 100.0f; /* Convert Pa to hPa */
    data->humidity = sensor_data[0].humidity;
    data->gas_resistance = sensor_data[0].gas_resistance;
    data->gas_valid = (sensor_data[0].status & BME68X_GASM_VALID_MSK) != 0;
    data->meas_valid = (sensor_data[0].status & BME68X_NEW_DATA_MSK) != 0;
    data->timestamp = bme680_get_timestamp_ms();

    ESP_LOGD(TAG, "Read data: T=%.2f°C, P=%.2fhPa, H=%.2f%%, G=%.2fkOhm", 
        data->temperature, data->pressure, data->humidity, data->gas_resistance / 1000.0f);

    return ESP_OK;
}

esp_err_t bme680_deinit(void)
{
    if (!is_initialized) {
        return ESP_OK; /* Already deinitialized */
    }

    /* Set to sleep mode */
    int8_t rslt = bme68x_set_op_mode(BME68X_SLEEP_MODE, &dev);
    if (rslt != BME68X_OK) {
        ESP_LOGW(TAG, "Failed to set sleep mode during deinitialization: %d", rslt);
        /* Continue with deinitialization anyway */
    }

    /* Deinitialize interface */
    esp_err_t ret = bme680_interface_deinit();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to deinitialize interface: %s", esp_err_to_name(ret));
        /* Continue with deinitialization anyway */
    }

    is_initialized = false;
    ESP_LOGI(TAG, "BME680 deinitialized");
    return ESP_OK;
} 