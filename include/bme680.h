/**
 * @file bme680.h
 * @brief BME680 sensor driver interface for ESP-IDF
 * 
 * ESP-IDF component for Bosch BME680 environmental sensor providing
 * temperature, pressure, humidity and gas resistance measurements.
 * 
 * @copyright Copyright (c) 2024
 * @license MIT
 */

#ifndef _BME680_ESP_H_
#define _BME680_ESP_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "bme68x.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sensor data structure
 */
typedef struct {
    float temperature;      /*!< Temperature in degrees Celsius */
    float pressure;         /*!< Pressure in hPa (hectopascal) */
    float humidity;         /*!< Humidity in %RH (relative humidity percentage) */
    float gas_resistance;   /*!< Gas resistance in Ohms */
    bool gas_valid;         /*!< Flag indicating if gas measurement is valid */
    bool meas_valid;        /*!< Flag indicating if T/P/H measurements are valid */
    uint32_t timestamp;     /*!< Timestamp of the measurement (ms) */
} bme680_data_t;

/**
 * @brief Initialize the BME680 sensor
 * 
 * This function initializes the BME680 sensor with the default configuration
 * based on Kconfig settings. It must be called before any other functions.
 * 
 * @return ESP_OK on success, or an error code from esp_err_t
 */
esp_err_t bme680_init(void);

/**
 * @brief Get the BME68x device handle
 * 
 * This function returns a pointer to the initialized BME68x device structure.
 * It can be used for direct access to the BME68x API if needed.
 * 
 * @return Pointer to the BME68x device structure, or NULL if not initialized
 */
struct bme68x_dev* bme680_get_device(void);

/**
 * @brief Configure the BME680 sensor
 * 
 * This function configures the BME680 sensor based on Kconfig settings.
 * 
 * @return ESP_OK on success, or an error code from esp_err_t
 */
esp_err_t bme680_configure(void);

/**
 * @brief Set the mode of operation for the BME680 sensor
 * 
 * This function sets the mode of operation for the BME680 sensor.
 * 
 * @param[in] mode Mode of operation (BME68X_SLEEP_MODE, BME68X_FORCED_MODE, etc.)
 * @return ESP_OK on success, or an error code from esp_err_t
 */
esp_err_t bme680_set_mode(uint8_t mode);

/**
 * @brief Read data from the BME680 sensor
 * 
 * This function reads the most recent measurement data from the BME680 sensor.
 * In forced mode, it triggers a new measurement and waits for it to complete.
 * 
 * @param[out] data Pointer to data structure to fill
 * @return ESP_OK on success, or an error code from esp_err_t
 */
esp_err_t bme680_read_data(bme680_data_t *data);

/**
 * @brief Get the time required to perform a measurement with current settings
 * 
 * This function calculates the time required to perform a measurement
 * with the current configuration settings.
 * 
 * @param[out] duration_ms Pointer to store the duration in milliseconds
 * @return ESP_OK on success, or an error code from esp_err_t
 */
esp_err_t bme680_get_measurement_duration(uint32_t *duration_ms);

/**
 * @brief Deinitialize the BME680 sensor
 * 
 * This function deinitializes the BME680 sensor and frees any resources used.
 * 
 * @return ESP_OK on success, or an error code from esp_err_t
 */
esp_err_t bme680_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* _BME680_ESP_H_ */ 