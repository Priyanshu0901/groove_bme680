/**
 * @file bme680_interface.h
 * @brief BME680 sensor hardware interface for ESP-IDF
 * 
 * This file contains the interface definitions for the BME680 sensor
 * with ESP-IDF specific I2C and SPI implementations.
 * 
 * @copyright Copyright (c) 2024
 * @license MIT
 */

#ifndef _BME680_INTERFACE_H_
#define _BME680_INTERFACE_H_

#include "bme68x.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the interface for BME680 sensor
 * 
 * This function initializes the hardware interface (I2C or SPI)
 * for the BME680 sensor.
 * 
 * @param[in,out] dev Pointer to BME68x device structure
 * @return ESP_OK on success, or an error code from esp_err_t
 */
esp_err_t bme680_interface_init(struct bme68x_dev *dev);

/**
 * @brief Deinitialize the interface for BME680 sensor
 * 
 * This function deinitializes the hardware interface (I2C or SPI)
 * for the BME680 sensor.
 * 
 * @return ESP_OK on success, or an error code from esp_err_t
 */
esp_err_t bme680_interface_deinit(void);

/**
 * @brief I2C read function for BME680 sensor
 * 
 * This function reads data from BME680 registers using I2C interface.
 * 
 * @param[in] reg_addr Register address to read from
 * @param[out] reg_data Pointer to store read data
 * @param[in] len Length of data to read
 * @param[in] intf_ptr Interface pointer (containing device address)
 * @return BME68X_OK on success, or an error code
 */
int8_t bme680_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
 * @brief I2C write function for BME680 sensor
 * 
 * This function writes data to BME680 registers using I2C interface.
 * 
 * @param[in] reg_addr Register address to write to
 * @param[in] reg_data Pointer to data to write
 * @param[in] len Length of data to write
 * @param[in] intf_ptr Interface pointer (containing device address)
 * @return BME68X_OK on success, or an error code
 */
int8_t bme680_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
 * @brief SPI read function for BME680 sensor
 * 
 * This function reads data from BME680 registers using SPI interface.
 * 
 * @param[in] reg_addr Register address to read from
 * @param[out] reg_data Pointer to store read data
 * @param[in] len Length of data to read
 * @param[in] intf_ptr Interface pointer (containing CS pin number)
 * @return BME68X_OK on success, or an error code
 */
int8_t bme680_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
 * @brief SPI write function for BME680 sensor
 * 
 * This function writes data to BME680 registers using SPI interface.
 * 
 * @param[in] reg_addr Register address to write to
 * @param[in] reg_data Pointer to data to write
 * @param[in] len Length of data to write
 * @param[in] intf_ptr Interface pointer (containing CS pin number)
 * @return BME68X_OK on success, or an error code
 */
int8_t bme680_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
 * @brief Delay function for BME680 sensor
 * 
 * This function provides a delay for the BME680 sensor operations.
 * 
 * @param[in] period Delay period in milliseconds
 * @param[in] intf_ptr Interface pointer (unused in ESP-IDF implementation)
 */
void bme680_delay_us(uint32_t period, void *intf_ptr);

/**
 * @brief Get timestamp in milliseconds
 * 
 * @return Current timestamp in milliseconds
 */
uint32_t bme680_get_timestamp_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* _BME680_INTERFACE_H_ */ 