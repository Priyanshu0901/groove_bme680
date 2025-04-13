/**
 * @file bme680_interface.c
 * @brief BME680 sensor hardware interface implementation for ESP-IDF
 *
 * This file contains the ESP-IDF specific interface implementations for
 * the BME680 sensor (I2C and SPI).
 *
 * @copyright Copyright (c) 2024
 * @license MIT
 */

#include "bme680_interface.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>

/* Logging tag */
static const char* TAG = "BME680_IF";

/* I2C timeout in milliseconds */
#define BME680_I2C_TIMEOUT_MS 100

/* Interface handle variables */
#if defined(CONFIG_BME680_INTERFACE_I2C)
static i2c_master_dev_handle_t i2c_dev = NULL;
static uint8_t i2c_dev_addr;
#elif defined(CONFIG_BME680_INTERFACE_SPI)
static spi_device_handle_t spi_dev = NULL;
#endif

/**
 * @brief Initialize I2C interface
 * 
 * @return ESP_OK on success, or an error code from esp_err_t
 */
static esp_err_t bme680_i2c_init(void)
{
#if defined(CONFIG_BME680_INTERFACE_I2C)
    esp_err_t ret;

    /* Configure I2C bus */
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = CONFIG_BME680_I2C_PORT,
        .scl_io_num = CONFIG_BME680_I2C_SCL_PIN,
        .sda_io_num = CONFIG_BME680_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    /* Create I2C master bus */
    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure I2C device */
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_BME680_I2C_ADDR,
        .scl_speed_hz = CONFIG_BME680_I2C_CLOCK_SPEED,
    };

    /* Add device to I2C bus */
    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(ret));
        i2c_del_master_bus(bus_handle);
        return ret;
    }

    i2c_dev_addr = CONFIG_BME680_I2C_ADDR;
    ESP_LOGI(TAG, "I2C interface initialized with address: 0x%02X", i2c_dev_addr);
    return ESP_OK;
#else
    ESP_LOGW(TAG, "I2C interface not enabled in config");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

/**
 * @brief Initialize SPI interface
 * 
 * @return ESP_OK on success, or an error code from esp_err_t
 */
static esp_err_t bme680_spi_init(void)
{
#if defined(CONFIG_BME680_INTERFACE_SPI)
    esp_err_t ret;

    /* Configure SPI bus */
    spi_bus_config_t bus_config = {
        .miso_io_num = CONFIG_BME680_SPI_MISO_PIN,
        .mosi_io_num = CONFIG_BME680_SPI_MOSI_PIN,
        .sclk_io_num = CONFIG_BME680_SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,  /* Max BME680 transaction size */
    };

    /* Initialize SPI bus */
    ret = spi_bus_initialize(CONFIG_BME680_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure SPI device */
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = CONFIG_BME680_SPI_CLOCK_SPEED,
        .mode = 0,                /* BME680 works in mode 0 (CPOL=0, CPHA=0) */
        .spics_io_num = CONFIG_BME680_SPI_CS_PIN,
        .queue_size = 7,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .cs_ena_pretrans = 5,     /* Assert CS 5 cycles before transaction */
        .cs_ena_posttrans = 5,    /* Keep CS active 5 cycles after transaction */
        .flags = 0,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    /* Add device to SPI bus */
    ret = spi_bus_add_device(CONFIG_BME680_SPI_HOST, &dev_config, &spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to SPI bus: %s", esp_err_to_name(ret));
        spi_bus_free(CONFIG_BME680_SPI_HOST);
        return ret;
    }

    ESP_LOGI(TAG, "SPI interface initialized with CS pin: %d", CONFIG_BME680_SPI_CS_PIN);
    return ESP_OK;
#else
    ESP_LOGW(TAG, "SPI interface not enabled in config");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t bme680_interface_init(struct bme68x_dev *dev)
{
    esp_err_t ret;

    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

#if defined(CONFIG_BME680_INTERFACE_I2C)
    /* Initialize I2C interface */
    ret = bme680_i2c_init();
    if (ret != ESP_OK) {
        return ret;
    }

    /* Set up device structure for I2C */
    dev->intf = BME68X_I2C_INTF;
    dev->read = bme680_i2c_read;
    dev->write = bme680_i2c_write;
    dev->intf_ptr = &i2c_dev_addr;
    
#elif defined(CONFIG_BME680_INTERFACE_SPI)
    /* Initialize SPI interface */
    ret = bme680_spi_init();
    if (ret != ESP_OK) {
        return ret;
    }

    /* Set up device structure for SPI */
    dev->intf = BME68X_SPI_INTF;
    dev->read = bme680_spi_read;
    dev->write = bme680_spi_write;
    uint8_t cs_pin = CONFIG_BME680_SPI_CS_PIN;
    dev->intf_ptr = &cs_pin;
    
#else
    #error "No BME680 interface selected in Kconfig. Please select either I2C or SPI."
#endif

    /* Set common device parameters */
    dev->delay_us = bme680_delay_us;
    dev->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */

    ESP_LOGI(TAG, "BME680 interface initialized successfully");
    return ESP_OK;
}

esp_err_t bme680_interface_deinit(void)
{
#if defined(CONFIG_BME680_INTERFACE_I2C)
    if (i2c_dev != NULL) {
        esp_err_t ret;
        i2c_master_bus_handle_t bus_handle;
        
        /* Remove device from bus */
        ret = i2c_master_bus_rm_device(i2c_dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error removing I2C device: %s", esp_err_to_name(ret));
        }

        /* Get bus handle and delete it */
        ret = i2c_master_get_bus_handle(CONFIG_BME680_I2C_PORT, &bus_handle);
        if (ret == ESP_OK && bus_handle != NULL) {
            i2c_del_master_bus(bus_handle);
        }

        i2c_dev = NULL;
        ESP_LOGI(TAG, "I2C interface deinitialized");
    }
    return ESP_OK;
#elif defined(CONFIG_BME680_INTERFACE_SPI)
    if (spi_dev != NULL) {
        /* Remove device from bus */
        spi_bus_remove_device(spi_dev);
        
        /* Free the bus */
        spi_bus_free(CONFIG_BME680_SPI_HOST);
        
        spi_dev = NULL;
        ESP_LOGI(TAG, "SPI interface deinitialized");
    }
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

int8_t bme680_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
#if defined(CONFIG_BME680_INTERFACE_I2C)
    esp_err_t ret;
    
    /* First write the register address */
    ret = i2c_master_transmit(i2c_dev, &reg_addr, 1, BME680_I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write address failed: %s", esp_err_to_name(ret));
        return BME68X_E_COM_FAIL;
    }
    
    /* Then read the data */
    ret = i2c_master_receive(i2c_dev, reg_data, len, BME680_I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read data failed: %s", esp_err_to_name(ret));
        return BME68X_E_COM_FAIL;
    }
    
    return BME68X_OK;
#else
    return BME68X_E_COM_FAIL;
#endif
}

int8_t bme680_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
#if defined(CONFIG_BME680_INTERFACE_I2C)
    esp_err_t ret;
    uint8_t *write_buf = malloc(len + 1);
    
    if (write_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for I2C write buffer");
        return BME68X_E_COM_FAIL;
    }
    
    /* Prepare write buffer with register address as first byte */
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, reg_data, len);

    /* Write data to device */
    ret = i2c_master_transmit(i2c_dev, write_buf, len + 1, BME680_I2C_TIMEOUT_MS);
    
    free(write_buf);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
        return BME68X_E_COM_FAIL;
    }

    return BME68X_OK;
#else
    return BME68X_E_COM_FAIL;
#endif
}

int8_t bme680_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
#if defined(CONFIG_BME680_INTERFACE_SPI)
    esp_err_t ret;
    
    /* BME680 requires the MSB of the register address to be set for read operations */
    uint8_t tx_addr = reg_addr | 0x80;
    uint8_t *tx_buffer = malloc(len + 1);
    uint8_t *rx_buffer = malloc(len + 1);
    
    if (tx_buffer == NULL || rx_buffer == NULL) {
        if (tx_buffer) free(tx_buffer);
        if (rx_buffer) free(rx_buffer);
        ESP_LOGE(TAG, "Failed to allocate memory for SPI buffers");
        return BME68X_E_COM_FAIL;
    }
    
    /* Fill TX buffer: first byte is address, rest are zeros for read operation */
    tx_buffer[0] = tx_addr;
    memset(tx_buffer + 1, 0, len);
    
    /* Create transaction */
    spi_transaction_t t = {
        .length = 8 * (len + 1),  /* bits */
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer
    };
    
    /* Execute transaction */
    ret = spi_device_transmit(spi_dev, &t);
    
    /* Copy result data (skip first byte which was address) */
    if (ret == ESP_OK) {
        memcpy(reg_data, rx_buffer + 1, len);
    }
    
    free(tx_buffer);
    free(rx_buffer);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read transaction failed: %s", esp_err_to_name(ret));
        return BME68X_E_COM_FAIL;
    }
    
    return BME68X_OK;
#else
    return BME68X_E_COM_FAIL;
#endif
}

int8_t bme680_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
#if defined(CONFIG_BME680_INTERFACE_SPI)
    esp_err_t ret;
    
    /* For BME680, clear MSB (0x7F) for write operations */
    uint8_t *tx_buffer = malloc(len + 1);
    
    if (tx_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for SPI write buffer");
        return BME68X_E_COM_FAIL;
    }
    
    /* First byte is register address (with MSB cleared for write operation) */
    tx_buffer[0] = reg_addr & 0x7F;
    memcpy(tx_buffer + 1, reg_data, len);
    
    /* Create transaction */
    spi_transaction_t t = {
        .length = 8 * (len + 1),  /* bits */
        .tx_buffer = tx_buffer,
        .rx_buffer = NULL
    };
    
    /* Execute transaction */
    ret = spi_device_transmit(spi_dev, &t);
    
    free(tx_buffer);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
        return BME68X_E_COM_FAIL;
    }
    
    return BME68X_OK;
#else
    return BME68X_E_COM_FAIL;
#endif
}

void bme680_delay_us(uint32_t period, void *intf_ptr)
{
    /* Convert microseconds to milliseconds for the FreeRTOS delay */
    if (period >= 1000) {
        vTaskDelay(pdMS_TO_TICKS(period / 1000));
    } else {
        /* For short delays use busy-wait */
        uint64_t start = esp_timer_get_time();
        while((esp_timer_get_time() - start) < period) {
            /* Busy wait */
        }
    }
}

uint32_t bme680_get_timestamp_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
} 