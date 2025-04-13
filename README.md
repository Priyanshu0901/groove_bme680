# BME680 Sensor Component for ESP-IDF

This is an ESP-IDF component for the Bosch BME680 environmental sensor, which measures temperature, pressure, humidity, and gas resistance. The component provides a simple API to initialize, configure, and read data from the sensor.

## Features

- Support for both I2C and SPI interfaces
- Configurable oversampling settings for temperature, pressure, and humidity
- Configurable IIR filter settings
- Configurable gas heater temperature and duration
- Simple API for reading sensor data
- Robust error handling and recovery mechanisms

## Dependencies

This component depends on the following ESP-IDF components:
- `driver` (for I2C and SPI drivers)

It also includes the [Bosch BME68x-Sensor-API](https://github.com/BoschSensortec/BME68x-Sensor-API) as a Git submodule.

## Installation

To use this component in your ESP-IDF project:

1. Add the component to your project's `components` directory:

```bash
cd your_project
git clone --recursive git@github.com:Priyanshu0901/grove_bme680.git components/bme680
```

If you've already cloned the repository without the `--recursive` flag, you can initialize the submodule with:

```bash
cd components/bme680
git submodule update --init --recursive
```

2. Configure the component using `menuconfig`:

```bash
idf.py menuconfig
```

Navigate to "BME680 Sensor Configuration" to configure the sensor settings.

## Configuration

The component can be configured through menuconfig. The following options are available:

- **Interface selection**: Choose between I2C and SPI interfaces
- **I2C/SPI configuration**: Configure pins and bus parameters
- **Oversampling settings**: Configure oversampling for temperature, pressure, and humidity
- **Filter settings**: Configure the IIR filter size
- **Gas heater settings**: Configure the gas heater temperature and duration

## Usage

Here's a simple example of how to use the component:

```c
#include "bme680.h"

void app_main(void)
{
    // Initialize BME680 sensor
    esp_err_t ret = bme680_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BME680 sensor");
        return;
    }
    
    // Wait for sensor to stabilize after initialization
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Read sensor data
    bme680_data_t data;
    ret = bme680_read_data(&data);
    if (ret == ESP_OK) {
        printf("Temperature: %.2f °C\n", data.temperature);
        printf("Pressure: %.2f hPa\n", data.pressure);
        printf("Humidity: %.2f %%\n", data.humidity);
        printf("Gas Resistance: %.2f kOhm\n", data.gas_resistance / 1000.0f);
    }
    
    // Deinitialize when done
    bme680_deinit();
}
```

For a more complete example, see the `examples/bme680_example.c` file.

## Troubleshooting

### Common Issues

1. **No new data available error**: The BME680 sensor needs some time to stabilize after initialization and between measurements. Make sure to add appropriate delays after initialization (at least 1 second) and ensure the read operation waits for the measurement to complete.

2. **Communication failures**: Check your wiring and make sure the I2C/SPI bus is properly set up. The default I2C address is 0x76.

3. **First reading fails, subsequent readings succeed**: This is a common behavior with the BME680 sensor. The example code includes retries for the first reading.

## API Reference

### Initialization and Deinitialization

```c
esp_err_t bme680_init(void);
esp_err_t bme680_deinit(void);
```

### Configuration

```c
esp_err_t bme680_configure(void);
esp_err_t bme680_set_mode(uint8_t mode);
```

### Data Reading

```c
esp_err_t bme680_read_data(bme680_data_t *data);
esp_err_t bme680_get_measurement_duration(uint32_t *duration_ms);
```

### Advanced Access

```c
struct bme68x_dev* bme680_get_device(void);
```

## Data Structure

The `bme680_data_t` structure contains the following fields:

```c
typedef struct {
    float temperature;      // Temperature in degrees Celsius
    float pressure;         // Pressure in hPa (hectopascal)
    float humidity;         // Humidity in %RH (relative humidity percentage)
    float gas_resistance;   // Gas resistance in Ohms
    bool gas_valid;         // Flag indicating if gas measurement is valid
    bool meas_valid;        // Flag indicating if T/P/H measurements are valid
    uint32_t timestamp;     // Timestamp of the measurement (ms)
} bme680_data_t;
```

## License

This component is licensed under the MIT License.

## Acknowledgements

This component uses the [Bosch BME68x-Sensor-API](https://github.com/BoschSensortec/BME68x-Sensor-API) for the low-level sensor functionality. 
