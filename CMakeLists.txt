idf_component_register(
    SRCS 
        "src/bme680.c" 
        "src/bme680_interface.c"
        "BME68x_SensorAPI/bme68x.c"
    INCLUDE_DIRS 
        "include"
        "BME68x_SensorAPI"
    REQUIRES 
        "driver" "esp_timer"
) 