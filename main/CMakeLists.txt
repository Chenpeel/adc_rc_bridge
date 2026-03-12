idf_component_register(
    SRCS
        "main.cpp"
        "adc_mux_sampler.cpp"
        "bridge_protocol.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_timer esp_adc driver esp_driver_gpio esp_driver_i2c
)
