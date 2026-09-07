idf_component_register(
    SRCS "main.cpp" "hal_esp32.cpp" "boot_id.cpp" "tmf8821.cpp" "tof_firmware.cpp" "tof_array.cpp"
    INCLUDE_DIRS "." "../include"
    REQUIRES driver esp_driver_i2c esp_driver_gpio esp_timer
)
