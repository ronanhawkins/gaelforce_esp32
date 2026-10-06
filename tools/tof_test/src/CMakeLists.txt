# The driver is compiled from the pod's tree, not copied, so this tests the
# exact bring-up code the pod runs.
idf_component_register(
    SRCS "main.cpp" "../../../src/tmf8821.cpp" "../../../src/tof_firmware.cpp"
    INCLUDE_DIRS "." "../../../include"
    REQUIRES driver esp_driver_i2c esp_driver_gpio esp_timer
)
