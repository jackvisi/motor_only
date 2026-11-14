idf_component_register(
    SRCS
        "batteries.c"
        "batteries_gps_motor_2.c"
        "gvi_motor.c"
        "motor_only.c"
        "cangps.c"
        "components/drivers/cooling.c"

    INCLUDE_DIRS
        "."
        "components"

    REQUIRES
        ad7190
        driver
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        mqtt
        esp_driver_gpio
        esp_driver_spi
        esp_driver_ledc
)
