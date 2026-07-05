idf_component_register(SRCS "batteries.c" "batteries_gps_motor_2.c" "gvi_motor.c" "motor_only.c" "cangps.c" "components/ad7124/ad7190.c" "components/drivers/cooling.c" "components/ad7124/ad7190_driver.c" 
    INCLUDE_DIRS "." "components"
                       REQUIRES driver nvs_flash esp_wifi esp_event esp_netif mqtt)