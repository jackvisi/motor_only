#Cmake ad7190
idf_component_register(SRCS "ad7190_driver.c" "ad7190.c" "cooling.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver fatfs esp_adc
                       WHOLE_ARCHIVE)
