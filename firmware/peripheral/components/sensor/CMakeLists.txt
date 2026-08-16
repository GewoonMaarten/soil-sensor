idf_component_register(
    SRCS "s_battery.c" "s_hdc1080.c" "s_i2c.c" "s_moisture.c" "sleep.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_adc_cal")
