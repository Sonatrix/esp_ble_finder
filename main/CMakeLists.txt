idf_component_register(
    SRCS "main.c" "clicker.c"
    INCLUDE_DIRS "."
    REQUIRES esp32_s3_touch_amoled_1_8 lvgl__lvgl bt nvs_flash esp_timer driver esp_codec_dev esp_io_expander
)
