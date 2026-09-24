idf_component_register(
    SRCS
        "main.c"
        "input.c"
        "zb_knob.c"
        "console_cmds.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES console nvs_flash esp_timer esp_driver_gpio
)
