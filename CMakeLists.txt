set(include_dirs
    "src"
)

set(srcs
    "src/hulp.c"
    "src/hulp_touch.c"
    "src/hulp_uart.c"
    "src/hulp_regwr.c"
    "src/hulp_debug.c"
)

set(requires
    ulp
    driver
    soc
    hal
    esp_adc
)

set(priv_requires
    esp_adc
)

idf_component_register(
    INCLUDE_DIRS "${include_dirs}"
    SRCS "${srcs}"
    REQUIRES "${requires}"
    PRIV_REQUIRES "${priv_requires}"
)
