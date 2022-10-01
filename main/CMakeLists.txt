idf_component_register(SRCS "lora.c" "connect.c" "bme280_support.c" "bme280.c" "pms5003.c" "main.c" "bme680.c" "bme680_platform.c" "esp8266_wrapper.c"
                    INCLUDE_DIRS ".")
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
