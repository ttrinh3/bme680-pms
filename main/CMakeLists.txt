idf_component_register(SRCS "connect.c" "bme280_support.c" "bme280.c" "pms5003.c" "main.c" 
                    INCLUDE_DIRS ".")
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
