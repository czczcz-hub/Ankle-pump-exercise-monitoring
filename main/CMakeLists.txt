idf_component_register(SRCS "shared_data.c" "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES BSP filter ble json)
