idf_component_register(SRCS "esp_hid_serial_bridge.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                    INCLUDE_DIRS "." "public_include"
                    REQUIRES nvs_flash esp_hid esp_tinyusb)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
