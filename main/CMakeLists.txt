idf_component_register(SRCS "esp_hid_serial_bridge.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_hid
		    PRIV_REQUIRES nvs_flash esp_hid app_update esp_driver_uart)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
