idf_component_register(SRCS "esp_hid_serial_bridge.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                    INCLUDE_DIRS "." "public_include"
                    REQUIRES nvs_flash esp_hid app_update esp_driver_uart tinyusb usb)

idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_include_directories(${tusb_lib} PRIVATE public_include)

if(NOT bcd_device)
    set(bcd_device "0x0001")
endif()
message(STATUS "bcdDevice value: ${bcd_device}")

target_compile_definitions(${COMPONENT_LIB} PRIVATE BCDDEVICE=${bcd_device})


target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
