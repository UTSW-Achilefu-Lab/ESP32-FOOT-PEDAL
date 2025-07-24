set(srcs "footpedal_main.c"
         "esp_hid_gap.c")
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid esp_driver_gpio
                       PRIV_REQUIRES usb nvs_flash)
