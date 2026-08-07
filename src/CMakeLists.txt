# osm_idf app — Arduino OSM tile viewer compiled under ESP-IDF 5.5.1
# via the espressif/arduino-esp32 component (the Arduino OSM library stack:
# OpenStreetMap-esp32 + LovyanGFX + PNGdec + Arduino BLE).
#
# Module folders:
#   display/  - ILI9341 + FT6336 init + low-power helpers
#   map/      - map state, tile fetch, pan/zoom, tile-source mode
#   ui/       - corner buttons, BLE scan panel, nav overlay
#   input/    - FT6336 gestures (tap / drag / long-press-to-sleep)
#   power/    - deep sleep, wake on touch
#   ble/      - nav GATT server + scanner
#   sd/       - SDMMC card mount + USB reader mode
idf_component_register(
    SRCS
        "main.cpp"
        "display/display_panel.cpp"
        "map/map_view.cpp"
        "ui/ui_controls.cpp"
        "input/input_touch.cpp"
        "power/power_mgr.cpp"
        "ble/ble_nav.cpp"
        "ble/ble_scan.cpp"
        "sd/sd_card.c"
        "sd/sd_upload.c"
    INCLUDE_DIRS
        "."
        "display"
        "map"
        "ui"
        "input"
        "power"
        "ble"
        "sd"
    REQUIRES
        espressif__arduino-esp32
        LovyanGFX
        OpenStreetMap-esp32
        PNGdec
        fatfs
        sdmmc
        esp_driver_sdmmc
        esp_driver_usb_serial_jtag
        esp_driver_gpio
        esp_wifi
        esp_hw_support
        esp_system)
