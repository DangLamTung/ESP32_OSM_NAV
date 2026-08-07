# OpenStreetMap-esp32 (CelliesProjects) vendored as an IDF component.
# Uses Arduino WiFi/WiFiClientSecure for tile fetching, PNGdec for decode,
# LovyanGFX LGFX_Sprite for composing the map.
idf_component_register(
    SRCS
        "src/OpenStreetMap-esp32.cpp"
        "src/MemoryBuffer.cpp"
        "src/ReusableTileFetcher.cpp"
    INCLUDE_DIRS "src"
    REQUIRES espressif__arduino-esp32 LovyanGFX PNGdec TJpgDec)
