# CMakeLists for ESP-IDF

set(COMPONENT_ADD_INCLUDEDIRS
    ${LGFX_ROOT}/src
    )
file(GLOB SRCS
     ${LGFX_ROOT}/src/lgfx/Fonts/efont/*.c
     ${LGFX_ROOT}/src/lgfx/Fonts/IPA/*.c
     ${LGFX_ROOT}/src/lgfx/utility/*.c
     ${LGFX_ROOT}/src/lgfx/v1/*.cpp
     ${LGFX_ROOT}/src/lgfx/v1/misc/*.cpp
     ${LGFX_ROOT}/src/lgfx/v1/panel/*.cpp
     ${LGFX_ROOT}/src/lgfx/v1/platforms/arduino_default/*.cpp
     ${LGFX_ROOT}/src/lgfx/v1/platforms/esp32/*.cpp
     ${LGFX_ROOT}/src/lgfx/v1/platforms/esp32c3/*.cpp
     ${LGFX_ROOT}/src/lgfx/v1/platforms/esp32s2/*.cpp
     ${LGFX_ROOT}/src/lgfx/v1/platforms/esp32s3/*.cpp
     ${LGFX_ROOT}/src/lgfx/v1/touch/*.cpp
     )

set(COMPONENT_SRCS ${SRCS})

# Not used on this board (SPI display, no RGB565 parallel bus) and its
# gpio_hal_func_sel() call does not match ESP-IDF 5.5.1's 3-arg signature.
list(REMOVE_ITEM COMPONENT_SRCS ${LGFX_ROOT}/src/lgfx/v1/platforms/esp32s3/Bus_RGB.cpp)

if (IDF_VERSION_MAJOR GREATER_EQUAL 5)
    set(COMPONENT_REQUIRES nvs_flash efuse esp_lcd driver esp_timer)
elseif ((IDF_VERSION_MAJOR EQUAL 4) AND (IDF_VERSION_MINOR GREATER 3) OR IDF_VERSION_MAJOR GREATER 4)
    set(COMPONENT_REQUIRES nvs_flash efuse esp_lcd)
else()
    set(COMPONENT_REQUIRES nvs_flash efuse)
endif()


### If you use arduino-esp32 components, please activate next comment line.
# CRITICAL FIX: we run inside a project that also compiles app code with the
# arduino-esp32 managed component. arduino-esp32 defines ARDUINO/ESP32 as
# PUBLIC compile options that only propagate to components listing it in
# REQUIRES. Without this, LovyanGFX is compiled with a DIFFERENT class layout
# than main.cpp (LovyanGFX headers branch on ESP32/ARDUINO macros) -> ODR
# violation -> _panel lands at different offsets -> NULL panel -> crash/black
# screen. So we MUST require the arduino component here too.
list(APPEND COMPONENT_REQUIRES espressif__arduino-esp32)


message(STATUS "LovyanGFX use components = ${COMPONENT_REQUIRES}")

register_component()
