cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# The app source lives in src/ (kept for PlatformIO compatibility). Register it
# as an ESP-IDF component so native idf.py builds find app_main.
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)

project(car_nav)

# Compiler optimization. GCC clamps -O4 to -O3 (there's no -O4). NOTE: enabling
# -O3 triggers a FULL ESP-IDF rebuild (~2h on this machine). Left disabled so
# incremental builds stay fast; uncomment + build once to apply it.
# idf_build_set_property(COMPILE_OPTIONS "-O3" APPEND)
