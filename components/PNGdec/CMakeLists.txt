# PNGdec (bitbank2) vendored as an IDF component.
# Standalone PNG decoder used by OpenStreetMap-esp32 (no LGFX dependency).
idf_component_register(
    SRCS
        "src/PNGdec.cpp"
        "src/adler32.c"
        "src/crc32.c"
        "src/infback.c"
        "src/inffast.c"
        "src/inflate.c"
        "src/inftrees.c"
        "src/zutil.c"
        "src/s3_simd_rgb565.S"
    INCLUDE_DIRS "src"
    REQUIRES espressif__arduino-esp32 espressif__esp-dsp)
