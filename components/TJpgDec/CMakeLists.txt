# TJpgDec (ChaN) — Tiny JPEG Decompressor, vendored from the vendor's
# TJpg_Decoder Arduino library (src/tjpgd.c, src/tjpgd.h, src/tjpgdcnf.h).
# Software JPEG decode for the ESP32-S3 (which has NO hardware JPEG codec).
# Config in tjpgdcnf.h: JD_FORMAT=1 (RGB565), JD_FASTDECODE=1.
idf_component_register(
    SRCS "tjpgd.c"
    INCLUDE_DIRS ".")
