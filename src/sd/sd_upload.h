/**
 * sd_upload.h — USB "reader mode": stream tile files from the PC over the
 * USB-Serial/JTAG port (COM9) straight into the SD card at /sdcard, so the
 * card never has to be removed from the board.
 *
 * Protocol (text lines over COM9, 115200 8N1):
 *   PC -> FW:  OSMUP1                    enter upload mode
 *   FW -> PC:  READY
 *   PC -> FW:  FILE <size> <relpath>     e.g. "FILE 6854 16/52130/30730.png"
 *              <size> raw bytes
 *   FW -> PC:  OK | ERR <reason>
 *   PC -> FW:  DONE                      finish -> FW reboots into nav mode
 *   FW -> PC:  OK
 *
 * Relpaths are written to /sdcard/<relpath> (directories created on the fly).
 * See scripts/upload_tiles_serial.py for the PC side.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Listen on the USB-Serial/JTAG port for the upload magic.
 *
 * Blocks for up to timeout_ms waiting for "OSMUP1\n". If it arrives, mounts
 * the SD card, runs the upload loop until DONE/ABRT, then returns true (the
 * caller should esp_restart() into the normal app). If no magic arrives,
 * returns false immediately so normal boot proceeds.
 *
 * @param timeout_ms how long to wait for the magic before giving up.
 * @return true if an upload session ran to completion, false otherwise.
 */
bool sd_upload_listen(int timeout_ms);

#ifdef __cplusplus
}
#endif
