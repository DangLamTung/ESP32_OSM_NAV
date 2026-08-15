#!/usr/bin/env python3
"""Generate a QR code encoding the NAV-OSM BLE connection info.

Scan with a phone/custom app to know what to connect to:
  name      NAV-OSM
  service   5a7e1000-2b2f-4f66-9f9a-5c0f8e1a2b3c
  nav char  5a7e1001-2b2f-4f66-9f9a-5c0f8e1a2b3c
"""
import qrcode
import qrcode.constants

content = (
    "NAV-OSM\n"
    "service 5a7e1000-2b2f-4f66-9f9a-5c0f8e1a2b3c\n"
    "char 5a7e1001-2b2f-4f66-9f9a-5c0f8e1a2b3c"
)

qr = qrcode.QRCode(
    version=None,               # auto-size
    error_correction=qrcode.constants.ERROR_CORRECT_M,
    box_size=12,
    border=4,
)
qr.add_data(content)
qr.make(fit=True)

img = qr.make_image(fill_color="black", back_color="white")
out = "docs/ble-connect-qr.png"
img.save(out)
print("QR generated:", out)
print("encoded content:")
print(content)
