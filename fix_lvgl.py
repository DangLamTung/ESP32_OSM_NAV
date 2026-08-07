import os, shutil

SRC = r"C:\Users\Tung\Documents\ESP32\ESP32\car_nav\managed_components\lvgl__lvgl"
DST = r"C:\Users\Tung\Documents\ESP32\ESP32\osm_idf\managed_components\lvgl__lvgl"


def size(p):
    return sum(os.path.getsize(os.path.join(r, f)) for r, _, fs in os.walk(p) for f in fs)


print("osm_idf lvgl:", len(os.listdir(DST)), "files", size(DST), "bytes")
print("car_nav lvgl:", len(os.listdir(SRC)), "files", size(SRC), "bytes")

if size(DST) < size(SRC) // 2:
    print("osm_idf copy looks partial -> replacing with car_nav copy")
    shutil.rmtree(DST, ignore_errors=True)
    shutil.copytree(SRC, DST)
    print("replaced. now:", len(os.listdir(DST)), "files", size(DST), "bytes")
else:
    print("osm_idf copy looks complete, keeping it")
