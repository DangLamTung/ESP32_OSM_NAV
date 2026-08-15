# verify_packets.py — check the new nav packets parse with the C parser logic.
def attr(buf, name):
    key = name + '="'
    i = buf.find(key)
    if i < 0:
        return None
    i += len(key)
    j = buf.find('"', i)
    return buf[i:j]

def parse_tag(buf):
    if "<pos" in buf:
        return dict(kind="pos", lat=float(attr(buf, "lat")), lon=float(attr(buf, "lon")),
                    spd=int(attr(buf, "spd")), hdg=int(attr(buf, "hdg")), sl=int(attr(buf, "sl") or 0))
    if "<nav" in buf:
        return dict(kind="nav", d=int(attr(buf, "d")), m=attr(buf, "m"), s=attr(buf, "s"))
    if "<eta" in buf:
        return dict(kind="eta", h=int(attr(buf, "h")), m=int(attr(buf, "m")), a=attr(buf, "a"))
    if "<clock" in buf:
        return dict(kind="clock", h=int(attr(buf, "h")), m=int(attr(buf, "m")))
    return dict(kind="?")

packets = [
    '<pos lat="10.772200" lon="106.699100" spd="30" hdg="90" sl="50"></pos>',
    '<pos lat="10.772200" lon="106.699100" spd="30" hdg="270"></pos>',
    '<nav d="85" m="slight-left" s="Nguyen Hue"></nav>',
    '<eta h="14" m="32" a="Ben Thanh"></eta>',
    '<clock h="14" m="30"></clock>',
]
closings = ["</route>", "</nav>", "</pos>", "</eta>", "</clock>"]
ok = True
for p in packets:
    closing = next((t for t in closings if p.endswith(t)), None)
    good = closing is not None
    ok = ok and good
    print(f"{'OK' if good else 'NO-CLOSE':9} {p[:58]:60} -> {parse_tag(p)}")
print("ALL OK" if ok else "FAILED")
