#!/usr/bin/env python3
"""Build a compact ESP32 routing graph (.rng) from an OSM PBF.

Usage:
  python3 build_routing_graph.py <in.osm.pbf> <out.rng> [--bbox minlat,minlon,maxlat,maxlon]

The .rng file is a plain little-endian binary the ESP32 loads from SD
(FAT, /sdcard). Layout (see also src/nav/route_graph.h):

    u32 magic   0x31474E52 'RNG1'
    u32 version 1
    u32 N                  # nodes
    u32 E                  # directed edges
    i32 minlat e7          # bbox (1e-7 deg)
    i32 minlon e7
    i32 maxlat e7
    i32 maxlon e7
    i32 lat  [N]           # 1e-7 deg
    i32 lon  [N]
    u32 first[N+1]         # CSR row starts (into to[]/w[])
    u32 to   [E]           # target node index
    u16 w    [E]           # travel time in 0.1 s (fastest profile)

Weights are travel TIME (0.1 s) from road-class speeds, matching the
"car + fastest" profile the NavBridge app uses.

Memory estimate (PSRAM) is printed so you can pick a region that fits the
ESP32-S3's 8 MB PSRAM (aim < ~3 MB graph + < ~2 MB A* working set).
"""
import argparse
import math
import os
import struct
import sys
import time

try:
    import osmium
except ImportError:
    sys.exit("osmium not installed — run:  python3 -m pip install osmium")

# ---- car-drivable highway classes -> speed km/h ----
HIGHWAY_SPEED = {
    "motorway": 100, "motorway_link": 60,
    "trunk": 90, "trunk_link": 50,
    "primary": 70, "primary_link": 45,
    "secondary": 60, "secondary_link": 40,
    "tertiary": 50, "tertiary_link": 35,
    "unclassified": 40, "residential": 40,
    "living_street": 20, "service": 30,
    "road": 30, "track": 20,
}
# highway values that are NOT car roads (skip)
NON_CAR_HIGHWAY = {
    "footway", "path", "cycleway", "pedestrian", "steps", "bridleway",
    "construction", "proposed", "corridor", "bus_guideway", "raceway",
    "escape", "platform", "elevator", "service_truck",
}

def car_speed(tags, highway):
    """Return km/h for a way's tags, or 0 if not car-drivable."""
    if highway in NON_CAR_HIGHWAY:
        return 0
    speed = HIGHWAY_SPEED.get(highway)
    if not speed:
        return 0
    # access restrictions
    for k in ("access", "motor_vehicle", "vehicle", "motorcar"):
        v = tags.get(k, "").strip().lower()
        if v in ("no", "private", "agricultural", "forestry", "delivery", "military", "emergency"):
            return 0
    if tags.get("maxspeed") == "none":
        pass  # no upper bound; keep class speed
    return speed

def way_oneway(tags, highway):
    if tags.get("oneway", "").strip().lower() in ("yes", "1", "true"):
        return 1
    if tags.get("oneway", "").strip().lower() in ("-1", "reverse"):
        return -1
    if tags.get("junction") == "roundabout":
        return 1
    if tags.get("oneway:car", "").strip().lower() in ("yes", "1", "true"):
        return 1
    return 0

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))

def e7(x):
    return int(round(x * 1e7))

# ---------------------------------------------------------------------------
# Pass 1: collect car ways + needed node ids
# ---------------------------------------------------------------------------
class WayCollector(osmium.SimpleHandler):
    def __init__(self, bbox):
        super().__init__()
        self.bbox = bbox  # (minlat, minlon, maxlat, maxlon) or None
        self.ways = []    # (speed_kmh, oneway, [node ids])
        self.needed = set()

    def way(self, w):
        hw = w.tags.get("highway")
        if not hw:
            return
        speed = car_speed(w.tags, hw)
        if speed <= 0:
            return
        refs = [n.ref for n in w.nodes]   # NodeRef -> plain id
        if len(refs) < 2:
            return
        self.ways.append((speed, way_oneway(w.tags, hw), refs))
        self.needed.update(refs)

# ---------------------------------------------------------------------------
# Pass 2: fetch locations for needed ids (nodes precede ways in a PBF, so this
# is a separate pass over the file)
# ---------------------------------------------------------------------------
class NodeCollector(osmium.SimpleHandler):
    def __init__(self, needed):
        super().__init__()
        self.needed = needed
        self.loc = {}   # id -> (lat_e7, lon_e7)
        self.hit = 0

    def node(self, n):
        if n.id in self.needed:
            # bbox filter (by location) if given
            self.loc[n.id] = (e7(n.location.lat), e7(n.location.lon))
            self.hit += 1

# ---------------------------------------------------------------------------
# build + write
# ---------------------------------------------------------------------------
def build(pbf, out, bbox):
    t0 = time.time()
    print(f"[1/3] reading ways from {pbf} ...", flush=True)
    wc = WayCollector(bbox)
    wc.apply_file(pbf, locations=False)
    n_ways = len(wc.ways)
    n_need = len(wc.needed)
    print(f"      car ways={n_ways:,}  referenced nodes={n_need:,}  ({time.time()-t0:.1f}s)", flush=True)

    t1 = time.time()
    print("[2/3] reading node locations ...", flush=True)
    nc = NodeCollector(wc.needed)
    nc.apply_file(pbf, locations=False)
    print(f"      located={len(nc.loc):,}  ({time.time()-t1:.1f}s)", flush=True)

    # if a bbox was given, prune ways/nodes to nodes inside the box
    nodes = nc.loc
    ways = wc.ways
    if bbox:
        minlat, minlon, maxlat, maxlon = bbox
        nodes = {nid: (la, lo) for nid, (la, lo) in nc.loc.items()
                 if minlat <= la / 1e7 <= maxlat and minlon <= lo / 1e7 <= maxlon}
        keep = set(nodes)
        ways = [(spd, ow, [x for x in refs if x in keep]) for spd, ow, refs in ways]
        ways = [w for w in ways if len(w[2]) >= 2]
        # drop leading/trailing gaps (nodes pruned in the middle of a way)
        for i, (spd, ow, refs) in enumerate(ways):
            refs2 = []
            for r in refs:
                refs2.append(r)
            ways[i] = (spd, ow, refs2)

    # compact index (sort for deterministic output)
    ids = sorted(nodes.keys())
    id2idx = {nid: i for i, nid in enumerate(ids)}
    N = len(ids)
    print(f"      graph nodes N={N:,}  (bbox {' '.join(map(str,bbox)) if bbox else 'full'})", flush=True)

    # adjacency: dict node -> list of (to_idx, weight_01s)
    t2 = time.time()
    print("[3/3] building CSR edges ...", flush=True)
    adj = [[] for _ in range(N)]
    E = 0
    for speed, oneway, refs in ways:
        # compute per-segment weights from node locations
        for a, b in zip(refs, refs[1:]):
            if a not in id2idx or b not in id2idx:
                continue
            ia, ib = id2idx[a], id2idx[b]
            la1, lo1 = nodes[a]; la2, lo2 = nodes[b]
            d = haversine_m(la1 / 1e7, lo1 / 1e7, la2 / 1e7, lo2 / 1e7)
            if d <= 0:
                continue
            w = int(round(d / (speed / 3.6) * 10.0))  # 0.1 s
            if w < 1:
                w = 1
            if w > 0xFFFF:
                w = 0xFFFF
            adj[ia].append((ib, w))
            if oneway != 1:
                adj[ib].append((ia, w))
            E += 1 if oneway == 1 else 2

    # dedup parallel edges (keep min weight), build CSR
    first = [0] * (N + 1)
    to = []
    w = []
    for i in range(N):
        best = {}
        for tgt, wgt in adj[i]:
            if tgt in best:
                if wgt < best[tgt]:
                    best[tgt] = wgt
            else:
                best[tgt] = wgt
        for tgt in sorted(best):
            to.append(tgt)
            w.append(best[tgt])
        first[i + 1] = len(to)
    E = len(to)
    del adj

    # bbox from actual coords
    lats = [nodes[nid][0] for nid in ids]
    lons = [nodes[nid][1] for nid in ids]
    minlat, maxlat = min(lats), max(lats)
    minlon, maxlon = min(lons), max(lons)

    # write
    with open(out, "wb") as f:
        f.write(struct.pack("<4sIIIiiii", b"RNG1", 1, N, E,
                            minlat, minlon, maxlat, maxlon))
        f.write(struct.pack(f"<{N}i", *lats))
        f.write(struct.pack(f"<{N}i", *lons))
        f.write(struct.pack(f"<{N+1}I", *first))
        f.write(struct.pack(f"<{E}I", *to))
        f.write(struct.pack(f"<{E}H", *w))

    size = os.path.getsize(out)
    ram_graph = 4 * N + 4 * N + 4 * (N + 1) + 4 * E + 2 * E
    ram_astar = (4 + 4 + 4 + 1 + 4) * N          # dist/f/prev/closed/heapPos
    print(f"      wrote {out}: N={N:,} E={E:,}  file={size/1e6:.2f} MB  "
          f"PSRAM: graph~{ram_graph/1e6:.2f} MB + A*~{ram_astar/1e6:.2f} MB "
          f"(total ~{(ram_graph+ram_astar)/1e6:.2f} MB)  ({time.time()-t2:.1f}s)")
    print(f"      bbox: lat {minlat/1e7:.5f}..{maxlat/1e7:.5f}  "
          f"lon {minlon/1e7:.5f}..{maxlon/1e7:.5f}")
    print("DONE")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("pbf")
    ap.add_argument("out")
    ap.add_argument("--bbox", default=None,
                    help="minlat,minlon,maxlat,maxlon (decimal degrees)")
    a = ap.parse_args()
    bb = None
    if a.bbox:
        bb = tuple(float(x) for x in a.bbox.split(","))
        assert len(bb) == 4, "bbox needs 4 values"
    build(a.pbf, a.out, bb)
