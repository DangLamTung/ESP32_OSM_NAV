#!/usr/bin/env python3
"""sim_window.py — simulate the ESP32 RNG2 windowed load in Python.

Mirrors route_graph.cpp rg_load_rng2: given an RNG2 file and a centre +
radius, read only the covered cells' nodes + edges, remap to a local CSR
(dropping out-of-window targets), then check connectivity/routing. Validates
the format and the loader's memory estimate before flashing to the board.

Usage: sim_window.py <in.rng2> [lat lon] [radius]
"""
import math
import struct
import sys

import numpy as np


def main():
    path = sys.argv[1]
    lat = float(sys.argv[2]) if len(sys.argv) > 2 else 10.7726
    lon = float(sys.argv[3]) if len(sys.argv) > 3 else 106.6980
    rad = float(sys.argv[4]) if len(sys.argv) > 4 else 0.06

    f = open(path, 'rb')
    magic, ver, N, E, mnla, mnlo, mxla, mxlo, cell = struct.unpack('<4sIIIiiiiI', f.read(36))
    gw, gh = struct.unpack('<HH', f.read(4))
    assert magic == b'RNG2'
    cnfOff = 40
    latOff = cnfOff + (gw * gh + 1) * 4
    lonOff = latOff + N * 4
    firstOff = lonOff + N * 4
    toOff = firstOff + (N + 1) * 4
    wOff = toOff + E * 4
    print(f"RNG2 N={N:,} E={E:,} cell={cell/1e7} grid={gw}x{gh}")

    # covered cells
    cx0 = max(0, int(math.floor((lon - rad - mnlo/1e7) / (cell/1e7))))
    cx1 = min(gw-1, int(math.floor((lon + rad - mnlo/1e7) / (cell/1e7))))
    cy0 = max(0, int(math.floor((lat - rad - mnla/1e7) / (cell/1e7))))
    cy1 = min(gh-1, int(math.floor((lat + rad - mnla/1e7) / (cell/1e7))))
    ncx, ncy = cx1-cx0+1, cy1-cy0+1
    print(f"covered cells x {cx0}..{cx1}, y {cy0}..{cy1}  ({ncx*ncy} cells)")

    # per covered row, read cnf[cy*gw+cx0 .. +ncx]
    cnf = np.empty((ncy, ncx+1), dtype=np.uint32)
    for r in range(ncy):
        cy = cy0 + r
        f.seek(cnfOff + (cy*gw + cx0)*4)
        cnf[r] = np.fromfile(f, dtype='<u4', count=ncx+1)

    # node + edge ranges per cell, build local arrays
    loc_lat, loc_lon = [], []
    loc_first = [0]
    loc_to, loc_w = [], []
    # old->local via covered ranges (binary search)
    ranges = []
    for r in range(ncy):
        for c in range(ncx):
            ranges.append((int(cnf[r, c]), int(cnf[r, c+1] - cnf[r, c])))
    # (ranges are disjoint sorted; build a simple lookup dict for speed)
    old_to_local = {}
    for base_, (s, cnt) in enumerate(ranges):
        for i in range(cnt):
            old_to_local[s + i] = (sum(r[1] for r in ranges[:base_]) + i)

    for r in range(ncy):
        for c in range(ncx):
            s = int(cnf[r, c]); cnt = int(cnf[r, c+1] - cnf[r, c])
            if cnt == 0:
                continue
            # coords
            f.seek(latOff + s*4); la = np.fromfile(f, dtype='<i4', count=cnt)
            f.seek(lonOff + s*4); lo = np.fromfile(f, dtype='<i4', count=cnt)
            # first
            f.seek(firstOff + s*4); fr = np.fromfile(f, dtype='<u4', count=cnt+1)
            e0, e1 = int(fr[0]), int(fr[cnt])
            f.seek(toOff + e0*4); tt = np.fromfile(f, dtype='<u4', count=e1-e0)
            f.seek(wOff + e0*2); ww = np.fromfile(f, dtype='<u2', count=e1-e0)
            base = len(loc_lat)
            loc_lat.extend(la.tolist()); loc_lon.extend(lo.tolist())
            for j in range(cnt):
                keep = 0
                for k in range(int(fr[j])-e0, int(fr[j+1])-e0):
                    tgt = int(tt[k])
                    tl = old_to_local.get(tgt)
                    if tl is not None:
                        loc_to.append(tl); loc_w.append(int(ww[k])); keep += 1
                loc_first.append(loc_first[-1] + keep)
            # window bbox
    M = len(loc_lat); Ew = len(loc_to)
    mb = (M*8 + (M+1)*4 + Ew*6) / 1e6
    print(f"window: N={M:,} E={Ew:,} (~{mb:.2f} MB graph + {M*21/1e6:.2f} MB A*)")

    # connectivity between two points in the window
    def nearest(la, lo):
        best, bd = -1, 1e18
        for i in range(M):
            d = (la - loc_lat[i]/1e7)**2 + (lo - loc_lon[i]/1e7)**2
            if d < bd: bd, best = d, i
        return best
    a = nearest(lat, lon); b = nearest(10.7780, 106.7020)
    vis = set([a]); stack = [a]
    while stack:
        u = stack.pop()
        for k in range(loc_first[u], loc_first[u+1]):
            v = loc_to[k]
            if v not in vis: vis.add(v); stack.append(v)
    print(f"start node {a} ({loc_lat[a]/1e7:.6f},{loc_lon[a]/1e7:.6f}) "
          f"end {b} in_comp={b in vis}, comp size={len(vis)}")


if __name__ == "__main__":
    main()
