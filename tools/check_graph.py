#!/usr/bin/env python3
"""Quick connectivity check for a .rng graph (offline, on the Mac)."""
import struct, sys

def load(path):
    f = open(path, 'rb')
    magic, ver, N, E, mnla, mnlo, mxla, mxlo = struct.unpack('<4sIIIiiii', f.read(32))
    lats = struct.unpack(f'<{N}i', f.read(4*N)); lons = struct.unpack(f'<{N}i', f.read(4*N))
    first = struct.unpack(f'<{N+1}I', f.read(4*(N+1)))
    to = struct.unpack(f'<{E}I', f.read(4*E)); w = struct.unpack(f'<{E}H', f.read(2*E))
    return N, lats, lons, first, to

def nearest(lats, lons, lat, lon):
    best, bd = -1, 1e18
    for i in range(len(lats)):
        d = (lat - lats[i]/1e7)**2 + (lon - lons[i]/1e7)**2
        if d < bd: bd, best = d, i
    return best

def comp_from(first, to, start):
    vis = set([start]); stack = [start]
    while stack:
        u = stack.pop()
        for k in range(first[u], first[u+1]):
            v = to[k]
            if v not in vis: vis.add(v); stack.append(v)
    return vis

def main():
    N, lats, lons, first, to = load(sys.argv[1])
    s = nearest(lats, lons, 10.7726, 106.6980)   # Ben Thanh
    vis = comp_from(first, to, s)
    print(f"largest comp from center: {len(vis)} of {N}")
    for name, la, lo in [("tapA", 10.770872, 106.698243), ("tapB", 10.774077, 106.702792),
                         ("cornerA", 10.737, 106.657), ("cornerB", 10.813, 106.743)]:
        n = nearest(lats, lons, la, lo)
        print(f"{name}: node {n} at {lats[n]/1e7:.6f},{lons[n]/1e7:.6f}  in_comp={n in vis}")

if __name__ == "__main__":
    main()
