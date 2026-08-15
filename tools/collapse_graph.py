#!/usr/bin/env python3
"""collapse_graph.py — collapse degree-2 chain nodes out of a .rng graph.

OSM stores every shape point (each bend in a road) as a node; routing only
needs JUNCTIONS (where roads meet / dead-ends). This collapses the degree-2
chains between junctions into single edges, shrinking a country graph ~8x
(Vietnam: 29M -> ~4.6M nodes, 702 MB -> ~87 MB).

Usage:  collapse_graph.py <in.rng> <out.rng>
Output uses the same RNG1 format (nodes = junctions, edges = collapsed chains).
"""
import struct
import sys

import numpy as np


def load(path):
    f = open(path, 'rb')
    magic, ver, N, E, mnla, mnlo, mxla, mxlo = struct.unpack('<4sIIIiiii', f.read(32))
    assert magic == b'RNG1', 'not an RNG1 file'
    lat = np.fromfile(f, dtype='<i4', count=N)
    lon = np.fromfile(f, dtype='<i4', count=N)
    first = np.fromfile(f, dtype='<u4', count=N + 1)
    to = np.fromfile(f, dtype='<u4', count=E)
    w = np.fromfile(f, dtype='<u2', count=E)
    return N, E, lat, lon, first, to, w, (mnla, mnlo, mxla, mxlo)


def main():
    fin, fout = sys.argv[1], sys.argv[2]
    print(f"loading {fin} ...", flush=True)
    N, E, lat, lon, first, to, w, bbox = load(fin)
    print(f"  N={N:,} E={E:,}", flush=True)

    # ---- undirected distinct-neighbour degree ----
    src = np.repeat(np.arange(N, dtype=np.uint32), np.diff(first))
    lo = np.minimum(src, to); hi = np.maximum(src, to)
    pairs = np.unique(np.stack([lo, hi], axis=1), axis=0)
    pairs = pairs[pairs[:, 0] != pairs[:, 1]]
    deg = np.zeros(N, dtype=np.uint32)
    np.add.at(deg, pairs[:, 0], 1)
    np.add.at(deg, pairs[:, 1], 1)

    is_jun = deg != 2
    old_jun = np.where(is_jun)[0]
    n_jun = len(old_jun)
    n_chain = N - n_jun
    print(f"  junctions={n_jun:,} chain={n_chain:,} ({100*n_chain/N:.1f}%)", flush=True)

    # ---- undirected adjacency CSR (from the directed CSR) ----
    order = np.argsort(src, kind='stable')
    usrc = src[order]
    unei = to[order].astype(np.int64)
    uw = w[order].astype(np.uint32)
    counts = np.bincount(usrc, minlength=N)
    ufirst = np.zeros(N + 1, dtype=np.int64)
    ufirst[1:] = np.cumsum(counts)

    new_id = np.full(N, -1, dtype=np.int64)
    new_id[old_jun] = np.arange(n_jun, dtype=np.int64)

    # ---- collapse chains: walk from each junction along unconsumed chains ----
    print("  collapsing chains ...", flush=True)
    consumed = np.zeros(N, dtype=bool)
    edges = {}   # (src_new, dst_new) -> min weight  (undirected)

    def add_edge(a, b, wt):
        if a == b:
            return
        key = (int(a), int(b)) if a < b else (int(b), int(a))
        if key not in edges or wt < edges[key]:
            edges[key] = int(wt)

    for j in old_jun:
        nj = new_id[j]
        for k in range(ufirst[j], ufirst[j + 1]):
            n = unei[k]
            if is_jun[n]:
                add_edge(nj, new_id[n], int(uw[k]))
            elif not consumed[n]:
                acc = int(uw[k])
                prev = j
                cur = n
                while not is_jun[cur]:
                    consumed[cur] = True
                    nxt = -1
                    ew = 0
                    for k2 in range(ufirst[cur], ufirst[cur + 1]):
                        nb = unei[k2]
                        if nb != prev:
                            nxt = nb
                            ew = int(uw[k2])
                            break
                    acc += ew
                    if nxt < 0:
                        break
                    prev = cur
                    cur = nxt
                if nxt >= 0:
                    add_edge(nj, new_id[cur], acc)

    del src, to, w, pairs, deg, unei, uw, usrc, counts

    # ---- build directed CSR for the collapsed graph ----
    edges_sorted = sorted(edges.items())
    E2 = len(edges_sorted) * 2          # both directions
    new_lat = lat[old_jun]
    new_lon = lon[old_jun]
    # adjacency as lists then CSR
    adj = [[] for _ in range(n_jun)]
    for (a, b), wt in edges_sorted:
        adj[a].append((b, wt))
        adj[b].append((a, wt))
    new_first = np.zeros(n_jun + 1, dtype=np.uint32)
    new_to = np.empty(E2, dtype=np.uint32)
    new_w = np.empty(E2, dtype=np.uint16)
    pos = 0
    for i in range(n_jun):
        new_first[i] = pos
        for (b, wt) in adj[i]:
            new_to[pos] = b
            new_w[pos] = min(wt, 0xFFFF)
            pos += 1
    new_first[n_jun] = pos
    assert pos == E2, (pos, E2)

    # ---- write ----
    mb = (n_jun * 8 + (n_jun + 1) * 4 + E2 * 6) / 1e6
    print(f"  writing {fout}: N={n_jun:,} E={E2:,} file ~{mb:.1f} MB", flush=True)
    with open(fout, 'wb') as f:
        f.write(struct.pack('<4sIIIiiii', b'RNG1', 1, n_jun, E2,
                            bbox[0], bbox[1], bbox[2], bbox[3]))
        new_lat.tofile(f)
        new_lon.tofile(f)
        new_first.tofile(f)
        new_to.tofile(f)
        new_w.tofile(f)
    print("DONE")


if __name__ == "__main__":
    main()
