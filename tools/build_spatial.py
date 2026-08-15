#!/usr/bin/env python3
"""build_spatial.py — add a spatial window index to a collapsed .rng.

Turns a plain RNG1 graph into RNG2: nodes are reordered into a grid of cells
(by lat/lon), and a per-cell cumulative node-range table is written up front.
The ESP32 can then fseek to a region and load ONLY the nodes/edges covering
the car (SD = memory, PSRAM = active window), enabling whole-country graphs
that would never fit in 8 MB PSRAM.

Layout (all little-endian):
  u32 magic 'RNG2'
  u32 version 1
  u32 N                  # junction nodes
  u32 E                  # directed edges
  i32 minlat, minlon, maxlat, maxlon (e7)   # whole-country bbox
  u32 cellDegE7          # cell size (e.g. 0.02 deg = 200000)
  u16 gridW, gridH       # cells per axis
  u32 cellNodeFirst[gridW*gridH]   # cumulative; cell c -> node ids [..c, ..c+1)
  i32 lat[N]  i32 lon[N]            # nodes reordered by cell
  u32 first[N+1]  u32 to[E]  u16 w[E]   # CSR over the reordered nodes

Usage: build_spatial.py <in.rng> <out.rng> [--cell 0.02]
"""
import argparse
import struct

import numpy as np


def load(path):
    f = open(path, 'rb')
    magic, ver, N, E, mnla, mnlo, mxla, mxlo = struct.unpack('<4sIIIiiii', f.read(32))
    assert magic == b'RNG1', 'input must be an RNG1 (collapsed) graph'
    lat = np.fromfile(f, dtype='<i4', count=N)
    lon = np.fromfile(f, dtype='<i4', count=N)
    first = np.fromfile(f, dtype='<u4', count=N + 1)
    to = np.fromfile(f, dtype='<u4', count=E)
    w = np.fromfile(f, dtype='<u2', count=E)
    return N, E, lat, lon, first, to, w, (mnla, mnlo, mxla, mxlo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("in_rng")
    ap.add_argument("out_rng")
    ap.add_argument("--cell", type=float, default=0.02, help="cell size in degrees")
    a = ap.parse_args()

    N, E, lat, lon, first, to, w, bbox = load(a.in_rng)
    print(f"input: N={N:,} E={E:,}", flush=True)

    cell = int(round(a.cell * 1e7))
    mnla, mnlo, mxla, mxlo = bbox
    gridW = int((mxlo - mnlo) / cell) + 1
    gridH = int((mxla - mnla) / cell) + 1
    nCells = gridW * gridH
    print(f"cells: {gridW}x{gridH} = {nCells:,}  ({a.cell} deg)", flush=True)

    # ---- assign each node to a cell, then reorder nodes by cell ----
    cx = np.clip(((lon - mnlo) / cell).astype(np.int64), 0, gridW - 1)
    cy = np.clip(((lat - mnla) / cell).astype(np.int64), 0, gridH - 1)
    cell_id = (cy * gridW + cx).astype(np.int64)

    order = np.argsort(cell_id, kind='stable')          # new position per old id
    new_id = np.empty(N, dtype=np.int64)
    new_id[order] = np.arange(N, dtype=np.int64)        # old id -> new position

    lat2 = lat[order]
    lon2 = lon[order]
    cell2 = cell_id[order]

    # ---- cellNodeFirst: cumulative node counts per cell (u32 in the file) ----
    cellCount = np.bincount(cell2, minlength=nCells)
    cellNodeFirst = np.zeros(nCells + 1, dtype=np.uint32)
    cellNodeFirst[1:] = np.cumsum(cellCount).astype(np.uint32)

    # ---- rebuild CSR in new order ----
    # edge rows: old source node -> new source position; remap targets
    old_src = np.repeat(np.arange(N, dtype=np.int64), np.diff(first.astype(np.int64)))
    new_src = new_id[old_src]
    new_tgt = new_id[to.astype(np.int64)]
    w2 = w

    # stable-sort edges by new source so rows are contiguous
    order_e = np.argsort(new_src, kind='stable')
    new_src = new_src[order_e]
    new_tgt = new_tgt[order_e]
    w2 = w2[order_e]

    first2 = np.zeros(N + 1, dtype=np.uint32)
    first2[1:] = np.cumsum(np.bincount(new_src, minlength=N)).astype(np.uint32)
    E2 = len(new_src)
    assert E2 == E

    mb = (nCells * 4 + N * 8 + (N + 1) * 4 + E * 6) / 1e6
    print(f"writing {a.out_rng}: N={N:,} E={E:,} file ~{mb:.1f} MB", flush=True)
    with open(a.out_rng, 'wb') as f:
        f.write(struct.pack('<4sIIIiiiiI', b'RNG2', 1, N, E2,
                            mnla, mnlo, mxla, mxlo, cell))
        f.write(struct.pack('<HH', gridW, gridH))
        cellNodeFirst.tofile(f)
        lat2.tofile(f)
        lon2.tofile(f)
        first2.tofile(f)
        new_tgt.astype(np.uint32).tofile(f)   # must be u32, not int64
        w2.tofile(f)
    print("DONE")


if __name__ == "__main__":
    main()
