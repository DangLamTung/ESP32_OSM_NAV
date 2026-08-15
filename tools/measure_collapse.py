#!/usr/bin/env python3
"""Measure the chain-collapsed size of a .rng graph without writing it out.

Loads the CSR graph, computes undirected node degrees, and counts how many
nodes are JUNCTIONS (degree != 2). Collapsing degree-2 chains turns the rest
into single edges, so this gives the routing-node count and a rough edge
estimate — the number that actually matters for a routing engine.
"""
import struct
import sys

import numpy as np


def load(path):
    f = open(path, 'rb')
    magic, ver, N, E, mnla, mnlo, mxla, mxlo = struct.unpack('<4sIIIiiii', f.read(32))
    assert magic == b'RNG1'
    lat = np.fromfile(f, dtype='<i4', count=N)
    lon = np.fromfile(f, dtype='<i4', count=N)
    first = np.fromfile(f, dtype='<u4', count=N + 1)
    to = np.fromfile(f, dtype='<u4', count=E)
    w = np.fromfile(f, dtype='<u2', count=E)
    return N, E, lat, lon, first, to, w


def main():
    path = sys.argv[1]
    N, E, lat, lon, first, to, w = load(path)
    print(f"N={N:,} E={E:,} file_est={((N*8 + (N+1)*4 + E*6)/1e6):.1f} MB")

    # UNDIRECTED degree = number of DISTINCT neighbours (ignoring one-way
    # direction). A chain node on any road (one-way or not) has exactly 2.
    src = np.repeat(np.arange(N, dtype=np.uint32), np.diff(first))
    dst = to
    lo = np.minimum(src, dst)
    hi = np.maximum(src, dst)
    pairs = np.unique(np.stack([lo, hi], axis=1), axis=0)
    pairs = pairs[pairs[:, 0] != pairs[:, 1]]
    deg = np.zeros(N, dtype=np.uint32)
    np.add.at(deg, pairs[:, 0], 1)
    np.add.at(deg, pairs[:, 1], 1)

    # junctions: undirected degree != 2 (>=3 = intersection, <=1 = dead-end)
    jun = (deg != 2)
    n_jun = int(jun.sum())
    n_chain = N - n_jun
    print(f"junctions (deg!=2): {n_jun:,}  chain nodes to collapse: {n_chain:,} "
          f"({100*n_chain/N:.1f}%)")

    # rough collapsed edges: each junction keeps its incident chains.
    # sum over junctions of (number of incident chains) / 2 approx = sum(deg_jun)/2,
    # but a chain contributes to both ends only if both are junctions.
    deg_jun = deg[jun]
    est_edges = int(deg_jun.sum() // 2)
    # collapsed file: coords 8B/jun + first 4B/jun + to 4B/edge + w 2B/edge
    mb = (n_jun * 8 + (n_jun + 1) * 4 + est_edges * 6) / 1e6
    # A* working set ~21B/node
    astar = n_jun * 21 / 1e6
    print(f"collapsed: ~{n_jun:,} nodes, ~{est_edges:,} edges, "
          f"file ~{mb:.1f} MB, A* ~{astar:.1f} MB")
    lat_span = (lat.max() - lat.min()) / 1e7
    lon_span = (lon.max() - lon.min()) / 1e7
    frac = (0.10 * 0.10) / (lat_span * lon_span)
    print(f"-> a 0.10 deg window ≈ {int(n_jun * frac):,} nodes")


if __name__ == "__main__":
    main()
