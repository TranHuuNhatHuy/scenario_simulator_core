#!/usr/bin/env python3
# Copyright 2026 The Autoware Contributors. Apache-2.0
"""
Print map-frame (x, y, yaw) for a lanelet id + s along a lanelet2 map.

Needed because OpenSCENARIO uses LanePosition (laneId + s) while handdrive.py,
verify_lifecycle.py and `ros2 service call` all need map-frame coordinates. Guessing them puts
the ego off-lane, and `path_generator` then silently produces nothing -- which looks exactly
like a broken integration.

Autoware projects kashiwanoha with MGRS. lanelet2's Python bindings do not expose
MGRSProjector, so this reproduces it directly: UTM for the map's zone, then modulo the 100 km
MGRS grid square. Verified against the map extent published by autoware_map_projection_loader.

    python3 tools/map_coords.py                          # the spawn/goal this repo uses
    python3 tools/map_coords.py --lanelet 34507 --s 50
    python3 tools/map_coords.py --list                   # every road lanelet
"""

import argparse
import math
import re

import pyproj


def load(osm_path):
    txt = open(osm_path).read()
    nodes = {
        int(m.group(1)): (float(m.group(2)), float(m.group(3)))
        for m in re.finditer(r'<node id="(\d+)" lat="([-\d.eE]+)" lon="([-\d.eE]+)"', txt)
    }
    if not nodes:
        raise SystemExit(f"no <node lat=.. lon=..> found in {osm_path}")
    lon0 = next(iter(nodes.values()))[1]
    zone = int((lon0 + 180) // 6) + 1
    tr = pyproj.Transformer.from_crs(
        "EPSG:4326", f"+proj=utm +zone={zone} +datum=WGS84 +units=m", always_xy=True)

    def loc(lat, lon):
        e, n = tr.transform(lon, lat)
        return e % 100000.0, n % 100000.0  # MGRS 100 km grid local frame

    ways = {
        int(w.group(1)): [int(n) for n in re.findall(r'<nd ref="(\d+)"', w.group(2))]
        for w in re.finditer(r'<way id="(\d+)"[^>]*>(.*?)</way>', txt, re.S)
    }
    lanelets = {}
    for r in re.finditer(r'<relation id="(\d+)"[^>]*>(.*?)</relation>', txt, re.S):
        rid, body = int(r.group(1)), r.group(2)
        if 'v="lanelet"' not in body:
            continue
        mem = {
            mm.group(1): int(mm.group(2))
            for mm in re.finditer(r'<member type="way" role="(left|right)" ref="(\d+)"', body)
        }
        if "left" not in mem or "right" not in mem:
            continue
        sub = re.search(r'k="subtype" v="(\w+)"', body)
        lanelets[rid] = (mem, sub.group(1) if sub else "?")
    return nodes, ways, lanelets, loc, zone


def centerline(lid, nodes, ways, lanelets, loc):
    mem, _ = lanelets[lid]
    left = [loc(*nodes[n]) for n in ways[mem["left"]] if n in nodes]
    right = [loc(*nodes[n]) for n in ways[mem["right"]] if n in nodes]
    k = min(len(left), len(right))
    if k < 2:
        raise SystemExit(f"lanelet {lid}: degenerate centerline")
    return [((left[i][0] + right[i][0]) / 2, (left[i][1] + right[i][1]) / 2) for i in range(k)]


def at_s(centre, s):
    acc = 0.0
    for i in range(len(centre) - 1):
        d = math.dist(centre[i], centre[i + 1])
        if d < 1e-9:
            continue
        if acc + d >= s:
            t = (s - acc) / d
            return (
                centre[i][0] + t * (centre[i + 1][0] - centre[i][0]),
                centre[i][1] + t * (centre[i + 1][1] - centre[i][1]),
                math.atan2(centre[i + 1][1] - centre[i][1], centre[i + 1][0] - centre[i][0]),
            )
        acc += d
    return (centre[-1][0], centre[-1][1],
            math.atan2(centre[-1][1] - centre[-2][1], centre[-1][0] - centre[-2][0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="maps/kashiwanoha/lanelet2_map.osm")
    ap.add_argument("--lanelet", type=int, default=None)
    ap.add_argument("--s", type=float, default=1.0)
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    nodes, ways, lanelets, loc, zone = load(args.map)
    xs, ys = zip(*[loc(la, lo) for la, lo in nodes.values()])
    print(f"map: {args.map}")
    print(f"  UTM zone {zone}, MGRS-local frame; {len(nodes)} nodes, {len(lanelets)} lanelets")
    print(f"  extent x [{min(xs):.1f}, {max(xs):.1f}]  y [{min(ys):.1f}, {max(ys):.1f}]")

    def emit(lid, s, label=""):
        centre = centerline(lid, nodes, ways, lanelets, loc)
        length = sum(math.dist(centre[i], centre[i + 1]) for i in range(len(centre) - 1))
        x, y, yaw = at_s(centre, s)
        print(f"\n{label}lanelet {lid} [{lanelets[lid][1]}] length={length:.1f} m, at s={s}:")
        print(f"  x={x:.2f}  y={y:.2f}  yaw={yaw:.4f} rad  qz={math.sin(yaw/2):.4f} qw={math.cos(yaw/2):.4f}")
        print(f"  --x {x:.2f} --y {y:.2f} --qz {math.sin(yaw/2):.4f} --qw {math.cos(yaw/2):.4f}")
        return x, y, yaw

    if args.list:
        print("\nroad lanelets:")
        for lid in sorted(lanelets):
            if lanelets[lid][1] != "road":
                continue
            centre = centerline(lid, nodes, ways, lanelets, loc)
            length = sum(math.dist(centre[i], centre[i + 1]) for i in range(len(centre) - 1))
            print(f"  {lid}  len={length:6.1f} m  start=({centre[0][0]:8.2f},{centre[0][1]:9.2f})"
                  f"  end=({centre[-1][0]:8.2f},{centre[-1][1]:9.2f})")
        return

    if args.lanelet is not None:
        emit(args.lanelet, args.s)
        return

    print("\n=== the spawn/goal used by scenarios/*.yaml ===")
    sx, sy, _ = emit(34513, 1.0, "SPAWN  ")
    gx, gy, _ = emit(34507, 50.0, "GOAL   ")
    print(f"\nstraight-line spawn->goal: {math.dist((sx, sy), (gx, gy)):.1f} m")


if __name__ == "__main__":
    main()
