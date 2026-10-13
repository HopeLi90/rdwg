#!/usr/bin/env python3
"""
Horizontal cross-check: compare `dwgparse`'s geometry against a ground-truth
DXF export, for **every fixture we have**.

Where the previous version hard-coded one R2007 pair, this one runs the same
comparison across every supported dialect *and* container:

  AC1015 (R2000, flat sections)
  AC1018 (R2004, section-tree container)
  AC1021 (R2007, page container)
  AC1024 (R2010) / AC1027 (R2013) / AC1032 (R2018) - reusing the R2004 container

That is the point of "horizontal" testing here: one logic bug shows up as one
dialect failing, instead of hiding behind an aggregate pass. It caught a real
one - `common_entity_data` gained fields at R2007a, R2010b and R2013, so
decoding every file with the R2007 layout silently produced garbage coordinates
for R2004 and R2010+.

For each fixture the script

1. runs `cargo run --example dump_geom -- <dwg>` to get our full-precision
   geometry dump (no intermediate file to get stale);
2. parses the ground-truth DXF produced by LibreDWG's `dwg2dxf`;
3. for every entity type, does a greedy 1:1 assignment by minimum
   max-coordinate difference and verifies each matched pair;
4. prints which truth entity types were **not** compared, so unverified
   coverage is visible instead of implied.

Usage:
    python3 tests/crosscheck_geom.py                 # every fixture
    python3 tests/crosscheck_geom.py --only AC1021   # one of them

A near-zero max difference is expected because DWG `BD`/`RD`/`DD` fields and
the DXF export are both IEEE-754 doubles of the same source values.

The truth DXFs come from LibreDWG's `dwg2dxf` (generate once):
    dwg2dxf -o /tmp/truth_2010.dxf tests/data/example_2010.dwg
"""
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

TOL = 1e-4          # coordinate tolerance for a PASS
WARN = 1e-6         # below TOL but above WARN -> print a warning

# label, dwg fixture, ground-truth DXF
FIXTURES = [
    ("AC1015", os.path.join(ROOT, "tests/data/sample_2000.dwg"), "/tmp/truth_2000.dxf"),
    ("AC1018", os.path.join(ROOT, "tests/data/example_2004.dwg"), "/tmp/truth_2004.dxf"),
    ("AC1021", os.path.join(ROOT, "tests/data/example_2007.dwg"), "/tmp/truth_2007.dxf"),
    ("AC1024", os.path.join(ROOT, "tests/data/example_2010.dwg"), "/tmp/truth_2010.dxf"),
    ("AC1027", os.path.join(ROOT, "tests/data/example_2013.dwg"), "/tmp/truth_2013.dxf"),
    ("AC1032", os.path.join(ROOT, "tests/data/example_2018.dwg"), "/tmp/truth_2018.dxf"),
]

# ---------------------------------------------------------------------------
# Truth DXF parsing
# ---------------------------------------------------------------------------

ENTITY_NAMES = ("POLYLINE", "VERTEX", "SEQEND", "LINE", "CIRCLE", "ARC",
                "POINT", "TEXT", "INSERT", "LWPOLYLINE", "ELLIPSE",
                "3DFACE", "SOLID", "TRACE", "BLOCK", "ENDBLK", "ATTRIB",
                "ATTDEF", "DIMENSION", "SPLINE", "HATCH", "MTEXT")


def parse_dxf(path):
    lines = open(path, encoding="utf-8", errors="replace").read().split("\n")
    # DXF: even line = group code (right-aligned, may have leading spaces),
    #      odd line  = value. Keep raw (str, str); convert codes to int later.
    raw = []
    i = 0
    n = len(lines)
    while i < n - 1:
        raw.append((lines[i].strip(), lines[i + 1].strip()))
        i += 2

    entities = []          # list of dicts: {section, type, groups}
    section = None
    i = 0
    L = len(raw)
    while i < L:
        code, val = raw[i]
        if code != "0":
            i += 1
            continue
        # A top-level entity begins here.
        if val == "SECTION":
            if i + 1 < L and raw[i + 1][0] == "2":
                section = raw[i + 1][1]
            i += 2
            continue
        if val == "ENDSEC":
            # `ENDSEC` is a single pair; the next pair is the following
            # `0/SECTION` (or `0/<entity>`), so advance by one only.
            section = None
            i += 1
            continue
        if val in ENTITY_NAMES:
            d = {"section": section, "type": val, "groups": {}}
            j = i + 1
            while j < L and raw[j][0] != "0":
                c, v = raw[j]
                d["groups"].setdefault(int(c), []).append(v)
                j += 1
            entities.append(d)
            i = j
            continue
        # unknown top-level: skip its groups
        j = i + 1
        while j < L and raw[j][0] != "0":
            j += 1
        i = j
        continue
    return entities


def num(x):
    try:
        return float(x)
    except ValueError:
        return float("nan")


def pt3(g, x=10, y=20, z=30):
    return (num(g[x][0]), num(g[y][0]), num(g.get(z, ["0.0"])[0]))


def deg2rad(d):
    """DXF stores ARC angles in degrees; our parser keeps radians."""
    return d * math.pi / 180.0


# --- per-type truth extractors ---------------------------------------------

def truth_line(e):
    g = e["groups"]
    return {"start": pt3(g, 10, 20, 30), "end": pt3(g, 11, 21, 31)}


def truth_circle(e):
    g = e["groups"]
    return {"center": pt3(g), "radius": num(g[40][0])}


def truth_arc(e):
    g = e["groups"]
    return {
        "center": pt3(g),
        "radius": num(g[40][0]),
        "start": deg2rad(num(g.get(50, ["0.0"])[0])),
        "end": deg2rad(num(g.get(51, ["0.0"])[0])),
    }


def truth_point(e):
    g = e["groups"]
    return {"p": pt3(g)}


def truth_text(e):
    g = e["groups"]
    # DXF splits long strings across 1 and 3, same as MTEXT.
    text = "".join(g.get(1, []) + g.get(3, []))
    return {"ins_pt": pt3(g), "height": num(g.get(40, ["0.0"])[0]), "text": text}


def truth_lwpolyline(e):
    g = e["groups"]
    xs, ys = g.get(10, []), g.get(20, [])
    return {
        "flag": int(num(g.get(70, ["0"])[0])),
        "points": [(num(a), num(b)) for a, b in zip(xs, ys)],
    }


def truth_insert(e):
    g = e["groups"]
    return {
        "block_name": g.get(2, [""])[0],
        "ins_pt": pt3(g),
        "scale": (num(g.get(41, ["1.0"])[0]),
                  num(g.get(42, ["1.0"])[0]),
                  num(g.get(43, ["1.0"])[0])),
    }


def truth_ellipse(e):
    g = e["groups"]
    return {
        "center": (num(g[10][0]), num(g[20][0]), num(g[30][0])),
        "major": (num(g[11][0]), num(g[21][0]), num(g[31][0])),
        "ratio": num(g[40][0]),
        "start": num(g[41][0]),
        "end": num(g[42][0]),
    }


def truth_face3d(e):
    g = e["groups"]
    return {
        "corners": [
            (num(g[10][0]), num(g[20][0]), num(g[30][0])),
            (num(g[11][0]), num(g[21][0]), num(g[31][0])),
            (num(g[12][0]), num(g[22][0]), num(g[32][0])),
            (num(g[13][0]), num(g[23][0]), num(g[33][0])),
        ]
    }


def truth_solid(e):
    g = e["groups"]
    return {
        "corners": [
            (num(g[10][0]), num(g[20][0]), num(g.get(30, ["0.0"])[0])),
            (num(g[11][0]), num(g[21][0]), num(g.get(31, ["0.0"])[0])),
            (num(g[12][0]), num(g[22][0]), num(g.get(32, ["0.0"])[0])),
            (num(g[13][0]), num(g[23][0]), num(g.get(33, ["0.0"])[0])),
        ],
        "elevation": num(g.get(38, ["0.0"])[0]),
    }


def truth_mtext(e):
    """MTEXT: insertion point (10/20/30), text height (40), text (1, or 3 when
    the string is longer than 250 chars — DXF splits it across both groups)."""
    g = e["groups"]
    text = "".join(g.get(1, []) + g.get(3, []))
    return {
        "ins_pt": pt3(g),
        "height": num(g.get(40, ["0.0"])[0]),
        "text": text,
    }


def truth_polyline(e, entities, idx):
    """Return dict with is_3d, flag, vertices for a POLYLINE in the pair stream.
    Child VERTEX entities follow until SEQEND."""
    g = e["groups"]
    flag = int(num(g.get(70, ["0"])[0]))
    is_3d = bool(flag & 0x8)
    is_pface = bool(flag & 0x40)
    verts = []
    j = idx + 1
    while j < len(entities):
        child = entities[j]
        j += 1
        if child["type"] == "SEQEND":
            break
        if child["type"] == "VERTEX":
            cg = child["groups"]
            # A polyface *face* record (VERTEX_PFACE_FACE) carries the 71..74
            # vertex indices instead of a coordinate; skip it. Note: the real
            # polyface mesh vertices also have bit 0x80 set in group 70 (flag
            # 192 = 0x40|0x80), so we must key off the presence of group 71,
            # not the 0x80 flag.
            if 71 in cg:
                continue
            if 10 in cg:  # a real point vertex
                verts.append((num(cg[10][0]), num(cg[20][0]), num(cg.get(30, ["0.0"])[0])))
    return {"is_3d": is_3d, "is_pface": is_pface, "flag": flag, "vertices": verts}


# ---------------------------------------------------------------------------
# Our dump parsing
# ---------------------------------------------------------------------------

def our_dump(dwg):
    """Run the dump_geom example and return its stdout lines."""
    r = subprocess.run(
        ["cargo", "run", "--quiet", "--example", "dump_geom", "--", dwg],
        cwd=ROOT, capture_output=True, text=True,
    )
    if r.returncode != 0:
        raise SystemExit(f"dump_geom failed for {dwg}:\n{r.stderr}")
    return r.stdout.split("\n")


def parse_ours(lines):
    out = {"LINE": [], "CIRCLE": [], "ARC": [], "POINT": [], "TEXT": [],
           "LWPOLYLINE": [], "INSERT": [], "ELLIPSE": [], "3DFACE": [],
           "SOLID": [], "POLYLINE_3D": [], "POLYLINE_PFACE": [], "MTEXT": []}
    vec3_re = re.compile(r"\(-?\d+\.?\d*,-?\d+\.?\d*,-?\d+\.?\d*\)")
    vec2_re = re.compile(r"\(-?\d+\.?\d*,-?\d+\.?\d*\)")
    for line in lines:
        m = re.match(r"\s*\[(\w+)\s*\]\s*#\s*(\d+)\s*layer=(\S+)\s*(.*)", line)
        if not m:
            continue
        tag, _idx, _layer, rest = m.groups()

        def f3(pattern):
            c = re.search(pattern, rest)
            return (float(c.group(1)), float(c.group(2)), float(c.group(3)))

        if tag == "LINE":
            c = re.search(r"LINE\s*"
                          r"\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)\s*->\s*"
                          r"\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)", rest)
            if c:
                out["LINE"].append({
                    "start": (float(c.group(1)), float(c.group(2)), float(c.group(3))),
                    "end": (float(c.group(4)), float(c.group(5)), float(c.group(6))),
                })
        elif tag == "CIRCLE":
            c = re.search(r"CIRCLE\s*c=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)\s*r=(-?[\d.]+)", rest)
            if c:
                out["CIRCLE"].append({
                    "center": (float(c.group(1)), float(c.group(2)), float(c.group(3))),
                    "radius": float(c.group(4)),
                })
        elif tag == "ARC":
            c = re.search(r"ARC\s*c=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)\s*r=(-?[\d.]+)\s*"
                          r"a=\[(-?[\d.]+),(-?[\d.]+)\]", rest)
            if c:
                out["ARC"].append({
                    "center": (float(c.group(1)), float(c.group(2)), float(c.group(3))),
                    "radius": float(c.group(4)),
                    "start": float(c.group(5)), "end": float(c.group(6)),
                })
        elif tag == "POINT":
            c = re.search(r"POINT\s*\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)", rest)
            if c:
                out["POINT"].append({"p": (float(c.group(1)), float(c.group(2)), float(c.group(3)))})
        elif tag == "TEXT":
            # `search`: layer names contain spaces, so `rest` may start with
            # the layer-name remainder.
            c = re.search(r'TEXT\s*@\((-?[\d.]+),(-?[\d.]+)\)\s*h=(-?[\d.]+)\s*"(.*)"\s*$', rest)
            if c:
                out["TEXT"].append({
                    "ins_pt": (float(c.group(1)), float(c.group(2)), 0.0),
                    "height": float(c.group(3)), "text": c.group(4),
                })
        elif tag == "LWPOLYLINE":
            fl = re.search(r"flag=0x([0-9A-Fa-f]+)", rest)
            pts = [tuple(map(float, p.strip("()").split(","))) for p in vec2_re.findall(rest)]
            out["LWPOLYLINE"].append({
                "flag": int(fl.group(1), 16) if fl else 0,
                "points": pts,
            })
        elif tag == "INSERT":
            c = re.search(r"INSERT\s*@\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)\s*"
                          r"scale=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)\s*blk=(\S+)", rest)
            if c:
                out["INSERT"].append({
                    "ins_pt": (float(c.group(1)), float(c.group(2)), float(c.group(3))),
                    "scale": (float(c.group(4)), float(c.group(5)), float(c.group(6))),
                    "block_name": c.group(7),
                })
        elif tag == "ELLIPSE":
            c = re.search(r"ELLIPSE\s*c=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)\s*"
                          r"m=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)\s*"
                          r"r=(-?[\d.]+)\s*a=\[(-?[\d.]+),(-?[\d.]+)\]", rest)
            if c:
                out["ELLIPSE"].append({
                    "center": (float(c.group(1)), float(c.group(2)), float(c.group(3))),
                    "major": (float(c.group(4)), float(c.group(5)), float(c.group(6))),
                    "ratio": float(c.group(7)),
                    "start": float(c.group(8)), "end": float(c.group(9)),
                })
        elif tag == "3DFACE":
            pts = [tuple(map(float, p.strip("()").split(","))) for p in vec3_re.findall(rest)]
            out["3DFACE"].append({"corners": pts})
        elif tag == "SOLID":
            pts = [tuple(map(float, p.strip("()").split(","))) for p in vec3_re.findall(rest)]
            ze = re.search(r"z=(-?[\d.]+)", rest)
            out["SOLID"].append({"corners": pts, "elevation": float(ze.group(1)) if ze else 0.0})
        elif tag in ("POLYLINE_3D", "POLYLINE_PFACE"):
            verts = [tuple(map(float, p.strip("()").split(","))) for p in vec3_re.findall(rest)]
            out[tag].append({"vertices": verts})
        elif tag == "MTEXT":
            m2 = re.search(r'MTEXT\s*@\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)\s*'
                           r'h=(-?[\d.]+)\s*"(.*)"\s*$', rest)
            if m2:
                out["MTEXT"].append({
                    "ins_pt": (float(m2.group(1)), float(m2.group(2)), float(m2.group(3))),
                    "height": float(m2.group(4)),
                    "text": m2.group(5),
                })
    return out


# ---------------------------------------------------------------------------
# Matching
# ---------------------------------------------------------------------------

def max_diff_coords(a, b):
    """Max abs coordinate difference between two equal-length point lists."""
    md = 0.0
    for (ax, ay, az), (bx, by, bz) in zip(a, b):
        md = max(md, abs(ax - bx), abs(ay - by), abs(az - bz))
    return md


def max_diff_scalar(a, b):
    return max(abs(a - b) for a, b in zip(a, b))


def cmp_line(o, t):
    return max_diff_coords([o["start"], o["end"]], [t["start"], t["end"]])


def cmp_circle(o, t):
    return max(max_diff_coords([o["center"]], [t["center"]]),
               abs(o["radius"] - t["radius"]))


def cmp_arc(o, t):
    return max(cmp_circle(o, t), abs(o["start"] - t["start"]), abs(o["end"] - t["end"]))


def cmp_point(o, t):
    return max_diff_coords([o["p"]], [t["p"]])


def cmp_text(o, t):
    # z is not printed for TEXT (it is 2D in our model), so compare x/y only.
    d = max(abs(o["ins_pt"][0] - t["ins_pt"][0]), abs(o["ins_pt"][1] - t["ins_pt"][1]))
    d = max(d, abs(o["height"] - t["height"]))
    if o["text"] != t["text"]:
        d = max(d, 1.0)      # string mismatch dominates any coordinate diff
    return d


def cmp_lwpolyline(o, t):
    if len(o["points"]) != len(t["points"]):
        return 1.0
    d = 0.0
    for (ax, ay), (bx, by) in zip(o["points"], t["points"]):
        d = max(d, abs(ax - bx), abs(ay - by))
    return d


def cmp_insert(o, t):
    d = max_diff_coords([o["ins_pt"]], [t["ins_pt"]])
    d = max(d, max_diff_scalar(o["scale"], t["scale"]))
    if o["block_name"] != t["block_name"]:
        d = max(d, 1.0)
    return d


def cmp_ellipse(o, t):
    return max(max_diff_coords([o["center"], o["major"]], [t["center"], t["major"]]),
               max_diff_scalar([o["ratio"], o["start"], o["end"]],
                               [t["ratio"], t["start"], t["end"]]))


def cmp_corners(o, t):
    return max_diff_coords(o["corners"], t["corners"])


def cmp_mtext(o, t):
    d = max(max_diff_coords([o["ins_pt"]], [t["ins_pt"]]),
            abs(o["height"] - t["height"]))
    if o["text"] != t["text"]:
        d = max(d, 1.0)
    return d


def cmp_polyline(o, t):
    if len(o["vertices"]) != len(t["vertices"]):
        return 1.0
    return max_diff_coords(o["vertices"], t["vertices"])


def greedy_match(ours, truth, cmpfn):
    """Greedy 1:1 assignment; returns (matches, unmatched_ours, unmatched_truth)."""
    used_t = set()
    matches = []
    cand = []
    for oi, o in enumerate(ours):
        for ti, t in enumerate(truth):
            if ti in used_t:
                continue
            cost = cmpfn(o, t)
            cand.append((cost, oi, ti))
    cand.sort()
    used_o = set()
    for cost, oi, ti in cand:
        if oi in used_o or ti in used_t:
            continue
        matches.append((oi, ti, cost))
        used_o.add(oi)
        used_t.add(ti)
    unmatched_ours = [i for i in range(len(ours)) if i not in used_o]
    unmatched_truth = [i for i in range(len(truth)) if i not in used_t]
    return matches, unmatched_ours, unmatched_truth


def report(name, ours, truth, cmpfn):
    if not ours and not truth:
        return 0
    print(f"\n=== {name} ===")
    print(f"  count  ours={len(ours)}  truth={len(truth)}")
    if len(ours) != len(truth):
        print("  <-- COUNT MISMATCH")
    matches, uo, ut = greedy_match(ours, truth, cmpfn)
    fails = 0
    worst = 0.0
    for oi, ti, cost in matches:
        flag = ""
        if cost > TOL:
            flag = "  <-- FAIL"
            fails += 1
        elif cost > WARN:
            flag = "  <-- WARN"
        worst = max(worst, cost)
        print(f"  ours[{oi}] <-> truth[{ti}]  maxdiff={cost:.3e}{flag}")
    if uo:
        print(f"  UNMATCHED ours: {uo}")
        fails += len(uo)
    if ut:
        print(f"  UNMATCHED truth: {ut}")
        fails += len(ut)
    if len(ours) != len(truth):
        fails += 1
    status = "PASS" if fails == 0 else "FAIL"
    print(f"  -> {status}  (worst matched diff={worst:.3e})")
    return fails


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

# Entity types we know how to compare, and how.
COMPARATORS = [
    ("LINE", "LINE", truth_line, cmp_line),
    ("CIRCLE", "CIRCLE", truth_circle, cmp_circle),
    ("ARC", "ARC", truth_arc, cmp_arc),
    ("POINT", "POINT", truth_point, cmp_point),
    ("TEXT", "TEXT", truth_text, cmp_text),
    ("LWPOLYLINE", "LWPOLYLINE", truth_lwpolyline, cmp_lwpolyline),
    ("INSERT", "INSERT", truth_insert, cmp_insert),
    ("ELLIPSE", "ELLIPSE", truth_ellipse, cmp_ellipse),
    ("3DFACE", "3DFACE", truth_face3d, cmp_corners),
    ("SOLID", "SOLID", truth_solid, cmp_corners),
    ("MTEXT", "MTEXT", truth_mtext, cmp_mtext),
]


def check_fixture(label, dwg, truth_path):
    print(f"\n{'=' * 70}\n{label}  {os.path.basename(dwg)}  vs  {truth_path}\n{'=' * 70}")
    if not os.path.exists(truth_path):
        print(f"  SKIPPED: truth DXF not found ({truth_path}).")
        print("  Generate it with:")
        print(f"    dwg2dxf -o {truth_path} {dwg}")
        return 0, None

    entities = parse_dxf(truth_path)
    by_type = {}
    for e in entities:
        by_type.setdefault(e["type"], []).append(e)

    truth_poly_3d, truth_poly_pface = [], []
    for i, e in enumerate(entities):
        if e["type"] == "POLYLINE":
            pl = truth_polyline(e, entities, i)
            if pl["is_pface"]:
                truth_poly_pface.append(pl)
            else:
                truth_poly_3d.append(pl)

    ours = parse_ours(our_dump(dwg))

    fails = 0
    for name, tname, extract, cmpfn in COMPARATORS:
        fails += report(name, ours[name], [extract(e) for e in by_type.get(tname, [])], cmpfn)
    fails += report("POLYLINE_3D", ours["POLYLINE_3D"], truth_poly_3d, cmp_polyline)
    fails += report("POLYLINE_PFACE", ours["POLYLINE_PFACE"], truth_poly_pface, cmp_polyline)

    # Make the *unverified* part explicit rather than implied.
    compared = {t for _n, t, _e, _c in COMPARATORS} | {"POLYLINE"}
    present = {t for t in by_type if t not in ("VERTEX", "SEQEND", "BLOCK", "ENDBLK")}
    gaps = sorted(present - compared)
    if gaps:
        print(f"\n  note: truth contains these entity types that this script "
              f"does NOT compare: {gaps}")
    return fails, gaps


def main():
    only = None
    if "--only" in sys.argv:
        only = sys.argv[sys.argv.index("--only") + 1]

    total = 0
    ran = 0
    for label, dwg, truth in FIXTURES:
        if only and label != only:
            continue
        if not os.path.exists(dwg):
            print(f"SKIP {label}: fixture missing ({dwg})")
            continue
        fails, _gaps = check_fixture(label, dwg, truth)
        total += fails
        ran += 1

    if ran == 0:
        print("nothing to check")
        return 1
    print("\n" + ("ALL CHECKS PASS" if total == 0 else f"{total} FAILURES"))
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
