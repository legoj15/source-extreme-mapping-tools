#!/usr/bin/env python3
"""Estimate what dropping the BSP faces bspwalk found no eye for would save (issue #102).

    python -I .agents/scripts/hidden_face_savings.py --bsp MAP.bsp --walk WALK.json [--json OUT.json]

bspwalk's -out JSON lists one entry per BSP face (face, sideids, tier, reason). The tiers are
"visible", "A" (the face is in no reachable eye's PVS: proven from the map's vis data) and "B"
(no sight line to it was found from the sampled eye positions: a search result, not a proof).
A and B are the hidden tiers; any other tier name stops the script. Reading that JSON and the
lit BSP it was run on, this prints for all faces, for the hidden faces together and for each
hidden tier:

  faces       count and share of all faces; how many are displacement faces (dispinfo != -1)
              and how many have no lightmap (lightofs == -1)
  triangles   drawn: a face with n edges is n - 2 triangles, a displacement 2 * (2^power)^2
              from its ddispinfo_t power; count and share
  lightmap    luxels ((w + 1) * (h + 1) of m_LightmapTextureSizeInLuxels) and lighting lump
              bytes: per used light style (styles[k] != 255) one block of luxels * 4 bytes,
              four blocks when the face's texinfo has SURF_BUMPLIGHT, plus 4 bytes of average
              colour per used style (the layout image_compare.py reads). The LDR lumps
              (faces 7, lighting 8) and the HDR ones (58, 53) are each reported when VRAD lit them

Then the BSP size a pass that drops the hidden faces would save, as an estimate (face lump
entries, original faces whose split faces all go, their lighting bytes and the surfedges only
they reference), the hidden faces' share of the lightmap samples VRAD lights as a VRAD work
proxy (a proxy, not a timing), and the ten hidden faces with the most luxels. Nothing is
removed. Exit 1 when a file cannot be read or a face lump does not hold as many faces as the
JSON lists.
"""
import argparse
import json
import os
import struct
import sys

LUMP_TEXINFO = 6
LUMP_FACES = 7
LUMP_LIGHTING = 8
LUMP_EDGES = 12
LUMP_SURFEDGES = 13
LUMP_DISPINFO = 26
LUMP_ORIGINALFACES = 27
LUMP_LIGHTING_HDR = 53
LUMP_FACES_HDR = 58

HEADER_LUMPS = 64
DFACE_SIZE = 56          # dface_t (BSPVERSION 20); bsp_reader.py unpacks the same layout
DFACE = struct.Struct("<HBBihhhh4BifiiiiiHHI")
TEXINFO_SIZE = 72        # texinfo_t: two 2x4 float matrices, int flags, int texdata
TEXINFO_FLAGS_OFS = 64
SURF_BUMPLIGHT = 0x0800  # src/public/bspflags.h: three bump lightmaps after the flat one, per style
DDISPINFO_SIZE = 176     # ddispinfo_t as its members align in src/public/bspfile.h
DDISPINFO_POWER_OFS = 20
VARIANTS = (("LDR", LUMP_FACES, LUMP_LIGHTING), ("HDR", LUMP_FACES_HDR, LUMP_LIGHTING_HDR))
KNOWN_TIERS = ("visible", "A", "B")   # bspwalk writes no other; a new one must not count as hidden unseen


def read_header(path):
    """(version, lumps, data): the BSP's version, its 64 lump directory entries and the file.

    Each entry is (fileofs, filelen, version, uncompressedSize); uncompressedSize is the LZMA
    size when the lump is compressed on disk and 0 when it is stored plain."""
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < 8 + HEADER_LUMPS * 16 + 4 or data[:4] != b"VBSP":
        raise ValueError(f"{path}: not a VBSP file (bad ident or truncated lump directory)")
    version = struct.unpack_from("<I", data, 4)[0]
    lumps = [struct.unpack_from("<4i", data, 8 + i * 16) for i in range(HEADER_LUMPS)]
    return version, lumps, data


def lump_bytes(data, lumps, lump_id):
    """The raw bytes of one lump, empty when the BSP does not have it."""
    ofs, length = lumps[lump_id][0], lumps[lump_id][1]
    if length <= 0:
        return b""
    if lumps[lump_id][3] != 0:
        raise ValueError(f"lump {lump_id} is LZMA-compressed on disk and this script reads lumps as stored; "
                         f"use the BSP as the compile tools wrote it (or repack it without compression)")
    if ofs < 0 or ofs + length > len(data):
        raise ValueError(f"lump {lump_id} runs past the end of the file ({ofs:,} + {length:,} bytes)")
    return data[ofs:ofs + length]


def read_faces(raw):
    """One dict per dface_t: its edge run, texinfo, dispinfo, styles, lightofs, size and origFace."""
    n = len(raw) // DFACE_SIZE
    return [
        {
            "firstedge": v[3],
            "numedges": v[4],
            "texinfo": v[5],
            "dispinfo": v[6],
            "styles": v[8:12],
            "lightofs": v[12],
            "size": (v[16], v[17]),
            "orig_face": v[18],
        }
        for v in DFACE.iter_unpack(raw[: n * DFACE_SIZE])
    ]


def read_texinfo_flags(raw):
    """The flags of each texinfo_t (the int at +64 of the 72-byte entry)."""
    return [
        struct.unpack_from("<i", raw, i * TEXINFO_SIZE + TEXINFO_FLAGS_OFS)[0]
        for i in range(len(raw) // TEXINFO_SIZE)
    ]


def read_disp_powers(raw):
    """The power of each ddispinfo_t (the int at +20); a displacement is 2 * (2^power)^2 triangles."""
    return [
        struct.unpack_from("<i", raw, i * DDISPINFO_SIZE + DDISPINFO_POWER_OFS)[0]
        for i in range(len(raw) // DDISPINFO_SIZE)
    ]


def face_rows(faces, texflags, powers):
    """Per face the triangles it draws, its lightmap luxels, its lighting lump bytes and the
    lightmap samples VRAD lights (one per luxel, x 4 on a bumped face, x 1 per used style).

    Returns (rows, bad_power); bad_power counts the displacements whose power the dispinfo
    lump does not hold, which fall back to their polygon's numedges - 2 triangles."""
    rows = []
    bad_power = 0
    for f in faces:
        used = sum(1 for s in f["styles"] if s != 255)
        luxels = lighting = samples = 0
        if f["lightofs"] != -1:
            luxels = (f["size"][0] + 1) * (f["size"][1] + 1)
            bumped = 0 <= f["texinfo"] < len(texflags) and bool(texflags[f["texinfo"]] & SURF_BUMPLIGHT)
            blocks = 4 if bumped else 1
            lighting = used * (luxels * 4 * blocks + 4)  # the style blocks and average colours
            samples = used * luxels * blocks
        power = powers[f["dispinfo"]] if 0 <= f["dispinfo"] < len(powers) else None
        if power in (2, 3, 4):
            triangles = (1 << power) * (1 << power) * 2  # NUM_DISP_POWER_TRIS in bspfile.h
        else:
            if f["dispinfo"] >= 0:
                bad_power += 1
            triangles = max(f["numedges"] - 2, 0)
        rows.append({
            "triangles": triangles, "luxels": luxels, "lighting": lighting, "samples": samples,
            "size": f["size"], "disp": f["dispinfo"] != -1, "no_lightmap": f["lightofs"] == -1,
        })
    return rows, bad_power


def pct(part, whole):
    return 100.0 * part / whole if whole else 0.0


def share(part, whole):
    return f"{pct(part, whole):.2f}%"


def print_group(title, g, all_g, order):
    """One block of the report: the faces, triangles and per-variant lightmap lines of a group."""
    print(f"\n{title}")
    print(f"    faces          {g['faces']:>13,} ({share(g['faces'], all_g['faces'])} of {all_g['faces']:,})"
          f"   displacement {g['displacement']:,}   without lightmap {g['no_lightmap']:,}")
    print(f"    triangles      {g['triangles']:>13,} ({share(g['triangles'], all_g['triangles'])}"
          f" of {all_g['triangles']:,})")
    for tag in order:
        lm, lm_all = g["lightmap"][tag], all_g["lightmap"][tag]
        print(f"    lightmap {tag}   luxels {lm['luxels']:>13,} ({share(lm['luxels'], lm_all['luxels'])})"
              f"   lighting {lm['lighting_bytes']:,} of {lm['lighting_lump_bytes']:,} lump bytes"
              f" ({share(lm['lighting_bytes'], lm['lighting_lump_bytes'])})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bsp", required=True, help="the lit BSP bspwalk was run on")
    ap.add_argument("--walk", required=True, help="bspwalk's -out JSON for that BSP")
    ap.add_argument("--json", metavar="OUT.json", help="also write the numbers as one JSON object here")
    a = ap.parse_args()

    try:
        with open(a.walk, "r", encoding="utf-8") as f:
            walk = json.load(f)
    except (OSError, ValueError, RecursionError) as e:
        print(f"error: cannot read {a.walk}: {e}", file=sys.stderr)
        return 1
    entries = walk.get("faces") if isinstance(walk, dict) else None
    if not isinstance(entries, list) or not entries:
        print(f"error: {a.walk} has no faces list; pass a bspwalk -out JSON", file=sys.stderr)
        return 1
    by_index = {}
    for e in entries:
        i, tier = (e.get("face"), e.get("tier")) if isinstance(e, dict) else (None, None)
        if not isinstance(i, int) or isinstance(i, bool) or not isinstance(tier, str):
            print(f"error: {a.walk}: a face entry without an integer face or a tier string: {e!r}", file=sys.stderr)
            return 1
        if tier not in KNOWN_TIERS:
            print(f"error: {a.walk}: face {i} has tier {tier!r}; this script knows {', '.join(KNOWN_TIERS)}",
                  file=sys.stderr)
            return 1
        if i in by_index:
            print(f"error: {a.walk}: face {i} is listed twice", file=sys.stderr)
            return 1
        by_index[i] = e
    n_json = len(entries)

    try:
        version, lumps, data = read_header(a.bsp)
        file_size = os.path.getsize(a.bsp)
        texflags = read_texinfo_flags(lump_bytes(data, lumps, LUMP_TEXINFO))
        powers = read_disp_powers(lump_bytes(data, lumps, LUMP_DISPINFO))
        edges_raw = lump_bytes(data, lumps, LUMP_EDGES)
        edges = list(struct.iter_unpack("<2H", edges_raw[: len(edges_raw) // 4 * 4]))
        se_raw = lump_bytes(data, lumps, LUMP_SURFEDGES)
        surfedges = [v[0] for v in struct.iter_unpack("<i", se_raw[: len(se_raw) // 4 * 4])]
        orig_count = len(lump_bytes(data, lumps, LUMP_ORIGINALFACES)) // DFACE_SIZE
    except (OSError, ValueError, struct.error) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    variants = {}
    for tag, face_lump, light_lump in VARIANTS:
        raw = lump_bytes(data, lumps, face_lump)
        if not raw:
            continue
        faces = read_faces(raw)
        if len(faces) != n_json:
            print(f"error: {a.bsp}: faces lump {face_lump} holds {len(faces):,} faces but {a.walk} lists "
                  f"{n_json:,}; --bsp must be the BSP bspwalk was run on (or that map, relit, so every "
                  f"face lump holds as many faces as the JSON lists)", file=sys.stderr)
            return 1
        rows, bad_power = face_rows(faces, texflags, powers)
        # the size estimate is in bytes of the file as stored, so a compressed lighting lump is
        # refused like any other lump (lump_bytes)
        variants[tag] = {
            "face_lump": face_lump, "lighting_lump": light_lump, "faces": faces, "rows": rows,
            "bad_power": bad_power, "lighting_length": len(lump_bytes(data, lumps, light_lump)),
        }
    if not variants:
        print(f"error: {a.bsp} holds no faces lump (neither {LUMP_FACES} nor {LUMP_FACES_HDR})", file=sys.stderr)
        return 1
    # A face lump whose lighting lump is empty beside a lit one is the copy of the faces VRAD did
    # not light (lump 7 of an -hdr compile, as VBSP left it: lightofs 0, style 0): only the pairs
    # VRAD lit are reported. In those, every face's lighting must lie in the lump, which is also
    # what refuses a BSP VRAD never ran on.
    lit = [tag for tag in variants if variants[tag]["lighting_length"] > 0]
    if lit:
        variants = {tag: variants[tag] for tag in lit}
    for v in variants.values():
        for i, (f, row) in enumerate(zip(v["faces"], v["rows"])):
            avg = 4 * sum(1 for s in f["styles"] if s != 255)   # the average colours sit before lightofs
            if row["lighting"] and not (avg <= f["lightofs"]
                                        and f["lightofs"] - avg + row["lighting"] <= v["lighting_length"]):
                raise ValueError(f"face {i} of faces lump {v['face_lump']} has {row['lighting']:,} lighting bytes "
                                 f"at offset {f['lightofs']:,}, outside the {v['lighting_length']:,}-byte lighting "
                                 f"lump {v['lighting_lump']}: the BSP is not lit by VRAD or is damaged")
    have = set(by_index)
    if have != set(range(n_json)):
        miss, extra = sorted(set(range(n_json)) - have), sorted(have - set(range(n_json)))
        print(f"error: {a.walk}: its face indices do not cover 0..{n_json - 1} (missing "
              f"{str(miss[:8])}{'...' if len(miss) > 8 else ''}"
              f"{', out of range ' + str(extra[:8]) if extra else ''})", file=sys.stderr)
        return 1

    order = [tag for tag, _, _ in VARIANTS if tag in variants]
    primary = "HDR" if "HDR" in variants else "LDR"
    hidden = [i for i in range(n_json) if by_index[i]["tier"] != "visible"]
    hidden_set = set(hidden)
    tier_ids = {}
    for i in range(n_json):
        tier_ids.setdefault(by_index[i]["tier"], []).append(i)
    hidden_tiers = sorted((t for t in tier_ids if t != "visible"), key=lambda t: (-len(tier_ids[t]), t))

    def lightmap_of(tag, indices):
        rows = variants[tag]["rows"]
        return {
            "faces_lump": variants[tag]["face_lump"],
            "lighting_lump": variants[tag]["lighting_lump"],
            "luxels": sum(rows[i]["luxels"] for i in indices),
            "lighting_bytes": sum(rows[i]["lighting"] for i in indices),
            "samples": sum(rows[i]["samples"] for i in indices),
            "lighting_lump_bytes": variants[tag]["lighting_length"],
        }

    def group(indices):
        rows = variants[primary]["rows"]
        return {
            "faces": len(indices),
            "displacement": sum(1 for i in indices if rows[i]["disp"]),
            "no_lightmap": sum(1 for i in indices if rows[i]["no_lightmap"]),
            "triangles": sum(rows[i]["triangles"] for i in indices),
            "lightmap": {tag: lightmap_of(tag, indices) for tag in order},
        }

    all_g, hidden_g = group(range(n_json)), group(hidden)
    tier_g = {t: group(tier_ids[t]) for t in hidden_tiers}

    lump_bits = "; ".join(f"faces lump {variants[tag]['face_lump']}: {n_json:,} faces" for tag in order)
    print(f"{a.bsp}: BSP version {version}, {file_size:,} bytes, {lump_bits}")
    tier_bits = ", ".join(f"{t} ({len(tier_ids[t]):,})" for t in hidden_tiers)
    print(f"{a.walk}: {n_json:,} faces, {len(hidden):,} hidden ({share(len(hidden), n_json)})"
          + (f" in tiers {tier_bits}" if tier_bits else " (every face is visible)"))
    if not lit:
        print("    the BSP holds no lighting (VRAD has not lit it): every lightmap figure below is zero")
    if variants[primary]["bad_power"]:
        print(f"    {variants[primary]['bad_power']:,} displacement faces have no readable power in the "
              f"dispinfo lump; their triangles are counted as numedges - 2")

    print_group("all faces", all_g, all_g, order)
    print_group("hidden faces (every hidden tier)", hidden_g, all_g, order)
    for t in hidden_tiers:
        print_group(f"tier {t}", tier_g[t], all_g, order)

    # Which surfedges, edges and vertexes only the hidden faces use. Both face lumps describe the
    # same faces, so both are walked: an entry is free only when no visible face in either uses it.
    se_vis, se_hid, ed_vis, ed_hid = set(), set(), set(), set()
    for tag in order:
        for i, f in enumerate(variants[tag]["faces"]):
            se_t, ed_t = (se_hid, ed_hid) if i in hidden_set else (se_vis, ed_vis)
            first = max(f["firstedge"], 0)
            for k in range(first, first + max(f["numedges"], 0)):
                if k >= len(surfedges):
                    break
                se_t.add(k)
                e = abs(surfedges[k])
                if e < len(edges):
                    ed_t.add(e)
    ve_vis = {v for e in ed_vis for v in edges[e]}
    ve_hid = {v for e in ed_hid for v in edges[e]}
    se_alone, ed_alone, ve_alone = len(se_hid - se_vis), len(ed_hid - ed_vis), len(ve_hid - ve_vis)

    # An original face goes when every face derived from it (origFace, any face lump) is hidden.
    orig_referenced, orig_kept = set(), set()
    for tag in order:
        for i, f in enumerate(variants[tag]["faces"]):
            if 0 <= f["orig_face"] < orig_count:
                orig_referenced.add(f["orig_face"])
                if i not in hidden_set:
                    orig_kept.add(f["orig_face"])
    orig_dropped = len(orig_referenced - orig_kept)

    face_lump_ids = ", ".join(str(variants[t]["face_lump"]) for t in order)
    face_entry_bytes = len(hidden) * DFACE_SIZE * len(order)
    lighting_bytes = {tag: hidden_g["lightmap"][tag]["lighting_bytes"] for tag in order}
    lighting_total = sum(lighting_bytes.values())
    surfedge_bytes = se_alone * 4
    estimate = face_entry_bytes + orig_dropped * DFACE_SIZE + lighting_total + surfedge_bytes

    print("\nBSP size the hidden faces cost, as an estimate (nothing is removed)")
    print("    an estimate: a real pass would also shrink leaf-face lists, primitives, the face id and")
    print("    side id lumps (a few bytes per face) and the pak's lightmap-related data")

    def est_line(label, detail, n_bytes):
        print(f"    {label:<26}{detail:<44}{n_bytes:>13,} bytes")

    est_line("face lump entries", f"{len(hidden):,} x {DFACE_SIZE} x {len(order)} lumps ({face_lump_ids})",
             face_entry_bytes)
    if orig_count:
        est_line("original face entries", f"{orig_dropped:,} of {orig_count:,} x {DFACE_SIZE} (all split faces hidden)",
                 orig_dropped * DFACE_SIZE)
    else:
        print(f"    {'original face entries':<26}left out: the original faces lump"
              f" ({LUMP_ORIGINALFACES}) is empty")
    est_line("lighting lump bytes", " + ".join(f"{tag} {lighting_bytes[tag]:,}" for tag in order),
             lighting_total)
    est_line("surfedges only they use", f"{se_alone:,} x 4", surfedge_bytes)
    print(f"    {'estimate':<26}{estimate:,} bytes of the {file_size:,}-byte file"
          f" ({share(estimate, file_size)})")
    print("    would also become unreferenced, left out of the estimate (freeing them needs the edge")
    print("    and vertex lumps compacted too):")
    est_line("edges", f"{ed_alone:,} x 4", ed_alone * 4)
    est_line("vertexes", f"{ve_alone:,} x 12", ve_alone * 12)

    print("\nVRAD work proxy (a proxy, not a timing): VRAD's lighting cost follows the lightmap sample")
    print("    count, so the hidden faces' share of the samples stands in for the VRAD time they cost.")
    print("    A sample is one luxel, x 4 on a SURF_BUMPLIGHT face, x 1 per used light style.")
    for tag in order:
        lm, lm_all = hidden_g["lightmap"][tag], all_g["lightmap"][tag]
        print(f"    {tag}   {lm['samples']:>13,} of {lm_all['samples']:,} samples"
              f" ({share(lm['samples'], lm_all['samples'])})")

    rows_p = variants[primary]["rows"]
    top = sorted(hidden, key=lambda i: (-rows_p[i]["luxels"], i))[:10]
    print(f"\nten hidden faces with the most luxels (lightmap sizes from faces lump"
          f" {variants[primary]['face_lump']})")
    if top:
        print(f"    {'face':>7}  {'tier':<12}{'reason':<34}{'size':>7}{'luxels':>10}  sideids")
    else:
        print("    (no hidden faces)")
    for i in top:
        e, r = by_index[i], rows_p[i]
        reason = str(e.get("reason") or "")
        if len(reason) > 32:
            reason = reason[:31] + "..."
        ids = e.get("sideids") if isinstance(e.get("sideids"), list) else []
        ids_text = ", ".join(str(s) for s in ids[:4]) + (" ..." if len(ids) > 4 else "")
        size_text = f"{r['size'][0]}x{r['size'][1]}"
        print(f"    {i:>7,}  {e['tier']:<12}{reason:<34}{size_text:>7}{r['luxels']:>10,}  {ids_text}")

    def with_shares(g):
        return {
            "faces": g["faces"], "share": pct(g["faces"], all_g["faces"]),
            "displacement": g["displacement"], "no_lightmap": g["no_lightmap"],
            "triangles": g["triangles"], "triangles_share": pct(g["triangles"], all_g["triangles"]),
            "lightmap": {
                tag: {
                    "faces_lump": lm["faces_lump"], "lighting_lump": lm["lighting_lump"],
                    "lighting_lump_bytes": lm["lighting_lump_bytes"],
                    "luxels": lm["luxels"],
                    "luxels_share": pct(lm["luxels"], all_g["lightmap"][tag]["luxels"]),
                    "lighting_bytes": lm["lighting_bytes"],
                    "lighting_share": pct(lm["lighting_bytes"], lm["lighting_lump_bytes"]),
                    "samples": lm["samples"],
                    "samples_share": pct(lm["samples"], all_g["lightmap"][tag]["samples"]),
                }
                for tag, lm in g["lightmap"].items()
            },
        }

    report = {
        "bsp": a.bsp, "bsp_version": version, "bsp_bytes": file_size,
        "walk": a.walk, "faces": n_json, "hidden_faces": len(hidden),
        "tiers": {t: len(ids) for t, ids in sorted(tier_ids.items())},
        "groups": {"all": with_shares(all_g), "hidden": with_shares(hidden_g),
                   "tiers": {t: with_shares(tier_g[t]) for t in hidden_tiers}},
        "size_estimate": {
            "bytes": estimate, "file_bytes": file_size, "share": pct(estimate, file_size),
            "face_lump_entries": {"faces": len(hidden), "bytes_each": DFACE_SIZE,
                                  "lumps": [variants[t]["face_lump"] for t in order],
                                  "bytes": face_entry_bytes},
            "original_faces": {"total": orig_count, "dropped": orig_dropped, "bytes_each": DFACE_SIZE,
                               "bytes": orig_dropped * DFACE_SIZE, "left_out": orig_count == 0},
            "lighting": {"per_lump": lighting_bytes, "bytes": lighting_total},
            "surfedges": {"alone_referenced": se_alone, "bytes_each": 4, "bytes": surfedge_bytes},
            "would_also_become_unreferenced": {
                "edges": {"count": ed_alone, "bytes_each": 4, "bytes": ed_alone * 4},
                "vertexes": {"count": ve_alone, "bytes_each": 12, "bytes": ve_alone * 12},
            },
            "note": "an estimate: a real pass would also shrink leaf-face lists, primitives, the face id "
                    "and side id lumps and the pak's lightmap-related data",
        },
        "top_by_luxels": [
            {"face": i, "tier": by_index[i]["tier"], "reason": by_index[i].get("reason") or "",
             "size": list(rows_p[i]["size"]), "luxels": rows_p[i]["luxels"],
             "sideids": by_index[i].get("sideids") or []}
            for i in top
        ],
    }
    if a.json:
        try:
            with open(a.json, "w", encoding="utf-8") as f:
                json.dump(report, f, indent=2)
                f.write("\n")
        except OSError as e:
            print(f"error: cannot write {a.json}: {e}", file=sys.stderr)
            return 1
        print(f"\nwrote {a.json}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, struct.error) as e:   # a malformed or truncated BSP
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)
