#!/usr/bin/env python3
"""
Offline Navigation Visualizer — World-Frame Map Builder  (v3)

Reads a JSONL log from the synthetic replay framework and renders a
top-down world-frame view.  The robot moves through the scene while
LiDAR wall-points accumulate to form the corridor / obstacle shape.

Visual elements
───────────────
  ● Dark dots          accumulated wall boundary (from projected LiDAR sectors)
  ● Tinted polygon     current LiDAR "free-space" footprint around the robot
  ● Green triangle     the robot, oriented in its heading direction
  ● Orange polyline    the trail the robot has taken so far
  ● Red arrow          velocity vector (speed + turn intent)
  ● Right panel        dashboard with state, reason, module, speeds, distances

Controls (interactive window)
─────────────────────────────
  SPACE   pause / resume
  A / ←   step backward  (auto-pauses)
  D / →   step forward   (auto-pauses)
  Q / Esc quit

Usage
─────
  python scripts/visualize_nav_log.py <log.jsonl>
  python scripts/visualize_nav_log.py <log.jsonl> --save output/video.mp4
  python scripts/visualize_nav_log.py <log.jsonl> --save out.mp4 --no-display
"""

import sys
import json
import math
import argparse
import os

import numpy as np
import cv2

# ═══════════════════════════════════════════════════════════════════════════
# Colour palette  (BGR for OpenCV)
# ═══════════════════════════════════════════════════════════════════════════
BG           = (245, 245, 240)       # warm off-white canvas
WALL_NEW     = (55,  55,  60)        # dark charcoal — recent wall points
WALL_OLD     = (195, 195, 190)       # faded — old wall points
TRAIL        = (45, 130, 210)        # warm orange trail
TRAIL_START  = (80, 180, 240)        # lighter start marker
ROBOT_FILL   = (60, 195, 95)         # bright green
ROBOT_EDGE   = (35, 120, 55)         # darker outline
VEL_ARROW    = (55,  55, 220)        # red arrow
GRID_LINE    = (225, 225, 220)       # subtle grid
GRID_TXT     = (175, 175, 170)
FOOTPRINT_FILL = (220, 235, 210)     # very light green free-space tint
FOOTPRINT_EDGE = (140, 180, 130)     # soft green boundary

# dashboard
DASH_BG      = (38, 40, 46)
DASH_SEP     = (65, 68, 74)
TXT          = (230, 230, 230)
TXT_DIM      = (140, 145, 150)
TXT_EMERG    = (60,  60, 235)
TXT_TURN     = (55, 185, 235)
TXT_RECOV    = (55, 210, 235)

STATE_CLR = {
    "CORRIDOR_FOLLOW": (60, 195, 95),
    "FOLLOW_GAP":      (60, 195, 95),
    "WALL_FOLLOW":     (60, 195, 95),
    "RECOVERY":        TXT_RECOV,
    "EMERGENCY_STOP":  TXT_EMERG,
    "TURNING_LEFT":    TXT_TURN,
    "TURNING_RIGHT":   TXT_TURN,
    "ALIGNING":        TXT_TURN,
    "SETTLING":        (180, 165, 50),
    "QR_SCAN":         (200, 125, 55),
    "SIGN_SLOW":       TXT_TURN,
}

# ═══════════════════════════════════════════════════════════════════════════
# Sector angle definitions  (degrees, 0 = forward, +left, -right)
# ═══════════════════════════════════════════════════════════════════════════
#  Ordered counter-clockwise so the polygon connects properly.
SECTOR_ARCS = [
    ("front_center",  -10,   10),
    ("front",         -20,   20),
    ("front_right",   -70,  -20),
    ("right",        -110,  -70),
    ("rear_right",   -170, -110),
    ("rear_left",     110,  170),
    ("left",           70,  110),
    ("front_left",     20,   70),
]

RAYS_PER_SECTOR = 8          # rays cast per sector for wall points
MAX_VIS_DIST    = 5.0         # clamp displayed distance (metres)
DASH_W          = 260         # dashboard panel width in pixels


# ═══════════════════════════════════════════════════════════════════════════
# Data loading
# ═══════════════════════════════════════════════════════════════════════════
def load_log(path):
    meta, steps = {}, []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                rec = json.loads(ln)
            except json.JSONDecodeError:
                continue
            if rec.get("record_type") == "metadata":
                meta = rec
            elif rec.get("record_type") == "step":
                steps.append(rec)
    return meta, steps


# ═══════════════════════════════════════════════════════════════════════════
# First pass — dead-reckoning + wall projection
# ═══════════════════════════════════════════════════════════════════════════
def _project_sector(x, y, heading, dist_m, a_min_deg, a_max_deg, n_rays):
    """Project one sector into world-frame (wx, wy) points."""
    pts = []
    d = min(dist_m, MAX_VIS_DIST) if math.isfinite(dist_m) and dist_m > 0 else None
    if d is None:
        return pts
    for a_deg in np.linspace(a_min_deg, a_max_deg, n_rays):
        a = math.radians(a_deg)
        wa = heading + a            # world angle
        pts.append((x + d * math.sin(wa),
                     y + d * math.cos(wa)))
    return pts


def precompute(steps):
    """Return  poses[(x,y,h)], wall_pts[[(wx,wy),...]], footprints[[(wx,wy),...]]"""
    x, y, h = 0.0, 0.0, 0.0
    poses, wall_pts, footprints = [], [], []

    grid_sz = 0.05
    visited_cells = set()

    for step in steps:
        poses.append((x, y, h))

        sectors = step.get("lidar", {}).get("sector_distance_m", {})
        sw = []   # wall boundary points this tick (spatially filtered)
        fp = []   # ordered footprint polygon this tick

        # Build a full-circle ordered polygon for the footprint and
        # collect wall points for the accumulated map.
        for sec_name, a_min, a_max in SECTOR_ARCS:
            dist = sectors.get(sec_name)
            if dist is None or not math.isfinite(dist) or dist <= 0:
                continue
            seg = _project_sector(x, y, h, dist, a_min, a_max, RAYS_PER_SECTOR)
            fp.extend(seg)
            
            # Spatial filter for the permanent walls (circuit shape)
            for wx, wy in seg:
                cx = int(math.floor(wx / grid_sz))
                cy = int(math.floor(wy / grid_sz))
                if (cx, cy) not in visited_cells:
                    visited_cells.add((cx, cy))
                    sw.append((wx, wy))

        wall_pts.append(sw)
        footprints.append(fp)

        # dead-reckon for next step  (heading=0 → +Y → "up" on screen)
        cmd = step.get("command", {})
        dt  = step.get("dt_s", 0.1)
        vx  = cmd.get("published_linear_x", 0.0)
        wz  = cmd.get("published_angular_z", 0.0)

        h += wz * dt
        x += vx * math.sin(h) * dt   # sin for x (left-right)
        y += vx * math.cos(h) * dt   # cos for y (forward)

    return poses, wall_pts, footprints


# ═══════════════════════════════════════════════════════════════════════════
# Viewport — fixed across the entire playback
# ═══════════════════════════════════════════════════════════════════════════
def viewport(poses, wall_pts, map_w, map_h, margin=50):
    xs, ys = [], []
    for px, py, _ in poses:
        xs.append(px); ys.append(py)
    for wps in wall_pts:
        for wx, wy in wps:
            xs.append(wx); ys.append(wy)
    if not xs:
        return 100.0, map_w // 2, map_h // 2
    pad = 0.4
    xmin, xmax = min(xs) - pad, max(xs) + pad
    ymin, ymax = min(ys) - pad, max(ys) + pad
    ww = max(xmax - xmin, 0.5)
    wh = max(ymax - ymin, 0.5)
    sc = min((map_w - 2 * margin) / ww, (map_h - 2 * margin) / wh)
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    ox = map_w // 2 - int(cx * sc)
    oy = map_h // 2 + int(cy * sc)   # Y flipped for screen
    return sc, ox, oy


def w2px(x, y, sc, ox, oy):
    return int(x * sc + ox), int(-y * sc + oy)


# ═══════════════════════════════════════════════════════════════════════════
# Drawing helpers
# ═══════════════════════════════════════════════════════════════════════════
def _grid(img, sc, ox, oy, mw, mh):
    step = 1.0
    if sc < 40: step = 2.0
    if sc > 200: step = 0.5
    xl = (0 - ox) / sc;  xr = (mw - ox) / sc
    yt = -(0 - oy) / sc; yb = -(mh - oy) / sc
    g = math.floor(xl / step) * step
    while g <= xr:
        px, _ = w2px(g, 0, sc, ox, oy)
        if 0 <= px < mw:
            cv2.line(img, (px, 0), (px, mh), GRID_LINE, 1)
            cv2.putText(img, f"{g:.0f}", (px + 2, mh - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, GRID_TXT, 1, cv2.LINE_AA)
        g += step
    g = math.floor(yb / step) * step
    while g <= yt:
        _, py = w2px(0, g, sc, ox, oy)
        if 0 <= py < mh:
            cv2.line(img, (0, py), (mw, py), GRID_LINE, 1)
            cv2.putText(img, f"{g:.0f}", (4, py - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, GRID_TXT, 1, cv2.LINE_AA)
        g += step


def _walls(img, wpts, idx, sc, ox, oy):
    for si in range(idx + 1):
        for wx, wy in wpts[si]:
            cv2.circle(img, w2px(wx, wy, sc, ox, oy), 2, WALL_NEW, -1)


def _footprint(img, fp_pts, sc, ox, oy):
    """Draw the current LiDAR free-space polygon (translucent fill + border)."""
    if len(fp_pts) < 3:
        return
    pxs = np.array([w2px(wx, wy, sc, ox, oy) for wx, wy in fp_pts], dtype=np.int32)
    overlay = img.copy()
    cv2.fillPoly(overlay, [pxs], FOOTPRINT_FILL)
    cv2.addWeighted(overlay, 0.45, img, 0.55, 0, img)
    cv2.polylines(img, [pxs], True, FOOTPRINT_EDGE, 1, cv2.LINE_AA)


def _trail(img, poses, up_to, sc, ox, oy):
    if up_to < 1:
        return
    pts = [w2px(p[0], p[1], sc, ox, oy) for p in poses[:up_to + 1]]
    cv2.polylines(img, [np.array(pts, np.int32)], False, TRAIL, 2, cv2.LINE_AA)
    cv2.circle(img, pts[0], 5, TRAIL_START, -1, cv2.LINE_AA)


def _robot(img, pose, sc, ox, oy):
    x, y, h = pose
    cx, cy = w2px(x, y, sc, ox, oy)
    sz = max(10, int(0.18 * sc))
    tri = []
    for a in [h, h + math.radians(140), h - math.radians(140)]:
        tri.append((int(cx + sz * math.sin(a)),
                     int(cy - sz * math.cos(a))))
    np_tri = np.array(tri, np.int32)
    cv2.fillConvexPoly(img, np_tri, ROBOT_FILL, cv2.LINE_AA)
    cv2.polylines(img, [np_tri], True, ROBOT_EDGE, 2, cv2.LINE_AA)


def _velocity(img, pose, step, sc, ox, oy):
    x, y, h = pose
    cx, cy = w2px(x, y, sc, ox, oy)
    vx = step.get("command", {}).get("published_linear_x", 0.0)
    wz = step.get("command", {}).get("published_angular_z", 0.0)
    arrow_len = max(vx * sc * 3.5, 5)
    fh = h + wz * 0.35          # slight look-ahead for visual turn hint
    ex = int(cx + arrow_len * math.sin(fh))
    ey = int(cy - arrow_len * math.cos(fh))
    cv2.arrowedLine(img, (cx, cy), (ex, ey), VEL_ARROW, 2, cv2.LINE_AA, tipLength=0.3)


# ═══════════════════════════════════════════════════════════════════════════
# Dashboard
# ═══════════════════════════════════════════════════════════════════════════
def _dash(img, step, meta, fi, n, cw, ch):
    dx = cw - DASH_W
    cv2.rectangle(img, (dx, 0), (cw, ch), DASH_BG, -1)
    cv2.line(img, (dx, 0), (dx, ch), DASH_SEP, 2)

    y, lh = 26, 20
    xl = dx + 12

    def label(t, yy, c=TXT_DIM, s=0.42):
        cv2.putText(img, t, (xl, yy), cv2.FONT_HERSHEY_SIMPLEX, s, c, 1, cv2.LINE_AA)
    def val(t, yy, c=TXT, s=0.52):
        cv2.putText(img, t, (xl, yy), cv2.FONT_HERSHEY_SIMPLEX, s, c, 1, cv2.LINE_AA)

    # scenario
    sc_name = step.get("scenario", meta.get("scenario", ""))
    val(sc_name, y, (180, 200, 255), 0.55); y += lh + 4

    # time
    ts = step.get("time_s", 0.0)
    label("Time", y); y += lh - 5
    val(f"{ts:.1f}s   frame {fi+1}/{n}", y, s=0.42); y += lh + 4

    # state
    state = step.get("state", "?")
    label("State", y); y += lh - 5
    val(state, y, STATE_CLR.get(state, TXT), 0.55); y += lh + 2

    # reason
    reason = step.get("reason", "")
    label("Reason", y); y += lh - 5
    if len(reason) > 26: reason = reason[:24] + ".."
    val(reason, y, s=0.40); y += lh + 4

    # module
    nm = step.get("nav", {}).get("module", meta.get("nav_module", ""))
    pf = step.get("profile_name", meta.get("profile_name", ""))
    label("Module / Profile", y); y += lh - 5
    val(f"{nm}  ({pf})", y, s=0.36); y += lh + 4

    # velocities
    cmd = step.get("command", {})
    vx = cmd.get("published_linear_x", 0.0)
    wz = cmd.get("published_angular_z", 0.0)
    label("Velocity", y); y += lh - 5
    val(f"v_x: {vx:+.3f} m/s", y, s=0.42); y += lh - 3
    val(f"w_z: {wz:+.3f} rad/s", y, s=0.42); y += lh + 4

    # lidar
    li = step.get("lidar", {})
    label("LiDAR (m)", y); y += lh - 5
    f_ = li.get("front_m", li.get("front_center_m", 0))
    fl = li.get("front_left_m", 0); fr = li.get("front_right_m", 0)
    l_ = li.get("left_m", 0);       r_ = li.get("right_m", 0)
    val(f"F: {f_:.2f}", y, s=0.40); y += lh - 4
    val(f"FL: {fl:.2f}   FR: {fr:.2f}", y, s=0.40); y += lh - 4
    val(f"L:  {l_:.2f}    R: {r_:.2f}", y, s=0.40); y += lh + 4

    # emergency
    em = step.get("emergency", {})
    if em.get("active"):
        label("⚠ EMERGENCY", y, TXT_EMERG); y += lh - 5
        val(str(em.get("reason", "")), y, TXT_EMERG, 0.40); y += lh + 2

    # turn
    tu = step.get("turn", {})
    if tu.get("turn_active"):
        label("Turn", y, TXT_TURN); y += lh - 5
        val(f"{tu.get('turn_direction','?')}  {tu.get('turn_phase','?')}", y, TXT_TURN, 0.40)
        y += lh + 2

    # signal
    sig = step.get("signal", {})
    if sig.get("fresh") or sig.get("direction", "none") != "none":
        label("Signal", y, TXT_TURN); y += lh - 5
        val(f"{sig.get('direction','?')}  conf={sig.get('confidence',0):.2f}", y, TXT_TURN, 0.40)
        y += lh + 2

    # qr
    qr = step.get("qr", {})
    if qr.get("visible") or qr.get("content"):
        label("QR", y, (200, 125, 55)); y += lh - 5
        val(str(qr.get("content", "scanning…")), y, (200, 125, 55), 0.40); y += lh + 2

    # ── legend ──
    ly = ch - 95
    cv2.line(img, (xl, ly - 8), (cw - 12, ly - 8), DASH_SEP, 1)
    label("Legend", ly, s=0.38); ly += lh - 3
    for clr, txt in [(WALL_NEW, "Walls (LiDAR)"), (TRAIL, "Robot trail"),
                      (ROBOT_FILL, "Robot"), (VEL_ARROW, "Velocity"),
                      (FOOTPRINT_EDGE, "Free space")]:
        cv2.circle(img, (xl + 6, ly - 4), 5, clr, -1)
        cv2.putText(img, txt, (xl + 18, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.36, TXT, 1, cv2.LINE_AA)
        ly += lh - 5
    ly += 6
    cv2.putText(img, "SPC:pause  A/D:step  Q:quit", (xl, ly),
                cv2.FONT_HERSHEY_SIMPLEX, 0.30, TXT_DIM, 1, cv2.LINE_AA)
    ly += 12
    cv2.putText(img, "L:toggle lidar  C:toggle circuit", (xl, ly),
                cv2.FONT_HERSHEY_SIMPLEX, 0.30, TXT_DIM, 1, cv2.LINE_AA)


# ═══════════════════════════════════════════════════════════════════════════
# Frame renderer
# ═══════════════════════════════════════════════════════════════════════════
def frame(fi, poses, wpts, fps, steps, meta, sc, ox, oy, cw, ch, show_lidar=True, show_circuit=True):
    img = np.full((ch, cw, 3), BG, dtype=np.uint8)
    mw = cw - DASH_W
    _grid(img, sc, ox, oy, mw, ch)
    if show_lidar:
        _walls(img, wpts[:fi + 1], fi, sc, ox, oy)
    if show_circuit:
        _footprint(img, fps[fi], sc, ox, oy)
    _trail(img, poses, fi, sc, ox, oy)
    _robot(img, poses[fi], sc, ox, oy)
    _velocity(img, poses[fi], steps[fi], sc, ox, oy)
    _dash(img, steps[fi], meta, fi, len(steps), cw, ch)
    return img


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="Nav JSONL visualizer (world-frame)")
    ap.add_argument("log_file")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--save", type=str, help="save to .mp4")
    ap.add_argument("--no-display", action="store_true")
    ap.add_argument("--width", type=int, default=1100)
    ap.add_argument("--height", type=int, default=700)
    args = ap.parse_args()

    if not os.path.exists(args.log_file):
        print(f"Error: {args.log_file} not found."); sys.exit(1)

    print(f"Loading {args.log_file} …")
    meta, steps = load_log(args.log_file)
    print(f"  {len(steps)} steps")
    if not steps:
        print("  No steps."); return

    print("  Precomputing trajectory + walls …")
    poses, wpts, fps = precompute(steps)

    cw, ch = args.width, args.height
    mw = cw - DASH_W
    sc, ox, oy = viewport(poses, wpts, mw, ch)
    print(f"  scale={sc:.1f} px/m   offset=({ox},{oy})")

    writer = None
    if args.save:
        writer = cv2.VideoWriter(args.save,
                                 cv2.VideoWriter_fourcc(*"mp4v"),
                                 args.fps, (cw, ch))
        print(f"  Writing → {args.save}")

    if not args.no_display:
        print("  SPACE=pause  A/D=step  Q=quit")
        print("  L=toggle lidar   C=toggle circuit")

    delay = max(1, 1000 // args.fps)
    paused = False
    show_lidar = True
    show_circuit = True
    i = 0

    while i < len(steps):
        f = frame(i, poses, wpts, fps, steps, meta, sc, ox, oy, cw, ch, show_lidar, show_circuit)
        if writer:
            writer.write(f)
        if not args.no_display:
            cv2.imshow("Nav Visualizer", f)
            k = cv2.waitKey(0 if paused else delay) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord(" "):
                paused = not paused; continue
            if k in (ord("l"), ord("L")):
                show_lidar = not show_lidar; continue
            if k in (ord("c"), ord("C")):
                show_circuit = not show_circuit; continue
            if k in (ord("a"), 81, 2):
                i = max(0, i - 1); paused = True; continue
            if k in (ord("d"), 83, 3):
                i = min(len(steps) - 1, i + 1); paused = True; continue
        if not paused or args.no_display:
            i += 1

    if writer:
        writer.release(); print(f"  Saved {args.save}")
    if not args.no_display:
        cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
