"""
path_planner.py  —  yarn-wrapping path planner
================================================
The robot releases yarn as it drives.  The yarn wraps around poles
like a belt around pulleys — it must stay TAUT at all times.

Tangent geometry:
  When yarn travels from source → pole (CCW wrap), the tangent line
  touches the circle on the side where continuing CCW keeps the yarn
  pressed against the pole.

  For CCW wrap (+1):
    ENTRY: yarn approaches from the CW side → offset = -wrap_sign
    EXIT:  yarn departs on the CCW side     → offset = +wrap_sign

  For CW wrap (-1): mirror of above.

  Between two poles (same wrap dir, same radius) the yarn runs along
  an external (parallel) tangent — both tangent points are at the same
  perpendicular offset from the centre-to-centre line.
"""

import math

# ==================================================================
# WORLD CONFIGURATION
# ==================================================================

POLES = {
    1: (-50,    0),
    2: (-200, -100),
    3: (-120,  120),
    4: ( 10,   150),
    5: ( 150, -150),
}

POLE_R  = 5
ROBOT_R = 17
SAFETY  = 12
ARC_R   = POLE_R + ROBOT_R + SAFETY

ROBOTS = {
    "A": {
        "start":   (0, 0),
        "seq":     [1, 3, 5, 2],
        "dir":     "ccw",
        "circles": 0,
    },
    "B": {
        "start":   (-120, 160),
        "seq":     [4, 2, 1, 3],
        "dir":     "cw",
        "circles": 0,
    },
}

TURN_WAIT = 30

# ==================================================================
# GEOMETRY
# ==================================================================

def dist(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def seg_pt_dist(ax, ay, bx, by, px, py):
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return dist(ax, ay, px, py)
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / (dx*dx + dy*dy)))
    return dist(ax + t*dx, ay + t*dy, px, py)

def norm_ang(a):
    while a > math.pi:   a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

# ==================================================================
# TANGENT POINTS  (correct for yarn tension)
# ==================================================================

def entry_tangent_from_point(src_x, src_y, cx, cy, r, wrap_sign):
    """
    Where yarn from (src) first touches circle (cx,cy,r) so that
    wrapping in wrap_sign direction keeps it taut.

    The yarn must arrive on the WRAP-DIRECTION side so that the
    arc immediately continues in the correct direction.
    """
    d = dist(src_x, src_y, cx, cy)
    if d <= r:
        ang = math.atan2(src_y - cy, src_x - cx)
        return (cx + r*math.cos(ang), cy + r*math.sin(ang))
    ang_to_src = math.atan2(src_y - cy, src_x - cx)
    offset = math.acos(r / d)
    # Touch on the wrap-direction side
    tang_ang = ang_to_src + wrap_sign * offset
    return (cx + r*math.cos(tang_ang), cy + r*math.sin(tang_ang))

def exit_tangent_to_point(cx, cy, r, dst_x, dst_y, wrap_sign):
    """
    Where yarn departs circle (cx,cy,r) heading toward (dst) so
    that the wrap stays taut.

    The yarn departs from the ANTI-WRAP side — the arc has swept
    past and the yarn peels off toward the next target.
    """
    d = dist(dst_x, dst_y, cx, cy)
    if d <= r:
        ang = math.atan2(dst_y - cy, dst_x - cx)
        return (cx + r*math.cos(ang), cy + r*math.sin(ang))
    ang_to_dst = math.atan2(dst_y - cy, dst_x - cx)
    offset = math.acos(r / d)
    # Depart from the anti-wrap side
    tang_ang = ang_to_dst - wrap_sign * offset
    return (cx + r*math.cos(tang_ang), cy + r*math.sin(tang_ang))

def transit_tangents(c1x, c1y, c2x, c2y, r, wrap_sign):
    """
    External tangent between two same-radius circles (same wrap dir).

    Exit from c1 is on the anti-wrap side (yarn peeling off after wrap).
    Entry to c2 is on the wrap side (yarn arriving to begin wrap).

    Returns (exit_on_c1, entry_on_c2).
    """
    d = dist(c1x, c1y, c2x, c2y)
    if d < 1e-6:
        return ((c1x + r, c1y), (c2x + r, c2y))
    base_ang = math.atan2(c2y - c1y, c2x - c1x)
    # Anti-wrap side perpendicular for exit from c1
    exit_ang  = base_ang - wrap_sign * math.pi / 2
    # Wrap side perpendicular for entry to c2
    entry_ang = base_ang - wrap_sign * math.pi / 2
    exit_pt  = (c1x + r*math.cos(exit_ang), c1y + r*math.sin(exit_ang))
    entry_pt = (c2x + r*math.cos(entry_ang), c2y + r*math.sin(entry_ang))
    return (exit_pt, entry_pt)

# ==================================================================
# COLLISION AVOIDANCE
# ==================================================================

def seg_clear_except(sx, sy, gx, gy, skip_poles):
    for pid, (px, py) in POLES.items():
        if pid in skip_poles:
            continue
        if seg_pt_dist(sx, sy, gx, gy, px, py) < ARC_R:
            return False
    return True

def safe_transit(sx, sy, gx, gy, skip_poles):
    if seg_clear_except(sx, sy, gx, gy, skip_poles):
        return [(gx, gy)]
    dx, dy = gx - sx, gy - sy
    length = math.hypot(dx, dy)
    if length == 0:
        return [(gx, gy)]
    nx, ny = -dy/length, dx/length
    mx, my = (sx+gx)/2.0, (sy+gy)/2.0
    for mult in (2.0, -2.0, 3.0, -3.0, 4.0, -4.0):
        wx = mx + nx*ARC_R*mult
        wy = my + ny*ARC_R*mult
        if (seg_clear_except(sx, sy, wx, wy, skip_poles) and
            seg_clear_except(wx, wy, gx, gy, skip_poles)):
            return [(wx, wy), (gx, gy)]
    for m1 in (3.0, -3.0, 4.0, -4.0):
        for m2 in (3.0, -3.0, 4.0, -4.0):
            w1x = sx + dx*0.33 + nx*ARC_R*m1
            w1y = sy + dy*0.33 + ny*ARC_R*m1
            w2x = sx + dx*0.66 + nx*ARC_R*m2
            w2y = sy + dy*0.66 + ny*ARC_R*m2
            if (seg_clear_except(sx, sy, w1x, w1y, skip_poles) and
                seg_clear_except(w1x, w1y, w2x, w2y, skip_poles) and
                seg_clear_except(w2x, w2y, gx, gy, skip_poles)):
                return [(w1x, w1y), (w2x, w2y), (gx, gy)]
    return [(mx + nx*ARC_R*5, my + ny*ARC_R*5), (gx, gy)]

# ==================================================================
# ARC GENERATION
# ==================================================================

def arc_points(cx, cy, r, entry_ang, exit_ang, wrap_sign,
               n_full_circles=1, steps_per_circle=120):
    """
    Arc from entry_ang to exit_ang in wrap_sign direction,
    with n_full_circles complete loops in between.
    """
    raw = norm_ang(exit_ang - entry_ang)
    if wrap_sign > 0:
        partial = raw if raw > 0 else raw + 2*math.pi
    else:
        partial = -raw if raw < 0 else -(raw - 2*math.pi)
        partial = abs(partial)

    total = n_full_circles * 2*math.pi + partial
    n = max(steps_per_circle, int(total / (2*math.pi) * steps_per_circle))

    pts = []
    for i in range(1, n+1):
        a = entry_ang + wrap_sign * (i/n) * total
        pts.append((cx + r*math.cos(a), cy + r*math.sin(a)))
    return pts

# ==================================================================
# PATH BUILDER
# ==================================================================

def build_path(robot_id):
    cfg    = ROBOTS[robot_id]
    seq    = cfg["seq"]
    sign   = 1 if cfg["dir"] == "ccw" else -1
    n_circ = cfg["circles"]
    sx, sy = cfg["start"]

    pts = [{"x": sx, "y": sy, "type": "start", "pole": 0}]
    n = len(seq)

    entries = {}
    exits   = {}

    for idx in range(n):
        pid = seq[idx]
        px, py = POLES[pid]

        # ENTRY
        if idx == 0:
            entries[idx] = entry_tangent_from_point(sx, sy, px, py, ARC_R, sign)
        else:
            prev_px, prev_py = POLES[seq[idx-1]]
            _, entry = transit_tangents(prev_px, prev_py, px, py, ARC_R, sign)
            entries[idx] = entry

        # EXIT
        if idx == n - 1:
            exits[idx] = entries[idx]
        else:
            next_px, next_py = POLES[seq[idx+1]]
            ex_pt, _ = transit_tangents(px, py, next_px, next_py, ARC_R, sign)
            exits[idx] = ex_pt

    cx, cy = sx, sy
    for idx in range(n):
        pid = seq[idx]
        px, py = POLES[pid]
        entry = entries[idx]
        exit_pt = exits[idx]

        skip = {pid}
        if idx > 0: skip.add(seq[idx-1])
        route = safe_transit(cx, cy, entry[0], entry[1], skip)
        for (wx, wy) in route:
            pts.append({"x": wx, "y": wy, "type": "move", "pole": pid})
            cx, cy = wx, wy

        entry_ang = math.atan2(entry[1] - py, entry[0] - px)
        exit_ang  = math.atan2(exit_pt[1] - py, exit_pt[0] - px)
        arc = arc_points(px, py, ARC_R, entry_ang, exit_ang, sign,
                         n_full_circles=n_circ)
        for (ax, ay) in arc:
            pts.append({"x": ax, "y": ay, "type": "wrap", "pole": pid})
        cx, cy = exit_pt

    return pts

# ==================================================================
# DISTANCE INDEXING
# ==================================================================

def path_distances(pts):
    cum = [0.0]
    for i in range(1, len(pts)):
        d = dist(pts[i-1]["x"], pts[i-1]["y"], pts[i]["x"], pts[i]["y"])
        cum.append(cum[-1] + d)
    return cum, cum[-1]

def pos_at_dist(pts, cum, d):
    d = max(0.0, min(d, cum[-1]))
    for i in range(1, len(cum)):
        if d <= cum[i]:
            seg_len = cum[i] - cum[i-1]
            t = (d - cum[i-1]) / seg_len if seg_len > 0 else 0.0
            x = pts[i-1]["x"] + (pts[i]["x"] - pts[i-1]["x"]) * t
            y = pts[i-1]["y"] + (pts[i]["y"] - pts[i-1]["y"]) * t
            return x, y, pts[i]
    last = pts[-1]
    return last["x"], last["y"], last

# ==================================================================
# COORDINATE CONVERSION
# ==================================================================

def world_to_local(wx, wy, robot_id):
    sx, sy = ROBOTS[robot_id]["start"]
    return (wx - sx, wy - sy)

def path_in_local(pts, robot_id):
    sx, sy = ROBOTS[robot_id]["start"]
    return [{**p, "x": p["x"] - sx, "y": p["y"] - sy} for p in pts]