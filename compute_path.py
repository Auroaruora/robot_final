"""
compute_path.py  —  Unified path planner for Robot A and Robot B
==================================================================
Single source of truth. Generates per-robot data files only:
  - path_<RID>.csv          (full waypoint list, WORLD frame, for debugging)
  - path_<RID>_robot.csv    (reduced waypoint list, LOCAL frame — what
                             run_robot.py loads at startup)
  - run_config_<RID>.json   (all per-robot constants run_robot.py needs:
                             direction, ARC_R, lights, bluetooth address, etc.)

The driver code itself lives in a hand-maintained run_robot.py and is
never regenerated. Set ROBOT_ID at the top of run_robot.py to choose
which robot to drive.

Exposed to the website (Pyodide) via build_path(rid),
reduce_for_robot(pts), robot_csv_text(rid, robot_pts),
and run_config_text(rid, robot_pts).
"""

import math, csv, os, io, json

# ==================================================================
# WORLD CONFIGURATION  (shared by both robots)
# ==================================================================

POLES = {
    1: (   0,   75),
    2: (-100, -200),
    3: (-120,  120),
    4: ( 175,   75),
    5: ( 150, -150),
}

POLE_R  = 5
ROBOT_R = 33
SAFETY  = 15
ARC_R   = POLE_R + ROBOT_R + SAFETY

# ==================================================================
# PER-ROBOT CONFIGURATION
# ==================================================================
# dir: "ccw" (counter-clockwise, arc_left)  or  "cw" (clockwise, arc_right)
# sign is derived: +1 for ccw, -1 for cw.

ROBOTS = {
    "A": {
        "start":   (0, 0),
        "seq":     [1, 4],
        "dir":     "ccw",
        "circles": 1,
    },
    "B": {
        "start":   (-120, 200),
        "seq":     [3, 2, 5, 4, 1, 5, 2],
        "dir":     "cw",
        "circles": 1,
    },
}

# Per-robot Bluetooth (BLE) addresses. On macOS these are the
# Create3's system-assigned UUID; on other platforms they may be
# the Bluetooth MAC. Flowed through into run_config_<RID>.json so
# run_robot.py can do Create3(Bluetooth(address=...)) deterministically.
BLUETOOTH_ADDRESSES = {
    "A": "38E42E1C-5EA3-F705-C05D-43EB8600C88B",
    "B": "BA5D90F5-FF5D-4870-0CCD-5034A75EE39F",
}

# How aggressively to reduce wrap-arc points for the robot driver.
# (Keep every Nth point within a pole's wrap group; first & last always kept.)
ROBOT_WRAP_STEP = 10

# ==================================================================
# GEOMETRY HELPERS
# ==================================================================

def dist(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def seg_pt_dist(ax, ay, bx, by, px, py):
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return dist(ax, ay, px, py)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return dist(ax + t * dx, ay + t * dy, px, py)

def norm_ang(a):
    while a > math.pi:    a -= 2 * math.pi
    while a <= -math.pi:  a += 2 * math.pi
    return a

# ==================================================================
# TANGENT POINTS
# ==================================================================

def entry_tangent_from_point(src_x, src_y, cx, cy, r, wrap_sign):
    d = dist(src_x, src_y, cx, cy)
    if d <= r:
        ang = math.atan2(src_y - cy, src_x - cx)
        return (cx + r * math.cos(ang), cy + r * math.sin(ang))
    ang_to_src = math.atan2(src_y - cy, src_x - cx)
    offset = math.acos(r / d)
    tang_ang = ang_to_src + wrap_sign * offset
    return (cx + r * math.cos(tang_ang), cy + r * math.sin(tang_ang))

def exit_tangent_to_point(cx, cy, r, dst_x, dst_y, wrap_sign):
    d = dist(dst_x, dst_y, cx, cy)
    if d <= r:
        ang = math.atan2(dst_y - cy, dst_x - cx)
        return (cx + r * math.cos(ang), cy + r * math.sin(ang))
    ang_to_dst = math.atan2(dst_y - cy, dst_x - cx)
    offset = math.acos(r / d)
    tang_ang = ang_to_dst - wrap_sign * offset
    return (cx + r * math.cos(tang_ang), cy + r * math.sin(tang_ang))

def transit_tangents(c1x, c1y, c2x, c2y, r, wrap_sign):
    d = dist(c1x, c1y, c2x, c2y)
    if d < 1e-6:
        return ((c1x + r, c1y), (c2x + r, c2y))
    base_ang = math.atan2(c2y - c1y, c2x - c1x)
    exit_ang  = base_ang - wrap_sign * math.pi / 2
    entry_ang = base_ang - wrap_sign * math.pi / 2
    exit_pt  = (c1x + r * math.cos(exit_ang),  c1y + r * math.sin(exit_ang))
    entry_pt = (c2x + r * math.cos(entry_ang), c2y + r * math.sin(entry_ang))
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
    nx, ny = -dy / length, dx / length
    mx, my = (sx + gx) / 2.0, (sy + gy) / 2.0
    for mult in (2.0, -2.0, 3.0, -3.0, 4.0, -4.0):
        wx = mx + nx * ARC_R * mult
        wy = my + ny * ARC_R * mult
        if (seg_clear_except(sx, sy, wx, wy, skip_poles) and
            seg_clear_except(wx, wy, gx, gy, skip_poles)):
            return [(wx, wy), (gx, gy)]
    for m1 in (3.0, -3.0, 4.0, -4.0):
        for m2 in (3.0, -3.0, 4.0, -4.0):
            w1x = sx + dx * 0.33 + nx * ARC_R * m1
            w1y = sy + dy * 0.33 + ny * ARC_R * m1
            w2x = sx + dx * 0.66 + nx * ARC_R * m2
            w2y = sy + dy * 0.66 + ny * ARC_R * m2
            if (seg_clear_except(sx, sy, w1x, w1y, skip_poles) and
                seg_clear_except(w1x, w1y, w2x, w2y, skip_poles) and
                seg_clear_except(w2x, w2y, gx, gy, skip_poles)):
                return [(w1x, w1y), (w2x, w2y), (gx, gy)]
    return [(mx + nx * ARC_R * 5, my + ny * ARC_R * 5), (gx, gy)]

# ==================================================================
# ARC GENERATION
# ==================================================================

def arc_points(cx, cy, r, entry_ang, exit_ang, wrap_sign,
               n_full_circles=1, steps_per_circle=120):
    raw = norm_ang(exit_ang - entry_ang)
    if wrap_sign > 0:
        partial = raw if raw > 0 else raw + 2 * math.pi
    else:
        partial = -raw if raw < 0 else -(raw - 2 * math.pi)
        partial = abs(partial)
    total = n_full_circles * 2 * math.pi + partial
    total_deg = math.degrees(total)
    n = max(steps_per_circle, int(total / (2 * math.pi) * steps_per_circle))
    pts = []
    for i in range(1, n + 1):
        a = entry_ang + wrap_sign * (i / n) * total
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts, total_deg

# ==================================================================
# PATH BUILDER  (per-robot)
# ==================================================================

def build_path(rid):
    """Generate the full (pre-reduction) waypoint list for robot `rid`."""
    cfg = ROBOTS[rid]
    start   = cfg["start"]
    seq     = cfg["seq"]
    sign    = 1 if cfg["dir"] == "ccw" else -1
    circles = cfg["circles"]

    sx, sy = start
    pts = [{"x": sx, "y": sy, "type": "start", "pole": 0, "arc_deg": 0}]
    n = len(seq)

    entries, exits = {}, {}

    for idx in range(n):
        pid = seq[idx]
        px, py = POLES[pid]

        if idx == 0:
            entries[idx] = entry_tangent_from_point(sx, sy, px, py, ARC_R, sign)
        else:
            prev_px, prev_py = POLES[seq[idx - 1]]
            _, entry = transit_tangents(prev_px, prev_py, px, py, ARC_R, sign)
            entries[idx] = entry

        if idx == n - 1:
            exits[idx] = entries[idx]
        else:
            next_px, next_py = POLES[seq[idx + 1]]
            ex_pt, _ = transit_tangents(px, py, next_px, next_py, ARC_R, sign)
            exits[idx] = ex_pt

    cx, cy = sx, sy
    for idx in range(n):
        pid = seq[idx]
        px, py = POLES[pid]
        entry   = entries[idx]
        exit_pt = exits[idx]

        skip = {pid}
        if idx > 0:
            skip.add(seq[idx - 1])
        route = safe_transit(cx, cy, entry[0], entry[1], skip)
        for (wx, wy) in route:
            pts.append({"x": wx, "y": wy, "type": "move", "pole": pid, "arc_deg": 0})
            cx, cy = wx, wy

        entry_ang = math.atan2(entry[1] - py, entry[0] - px)
        exit_ang  = math.atan2(exit_pt[1] - py, exit_pt[0] - px)
        arc, arc_deg = arc_points(px, py, ARC_R, entry_ang, exit_ang, sign,
                                  n_full_circles=circles)
        for j, (ax, ay) in enumerate(arc):
            wp = {"x": ax, "y": ay, "type": "wrap", "pole": pid, "arc_deg": 0}
            if j == 0:
                wp["arc_deg"] = round(arc_deg, 2)
            pts.append(wp)
        cx, cy = exit_pt

    return pts

# ==================================================================
# WAYPOINT REDUCTION for robot driver
# ==================================================================

def reduce_for_robot(pts, wrap_step=ROBOT_WRAP_STEP):
    """Thin out dense arc points — robot doesn't need 120 waypoints per circle."""
    robot_pts = []
    i = 0
    while i < len(pts):
        p = pts[i]
        if p["type"] != "wrap":
            robot_pts.append(p)
            i += 1
            continue
        pole = p["pole"]
        group_start = i
        while i < len(pts) and pts[i]["type"] == "wrap" and pts[i]["pole"] == pole:
            i += 1
        group = pts[group_start:i]
        if len(group) <= 2:
            robot_pts.extend(group)
            continue
        first = dict(group[0])
        robot_pts.append(first)
        for j in range(wrap_step, len(group) - 1, wrap_step):
            mid = dict(group[j])
            mid["arc_deg"] = 0
            robot_pts.append(mid)
        last = dict(group[-1])
        last["arc_deg"] = 0
        robot_pts.append(last)
    return robot_pts

# ==================================================================
# CSV EXPORT
# ==================================================================

def save_csv(pts, filename):
    filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    with open(filepath, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "x", "y", "type", "pole", "arc_deg"])
        for i, p in enumerate(pts):
            w.writerow([i, round(p["x"], 4), round(p["y"], 4),
                        p["type"], p["pole"], p.get("arc_deg", 0)])
    return filepath

def robot_csv_text(rid, robot_pts):
    """Return the path_<rid>_robot.csv content in the robot's LOCAL frame.

    run_robot_<rid>.py loads this file at startup. reset_navigation()
    makes the robot's odometry origin equal its starting pose, so all
    coords must be translated by subtracting the world start position.
    """
    cfg = ROBOTS[rid]
    sx, sy = cfg["start"]
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(["step", "x", "y", "type", "pole", "arc_deg"])
    for i, p in enumerate(robot_pts):
        w.writerow([i, round(p["x"] - sx, 4), round(p["y"] - sy, 4),
                    p["type"], p["pole"], p.get("arc_deg", 0)])
    return buf.getvalue()

# ==================================================================
# RUN CONFIG  (per-robot JSON consumed by run_robot.py)
# ==================================================================

def run_config_dict(rid, robot_pts):
    """Build the run_config_<rid>.json payload for run_robot.py.

    Includes every per-robot value the driver needs at runtime.
    """
    cfg = ROBOTS[rid]
    sx, sy = cfg["start"]
    direction = cfg["dir"]

    arc_cmd = "arc_left" if direction == "ccw" else "arc_right"
    if rid == "A":
        light_rgb, color_name = [0, 255, 0], "Green"
    else:
        light_rgb, color_name = [0, 0, 255], "Blue"

    # Distance is computed from the full (pre-reduction) pts ideally, but
    # robot_pts is what the driver loads — use that so the number matches
    # what run_robot.py will see.
    total = 0.0
    for i in range(1, len(robot_pts)):
        total += math.hypot(robot_pts[i]["x"] - robot_pts[i-1]["x"],
                            robot_pts[i]["y"] - robot_pts[i-1]["y"])

    return {
        "rid":                rid,
        "dir":                direction,
        "arc_cmd":            arc_cmd,
        "arc_r":              ARC_R,
        "circles":            cfg["circles"],
        "light_rgb":          light_rgb,
        "color_name":         color_name,
        "seq":                list(cfg["seq"]),
        "start_world":        [sx, sy],
        "path_csv":           "path_" + rid + "_robot.csv",
        "bluetooth_address":  BLUETOOTH_ADDRESSES.get(rid, ""),
        "waypoint_count":     len(robot_pts),
        "total_distance_cm":  round(total, 2),
    }


def run_config_text(rid, robot_pts):
    """JSON text for run_config_<rid>.json (what both compute_path.py and
    the website write to disk)."""
    return json.dumps(run_config_dict(rid, robot_pts), indent=2)

# ==================================================================
# MAIN — regenerate all six output files
# ==================================================================

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    for rid in ("A", "B"):
        pts = build_path(rid)
        robot_pts = reduce_for_robot(pts)

        total = sum(
            math.hypot(pts[i]["x"] - pts[i-1]["x"], pts[i]["y"] - pts[i-1]["y"])
            for i in range(1, len(pts))
        )

        save_csv(pts, "path_" + rid + ".csv")

        robot_csv_path = os.path.join(script_dir, "path_" + rid + "_robot.csv")
        with open(robot_csv_path, "w", newline="") as f:
            f.write(robot_csv_text(rid, robot_pts))

        config_path = os.path.join(script_dir, "run_config_" + rid + ".json")
        with open(config_path, "w") as f:
            f.write(run_config_text(rid, robot_pts))

        cfg = ROBOTS[rid]
        print("Robot " + rid + " | start=" + str(cfg["start"]) +
              " seq=" + str(cfg["seq"]) +
              " dir=" + cfg["dir"] +
              " circles=" + str(cfg["circles"]))
        print("  full: " + str(len(pts)) + " wpts, " +
              str(round(total, 1)) + " cm | robot: " + str(len(robot_pts)) + " wpts")
        print("  Saved path_" + rid + ".csv, path_" + rid + "_robot.csv, run_config_" + rid + ".json")

if __name__ == "__main__":
    main()