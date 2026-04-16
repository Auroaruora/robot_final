"""
compute_path.py  —  Unified path planner for Robot A and Robot B
==================================================================
Single source of truth. Generates, for BOTH robots:
  - path_<RID>.csv          (full waypoint list, for debugging)
  - path_<RID>_robot.csv    (reduced waypoint list, what the robot uses)
  - run_robot_<RID>.py      (playground-ready driver, path embedded,
                             plus IR-based pole centering using the
                             world POLES dict as ground truth)

Run this file directly to regenerate all six output files at once.
Also exposed to the website (Pyodide) via build_path(rid) and
generate_driver_code(rid, robot_pts).
"""

import math, csv, os

# ==================================================================
# WORLD CONFIGURATION  (shared by both robots)
# ==================================================================

POLES = {
    1: (   0,   75),
    2: (-100, -200),
    3: (-120,  120),
    4: ( 175,   70),
    5: ( 150, -150),
}

POLE_R  = 5
ROBOT_R = 33
SAFETY  = 20
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
        "seq":     [3, 1, 4, 5],
        "dir":     "cw",
        "circles": 1,
    },
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

# ==================================================================
# DRIVER CODE GENERATION
# ==================================================================
# Emits a fully self-contained run_robot_<rid>.py suitable for
# pasting into the iRobot Python Playground. The generated file
# includes:
#   - embedded PATH (local-frame coordinates)
#   - embedded POLE_POSITIONS (local-frame, used by center_on_pole)
#   - IR-centering helper that runs before each wrap

def _path_literal(local_pts):
    """Render the PATH dict list as valid Python source."""
    lines = []
    for p in local_pts:
        lines.append(
            '    {"x": ' + str(p["x"]) +
            ', "y": ' + str(p["y"]) +
            ', "type": "' + p["type"] + '"' +
            ', "pole": ' + str(p["pole"]) +
            ', "arc_deg": ' + str(p["arc_deg"]) + '},'
        )
    return "[\n" + "\n".join(lines) + "\n]"


def _pole_positions_literal(rid):
    """Render POLE_POSITIONS in the robot's local frame.

    The robot does reset_navigation() at startup — its origin is
    its own starting pose. So we translate world pole coords by
    subtracting the robot's world start position. Only include poles
    that appear in this robot's sequence (the only ones it centers on).
    """
    cfg = ROBOTS[rid]
    sx, sy = cfg["start"]
    used = sorted(set(cfg["seq"]))
    lines = []
    for pid in used:
        px, py = POLES[pid]
        lines.append(
            "    " + str(pid) + ": (" +
            str(round(px - sx, 4)) + ", " +
            str(round(py - sy, 4)) + "),"
        )
    return "{\n" + "\n".join(lines) + "\n}"


def generate_driver_code(rid, robot_pts):
    """Return the full source text of run_robot_<rid>.py."""
    cfg = ROBOTS[rid]
    direction = cfg["dir"]
    circles   = cfg["circles"]
    sx, sy    = cfg["start"]

    # Translate path waypoints into the robot's local frame
    local = [{
        "x": round(p["x"] - sx, 4),
        "y": round(p["y"] - sy, 4),
        "type": p["type"],
        "pole": p["pole"],
        "arc_deg": p.get("arc_deg", 0),
    } for p in robot_pts]

    path_lit = _path_literal(local)
    poles_lit = _pole_positions_literal(rid)

    arc_cmd = "arc_left" if direction == "ccw" else "arc_right"
    if rid == "A":
        light_rgb, color_name = "0, 255, 0", "Green"
    else:
        light_rgb, color_name = "0, 0, 255", "Blue"

    # Build the file as a list of lines (easier to read than one giant f-string)
    L = []
    L.append('"""')
    L.append('run_robot_' + rid + '.py  (auto-generated)')
    L.append('=====================================================')
    L.append('Path is embedded below - no CSV file needed.')
    L.append('Re-run compute_path.py or use the website to regenerate.')
    L.append('')
    L.append('TWO WRAPPING MODES - change WRAP_MODE to swap:')
    L.append('  "arc"      = ' + arc_cmd + ' using computed sweep angle (fast, more accurate)')
    L.append('  "navigate" = follow reduced waypoints via navigate_to (slow, most accurate)')
    L.append('')
    L.append('IR POLE CENTERING:')
    L.append('  Before each wrap, the robot turns to face the known pole position,')
    L.append('  then dithers a few degrees to find the peak IR return. This corrects')
    L.append('  accumulated heading drift from wheel odometry. Set CENTER_ON_POLE=False')
    L.append('  to disable.')
    L.append('"""')
    L.append('')
    L.append('from irobot_edu_sdk.backend.bluetooth import Bluetooth')
    L.append('from irobot_edu_sdk.robots import event, Create3')
    L.append('import math')
    L.append('')
    L.append('# ==================================================================')
    L.append('# CONFIG')
    L.append('# ==================================================================')
    L.append('')
    L.append('DIR     = "' + direction + '"')
    L.append('CIRCLES = ' + str(circles))
    L.append('ARC_R   = ' + str(ARC_R))
    L.append('')
    L.append('# ==================================================================')
    L.append('# WRAPPING MODE - CHANGE THIS TO SWAP')
    L.append('# ==================================================================')
    L.append('WRAP_MODE = "arc"')
    L.append('# WRAP_MODE = "navigate"')
    L.append('')
    L.append('# ==================================================================')
    L.append('# IR CENTERING  (fine heading correction before each wrap)')
    L.append('# ==================================================================')
    L.append('CENTER_ON_POLE    = True    # set False to disable')
    L.append('CENTER_SEARCH_DEG = 8       # +/- degrees to dither')
    L.append('CENTER_STEP_DEG   = 1       # degree increment per dither step')
    L.append('CENTER_SAMPLES    = 3       # IR samples averaged per step')
    L.append('CENTER_SENSOR_IDX = 3       # center sensor of 7-sensor IR array')
    L.append('')
    L.append('# Pole positions in the robot\'s LOCAL frame (world coords minus')
    L.append('# start position). reset_navigation() makes local = robot odom frame.')
    L.append('POLE_POSITIONS = ' + poles_lit)
    L.append('')
    L.append('# ==================================================================')
    L.append('# EMBEDDED PATH  (' + str(len(local)) + ' waypoints, reduced for robot)')
    L.append('# arc_deg: total sweep angle for each pole wrap (on first wrap pt)')
    L.append('# ==================================================================')
    L.append('')
    L.append('PATH = ' + path_lit)
    L.append('')
    L.append('# ==================================================================')
    L.append('# ROBOT DRIVER')
    L.append('# ==================================================================')
    L.append('')
    L.append('robot = Create3(Bluetooth())')
    L.append('')
    L.append('')
    L.append('async def center_on_pole(robot, pole_id):')
    L.append('    """Face the pole using known coords, then dither IR for peak."""')
    L.append('    if pole_id not in POLE_POSITIONS:')
    L.append('        print("  [center] no known position for pole " + str(pole_id) + ", skipping")')
    L.append('        return')
    L.append('')
    L.append('    pose = await robot.get_position()')
    L.append('    rx, ry, rh = pose.x, pose.y, pose.heading')
    L.append('')
    L.append('    px, py = POLE_POSITIONS[pole_id]')
    L.append('    target_heading = math.degrees(math.atan2(py - ry, px - rx))')
    L.append('')
    L.append('    delta = target_heading - rh')
    L.append('    while delta > 180:  delta -= 360')
    L.append('    while delta < -180: delta += 360')
    L.append('')
    L.append('    print("  [center] facing pole " + str(pole_id) +')
    L.append('          " (turning " + str(round(delta, 1)) + " deg)")')
    L.append('')
    L.append('    if delta >= 0:')
    L.append('        await robot.turn_left(delta)')
    L.append('    else:')
    L.append('        await robot.turn_right(-delta)')
    L.append('')
    L.append('    # Dither: turn right to leftmost edge of sweep, then step LEFT across it')
    L.append('    await robot.turn_right(CENTER_SEARCH_DEG)')
    L.append('')
    L.append('    best_reading = -1')
    L.append('    best_index   = 0')
    L.append('    num_steps    = int(2 * CENTER_SEARCH_DEG / CENTER_STEP_DEG)')
    L.append('')
    L.append('    for step in range(num_steps + 1):')
    L.append('        total = 0')
    L.append('        for _ in range(CENTER_SAMPLES):')
    L.append('            ir = await robot.get_ir_proximity()')
    L.append('            total += ir.sensors[CENTER_SENSOR_IDX]')
    L.append('        avg = total / CENTER_SAMPLES')
    L.append('        if avg > best_reading:')
    L.append('            best_reading = avg')
    L.append('            best_index   = step')
    L.append('        if step < num_steps:')
    L.append('            await robot.turn_left(CENTER_STEP_DEG)')
    L.append('')
    L.append('    back = (num_steps - best_index) * CENTER_STEP_DEG')
    L.append('    if back > 0:')
    L.append('        await robot.turn_right(back)')
    L.append('')
    L.append('    offset = (best_index * CENTER_STEP_DEG) - CENTER_SEARCH_DEG')
    L.append('    print("  [center] locked on pole " + str(pole_id) +')
    L.append('          " | best_ir=" + str(round(best_reading, 1)) +')
    L.append('          " | heading offset=" + str(round(offset, 1)) + " deg")')
    L.append('')
    L.append('')
    L.append('@event(robot.when_play)')
    L.append('async def play(robot):')
    L.append('    await robot.reset_navigation()')
    L.append('    await robot.set_lights_on_rgb(' + light_rgb + ')       # ' + color_name + ' = Robot ' + rid)
    L.append('')
    L.append('    print("Robot ' + rid + ' | dir=" + DIR + " mode=" + WRAP_MODE + " | " + str(len(PATH)) + " waypoints")')
    L.append('')
    L.append('    current_pole = None')
    L.append('')
    L.append('    for i, pt in enumerate(PATH):')
    L.append('        if pt["type"] == "start":')
    L.append('            continue')
    L.append('')
    L.append('        if pt["type"] == "move":')
    L.append('            if pt["pole"] != current_pole:')
    L.append('                current_pole = pt["pole"]')
    L.append('                print("\\n--- Heading to Pole " + str(current_pole) + " ---")')
    L.append('            await robot.navigate_to(pt["x"], pt["y"])')
    L.append('')
    L.append('        elif pt["type"] == "wrap":')
    L.append('')
    L.append('            prev = PATH[i - 1] if i > 0 else None')
    L.append('            is_first_wrap = (prev is None or prev["type"] != "wrap"')
    L.append('                             or prev["pole"] != pt["pole"])')
    L.append('')
    L.append('            if is_first_wrap and CENTER_ON_POLE:')
    L.append('                await center_on_pole(robot, pt["pole"])')
    L.append('')
    L.append('            # ARC MODE')
    L.append('            if WRAP_MODE == "arc":')
    L.append('                if is_first_wrap:')
    L.append('                    await robot.set_lights_spin_rgb(255, 115, 0)')
    L.append('                    sweep = pt.get("arc_deg", 360)')
    L.append('                    print("  Wrapping pole " + str(pt["pole"]) + " (" + DIR + ") [arc mode, " + str(round(sweep, 1)) + " deg]")')
    L.append('                    await robot.' + arc_cmd + '(sweep, ARC_R)')
    L.append('                    await robot.set_lights_on_rgb(' + light_rgb + ')')
    L.append('                    print("  Done wrapping pole " + str(pt["pole"]))')
    L.append('                continue')
    L.append('')
    L.append('            # NAVIGATE MODE')
    L.append('            elif WRAP_MODE == "navigate":')
    L.append('                if is_first_wrap:')
    L.append('                    await robot.set_lights_spin_rgb(255, 115, 0)')
    L.append('                    print("  Wrapping pole " + str(pt["pole"]) + " (" + DIR + ") [navigate mode]")')
    L.append('')
    L.append('                await robot.navigate_to(pt["x"], pt["y"])')
    L.append('')
    L.append('                nxt = PATH[i + 1] if i + 1 < len(PATH) else None')
    L.append('                if nxt is None or nxt["type"] != "wrap" or nxt["pole"] != pt["pole"]:')
    L.append('                    await robot.set_lights_on_rgb(' + light_rgb + ')')
    L.append('                    print("  Done wrapping pole " + str(pt["pole"]))')
    L.append('')
    L.append('    await robot.set_lights_blink_rgb(255, 255, 255)')
    L.append('    await robot.play_note(440, 0.5)')
    L.append('    await robot.play_note(880, 0.5)')
    L.append('    await robot.stop_sound()')
    L.append('    print("\\nRobot ' + rid + ' complete!")')
    L.append('')
    L.append('robot.play()')
    L.append('')

    return "\n".join(L)

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

        save_csv(pts,       "path_" + rid + ".csv")
        save_csv(robot_pts, "path_" + rid + "_robot.csv")

        drv_path = os.path.join(script_dir, "run_robot_" + rid + ".py")
        with open(drv_path, "w") as f:
            f.write(generate_driver_code(rid, robot_pts))

        cfg = ROBOTS[rid]
        print("Robot " + rid + " | start=" + str(cfg["start"]) +
              " seq=" + str(cfg["seq"]) +
              " dir=" + cfg["dir"] +
              " circles=" + str(cfg["circles"]))
        print("  full: " + str(len(pts)) + " wpts, " +
              str(round(total, 1)) + " cm | robot: " + str(len(robot_pts)) + " wpts")
        print("  Saved path_" + rid + ".csv, path_" + rid + "_robot.csv, run_robot_" + rid + ".py")

if __name__ == "__main__":
    main()