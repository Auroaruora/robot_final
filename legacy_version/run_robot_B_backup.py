"""
run_robot_B.py  —  Standalone runner for Robot B
=================================================
"""

import math
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# ==================================================================
# WORLD CONFIGURATION
# ==================================================================

POLES = {
    1: ( -50,    0),
    2: (-200, -100),
    3: (-120,  120),
    4: (  10,  150),
    5: ( 150, -150),
}

POLE_R  = 5
ROBOT_R = 17
SAFETY  = 12
ARC_R   = POLE_R + ROBOT_R + SAFETY

# --- Robot B config ---
RID = "B"
START  = (-120, 160)
SEQ    = [4, 5, 2, 1, 5]
DIR    = "cw"
CIRCLES = 1
SIGN    = -1         # cw = -1

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
# TANGENT POINTS
# ==================================================================

def entry_tangent_from_point(src_x, src_y, cx, cy, r, wrap_sign):
    d = dist(src_x, src_y, cx, cy)
    if d <= r:
        ang = math.atan2(src_y - cy, src_x - cx)
        return (cx + r*math.cos(ang), cy + r*math.sin(ang))
    ang_to_src = math.atan2(src_y - cy, src_x - cx)
    offset = math.acos(r / d)
    tang_ang = ang_to_src + wrap_sign * offset
    return (cx + r*math.cos(tang_ang), cy + r*math.sin(tang_ang))

def exit_tangent_to_point(cx, cy, r, dst_x, dst_y, wrap_sign):
    d = dist(dst_x, dst_y, cx, cy)
    if d <= r:
        ang = math.atan2(dst_y - cy, dst_x - cx)
        return (cx + r*math.cos(ang), cy + r*math.sin(ang))
    ang_to_dst = math.atan2(dst_y - cy, dst_x - cx)
    offset = math.acos(r / d)
    tang_ang = ang_to_dst - wrap_sign * offset
    return (cx + r*math.cos(tang_ang), cy + r*math.sin(tang_ang))

def transit_tangents(c1x, c1y, c2x, c2y, r, wrap_sign):
    d = dist(c1x, c1y, c2x, c2y)
    if d < 1e-6:
        return ((c1x + r, c1y), (c2x + r, c2y))
    base_ang = math.atan2(c2y - c1y, c2x - c1x)
    exit_ang  = base_ang - wrap_sign * math.pi / 2
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

def build_path():
    sx, sy = START
    pts = [{"x": sx, "y": sy, "type": "start", "pole": 0}]
    n = len(SEQ)

    entries = {}
    exits   = {}

    for idx in range(n):
        pid = SEQ[idx]
        px, py = POLES[pid]

        if idx == 0:
            entries[idx] = entry_tangent_from_point(sx, sy, px, py, ARC_R, SIGN)
        else:
            prev_px, prev_py = POLES[SEQ[idx-1]]
            _, entry = transit_tangents(prev_px, prev_py, px, py, ARC_R, SIGN)
            entries[idx] = entry

        if idx == n - 1:
            exits[idx] = entries[idx]
        else:
            next_px, next_py = POLES[SEQ[idx+1]]
            ex_pt, _ = transit_tangents(px, py, next_px, next_py, ARC_R, SIGN)
            exits[idx] = ex_pt

    cx, cy = sx, sy
    for idx in range(n):
        pid = SEQ[idx]
        px, py = POLES[pid]
        entry = entries[idx]
        exit_pt = exits[idx]

        skip = {pid}
        if idx > 0: skip.add(SEQ[idx-1])
        route = safe_transit(cx, cy, entry[0], entry[1], skip)
        for (wx, wy) in route:
            pts.append({"x": wx, "y": wy, "type": "move", "pole": pid})
            cx, cy = wx, wy

        entry_ang = math.atan2(entry[1] - py, entry[0] - px)
        exit_ang  = math.atan2(exit_pt[1] - py, exit_pt[0] - px)
        arc = arc_points(px, py, ARC_R, entry_ang, exit_ang, SIGN,
                         n_full_circles=CIRCLES)
        for (ax, ay) in arc:
            pts.append({"x": ax, "y": ay, "type": "wrap", "pole": pid})
        cx, cy = exit_pt

    return pts

def path_in_local(pts):
    sx, sy = START
    return [{**p, "x": p["x"] - sx, "y": p["y"] - sy} for p in pts]

# ==================================================================
# ROBOT RUNNER
# ==================================================================

robot = Create3(Bluetooth())

@event(robot.when_play)
async def play(robot):
    await robot.reset_navigation()
    await robot.set_lights_on_rgb(0, 0, 255)   # Blue for B

    world_pts = build_path()
    local_pts = path_in_local(world_pts)

    print(f"Robot B | start={START} seq={SEQ} dir={DIR} | {len(local_pts)} waypoints")

    current_pole = None

    for i, pt in enumerate(local_pts):
        if pt["type"] == "start":
            continue

        if pt["type"] == "move":
            if pt["pole"] != current_pole:
                current_pole = pt["pole"]
                print(f"\n--- Heading to Pole {current_pole} ---")
                await robot.wait(TURN_WAIT // 2)

            await robot.navigate_to(pt["x"], pt["y"])

        elif pt["type"] == "wrap":
            prev = local_pts[i - 1] if i > 0 else None
            if prev is None or prev["type"] != "wrap" or prev["pole"] != pt["pole"]:
                await robot.set_lights_spin_rgb(255, 115, 0)
                print(f"  Wrapping pole {pt['pole']} ({DIR})")

                for _ in range(CIRCLES):
                    await robot.arc_right(360, ARC_R)   # CW

                await robot.set_lights_on_rgb(0, 0, 255)
                print(f"  Done wrapping pole {pt['pole']}")
                await robot.wait(2.0)
            continue

    await robot.set_lights_blink_rgb(255, 255, 255)
    await robot.play_note(440, 0.5)
    await robot.play_note(880, 0.5)
    await robot.stop_sound()
    print(f"\nRobot B complete!")

robot.play()