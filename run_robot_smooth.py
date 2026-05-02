"""
run_robot_smooth.py  —  single-robot driver, smoothed transits
===================================================================
Same loader/event flow as run_robot.py. The difference: consecutive
"move" waypoints (the slice_transit chunks) are driven with a
constant-speed follower instead of one navigate_to per slice.

Per-transit behavior:
  - Re-aim with turn_left/turn_right when heading error exceeds
    HEADING_REAIM_DEG (preserves the slicing's accuracy benefit).
  - Cruise between slices at SMOOTH_V_CMS via set_wheel_speeds —
    no per-slice deceleration.
  - Final slice of each transit run uses move() so the robot decels
    cleanly before the next wrap.
Wraps are still followed with navigate_to per waypoint, same as the
non-smooth driver.

No external sensing required — get_position() reads the robot's
internal odometry, the same source navigate_to consults.
"""

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3
import asyncio, csv, json, math, os

# ==================================================================
# ROBOT SELECTION
# ==================================================================
ROBOT_ID = os.environ.get("ROBOT_ID", "A")

# ==================================================================
# SMOOTH-FOLLOWER TUNING
# ==================================================================
SMOOTH_V_CMS      = 15      # cruise speed between transit slices (cm/s)
HEADING_REAIM_DEG = 5       # turn-in-place if heading off by more than this
POLL_INTERVAL_S   = 0.05    # 20 Hz position polling while cruising
ARRIVE_TOL_CM     = 2.5     # consider a slice reached within this radius

# ==================================================================
# LOADERS
# ==================================================================

_HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_INFO_DIR = os.path.join(_HERE, "robot_info")

def load_run_config(rid):
    with open(os.path.join(ROBOT_INFO_DIR, "run_config_" + rid + ".json")) as f:
        cfg = json.load(f)
    cfg["light_rgb"] = tuple(cfg["light_rgb"])
    if not cfg.get("bluetooth_address"):
        raise SystemExit(
            "Robot " + rid + " has no bluetooth_address in run_config_" + rid +
            ".json. Run `python setup_robots.py` to pair the robots first."
        )
    return cfg

def load_path(csv_name):
    pts = []
    with open(os.path.join(ROBOT_INFO_DIR, csv_name), newline="") as f:
        for row in csv.DictReader(f):
            pts.append({
                "x":       float(row["x"]),
                "y":       float(row["y"]),
                "type":    row["type"],
                "pole":    int(row["pole"]),
                "arc_deg": float(row["arc_deg"]),
            })
    return pts

CFG  = load_run_config(ROBOT_ID)
PATH = load_path(CFG["path_csv"])

# ==================================================================
# SMOOTH FOLLOWER
# ==================================================================

def _wrap180(d):
    while d > 180:    d -= 360
    while d <= -180:  d += 360
    return d

async def drive_transit_run(robot, slices):
    """Drive a chain of (x, y) waypoints as one continuous motion.

    Wheels stay rolling between slices; only the final slice decels.
    Heading is re-aimed by stopping briefly and turning in place when
    the error exceeds HEADING_REAIM_DEG, which keeps the slicing's
    drift-correction property without the per-slice decel cycle.
    """
    if not slices:
        return
    last_idx = len(slices) - 1
    for i, (gx, gy) in enumerate(slices):
        pos = await robot.get_position()
        dx, dy = gx - pos.x, gy - pos.y
        d = math.hypot(dx, dy)
        if d < 0.5:
            continue

        target_h = math.degrees(math.atan2(dy, dx))
        err = _wrap180(target_h - pos.heading)
        if abs(err) > HEADING_REAIM_DEG:
            await robot.set_wheel_speeds(0, 0)
            if err > 0:
                await robot.turn_left(err)
            else:
                await robot.turn_right(-err)
            pos = await robot.get_position()
            d = math.hypot(gx - pos.x, gy - pos.y)

        if i == last_idx:
            # Final slice of this transit run: hand off to move() so the
            # SDK's trapezoidal profile decels cleanly to zero before the
            # next wrap's navigate_to picks up.
            await robot.set_wheel_speeds(0, 0)
            await robot.move(d)
        else:
            await robot.set_wheel_speeds(SMOOTH_V_CMS, SMOOTH_V_CMS)
            while True:
                await robot.wait(POLL_INTERVAL_S)
                pos = await robot.get_position()
                dgx, dgy = gx - pos.x, gy - pos.y
                if math.hypot(dgx, dgy) < ARRIVE_TOL_CM:
                    break
                # Forward-projected distance: if we've passed the goal
                # along the heading direction, advance even if Euclidean
                # distance is still > tolerance (handles overshoot).
                rh = math.radians(pos.heading)
                if dgx * math.cos(rh) + dgy * math.sin(rh) < 0:
                    break

# ==================================================================
# ROBOT DRIVER
# ==================================================================

robot = Create3(Bluetooth(address=CFG["bluetooth_address"]))


@event(robot.when_play)
async def play(robot):
    await robot.reset_navigation()
    await robot.set_lights_on_rgb(*CFG["light_rgb"])

    print("Robot " + CFG["rid"] +
          " | dir=" + CFG["dir"] +
          " | " + str(len(PATH)) + " waypoints (smooth)")

    on_color = CFG["light_rgb"]
    current_pole = None

    i = 0
    while i < len(PATH):
        pt = PATH[i]

        if pt["type"] == "start":
            i += 1
            continue

        if pt["type"] == "move":
            # Gather the consecutive run of move waypoints (one transit).
            run = []
            run_pole = pt["pole"]
            j = i
            while j < len(PATH) and PATH[j]["type"] == "move":
                run.append((PATH[j]["x"], PATH[j]["y"]))
                j += 1
            if run_pole != current_pole:
                current_pole = run_pole
                print("\n--- Heading to Pole " + str(current_pole) + " ---")
            await drive_transit_run(robot, run)
            i = j

        elif pt["type"] == "wrap":
            prev = PATH[i - 1] if i > 0 else None
            is_first_wrap = (prev is None or prev["type"] != "wrap"
                             or prev["pole"] != pt["pole"])
            if is_first_wrap:
                await robot.set_lights_spin_rgb(255, 115, 0)
                print("  Wrapping pole " + str(pt["pole"]) +
                      " (" + CFG["dir"] + ")")

            await robot.navigate_to(pt["x"], pt["y"])

            nxt = PATH[i + 1] if i + 1 < len(PATH) else None
            is_last_wrap = (nxt is None or nxt["type"] != "wrap"
                            or nxt["pole"] != pt["pole"])
            if is_last_wrap:
                await robot.set_lights_on_rgb(*on_color)
                print("  Done wrapping pole " + str(pt["pole"]))
            i += 1

        else:
            i += 1

    await robot.set_lights_blink_rgb(255, 255, 255)
    await robot.play_note(440, 0.5)
    await robot.play_note(880, 0.5)
    await robot.stop_sound()
    print("\nRobot " + CFG["rid"] + " complete!")

    loop = asyncio.get_event_loop()
    loop.call_later(0.5, loop.stop)

robot.play()
