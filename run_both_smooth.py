"""
run_both_smooth.py  —  drive Robot A and Robot B simultaneously, smoothed transits

Mirrors run_both.py (one Bluetooth backend per robot, single .play() call
drives both, startup barrier + finish counter for clean exit). The
difference: consecutive "move" waypoints in each robot's path are driven
with a constant-speed follower instead of one navigate_to per slice.

Per-transit behavior:
  - Re-aim with turn_left/turn_right when heading error exceeds
    HEADING_REAIM_DEG (preserves slicing's drift-correction).
  - Cruise between slices at SMOOTH_V_CMS via set_wheel_speeds —
    no per-slice deceleration.
  - Final slice of each transit run uses move() so the robot decels
    cleanly before the next wrap.
Wraps are still followed with navigate_to per waypoint, same as run_both.

Pure odometry — get_position() reads the robot's internal pose, no
external tracking required.
"""

import asyncio, csv, json, math, os

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# ==================================================================
# SMOOTH-FOLLOWER TUNING
# ==================================================================
SMOOTH_V_CMS      = 15      # cruise speed between transit slices (cm/s)
HEADING_REAIM_DEG = 5       # turn-in-place if heading off by more than this
POLL_INTERVAL_S   = 0.05    # 20 Hz position polling while cruising
ARRIVE_TOL_CM     = 2.5     # consider a slice reached within this radius

# Startup barrier — both robots reset_navigation, then wait here. Once
# both have arrived, they cross together and start moving at the same
# moment. Without this, whichever robot connects first starts driving
# before the other is even on Bluetooth.
_NUM_ROBOTS = 2
_start_barrier = asyncio.Barrier(_NUM_ROBOTS)

# Finish counter — when every robot's play handler reaches the end, the
# last one schedules loop.stop() so the SDK's run_forever() returns and
# the process exits cleanly (exit code 0).
_done_count = 0

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
    the error exceeds HEADING_REAIM_DEG.
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
                rh = math.radians(pos.heading)
                if dgx * math.cos(rh) + dgy * math.sin(rh) < 0:
                    break


def register_play(robot, cfg, path):
    """Attach a when_play handler that walks `path` on `robot` smoothly."""

    on_color  = cfg["light_rgb"]
    rid       = cfg["rid"]
    direction = cfg["dir"]

    @event(robot.when_play)
    async def play(robot):
        await robot.reset_navigation()
        await robot.set_lights_on_rgb(*on_color)
        print("[" + rid + "] ready — waiting for the other robot...")

        await _start_barrier.wait()

        print("Robot " + rid +
              " | dir=" + direction +
              " | " + str(len(path)) + " waypoints (smooth)")

        current_pole = None

        i = 0
        while i < len(path):
            pt = path[i]

            if pt["type"] == "start":
                i += 1
                continue

            if pt["type"] == "move":
                run = []
                run_pole = pt["pole"]
                j = i
                while j < len(path) and path[j]["type"] == "move":
                    run.append((path[j]["x"], path[j]["y"]))
                    j += 1
                if run_pole != current_pole:
                    current_pole = run_pole
                    print("[" + rid + "] --- Heading to Pole " + str(current_pole) + " ---")
                await drive_transit_run(robot, run)
                i = j

            elif pt["type"] == "wrap":
                prev = path[i - 1] if i > 0 else None
                is_first_wrap = (prev is None or prev["type"] != "wrap"
                                 or prev["pole"] != pt["pole"])
                if is_first_wrap:
                    await robot.set_lights_spin_rgb(255, 115, 0)
                    print("[" + rid + "] Wrapping pole " + str(pt["pole"]) +
                          " (" + direction + ")")

                await robot.navigate_to(pt["x"], pt["y"])

                nxt = path[i + 1] if i + 1 < len(path) else None
                is_last_wrap = (nxt is None or nxt["type"] != "wrap"
                                or nxt["pole"] != pt["pole"])
                if is_last_wrap:
                    await robot.set_lights_on_rgb(*on_color)
                    print("[" + rid + "] Done wrapping pole " + str(pt["pole"]))
                i += 1

            else:
                i += 1

        await robot.set_lights_blink_rgb(255, 255, 255)
        await robot.play_note(440, 0.5)
        await robot.play_note(880, 0.5)
        await robot.stop_sound()
        print("[" + rid + "] complete!")

        global _done_count
        _done_count += 1
        if _done_count >= _NUM_ROBOTS:
            loop = asyncio.get_event_loop()
            loop.call_later(0.5, loop.stop)


# ---------- build both robots ----------
CFG_A,  CFG_B  = load_run_config("A"), load_run_config("B")
PATH_A, PATH_B = load_path(CFG_A["path_csv"]), load_path(CFG_B["path_csv"])

print("Connecting Robot A (" + CFG_A["color_name"] + ") at " + CFG_A["bluetooth_address"])
print("Connecting Robot B (" + CFG_B["color_name"] + ") at " + CFG_B["bluetooth_address"])

robot_a = Create3(Bluetooth(address=CFG_A["bluetooth_address"]))
robot_b = Create3(Bluetooth(address=CFG_B["bluetooth_address"]))

register_play(robot_a, CFG_A, PATH_A)
register_play(robot_b, CFG_B, PATH_B)

# Single blocking call drives both robots.
robot_a.play()
