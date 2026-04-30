"""
run_both.py  —  drive Robot A and Robot B at the same time

Uses the multi-robot pattern from the SDK's multidrive.py (one Bluetooth
backend per robot, one .play() call drives both). The per-robot path-
following logic mirrors run_robot.py: pure odometry, no Vicon, navigate
mode only.
"""

import asyncio, csv, json, os

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# Startup barrier — both robots reset_navigation, then wait here. Once
# both have arrived, they cross together and start moving at the same
# moment. Without this, whichever robot connects first starts driving
# before the other is even on Bluetooth.
_NUM_ROBOTS = 2
_start_barrier = asyncio.Barrier(_NUM_ROBOTS)

_HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_INFO_DIR = os.path.join(_HERE, "robot_info")


def load_run_config(rid):
    with open(os.path.join(ROBOT_INFO_DIR, "run_config_" + rid + ".json")) as f:
        cfg = json.load(f)
    cfg["light_rgb"] = tuple(cfg["light_rgb"])
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


def register_play(robot, cfg, path):
    """Attach a when_play handler that walks `path` on `robot`."""

    on_color  = cfg["light_rgb"]
    rid       = cfg["rid"]
    direction = cfg["dir"]

    @event(robot.when_play)
    async def play(robot):
        await robot.reset_navigation()
        await robot.set_lights_on_rgb(*on_color)
        print("[" + rid + "] ready — waiting for the other robot...")

        # Hold here until the other robot has also reset and is ready.
        # Both then release together, so motion starts at the same instant.
        await _start_barrier.wait()

        print("Robot " + rid +
              " | dir=" + direction +
              " | " + str(len(path)) + " waypoints")

        current_pole = None

        for i, pt in enumerate(path):
            if pt["type"] == "start":
                continue

            if pt["type"] == "move":
                if pt["pole"] != current_pole:
                    current_pole = pt["pole"]
                    print("[" + rid + "] --- Heading to Pole " + str(current_pole) + " ---")
                await robot.navigate_to(pt["x"], pt["y"])

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

        await robot.set_lights_blink_rgb(255, 255, 255)
        await robot.play_note(440, 0.5)
        await robot.play_note(880, 0.5)
        await robot.stop_sound()
        print("[" + rid + "] complete!")


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
