"""
test_2_connection.py  —  verify BOTH Create3 robots are reachable at the
same time, using the multi-robot pattern from the SDK's multidrive.py
(one Bluetooth backend per robot, one .play() call drives them all).

Usage:
    python test_2_connection.py

Each robot lights up in its configured color (from run_config_<RID>.json),
plays a short tune, then blinks. If only one lights up, that slot's
Bluetooth address is wrong or the robot is off/out of range.
"""

import os, json
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# ---------- fixed addresses ----------
BLUETOOTH_ADDRESSES = {
    "A": "38E42E1C-5EA3-F705-C05D-43EB8600C88B",
    "B": "BDAFD1BE-45FC-A601-E2B1-A0AD8A97AFB6",
}

# ---------- load per-robot color from run_config_<RID>.json ----------
HERE = os.path.dirname(os.path.abspath(__file__))

def load_color(rid):
    cfg_path = os.path.join(HERE, "run_config_" + rid + ".json")
    if not os.path.exists(cfg_path):
        raise SystemExit("Missing " + cfg_path + ". Run `python compute_path.py` first.")
    with open(cfg_path) as f:
        cfg = json.load(f)
    return tuple(cfg["light_rgb"]), cfg["color_name"]

COLOR_A, NAME_A = load_color("A")
COLOR_B, NAME_B = load_color("B")

print("Connecting to Robot A (" + NAME_A + ") at " + BLUETOOTH_ADDRESSES["A"])
print("Connecting to Robot B (" + NAME_B + ") at " + BLUETOOTH_ADDRESSES["B"])

# ---------- two backends, two robots (mirrors multidrive.py) ----------
backend_a = Bluetooth(address=BLUETOOTH_ADDRESSES["A"])
backend_b = Bluetooth(address=BLUETOOTH_ADDRESSES["B"])
robot_a = Create3(backend_a)
robot_b = Create3(backend_b)


@event(robot_a.when_play)
async def play_a(robot):
    print("[A] connected. Lighting " + NAME_A + " and playing tune...")
    await robot.set_lights_on_rgb(*COLOR_A)
    await robot.play_note(440, 0.25)
    await robot.play_note(660, 0.25)
    await robot.play_note(880, 0.5)
    await robot.stop_sound()
    await robot.set_lights_blink_rgb(*COLOR_A)
    print("[A] done.")


@event(robot_b.when_play)
async def play_b(robot):
    print("[B] connected. Lighting " + NAME_B + " and playing tune...")
    await robot.set_lights_on_rgb(*COLOR_B)
    await robot.play_note(523, 0.25)   # slightly different tune to tell them apart
    await robot.play_note(784, 0.25)
    await robot.play_note(1047, 0.5)
    await robot.stop_sound()
    await robot.set_lights_blink_rgb(*COLOR_B)
    print("[B] done.")


# Only one .play() call — it's blocking and dispatches events to both robots.
robot_a.play()
