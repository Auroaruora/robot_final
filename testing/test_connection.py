"""
test_connection.py  —  verify the Create3 reachable at the address listed
in run_config_<RID>.json is the robot you expect.

Usage:
    python test_connection.py A
    python test_connection.py B

The robot will light up in its configured color (Green for A, Blue for B),
play a short tune, then blink. If the wrong robot lights up, the
Bluetooth address for that slot in compute_path.py is wrong.
"""

import sys, os, json
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# ---------- parse arg ----------
if len(sys.argv) != 2 or sys.argv[1].upper() not in ("A", "B"):
    print("Usage: python test_connection.py <A|B>")
    sys.exit(1)

ROBOT_ID = sys.argv[1].upper()

# ---------- load config ----------
HERE = os.path.dirname(os.path.abspath(__file__))
cfg_path = os.path.join(HERE, "run_config_" + ROBOT_ID + ".json")
if not os.path.exists(cfg_path):
    print("Missing " + cfg_path + ". Run `python compute_path.py` first.")
    sys.exit(1)

with open(cfg_path) as f:
    CFG = json.load(f)

ADDR  = CFG.get("bluetooth_address", "")
COLOR = tuple(CFG["light_rgb"])
NAME  = CFG["color_name"]

if not ADDR or "REPLACE" in ADDR:
    print("Robot " + ROBOT_ID + " has no valid bluetooth_address in " + cfg_path + ".")
    print("Fill BLUETOOTH_ADDRESSES[\"" + ROBOT_ID + "\"] in compute_path.py and re-run it.")
    sys.exit(1)

print("Connecting to Robot " + ROBOT_ID + " (" + NAME + ") at " + ADDR + " ...")

# ---------- connect + play ----------
robot = Create3(Bluetooth(address=ADDR))


@event(robot.when_play)
async def play(robot):
    print("Connected. Lighting up " + NAME + " and playing tune...")

    # Solid color — same as run_robot.py's "idle" color.
    await robot.set_lights_on_rgb(*COLOR)

    # Short rising tune. If you see Green + hear this on Robot A, address is good.
    await robot.play_note(440, 0.25)
    await robot.play_note(660, 0.25)
    await robot.play_note(880, 0.5)
    await robot.stop_sound()

    # Blink in the same color so the robot is easy to spot across a room.
    await robot.set_lights_blink_rgb(*COLOR)

    print("Done. If the robot that lit up " + NAME + " is the one you intended")
    print("as Robot " + ROBOT_ID + ", run_robot.py will connect to the same device.")


robot.play()
