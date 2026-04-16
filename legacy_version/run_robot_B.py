"""
run_robot_B.py  (auto-generated)
=====================================================
Path is embedded below - no CSV file needed.
Re-run compute_path_B.py or use the website to regenerate.

TWO WRAPPING MODES - change WRAP_MODE to swap:
  "arc"      = arc_right using computed sweep angle (fast, more accurate)
  "navigate" = follow reduced waypoints via navigate_to (slow, most accurate)
"""

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# ==================================================================
# CONFIG
# ==================================================================

DIR     = "cw"
CIRCLES = 1
ARC_R   = 58

# ==================================================================
# WRAPPING MODE - CHANGE THIS TO SWAP
# ==================================================================
WRAP_MODE = "arc"
# WRAP_MODE = "navigate"

# ==================================================================
# EMBEDDED PATH  (96 waypoints, reduced for robot)
# arc_deg: total sweep angle for each pole wrap (on first wrap pt)
# ==================================================================

PATH = [
    {"x": 0, "y": 0, "type": "start", "pole": 0, "arc_deg": 0},
    {"x": 39.9474, "y": -37.95, "type": "move", "pole": 3, "arc_deg": 0},
    {"x": 42.0965, "y": -40.1016, "type": "wrap", "pole": 3, "arc_deg": 697.02},
    {"x": 56.4162, "y": -66.5387, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 55.5758, "y": -96.5931, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 39.801, "y": -122.1887, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 13.3308, "y": -136.4472, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -16.7216, "y": -135.5373, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -42.2806, "y": -119.7033, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -56.4779, "y": -93.2002, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -55.4984, "y": -63.1499, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -39.6054, "y": -37.6277, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -13.0695, "y": -23.4917, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 16.9784, "y": -24.5407, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 42.4638, "y": -40.4928, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 56.5384, "y": -67.0612, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 55.4199, "y": -97.1066, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 39.4089, "y": -122.5551, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 12.808, "y": -136.5682, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -17.2348, "y": -135.3802, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -42.6461, "y": -119.3104, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -56.5976, "y": -92.6771, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -55.3402, "y": -62.6372, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -39.2116, "y": -37.263, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": -12.5461, "y": -23.3732, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 17.4908, "y": -24.7002, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 20.3652, "y": -25.6929, "type": "wrap", "pole": 3, "arc_deg": 0},
    {"x": 140.3652, "y": -70.6929, "type": "move", "pole": 1, "arc_deg": 0},
    {"x": 143.1877, "y": -71.8368, "type": "wrap", "pole": 1, "arc_deg": 701.08},
    {"x": 166.7164, "y": -90.6259, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 177.6546, "y": -118.6792, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 173.0542, "y": -148.4361, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 154.1553, "y": -171.8766, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 126.0511, "y": -182.6835, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 96.3162, "y": -177.9441, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 72.9642, "y": -158.9357, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 62.2889, "y": -130.7814, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 67.1672, "y": -101.0689, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 86.2845, "y": -77.8061, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 114.4885, "y": -67.2625, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 144.1779, "y": -72.2797, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 167.3511, "y": -91.5056, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 177.7627, "y": -119.7585, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 172.6067, "y": -149.4241, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 153.2727, "y": -172.5071, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 124.9714, "y": -182.7866, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 95.3302, "y": -177.4919, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 72.3378, "y": -158.0502, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 62.1908, "y": -129.7011, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 67.624, "y": -100.0851, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 87.173, "y": -77.1838, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 115.5692, "y": -67.1695, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 121.6565, "y": -67.0237, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 296.6565, "y": -72.0237, "type": "move", "pole": 4, "arc_deg": 0},
    {"x": 299.7008, "y": -72.1908, "type": "wrap", "pole": 4, "arc_deg": 454.85},
    {"x": 328.0774, "y": -82.3566, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 347.5201, "y": -105.3904, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 352.7779, "y": -135.0708, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 342.4306, "y": -163.3817, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 319.2729, "y": -182.6766, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 289.5593, "y": -187.7443, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 261.3153, "y": -177.2159, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 242.169, "y": -153.9351, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 237.2918, "y": -124.1897, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 248.0008, "y": -96.0137, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 271.4037, "y": -77.0169, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 301.1796, "y": -72.3301, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 329.2866, "y": -83.2193, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 348.1331, "y": -106.7434, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 352.6291, "y": -136.5488, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 327.6291, "y": -356.5488, "type": "move", "pole": 5, "arc_deg": 0},
    {"x": 327.2074, "y": -359.5559, "type": "wrap", "pole": 5, "arc_deg": 720.0},
    {"x": 314.7651, "y": -386.8793, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 290.3281, "y": -404.321, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 260.4441, "y": -407.2074, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 233.1207, "y": -394.7651, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 215.679, "y": -370.3281, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 212.7926, "y": -340.4441, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 225.2349, "y": -313.1207, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 249.6719, "y": -295.679, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 279.5559, "y": -292.7926, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 306.8793, "y": -305.2349, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 324.321, "y": -329.6719, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 327.2074, "y": -359.5559, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 314.7651, "y": -386.8793, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 290.3281, "y": -404.321, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 260.4441, "y": -407.2074, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 233.1207, "y": -394.7651, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 215.679, "y": -370.3281, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 212.7926, "y": -340.4441, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 225.2349, "y": -313.1207, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 249.6719, "y": -295.679, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 279.5559, "y": -292.7926, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 306.8793, "y": -305.2349, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 324.321, "y": -329.6719, "type": "wrap", "pole": 5, "arc_deg": 0},
    {"x": 327.6291, "y": -356.5488, "type": "wrap", "pole": 5, "arc_deg": 0},
]

# ==================================================================
# ROBOT DRIVER
# ==================================================================

robot = Create3(Bluetooth())

@event(robot.when_play)
async def play(robot):
    await robot.reset_navigation()
    await robot.set_lights_on_rgb(0, 0, 255)       # Blue = Robot B

    print("Robot B | dir=" + DIR + " mode=" + WRAP_MODE + " | " + str(len(PATH)) + " waypoints")

    current_pole = None

    for i, pt in enumerate(PATH):
        if pt["type"] == "start":
            continue

        if pt["type"] == "move":
            if pt["pole"] != current_pole:
                current_pole = pt["pole"]
                print("\n--- Heading to Pole " + str(current_pole) + " ---")
            await robot.navigate_to(pt["x"], pt["y"])

        elif pt["type"] == "wrap":

            # ARC MODE
            if WRAP_MODE == "arc":
                prev = PATH[i - 1] if i > 0 else None
                if prev is None or prev["type"] != "wrap" or prev["pole"] != pt["pole"]:
                    await robot.set_lights_spin_rgb(255, 115, 0)
                    sweep = pt.get("arc_deg", 360)
                    print("  Wrapping pole " + str(pt["pole"]) + " (" + DIR + ") [arc mode, " + str(round(sweep, 1)) + " deg]")
                    await robot.arc_right(sweep, ARC_R)
                    await robot.set_lights_on_rgb(0, 0, 255)
                    print("  Done wrapping pole " + str(pt["pole"]))
                continue

            # NAVIGATE MODE
            elif WRAP_MODE == "navigate":
                prev = PATH[i - 1] if i > 0 else None
                if prev is None or prev["type"] != "wrap" or prev["pole"] != pt["pole"]:
                    await robot.set_lights_spin_rgb(255, 115, 0)
                    print("  Wrapping pole " + str(pt["pole"]) + " (" + DIR + ") [navigate mode]")

                await robot.navigate_to(pt["x"], pt["y"])

                nxt = PATH[i + 1] if i + 1 < len(PATH) else None
                if nxt is None or nxt["type"] != "wrap" or nxt["pole"] != pt["pole"]:
                    await robot.set_lights_on_rgb(0, 0, 255)
                    print("  Done wrapping pole " + str(pt["pole"]))

    await robot.set_lights_blink_rgb(255, 255, 255)
    await robot.play_note(440, 0.5)
    await robot.play_note(880, 0.5)
    await robot.stop_sound()
    print("\nRobot B complete!")

robot.play()
