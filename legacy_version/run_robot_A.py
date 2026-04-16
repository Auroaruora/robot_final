"""
run_robot_A.py  (auto-generated)
=====================================================
Path is embedded below - no CSV file needed.
Re-run compute_path_A.py or use the website to regenerate.

TWO WRAPPING MODES - change WRAP_MODE to swap:
  "arc"      = arc_left using computed sweep angle (fast, more accurate)
  "navigate" = follow reduced waypoints via navigate_to (slow, most accurate)
"""

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# ==================================================================
# CONFIG
# ==================================================================

DIR     = "ccw"
CIRCLES = 1
ARC_R   = 58

# ==================================================================
# WRAPPING MODE - CHANGE THIS TO SWAP
# ==================================================================
WRAP_MODE = "arc"
# WRAP_MODE = "navigate"

# ==================================================================
# EMBEDDED PATH  (92 waypoints, reduced for robot)
# arc_deg: total sweep angle for each pole wrap (on first wrap pt)
# ==================================================================

PATH = [
    {"x": 0, "y": 0, "type": "start", "pole": 0, "arc_deg": 0},
    {"x": 36.772, "y": 30.1467, "type": "move", "pole": 1, "arc_deg": 0},
    {"x": 39.0717, "y": 32.135, "type": "wrap", "pole": 1, "arc_deg": 570.67},
    {"x": 55.2804, "y": 57.4478, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 56.6426, "y": 87.4744, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 42.7926, "y": 114.1509, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 17.45, "y": 130.3127, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -12.5791, "y": 131.6195, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -39.2299, "y": 117.7202, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -55.3449, "y": 92.3477, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -56.5961, "y": 62.3162, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -42.6475, "y": 35.6911, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -17.2453, "y": 19.6231, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 12.7885, "y": 18.4274, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 39.3877, "y": 32.4252, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 55.4087, "y": 57.8572, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 56.5488, "y": 87.8931, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 42.5019, "y": 114.4663, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 17.0403, "y": 130.4403, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -12.9976, "y": 131.5249, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -39.5449, "y": 117.4288, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -54.508, "y": 94.8211, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -154.508, "y": -180.1789, "type": "move", "pole": 2, "arc_deg": 0},
    {"x": -155.4707, "y": -183.0588, "type": "wrap", "pole": 2, "arc_deg": 540.0},
    {"x": -156.5096, "y": -213.0638, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -142.4068, "y": -239.5684, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -116.9412, "y": -255.4707, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -86.9362, "y": -256.5096, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -60.4316, "y": -242.4068, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -44.5293, "y": -216.9412, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -43.4904, "y": -186.9362, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -57.5932, "y": -160.4316, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -83.0588, "y": -144.5293, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -113.0638, "y": -143.4904, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -139.5684, "y": -157.5932, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -155.4707, "y": -183.0588, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -156.5096, "y": -213.0638, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -142.4068, "y": -239.5684, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -116.9412, "y": -255.4707, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -86.9362, "y": -256.5096, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -60.4316, "y": -242.4068, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": -45.492, "y": -219.8211, "type": "wrap", "pole": 2, "arc_deg": 0},
    {"x": 54.508, "y": 55.1789, "type": "move", "pole": 1, "arc_deg": 0},
    {"x": 55.4711, "y": 58.0603, "type": "wrap", "pole": 1, "arc_deg": 648.35},
    {"x": 56.5056, "y": 88.0812, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 42.3836, "y": 114.5933, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 16.893, "y": 130.4854, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -13.1287, "y": 131.4946, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -39.6289, "y": 117.3503, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -55.4995, "y": 91.8464, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -56.4835, "y": 61.8239, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -42.317, "y": 35.3355, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -16.7998, "y": 19.4863, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 13.2236, "y": 18.5276, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 39.7, "y": 32.7163, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 55.5278, "y": 58.2468, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 56.4613, "y": 88.271, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 42.2503, "y": 114.7355, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 16.7065, "y": 130.5418, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -13.3184, "y": 131.4501, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -39.771, "y": 117.2169, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -55.5558, "y": 91.6599, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -56.4389, "y": 61.6341, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -42.1835, "y": 35.1935, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -16.6132, "y": 19.4302, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -1.6565, "y": 17.0237, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 173.3435, "y": 12.0237, "type": "move", "pole": 4, "arc_deg": 0},
    {"x": 176.3801, "y": 12.0164, "type": "wrap", "pole": 4, "arc_deg": 720.0},
    {"x": 205.1869, "y": 20.4748, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 225.9053, "y": 42.2034, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 232.9836, "y": 71.3801, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 224.5252, "y": 100.1869, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 202.7966, "y": 120.9053, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 173.6199, "y": 127.9836, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 144.8131, "y": 119.5252, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 124.0947, "y": 97.7966, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 117.0164, "y": 68.6199, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 125.4748, "y": 39.8131, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 147.2034, "y": 19.0947, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 176.3801, "y": 12.0164, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 205.1869, "y": 20.4748, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 225.9053, "y": 42.2034, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 232.9836, "y": 71.3801, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 224.5252, "y": 100.1869, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 202.7966, "y": 120.9053, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 173.6199, "y": 127.9836, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 144.8131, "y": 119.5252, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 124.0947, "y": 97.7966, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 117.0164, "y": 68.6199, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 125.4748, "y": 39.8131, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 147.2034, "y": 19.0947, "type": "wrap", "pole": 4, "arc_deg": 0},
    {"x": 173.3435, "y": 12.0237, "type": "wrap", "pole": 4, "arc_deg": 0},
]

# ==================================================================
# ROBOT DRIVER
# ==================================================================

robot = Create3(Bluetooth())

@event(robot.when_play)
async def play(robot):
    await robot.reset_navigation()
    await robot.set_lights_on_rgb(0, 255, 0)       # Green = Robot A

    print("Robot A | dir=" + DIR + " mode=" + WRAP_MODE + " | " + str(len(PATH)) + " waypoints")

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
                    await robot.arc_left(sweep, ARC_R)
                    await robot.set_lights_on_rgb(0, 255, 0)
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
                    await robot.set_lights_on_rgb(0, 255, 0)
                    print("  Done wrapping pole " + str(pt["pole"]))

    await robot.set_lights_blink_rgb(255, 255, 255)
    await robot.play_note(440, 0.5)
    await robot.play_note(880, 0.5)
    await robot.stop_sound()
    print("\nRobot A complete!")

robot.play()
