"""
run_robot_A.py  (auto-generated)
=====================================================
Path is embedded below - no CSV file needed.
Re-run compute_path_A.py or use the website to regenerate.

TWO WRAPPING MODES - change WRAP_MODE to swap:
  "arc"      = single arc_left command (fast, less accurate)
  "navigate" = follow reduced waypoints via navigate_to (slow, more accurate)
"""

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# ==================================================================
# CONFIG
# ==================================================================

DIR     = "ccw"
CIRCLES = 1
ARC_R   = 50

# ==================================================================
# WRAPPING MODE - CHANGE THIS TO SWAP
# ==================================================================
WRAP_MODE = "arc"
# WRAP_MODE = "navigate"

# ==================================================================
# EMBEDDED PATH  (92 waypoints, reduced for robot)
# ==================================================================

PATH = [
    {"x": 0, "y": 0, "type": "start", "pole": 0},
    {"x": 37.2678, "y": 41.6667, "type": "move", "pole": 1},
    {"x": 38.9637, "y": 43.6658, "type": "wrap", "pole": 1},
    {"x": 49.4165, "y": 67.3838, "type": "wrap", "pole": 1},
    {"x": 46.5901, "y": 93.1484, "type": "wrap", "pole": 1},
    {"x": 31.2439, "y": 114.0361, "type": "wrap", "pole": 1},
    {"x": 7.5017, "y": 124.434, "type": "wrap", "pole": 1},
    {"x": -18.2563, "y": 121.5479, "type": "wrap", "pole": 1},
    {"x": -39.1084, "y": 106.1534, "type": "wrap", "pole": 1},
    {"x": -49.4513, "y": 82.3872, "type": "wrap", "pole": 1},
    {"x": -46.5055, "y": 56.636, "type": "wrap", "pole": 1},
    {"x": -31.0627, "y": 35.8195, "type": "wrap", "pole": 1},
    {"x": -7.2727, "y": 25.5317, "type": "wrap", "pole": 1},
    {"x": 18.4717, "y": 28.5371, "type": "wrap", "pole": 1},
    {"x": 39.2523, "y": 44.0281, "type": "wrap", "pole": 1},
    {"x": 49.485, "y": 67.8419, "type": "wrap", "pole": 1},
    {"x": 46.4199, "y": 93.5793, "type": "wrap", "pole": 1},
    {"x": 30.8809, "y": 114.3239, "type": "wrap", "pole": 1},
    {"x": 7.0434, "y": 124.5014, "type": "wrap", "pole": 1},
    {"x": -18.6867, "y": 121.3768, "type": "wrap", "pole": 1},
    {"x": -39.3954, "y": 105.7897, "type": "wrap", "pole": 1},
    {"x": -46.9897, "y": 92.0872, "type": "wrap", "pole": 1},
    {"x": -146.9897, "y": -182.9128, "type": "move", "pole": 2},
    {"x": -147.8195, "y": -185.3955, "type": "wrap", "pole": 2},
    {"x": -148.7152, "y": -211.2619, "type": "wrap", "pole": 2},
    {"x": -136.5576, "y": -234.1107, "type": "wrap", "pole": 2},
    {"x": -114.6045, "y": -247.8195, "type": "wrap", "pole": 2},
    {"x": -88.7381, "y": -248.7152, "type": "wrap", "pole": 2},
    {"x": -65.8893, "y": -236.5576, "type": "wrap", "pole": 2},
    {"x": -52.1805, "y": -214.6045, "type": "wrap", "pole": 2},
    {"x": -51.2848, "y": -188.7381, "type": "wrap", "pole": 2},
    {"x": -63.4424, "y": -165.8893, "type": "wrap", "pole": 2},
    {"x": -85.3955, "y": -152.1805, "type": "wrap", "pole": 2},
    {"x": -111.2619, "y": -151.2848, "type": "wrap", "pole": 2},
    {"x": -134.1107, "y": -163.4424, "type": "wrap", "pole": 2},
    {"x": -147.8195, "y": -185.3955, "type": "wrap", "pole": 2},
    {"x": -148.7152, "y": -211.2619, "type": "wrap", "pole": 2},
    {"x": -136.5576, "y": -234.1107, "type": "wrap", "pole": 2},
    {"x": -114.6045, "y": -247.8195, "type": "wrap", "pole": 2},
    {"x": -88.7381, "y": -248.7152, "type": "wrap", "pole": 2},
    {"x": -65.8893, "y": -236.5576, "type": "wrap", "pole": 2},
    {"x": -53.0103, "y": -217.0872, "type": "wrap", "pole": 2},
    {"x": 46.9897, "y": 57.9128, "type": "move", "pole": 1},
    {"x": 47.82, "y": 60.3969, "type": "wrap", "pole": 1},
    {"x": 48.7117, "y": 86.2769, "type": "wrap", "pole": 1},
    {"x": 36.5376, "y": 109.1322, "type": "wrap", "pole": 1},
    {"x": 14.563, "y": 122.8322, "type": "wrap", "pole": 1},
    {"x": -11.3178, "y": 123.7022, "type": "wrap", "pole": 1},
    {"x": -34.1629, "y": 111.5089, "type": "wrap", "pole": 1},
    {"x": -47.8444, "y": 89.5228, "type": "wrap", "pole": 1},
    {"x": -48.6927, "y": 63.6413, "type": "wrap", "pole": 1},
    {"x": -36.4802, "y": 40.8065, "type": "wrap", "pole": 1},
    {"x": -14.4826, "y": 27.1434, "type": "wrap", "pole": 1},
    {"x": 11.3996, "y": 26.3169, "type": "wrap", "pole": 1},
    {"x": 34.2242, "y": 38.5486, "type": "wrap", "pole": 1},
    {"x": 47.8688, "y": 60.5576, "type": "wrap", "pole": 1},
    {"x": 48.6735, "y": 86.4405, "type": "wrap", "pole": 1},
    {"x": 36.4227, "y": 109.2548, "type": "wrap", "pole": 1},
    {"x": 14.4022, "y": 122.8809, "type": "wrap", "pole": 1},
    {"x": -11.4814, "y": 123.6639, "type": "wrap", "pole": 1},
    {"x": -34.2853, "y": 111.3939, "type": "wrap", "pole": 1},
    {"x": -47.8929, "y": 89.3619, "type": "wrap", "pole": 1},
    {"x": -48.6543, "y": 63.4777, "type": "wrap", "pole": 1},
    {"x": -36.3651, "y": 40.6841, "type": "wrap", "pole": 1},
    {"x": -14.3217, "y": 27.095, "type": "wrap", "pole": 1},
    {"x": -1.428, "y": 25.0204, "type": "wrap", "pole": 1},
    {"x": 173.572, "y": 20.0204, "type": "move", "pole": 4},
    {"x": 176.1897, "y": 20.0142, "type": "wrap", "pole": 4},
    {"x": 201.0232, "y": 27.3058, "type": "wrap", "pole": 4},
    {"x": 218.8839, "y": 46.0374, "type": "wrap", "pole": 4},
    {"x": 224.9858, "y": 71.1897, "type": "wrap", "pole": 4},
    {"x": 217.6942, "y": 96.0232, "type": "wrap", "pole": 4},
    {"x": 198.9626, "y": 113.8839, "type": "wrap", "pole": 4},
    {"x": 173.8103, "y": 119.9858, "type": "wrap", "pole": 4},
    {"x": 148.9768, "y": 112.6942, "type": "wrap", "pole": 4},
    {"x": 131.1161, "y": 93.9626, "type": "wrap", "pole": 4},
    {"x": 125.0142, "y": 68.8103, "type": "wrap", "pole": 4},
    {"x": 132.3058, "y": 43.9768, "type": "wrap", "pole": 4},
    {"x": 151.0374, "y": 26.1161, "type": "wrap", "pole": 4},
    {"x": 176.1897, "y": 20.0142, "type": "wrap", "pole": 4},
    {"x": 201.0232, "y": 27.3058, "type": "wrap", "pole": 4},
    {"x": 218.8839, "y": 46.0374, "type": "wrap", "pole": 4},
    {"x": 224.9858, "y": 71.1897, "type": "wrap", "pole": 4},
    {"x": 217.6942, "y": 96.0232, "type": "wrap", "pole": 4},
    {"x": 198.9626, "y": 113.8839, "type": "wrap", "pole": 4},
    {"x": 173.8103, "y": 119.9858, "type": "wrap", "pole": 4},
    {"x": 148.9768, "y": 112.6942, "type": "wrap", "pole": 4},
    {"x": 131.1161, "y": 93.9626, "type": "wrap", "pole": 4},
    {"x": 125.0142, "y": 68.8103, "type": "wrap", "pole": 4},
    {"x": 132.3058, "y": 43.9768, "type": "wrap", "pole": 4},
    {"x": 151.0374, "y": 26.1161, "type": "wrap", "pole": 4},
    {"x": 173.572, "y": 20.0204, "type": "wrap", "pole": 4},
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
                    print("  Wrapping pole " + str(pt["pole"]) + " (" + DIR + ") [arc mode]")
                    for _ in range(CIRCLES):
                        await robot.arc_left(360, ARC_R)
                    await robot.set_lights_on_rgb(0, 255, 0)
                    print("  Done wrapping pole " + str(pt["pole"]))
                    await robot.wait(15)
                    await robot.wait(2.0)
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
                    await robot.wait(15)
                    await robot.wait(2.0)

    await robot.set_lights_blink_rgb(255, 255, 255)
    await robot.play_note(440, 0.5)
    await robot.play_note(880, 0.5)
    await robot.stop_sound()
    print("\nRobot A complete!")

robot.play()
