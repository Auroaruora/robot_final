"""
run_robot_B.py  (auto-generated)
=====================================================
Path is embedded below - no CSV file needed.
Re-run compute_path_B.py or use the website to regenerate.

TWO WRAPPING MODES - change WRAP_MODE to swap:
  "arc"      = single arc_right command (fast, less accurate)
  "navigate" = follow reduced waypoints via navigate_to (slow, more accurate)
"""

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

# ==================================================================
# CONFIG
# ==================================================================

DIR     = "cw"
CIRCLES = 1
ARC_R   = 50

# ==================================================================
# WRAPPING MODE - CHANGE THIS TO SWAP
# ==================================================================
WRAP_MODE = "arc"
# WRAP_MODE = "navigate"

# ==================================================================
# EMBEDDED PATH  (95 waypoints, reduced for robot)
# ==================================================================

PATH = [
    {"x": 0, "y": 0, "type": "start", "pole": 0},
    {"x": 39.0312, "y": -48.75, "type": "move", "pole": 3},
    {"x": 40.6182, "y": -50.8425, "type": "wrap", "pole": 3},
    {"x": 49.7635, "y": -75.1429, "type": "wrap", "pole": 3},
    {"x": 45.4897, "y": -100.753, "type": "wrap", "pole": 3},
    {"x": 28.9493, "y": -120.7669, "type": "wrap", "pole": 3},
    {"x": 4.6024, "y": -129.7877, "type": "wrap", "pole": 3},
    {"x": -20.9855, "y": -125.3829, "type": "wrap", "pole": 3},
    {"x": -40.9145, "y": -108.7402, "type": "wrap", "pole": 3},
    {"x": -49.8106, "y": -84.3476, "type": "wrap", "pole": 3},
    {"x": -45.2749, "y": -58.7825, "type": "wrap", "pole": 3},
    {"x": -28.5305, "y": -38.9389, "type": "wrap", "pole": 3},
    {"x": -4.0926, "y": -30.1678, "type": "wrap", "pole": 3},
    {"x": 21.4489, "y": -34.8343, "type": "wrap", "pole": 3},
    {"x": 41.2065, "y": -51.68, "type": "wrap", "pole": 3},
    {"x": 49.8525, "y": -76.1625, "type": "wrap", "pole": 3},
    {"x": 45.0554, "y": -101.6798, "type": "wrap", "pole": 3},
    {"x": 28.1087, "y": -121.3509, "type": "wrap", "pole": 3},
    {"x": 3.5823, "y": -129.8715, "type": "wrap", "pole": 3},
    {"x": -21.9101, "y": -124.9438, "type": "wrap", "pole": 3},
    {"x": -41.4942, "y": -107.8967, "type": "wrap", "pole": 3},
    {"x": -49.8892, "y": -83.3271, "type": "wrap", "pole": 3},
    {"x": -44.8311, "y": -57.8602, "type": "wrap", "pole": 3},
    {"x": -27.684, "y": -38.3635, "type": "wrap", "pole": 3},
    {"x": -3.0717, "y": -30.0944, "type": "wrap", "pole": 3},
    {"x": 17.5562, "y": -33.1835, "type": "wrap", "pole": 3},
    {"x": 137.5562, "y": -78.1835, "type": "move", "pole": 1},
    {"x": 139.9894, "y": -79.1696, "type": "wrap", "pole": 1},
    {"x": 160.2728, "y": -95.3672, "type": "wrap", "pole": 1},
    {"x": 169.7022, "y": -119.5511, "type": "wrap", "pole": 1},
    {"x": 165.7364, "y": -145.2035, "type": "wrap", "pole": 1},
    {"x": 149.4442, "y": -165.4109, "type": "wrap", "pole": 1},
    {"x": 125.2165, "y": -174.7271, "type": "wrap", "pole": 1},
    {"x": 99.5829, "y": -170.6414, "type": "wrap", "pole": 1},
    {"x": 79.4519, "y": -154.2549, "type": "wrap", "pole": 1},
    {"x": 70.249, "y": -129.9839, "type": "wrap", "pole": 1},
    {"x": 74.4545, "y": -104.3697, "type": "wrap", "pole": 1},
    {"x": 90.935, "y": -84.3156, "type": "wrap", "pole": 1},
    {"x": 115.2487, "y": -75.2263, "type": "wrap", "pole": 1},
    {"x": 140.843, "y": -79.5515, "type": "wrap", "pole": 1},
    {"x": 160.8199, "y": -96.1255, "type": "wrap", "pole": 1},
    {"x": 169.7954, "y": -120.4815, "type": "wrap", "pole": 1},
    {"x": 165.3506, "y": -146.0553, "type": "wrap", "pole": 1},
    {"x": 148.6833, "y": -165.9544, "type": "wrap", "pole": 1},
    {"x": 124.2857, "y": -174.816, "type": "wrap", "pole": 1},
    {"x": 98.7329, "y": -170.2517, "type": "wrap", "pole": 1},
    {"x": 78.9119, "y": -153.4916, "type": "wrap", "pole": 1},
    {"x": 70.1645, "y": -129.0527, "type": "wrap", "pole": 1},
    {"x": 74.8483, "y": -103.5216, "type": "wrap", "pole": 1},
    {"x": 91.7009, "y": -83.7792, "type": "wrap", "pole": 1},
    {"x": 116.1803, "y": -75.1461, "type": "wrap", "pole": 1},
    {"x": 121.428, "y": -75.0204, "type": "wrap", "pole": 1},
    {"x": 296.428, "y": -80.0204, "type": "move", "pole": 4},
    {"x": 299.0524, "y": -80.1645, "type": "wrap", "pole": 4},
    {"x": 323.515, "y": -88.9281, "type": "wrap", "pole": 4},
    {"x": 340.276, "y": -108.7848, "type": "wrap", "pole": 4},
    {"x": 344.8085, "y": -134.3714, "type": "wrap", "pole": 4},
    {"x": 335.8884, "y": -158.7773, "type": "wrap", "pole": 4},
    {"x": 315.9249, "y": -175.4109, "type": "wrap", "pole": 4},
    {"x": 290.3098, "y": -179.7795, "type": "wrap", "pole": 4},
    {"x": 265.9614, "y": -170.7033, "type": "wrap", "pole": 4},
    {"x": 249.4561, "y": -150.6337, "type": "wrap", "pole": 4},
    {"x": 245.2515, "y": -124.9911, "type": "wrap", "pole": 4},
    {"x": 254.4834, "y": -100.7014, "type": "wrap", "pole": 4},
    {"x": 274.6583, "y": -84.3249, "type": "wrap", "pole": 4},
    {"x": 300.3273, "y": -80.2846, "type": "wrap", "pole": 4},
    {"x": 324.5574, "y": -89.6718, "type": "wrap", "pole": 4},
    {"x": 340.8044, "y": -109.9512, "type": "wrap", "pole": 4},
    {"x": 344.6803, "y": -135.6455, "type": "wrap", "pole": 4},
    {"x": 319.6803, "y": -355.6455, "type": "move", "pole": 5},
    {"x": 319.3167, "y": -358.2378, "type": "wrap", "pole": 5},
    {"x": 308.5906, "y": -381.7925, "type": "wrap", "pole": 5},
    {"x": 287.5242, "y": -396.8284, "type": "wrap", "pole": 5},
    {"x": 261.7622, "y": -399.3167, "type": "wrap", "pole": 5},
    {"x": 238.2075, "y": -388.5906, "type": "wrap", "pole": 5},
    {"x": 223.1716, "y": -367.5242, "type": "wrap", "pole": 5},
    {"x": 220.6833, "y": -341.7622, "type": "wrap", "pole": 5},
    {"x": 231.4094, "y": -318.2075, "type": "wrap", "pole": 5},
    {"x": 252.4758, "y": -303.1716, "type": "wrap", "pole": 5},
    {"x": 278.2378, "y": -300.6833, "type": "wrap", "pole": 5},
    {"x": 301.7925, "y": -311.4094, "type": "wrap", "pole": 5},
    {"x": 316.8284, "y": -332.4758, "type": "wrap", "pole": 5},
    {"x": 319.3167, "y": -358.2378, "type": "wrap", "pole": 5},
    {"x": 308.5906, "y": -381.7925, "type": "wrap", "pole": 5},
    {"x": 287.5242, "y": -396.8284, "type": "wrap", "pole": 5},
    {"x": 261.7622, "y": -399.3167, "type": "wrap", "pole": 5},
    {"x": 238.2075, "y": -388.5906, "type": "wrap", "pole": 5},
    {"x": 223.1716, "y": -367.5242, "type": "wrap", "pole": 5},
    {"x": 220.6833, "y": -341.7622, "type": "wrap", "pole": 5},
    {"x": 231.4094, "y": -318.2075, "type": "wrap", "pole": 5},
    {"x": 252.4758, "y": -303.1716, "type": "wrap", "pole": 5},
    {"x": 278.2378, "y": -300.6833, "type": "wrap", "pole": 5},
    {"x": 301.7925, "y": -311.4094, "type": "wrap", "pole": 5},
    {"x": 316.8284, "y": -332.4758, "type": "wrap", "pole": 5},
    {"x": 319.6803, "y": -355.6455, "type": "wrap", "pole": 5},
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
                    print("  Wrapping pole " + str(pt["pole"]) + " (" + DIR + ") [arc mode]")
                    for _ in range(CIRCLES):
                        await robot.arc_right(360, ARC_R)
                    await robot.set_lights_on_rgb(0, 0, 255)
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
                    await robot.set_lights_on_rgb(0, 0, 255)
                    print("  Done wrapping pole " + str(pt["pole"]))
                    await robot.wait(15)
                    await robot.wait(2.0)

    await robot.set_lights_blink_rgb(255, 255, 255)
    await robot.play_note(440, 0.5)
    await robot.play_note(880, 0.5)
    await robot.stop_sound()
    print("\nRobot B complete!")

robot.play()
