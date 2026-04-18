"""
run_robot_A_ros2.py  (ROS 2 port)
=====================================================
Same pole-wrapping logic as run_robot_A.py, but driven over ROS 2
using the Create3Ros wrapper in create3_driver.py.

The PATH, POLE_POSITIONS, mode toggles, and ir_correct_pose logic
are IDENTICAL to the SDK version.

Launch this file directly for single-robot testing, or import
run_robot_A() from run_both.py to run A and B concurrently.
"""

import asyncio
import math
import rclpy

from create3_driver import Create3Ros

# ==================================================================
# CONFIG  (unchanged from SDK version)
# ==================================================================

DIR     = "ccw"
CIRCLES = 1
ARC_R   = 58

# ==================================================================
# WRAP MODE - how to draw the circle around the pole
# ==================================================================
WRAP_MODE = "arc"
# WRAP_MODE = "navigate"

# ==================================================================
# CORRECTION MODE - how to fix odometry drift before/after wraps
# ==================================================================
CORRECTION_MODE = "ir"
# CORRECTION_MODE = "none"

# ==================================================================
# IR CORRECTION PARAMETERS
# ==================================================================
CENTER_SEARCH_DEG = 8
CENTER_STEP_DEG   = 1
CENTER_SAMPLES    = 3
CENTER_SENSOR_IDX = 3

POLE_POSITIONS = {
    1: (0, 75),
    4: (175, 70),
}

# ==================================================================
# EMBEDDED PATH  (identical to SDK version)
# ==================================================================
PATH = [
    {"x": 0, "y": 0, "type": "start", "pole": 0, "arc_deg": 0},
    {"x": 36.772, "y": 30.1467, "type": "move", "pole": 1, "arc_deg": 0},
    {"x": 39.0724, "y": 32.1357, "type": "wrap", "pole": 1, "arc_deg": 679.02},
    {"x": 55.2836, "y": 57.4581, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 56.6382, "y": 87.4946, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 42.7721, "y": 114.1733, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 17.4115, "y": 130.3248, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -12.6281, "y": 131.6086, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -39.2741, "y": 117.6796, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -55.3657, "y": 92.2811, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -56.5787, "y": 62.2385, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -42.5869, "y": 35.6254, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -17.1505, "y": 19.5937, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 12.8948, "y": 18.4516, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 39.4749, "y": 32.5061, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 55.4466, "y": 57.9802, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 56.5179, "y": 88.0281, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 42.4007, "y": 114.5749, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": 16.8891, "y": 130.4866, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -13.1613, "y": 131.487, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -39.6748, "y": 117.3073, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -55.5262, "y": 91.7582, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -56.4558, "y": 61.7055, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -42.2137, "y": 35.2256, "type": "wrap", "pole": 1, "arc_deg": 0},
    {"x": -16.6272, "y": 19.4344, "type": "wrap", "pole": 1, "arc_deg": 0},
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
# IR CORRECTION - identical logic to SDK version
# ==================================================================
async def ir_correct_pose(robot, pole_id, wrap_x, wrap_y):
    if pole_id not in POLE_POSITIONS:
        print(f"  [A][ir-fix] no known pos for pole {pole_id}, skipping")
        return

    pose = await robot.get_position()
    rx, ry, rh = pose.x, pose.y, pose.heading
    px, py = POLE_POSITIONS[pole_id]

    target_heading = math.degrees(math.atan2(py - ry, px - rx))
    delta = target_heading - rh
    while delta > 180:  delta -= 360
    while delta < -180: delta += 360

    if delta >= 0:
        await robot.turn_left(delta)
    else:
        await robot.turn_right(-delta)

    await robot.turn_right(CENTER_SEARCH_DEG)

    best_reading = -1
    best_index   = 0
    num_steps    = int(2 * CENTER_SEARCH_DEG / CENTER_STEP_DEG)

    for step in range(num_steps + 1):
        total = 0
        for _ in range(CENTER_SAMPLES):
            ir = await robot.get_ir_proximity()
            total += ir.sensors[CENTER_SENSOR_IDX]
        avg = total / CENTER_SAMPLES
        if avg > best_reading:
            best_reading = avg
            best_index   = step
        if step < num_steps:
            await robot.turn_left(CENTER_STEP_DEG)

    back = (num_steps - best_index) * CENTER_STEP_DEG
    if back > 0:
        await robot.turn_right(back)

    offset = (best_index * CENTER_STEP_DEG) - CENTER_SEARCH_DEG

    pose2 = await robot.get_position()
    dx = px - pose2.x
    dy = py - pose2.y
    measured_dist = math.sqrt(dx*dx + dy*dy)
    nudge = measured_dist - ARC_R

    if nudge > 10:   nudge = 10
    if nudge < -10:  nudge = -10

    if abs(nudge) > 0.5:
        await robot.move(nudge)

    print(f"  [A][ir-fix] pole {pole_id}"
          f" | ir={round(best_reading,1)}"
          f" | heading_off={round(offset,1)}deg"
          f" | dist_nudge={round(nudge,1)}cm")


async def apply_correction(robot, pt):
    if CORRECTION_MODE == "none":
        return
    elif CORRECTION_MODE == "ir":
        await ir_correct_pose(robot, pt["pole"], pt["x"], pt["y"])
    else:
        print(f"  [A][correction] unknown mode '{CORRECTION_MODE}', skipping")


# ==================================================================
# MAIN LOGIC - identical structure to SDK play() handler
# ==================================================================
async def run_robot_A(robot: Create3Ros):
    await robot.wait_until_ready()
    await robot.reset_navigation()
    await robot.set_lights_on_rgb(0, 255, 0)      # Green = Robot A

    print(f"Robot A | dir={DIR} | wrap={WRAP_MODE}"
          f" | correction={CORRECTION_MODE} | {len(PATH)} waypoints")

    current_pole = None

    for i, pt in enumerate(PATH):
        if pt["type"] == "start":
            continue

        if pt["type"] == "move":
            if pt["pole"] != current_pole:
                current_pole = pt["pole"]
                print(f"\n[A] --- Heading to Pole {current_pole} ---")
            await robot.navigate_to(pt["x"], pt["y"])

        elif pt["type"] == "wrap":
            prev = PATH[i - 1] if i > 0 else None
            nxt  = PATH[i + 1] if i + 1 < len(PATH) else None

            is_first_wrap = (prev is None or prev["type"] != "wrap"
                             or prev["pole"] != pt["pole"])
            is_last_wrap  = (nxt is None or nxt["type"] != "wrap"
                             or nxt["pole"] != pt["pole"])

            if WRAP_MODE == "arc":
                if is_first_wrap:
                    await apply_correction(robot, pt)
                    await robot.set_lights_spin_rgb(255, 115, 0)
                    sweep = pt.get("arc_deg", 360)
                    print(f"[A]   Wrapping pole {pt['pole']} ({DIR})"
                          f" [arc mode, {round(sweep,1)} deg]")
                    await robot.arc_left(sweep, ARC_R)
                    await robot.set_lights_on_rgb(0, 255, 0)
                    print(f"[A]   Done wrapping pole {pt['pole']}")
                    await apply_correction(robot, pt)
                continue

            elif WRAP_MODE == "navigate":
                if is_first_wrap:
                    await apply_correction(robot, pt)
                    await robot.set_lights_spin_rgb(255, 115, 0)
                    print(f"[A]   Wrapping pole {pt['pole']} ({DIR}) [navigate mode]")

                await robot.navigate_to(pt["x"], pt["y"])

                if is_last_wrap:
                    await apply_correction(robot, pt)
                    await robot.set_lights_on_rgb(0, 255, 0)
                    print(f"[A]   Done wrapping pole {pt['pole']}")

    await robot.set_lights_blink_rgb(255, 255, 255)
    await robot.play_note(440, 0.5)
    await robot.play_note(880, 0.5)
    print("\nRobot A complete!")


# ==================================================================
# Standalone entry point (single robot)
# ==================================================================
def main():
    rclpy.init()
    robot = Create3Ros(namespace='robot_a')   # change to '' if no namespace

    import threading
    executor_thread = threading.Thread(
        target=rclpy.spin, args=(robot,), daemon=True)
    executor_thread.start()

    try:
        asyncio.run(run_robot_A(robot))
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
