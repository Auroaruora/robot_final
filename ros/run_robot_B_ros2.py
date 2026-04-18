"""
run_robot_B_ros2.py  (ROS 2 port)
=====================================================
Mirror of run_robot_B.py (SDK version) running over ROS 2.
"""

import asyncio
import math
import rclpy

from create3_driver import Create3Ros

# ==================================================================
# CONFIG
# ==================================================================

DIR     = "cw"
CIRCLES = 1
ARC_R   = 58

WRAP_MODE = "arc"
# WRAP_MODE = "navigate"

CORRECTION_MODE = "ir"
# CORRECTION_MODE = "none"

CENTER_SEARCH_DEG = 8
CENTER_STEP_DEG   = 1
CENTER_SAMPLES    = 3
CENTER_SENSOR_IDX = 3

POLE_POSITIONS = {
    1: (120, -125),
    3: (0, -80),
    4: (295, -130),
    5: (270, -350),
}

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


async def ir_correct_pose(robot, pole_id, wrap_x, wrap_y):
    if pole_id not in POLE_POSITIONS:
        print(f"  [B][ir-fix] no known pos for pole {pole_id}, skipping")
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

    print(f"  [B][ir-fix] pole {pole_id}"
          f" | ir={round(best_reading,1)}"
          f" | heading_off={round(offset,1)}deg"
          f" | dist_nudge={round(nudge,1)}cm")


async def apply_correction(robot, pt):
    if CORRECTION_MODE == "none":
        return
    elif CORRECTION_MODE == "ir":
        await ir_correct_pose(robot, pt["pole"], pt["x"], pt["y"])
    else:
        print(f"  [B][correction] unknown mode '{CORRECTION_MODE}', skipping")


async def run_robot_B(robot: Create3Ros):
    await robot.wait_until_ready()
    await robot.reset_navigation()
    await robot.set_lights_on_rgb(0, 0, 255)      # Blue = Robot B

    print(f"Robot B | dir={DIR} | wrap={WRAP_MODE}"
          f" | correction={CORRECTION_MODE} | {len(PATH)} waypoints")

    current_pole = None

    for i, pt in enumerate(PATH):
        if pt["type"] == "start":
            continue

        if pt["type"] == "move":
            if pt["pole"] != current_pole:
                current_pole = pt["pole"]
                print(f"\n[B] --- Heading to Pole {current_pole} ---")
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
                    print(f"[B]   Wrapping pole {pt['pole']} ({DIR})"
                          f" [arc mode, {round(sweep,1)} deg]")
                    await robot.arc_right(sweep, ARC_R)
                    await robot.set_lights_on_rgb(0, 0, 255)
                    print(f"[B]   Done wrapping pole {pt['pole']}")
                    await apply_correction(robot, pt)
                continue

            elif WRAP_MODE == "navigate":
                if is_first_wrap:
                    await apply_correction(robot, pt)
                    await robot.set_lights_spin_rgb(255, 115, 0)
                    print(f"[B]   Wrapping pole {pt['pole']} ({DIR}) [navigate mode]")

                await robot.navigate_to(pt["x"], pt["y"])

                if is_last_wrap:
                    await apply_correction(robot, pt)
                    await robot.set_lights_on_rgb(0, 0, 255)
                    print(f"[B]   Done wrapping pole {pt['pole']}")

    await robot.set_lights_blink_rgb(255, 255, 255)
    await robot.play_note(440, 0.5)
    await robot.play_note(880, 0.5)
    print("\nRobot B complete!")


def main():
    rclpy.init()
    robot = Create3Ros(namespace='robot_b')

    import threading
    executor_thread = threading.Thread(
        target=rclpy.spin, args=(robot,), daemon=True)
    executor_thread.start()

    try:
        asyncio.run(run_robot_B(robot))
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
