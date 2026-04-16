"""
run_robot.py  —  iRobot Create 3 runner
=========================================
Imports planning logic from path_planner.py.
Plans in world coords, converts to robot-local for navigate_to().

Usage:
    python run_robot.py A
    python run_robot.py B
"""

import sys
from legacy_version.path_planner import (
    ROBOTS, POLES, ARC_R, TURN_WAIT,
    build_path, path_in_local,
)
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3


def main():
    if len(sys.argv) < 2 or sys.argv[1].upper() not in ROBOTS:
        print("Usage:  python run_robot.py A|B")
        sys.exit(1)

    RID = sys.argv[1].upper()
    cfg = ROBOTS[RID]
    robot = Create3(Bluetooth())

    @event(robot.when_play)
    async def play(robot):
        await robot.reset_navigation()

        if RID == "A":
            await robot.set_lights_on_rgb(0, 255, 0)
        else:
            await robot.set_lights_on_rgb(0, 0, 255)

        # Build path in world coords → convert to local
        world_pts = build_path(RID)
        local_pts = path_in_local(world_pts, RID)

        direction = cfg["dir"]
        print(f"Robot {RID} | start={cfg['start']} seq={cfg['seq']} "
              f"dir={direction} | {len(local_pts)} waypoints")

        current_pole = None

        for i, pt in enumerate(local_pts):
            if pt["type"] == "start":
                continue

            if pt["type"] == "move":
                if pt["pole"] != current_pole:
                    current_pole = pt["pole"]
                    print(f"\n--- Heading to Pole {current_pole} ---")
                    if RID == "B":
                        await robot.wait(TURN_WAIT // 2)

                await robot.navigate_to(pt["x"], pt["y"])

            elif pt["type"] == "wrap":
                prev = local_pts[i - 1] if i > 0 else None
                if prev is None or prev["type"] != "wrap" or prev["pole"] != pt["pole"]:
                    await robot.set_lights_spin_rgb(255, 115, 0)
                    print(f"  Wrapping pole {pt['pole']} ({direction})")

                    for _ in range(cfg["circles"]):
                        if direction == "ccw":
                            await robot.arc_left(360, ARC_R)
                        else:
                            await robot.arc_right(360, ARC_R)

                    if RID == "A":
                        await robot.set_lights_on_rgb(0, 255, 0)
                    else:
                        await robot.set_lights_on_rgb(0, 0, 255)

                    print(f"  Done wrapping pole {pt['pole']}")
                    if RID == "A":
                        await robot.wait(TURN_WAIT // 2)
                    await robot.wait(2.0)
                continue

        await robot.set_lights_blink_rgb(255, 255, 255)
        await robot.play_note(440, 0.5)
        await robot.play_note(880, 0.5)
        await robot.stop_sound()
        print(f"\nRobot {RID} complete!")

    robot.play()


if __name__ == "__main__":
    main()