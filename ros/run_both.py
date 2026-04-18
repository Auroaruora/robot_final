"""
run_both.py
================================================================
Launch Robot A and Robot B concurrently, each in its own namespace,
in a single ROS 2 process.

This is the file you actually run when you want both robots going
at once. Single-robot testing can still use the individual files.

Usage:
    python3 run_both.py
"""

import asyncio
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from create3_driver import Create3Ros
from run_robot_A_ros2 import run_robot_A
from run_robot_B_ros2 import run_robot_B


# ------------------------------------------------------------------
# Namespaces - must match what you set in each robot's webserver
# ------------------------------------------------------------------
ROBOT_A_NAMESPACE = 'robot_a'
ROBOT_B_NAMESPACE = 'robot_b'


async def run_all(robot_a, robot_b):
    """Kick off both robots and wait for both to finish."""
    task_a = asyncio.create_task(run_robot_A(robot_a))
    task_b = asyncio.create_task(run_robot_B(robot_b))
    await asyncio.gather(task_a, task_b)


def main():
    rclpy.init()

    # Create one node per robot, each scoped to its own namespace
    robot_a = Create3Ros(namespace=ROBOT_A_NAMESPACE)
    robot_b = Create3Ros(namespace=ROBOT_B_NAMESPACE)

    # One executor spins both nodes in a background thread, so
    # subscription callbacks (odom, IR) fire for both robots while
    # our asyncio logic drives actions.
    executor = MultiThreadedExecutor()
    executor.add_node(robot_a)
    executor.add_node(robot_b)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        asyncio.run(run_all(robot_a, robot_b))
    finally:
        executor.shutdown()
        robot_a.destroy_node()
        robot_b.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
