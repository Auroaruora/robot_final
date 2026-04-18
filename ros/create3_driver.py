"""
create3_driver.py
================================================================
ROS 2 wrapper that mimics the irobot_edu_sdk Create3 API.

The goal is to let your existing pole-wrapping logic run UNCHANGED
against a ROS 2 robot. Method names and semantics match the SDK
(cm + degrees in, same returns), but under the hood everything
goes through ROS 2 actions/topics/services.

Usage:
    rclpy.init()
    robot = Create3Ros('robot_a')          # namespace set via webserver
    await robot.wait_until_ready()
    await robot.reset_navigation()
    await robot.arc_left(90, 58)           # 90 deg, radius 58 cm
    pose = await robot.get_position()      # .x .y .heading (cm, deg)
"""

import math
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from irobot_create_msgs.msg import (
    IrIntensityVector, LightringLeds, LedColor, InterfaceButtons
)
from irobot_create_msgs.action import (
    DriveArc, DriveDistance, RotateAngle, NavigateToPosition,
    AudioNoteSequence, LedAnimation
)
from irobot_create_msgs.srv import ResetPose

# ------------------------------------------------------------------
# Unit conversions: SDK uses cm/deg, ROS 2 uses m/rad
# ------------------------------------------------------------------
def cm_to_m(cm):   return cm / 100.0
def m_to_cm(m):    return m * 100.0
def deg_to_rad(d): return math.radians(d)
def rad_to_deg(r): return math.degrees(r)


def yaw_from_quat(q: Quaternion) -> float:
    """Extract yaw (heading) in radians from a geometry_msgs Quaternion."""
    # Standard ZYX extraction, reduced to yaw only (roll/pitch ~= 0 on flat ground)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Pose:
    """Lightweight pose object matching SDK's get_position() return shape."""
    def __init__(self, x_cm, y_cm, heading_deg):
        self.x = x_cm
        self.y = y_cm
        self.heading = heading_deg
    def __repr__(self):
        return f"Pose(x={self.x:.1f}, y={self.y:.1f}, h={self.heading:.1f})"


class IrReading:
    """Matches SDK's get_ir_proximity() return (has .sensors list)."""
    def __init__(self, sensors):
        self.sensors = sensors


# ------------------------------------------------------------------
# Main driver class
# ------------------------------------------------------------------
class Create3Ros(Node):
    """
    One instance = one robot. Pass the namespace you set in the
    robot's webserver (e.g. 'robot_a'). Without a namespace, pass ''.
    """

    # Default speeds (tune for your robot)
    DEFAULT_LINEAR_SPEED  = 0.15   # m/s
    DEFAULT_ANGULAR_SPEED = 1.0    # rad/s

    def __init__(self, namespace: str = ''):
        super().__init__(f'create3_driver_{namespace or "default"}')
        self.namespace = namespace
        ns = f'/{namespace}' if namespace else ''

        # ----- Sensor subscriptions (best-effort QoS matches robot) -----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._latest_odom = None
        self._latest_ir   = None

        self.create_subscription(
            Odometry, f'{ns}/odom', self._on_odom, sensor_qos)
        self.create_subscription(
            IrIntensityVector, f'{ns}/ir_intensity', self._on_ir, sensor_qos)

        # ----- Publishers -----
        self._lightring_pub = self.create_publisher(
            LightringLeds, f'{ns}/cmd_lightring', 10)

        # ----- Action clients -----
        self._arc     = ActionClient(self, DriveArc,           f'{ns}/drive_arc')
        self._drive   = ActionClient(self, DriveDistance,      f'{ns}/drive_distance')
        self._rotate  = ActionClient(self, RotateAngle,        f'{ns}/rotate_angle')
        self._nav     = ActionClient(self, NavigateToPosition, f'{ns}/navigate_to_position')
        self._audio   = ActionClient(self, AudioNoteSequence,  f'{ns}/audio_note_sequence')
        self._led_ani = ActionClient(self, LedAnimation,       f'{ns}/led_animation')

        # ----- Service clients -----
        self._reset_pose_srv = self.create_client(ResetPose, f'{ns}/reset_pose')

    # ------------- sensor callbacks -------------
    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg

    def _on_ir(self, msg: IrIntensityVector):
        self._latest_ir = msg

    # ------------- helpers -------------
    async def wait_until_ready(self, timeout_sec: float = 15.0):
        """Block until action servers are reachable and odom has arrived."""
        self.get_logger().info(f'[{self.namespace}] waiting for action servers...')
        self._arc.wait_for_server(timeout_sec=timeout_sec)
        self._drive.wait_for_server(timeout_sec=timeout_sec)
        self._rotate.wait_for_server(timeout_sec=timeout_sec)
        self._nav.wait_for_server(timeout_sec=timeout_sec)
        self.get_logger().info(f'[{self.namespace}] action servers up')

        # wait for first odom message
        start = self.get_clock().now()
        while self._latest_odom is None:
            if (self.get_clock().now() - start).nanoseconds > timeout_sec * 1e9:
                raise RuntimeError('no odometry received - check namespace/network')
            await asyncio.sleep(0.05)
        self.get_logger().info(f'[{self.namespace}] ready')

    async def _send_and_wait(self, client, goal_msg):
        """Send an action goal and await its result."""
        send_future = client.send_goal_async(goal_msg)
        goal_handle = await send_future
        if not goal_handle.accepted:
            self.get_logger().warn('goal rejected')
            return None
        result_future = goal_handle.get_result_async()
        return await result_future

    # ==================================================================
    # SDK-compatible API
    # ==================================================================

    # --- Motion ---
    async def move(self, distance_cm):
        goal = DriveDistance.Goal()
        goal.distance = cm_to_m(distance_cm)
        goal.max_translation_speed = self.DEFAULT_LINEAR_SPEED
        await self._send_and_wait(self._drive, goal)

    async def turn_left(self, angle_deg):
        goal = RotateAngle.Goal()
        goal.angle = deg_to_rad(angle_deg)
        goal.max_rotation_speed = self.DEFAULT_ANGULAR_SPEED
        await self._send_and_wait(self._rotate, goal)

    async def turn_right(self, angle_deg):
        goal = RotateAngle.Goal()
        goal.angle = -deg_to_rad(angle_deg)
        goal.max_rotation_speed = self.DEFAULT_ANGULAR_SPEED
        await self._send_and_wait(self._rotate, goal)

    async def arc_left(self, angle_deg, radius_cm):
        """Counter-clockwise arc. SDK convention: positive = ccw."""
        goal = DriveArc.Goal()
        goal.angle = deg_to_rad(angle_deg)
        goal.radius = cm_to_m(radius_cm)
        goal.translate_direction = 1       # 1 = forward
        goal.max_translation_speed = self.DEFAULT_LINEAR_SPEED
        await self._send_and_wait(self._arc, goal)

    async def arc_right(self, angle_deg, radius_cm):
        """Clockwise arc. Uses negative angle."""
        goal = DriveArc.Goal()
        goal.angle = -deg_to_rad(angle_deg)
        goal.radius = cm_to_m(radius_cm)
        goal.translate_direction = 1
        goal.max_translation_speed = self.DEFAULT_LINEAR_SPEED
        await self._send_and_wait(self._arc, goal)

    async def navigate_to(self, x_cm, y_cm, heading_deg=None):
        goal = NavigateToPosition.Goal()
        goal.goal_pose.pose.position.x = cm_to_m(x_cm)
        goal.goal_pose.pose.position.y = cm_to_m(y_cm)
        goal.goal_pose.pose.position.z = 0.0
        if heading_deg is not None:
            # Build quaternion from yaw
            yaw = deg_to_rad(heading_deg)
            goal.goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal.achieve_goal_heading = True
        else:
            goal.goal_pose.pose.orientation.w = 1.0
            goal.achieve_goal_heading = False
        goal.max_translation_speed = self.DEFAULT_LINEAR_SPEED
        goal.max_rotation_speed    = self.DEFAULT_ANGULAR_SPEED
        await self._send_and_wait(self._nav, goal)

    async def reset_navigation(self):
        """Reset robot's odom frame to origin. SDK sets (0, 0, 90 deg)."""
        if not self._reset_pose_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('reset_pose service not available')
            return
        req = ResetPose.Request()
        # Leave pose unset -> robot defaults to (0, 0) with current heading kept.
        # That differs slightly from SDK (which forces heading=90). If you need
        # exact SDK behavior, set req.pose here.
        await self._reset_pose_srv.call_async(req)

    # --- Sensors ---
    async def get_position(self):
        if self._latest_odom is None:
            return Pose(0.0, 0.0, 0.0)
        p = self._latest_odom.pose.pose.position
        q = self._latest_odom.pose.pose.orientation
        return Pose(m_to_cm(p.x), m_to_cm(p.y), rad_to_deg(yaw_from_quat(q)))

    async def get_ir_proximity(self):
        """Returns object with .sensors (list of 7 ints) like SDK."""
        if self._latest_ir is None:
            return IrReading([0] * 7)
        return IrReading([r.value for r in self._latest_ir.readings])

    # --- Lights ---
    def _build_lightring_msg(self, r, g, b):
        msg = LightringLeds()
        msg.override_system = True
        color = LedColor()
        color.red, color.green, color.blue = int(r), int(g), int(b)
        msg.leds = [color] * 6
        return msg

    async def set_lights_on_rgb(self, r, g, b):
        self._lightring_pub.publish(self._build_lightring_msg(r, g, b))

    async def set_lights_spin_rgb(self, r, g, b):
        # "Spin" animation via LedAnimation action. Fire-and-forget here
        # so it doesn't block the main loop.
        goal = LedAnimation.Goal()
        goal.animation_type = 2        # 2 = spin
        goal.max_runtime.sec = 3600    # effectively "until cancelled"
        goal.lightring = self._build_lightring_msg(r, g, b)
        self._led_ani.send_goal_async(goal)

    async def set_lights_blink_rgb(self, r, g, b):
        goal = LedAnimation.Goal()
        goal.animation_type = 1        # 1 = blink
        goal.max_runtime.sec = 3600
        goal.lightring = self._build_lightring_msg(r, g, b)
        self._led_ani.send_goal_async(goal)

    async def set_lights_off(self):
        await self.set_lights_on_rgb(0, 0, 0)

    # --- Audio ---
    async def play_note(self, freq_hz, duration_sec):
        goal = AudioNoteSequence.Goal()
        goal.iterations = 1
        goal.note_sequence.append = False
        # Build a single-note sequence
        from irobot_create_msgs.msg import AudioNote
        from builtin_interfaces.msg import Duration
        note = AudioNote()
        note.frequency = int(freq_hz)
        note.max_runtime = Duration()
        note.max_runtime.sec = int(duration_sec)
        note.max_runtime.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal.note_sequence.notes = [note]
        await self._send_and_wait(self._audio, goal)

    async def stop_sound(self):
        # No direct "stop" action - play a silent note of duration 0
        pass

    async def wait(self, seconds):
        await asyncio.sleep(seconds)
