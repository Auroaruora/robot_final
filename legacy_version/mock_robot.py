"""
mock_robot.py  — drop-in replacement for Create3 for at-home testing.

Usage:
    from mock_robot import MockCreate3, event

    robot = MockCreate3()

    @event(robot.when_play)
    async def play(robot):
        await robot.move(10)
        await robot.turn_left(90)
        ...

    robot.play()   # runs the registered async function synchronously

Simulates (x, y, heading) state in the same units/convention as the real
Create 3 SDK:
  - x, y in centimeters
  - heading in degrees, math convention (0° = +x axis, 90° = +y axis)
  - reset_navigation() sets state to (0, 0, 90°) — matches the real SDK,
    where +y is "forward" out of the robot's bumper.

Every move/turn/arc updates state and prints what the robot "would" do.
At the end of play(), you can read robot.trajectory for the full (x,y,h)
history, useful for plotting or sanity-checking the path.
"""

import asyncio
import math


# A stand-in for the real SDK's @event decorator.
# The real decorator hooks into an event loop; ours just registers the
# callback on a sentinel object so robot.play() knows what to run.
def event(trigger):
    """Decorator that registers an async fn as the robot's play callback.
    `trigger` is the object returned by robot.when_play — we attach the
    function to it so robot.play() can find it later."""
    def decorator(fn):
        trigger._callback = fn
        return fn
    return decorator


class _PlayTrigger:
    """Sentinel object returned by robot.when_play. The @event decorator
    attaches the user's async function to _callback."""
    def __init__(self):
        self._callback = None


class _Position:
    """Matches the shape of Create3's get_position() return value,
    which has .x, .y, .heading attributes."""
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

    def __repr__(self):
        return f"Position(x={self.x:.1f}, y={self.y:.1f}, heading={self.heading:.1f})"


class MockCreate3:
    """Fake Create3 that tracks simulated pose instead of driving real hardware."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 90.0   # degrees; matches real SDK reset state
        self.trajectory = [(0.0, 0.0, 90.0)]   # list of (x, y, heading) snapshots
        self.when_play = _PlayTrigger()

    # ----- internal helpers -----
    def _log_pose(self, tag):
        self.trajectory.append((self.x, self.y, self.heading))
        print(f"    [mock] {tag:<18s} -> pose=({self.x:+7.1f}, {self.y:+7.1f}) "
              f"heading={self.heading:+6.1f}")

    def _normalise_heading(self):
        while self.heading > 180:   self.heading -= 360
        while self.heading <= -180: self.heading += 360

    # ----- navigation -----
    async def reset_navigation(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 90.0
        self.trajectory = [(0.0, 0.0, 90.0)]
        self._log_pose("reset_navigation")

    async def move(self, distance_cm):
        rad = math.radians(self.heading)
        self.x += distance_cm * math.cos(rad)
        self.y += distance_cm * math.sin(rad)
        self._log_pose(f"move({distance_cm:.1f})")

    async def turn_left(self, deg):
        self.heading += deg
        self._normalise_heading()
        self._log_pose(f"turn_left({deg:.1f})")

    async def turn_right(self, deg):
        self.heading -= deg
        self._normalise_heading()
        self._log_pose(f"turn_right({deg:.1f})")

    async def arc_left(self, deg, radius_cm):
        """Arc counter-clockwise: circle center is to the robot's left,
        perpendicular to current heading."""
        self._arc(deg, radius_cm, direction=+1)
        self._log_pose(f"arc_left({deg:.1f}, r={radius_cm})")

    async def arc_right(self, deg, radius_cm):
        """Arc clockwise: circle center is to the robot's right."""
        self._arc(deg, radius_cm, direction=-1)
        self._log_pose(f"arc_right({deg:.1f}, r={radius_cm})")

    def _arc(self, deg, radius_cm, direction):
        # Center of the circle is perpendicular to heading, offset by radius.
        # direction = +1 for left (CCW), -1 for right (CW)
        perp = self.heading + direction * 90
        prad = math.radians(perp)
        cx = self.x + radius_cm * math.cos(prad)
        cy = self.y + radius_cm * math.sin(prad)

        # Angle from centre to robot right now
        start_angle = math.degrees(math.atan2(self.y - cy, self.x - cx))

        # Sweep: positive deg rotates CCW around centre for left arc,
        # CW for right arc
        end_angle = start_angle + direction * deg
        end_rad = math.radians(end_angle)
        self.x = cx + radius_cm * math.cos(end_rad)
        self.y = cy + radius_cm * math.sin(end_rad)
        self.heading += direction * deg
        self._normalise_heading()

    async def navigate_to(self, x, y, heading=None):
        self.x = float(x)
        self.y = float(y)
        if heading is not None:
            self.heading = float(heading)
            self._normalise_heading()
        args = f"{x:.1f}, {y:.1f}" + ("" if heading is None else f", h={heading:.1f}")
        self._log_pose(f"navigate_to({args})")

    async def get_position(self):
        return _Position(self.x, self.y, self.heading)

    # ----- lights -----
    async def set_lights_on_rgb(self, r, g, b):
        print(f"    [mock] lights ON  rgb=({r},{g},{b})")

    async def set_lights_spin_rgb(self, r, g, b):
        print(f"    [mock] lights SPIN rgb=({r},{g},{b})")

    async def set_lights_blink_rgb(self, r, g, b):
        print(f"    [mock] lights BLINK rgb=({r},{g},{b})")

    async def set_lights_off(self):
        print(f"    [mock] lights OFF")

    # ----- sounds -----
    async def play_note(self, hz, duration):
        print(f"    [mock] play_note({hz}Hz, {duration}s)")

    async def stop_sound(self):
        print(f"    [mock] stop_sound")

    # ----- sensors (stubbed) -----
    async def get_ir_proximity(self):
        # Returns an object with .sensors: list of 7 readings
        class _IR:
            sensors = [0, 0, 0, 0, 0, 0, 0]
        return _IR()

    async def get_bumpers(self):
        class _B:
            left = False
            right = False
        return _B()

    async def get_battery_level(self):
        return 100

    # ----- lifecycle -----
    def play(self):
        """Run the registered async callback synchronously (so the script
        behaves the same way the real SDK behaves when 'play' is pressed)."""
        cb = self.when_play._callback
        if cb is None:
            print("[mock] no @event(robot.when_play) handler registered")
            return
        print("[mock] === robot.play() starting ===")
        asyncio.run(cb(self))
        print(f"[mock] === robot.play() done "
              f"({len(self.trajectory)} pose snapshots) ===")