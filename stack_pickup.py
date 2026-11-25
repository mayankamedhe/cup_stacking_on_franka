"""Force-guided pickup of nested, rim-down cups from one fixed XY source.

The shimmy is a separation attempt. Counting cups requires a calibrated payload
band (and enough force resolution); no count is claimed without that check.
"""
import math
import time

import numpy as np

import config as C


class StackPickupError(RuntimeError):
    pass


def _finite(value, label):
    if value is None or not np.isfinite(value):
        raise ValueError(f"Measure and set {label} in config.py before stack pickup")
    return float(value)


def validate_stack_config(c=C):
    """Check settings before constructing a FrankaArm or sending any command."""
    xy = np.asarray(c.STACK_XY, dtype=float)
    if xy.shape != (2,) or not np.all(np.isfinite(xy)):
        raise ValueError("STACK_XY must contain two finite coordinates")
    top = _finite(c.STACK_SEARCH_TOP_Z, "STACK_SEARCH_TOP_Z")
    bottom = _finite(c.STACK_SEARCH_BOTTOM_Z, "STACK_SEARCH_BOTTOM_Z")
    offset = _finite(c.STACK_GRASP_OFFSET_Z, "STACK_GRASP_OFFSET_Z")
    if not c.STACK_MIN_TOOL_Z < bottom < top < c.STACK_MAX_TOOL_Z:
        raise ValueError("Require STACK_MIN_TOOL_Z < search bottom < search top < STACK_MAX_TOOL_Z")
    if not -0.04 <= offset < 0:
        raise ValueError("STACK_GRASP_OFFSET_Z must be negative and no lower than -0.04 m")
    positive = ("STACK_PROBE_SPEED", "STACK_CONTACT_FORCE", "STACK_MAX_FORCE",
                "STACK_MAX_TORQUE", "STACK_FORCE_NOISE", "STACK_PROBE_STIFFNESS",
                "STACK_INITIAL_LIFT", "STACK_SHIMMY_XY", "STACK_SHIMMY_DURATION",
                "STACK_SHIMMY_RISE")
    for key in positive:
        if _finite(getattr(c, key), key) <= 0:
            raise ValueError(f"{key} must be positive")
    if not 0 < c.STACK_PROBE_SPEED <= .01:
        raise ValueError("STACK_PROBE_SPEED must be <= 0.01 m/s")
    if not c.STACK_CONTACT_FORCE < c.STACK_MAX_FORCE <= 10:
        raise ValueError("Require STACK_CONTACT_FORCE < STACK_MAX_FORCE <= 10 N")
    if c.STACK_CONTACT_FORCE <= 3*c.STACK_FORCE_NOISE:
        raise ValueError("Contact threshold must exceed three times STACK_FORCE_NOISE")
    if not 2 <= c.STACK_CONTACT_SAMPLES <= 10 or int(c.STACK_CONTACT_SAMPLES) != c.STACK_CONTACT_SAMPLES:
        raise ValueError("STACK_CONTACT_SAMPLES must be an integer between 2 and 10")
    if not 1 <= c.STACK_SHIMMY_CYCLES <= 5 or int(c.STACK_SHIMMY_CYCLES) != c.STACK_SHIMMY_CYCLES:
        raise ValueError("STACK_SHIMMY_CYCLES must be an integer between 1 and 5")
    if not 0 < c.STACK_SHIMMY_XY <= .005 or not 0 <= c.STACK_SHIMMY_YAW_DEG <= 5:
        raise ValueError("Limit shimmy to 5 mm XY and 5 degrees yaw")
    if c.STACK_SHIMMY_DURATION < .5 or c.STACK_SHIMMY_RISE > .01 or c.STACK_INITIAL_LIFT > .02:
        raise ValueError("Shimmy segments must be >= 0.5 s; rise <= 10 mm/cycle; initial lift <= 20 mm")
    if bottom + offset < c.STACK_MIN_TOOL_Z:
        raise ValueError("The lowest possible grasp is below STACK_MIN_TOOL_Z")
    if top + c.CUP_HEIGHT + c.CLEARANCE > c.STACK_MAX_TOOL_Z:
        raise ValueError("Stack plus carried-cup clearance exceeds STACK_MAX_TOOL_Z")
    for axis, value in zip(("x", "y"), xy):
        low, high = c.WORKSPACE[axis]
        if value-c.STACK_SHIMMY_XY < low or value+c.STACK_SHIMMY_XY > high:
            raise ValueError("Stack / shimmy envelope is outside WORKSPACE")
    if (c.STACK_SINGLE_WEIGHT is None) != (c.STACK_WEIGHT_TOLERANCE is None):
        raise ValueError("Set both STACK_SINGLE_WEIGHT and STACK_WEIGHT_TOLERANCE, or leave both None")
    if c.STACK_SINGLE_WEIGHT is not None:
        weight = _finite(c.STACK_SINGLE_WEIGHT, "STACK_SINGLE_WEIGHT")
        tol = _finite(c.STACK_WEIGHT_TOLERANCE, "STACK_WEIGHT_TOLERANCE")
        if not 0 < tol < weight/2 or weight+tol >= c.STACK_MAX_FORCE:
            raise ValueError("Weight tolerance must be positive, less than half one cup's weight, and below the force cap")
    return xy


def shimmy_waypoints(xy, z, c=C):
    """Small lateral loops with alternating yaw and a gradual upward peel.

    Ends exactly over the stack at zero yaw. Durations are set by the caller.
    """
    x, y = xy
    a = c.STACK_SHIMMY_XY
    for cycle in range(c.STACK_SHIMMY_CYCLES):
        for step, (dx, dy, sign) in enumerate(((a, 0, 1), (0, a, -1),
                                              (-a, 0, 1), (0, -a, -1), (0, 0, 0)), 1):
            yield ((x+dx, y+dy), z+c.STACK_SHIMMY_RISE*(cycle+step/5),
                   math.radians(sign*c.STACK_SHIMMY_YAW_DEG))


class StackPicker:
    def __init__(self, robot, settings=C, clock=time):
        self.robot, self.fa, self.c, self.clock = robot, robot.fa, settings, clock
        self.xy = validate_stack_config(settings)
        self.clear_z = max(robot.travel_z, settings.STACK_SEARCH_TOP_Z + settings.CUP_HEIGHT + settings.CLEARANCE)
        if self.clear_z > settings.STACK_MAX_TOOL_Z:
            raise ValueError("Pyramid/stack travel height exceeds STACK_MAX_TOOL_Z")
        robot.travel_z = self.clear_z
        self.baseline = None
        self.last_std = None
        self.previous_top = None
        self.records = []

    def _read(self):
        started = self.clock.monotonic()
        state = self.fa.get_robot_state()
        if self.clock.monotonic()-started > 0.5:
            raise StackPickupError("Stack telemetry took more than 0.5 s")
        wrench = np.asarray(state["ee_force_torque"], dtype=float)
        pose = state["pose"] * self.fa.get_tool_base_pose()
        xyz = np.asarray(pose.translation, dtype=float)
        if wrench.shape != (6,) or xyz.shape != (3,) or not np.all(np.isfinite(np.r_[wrench, xyz])):
            raise StackPickupError("Invalid stack force/pose feedback")
        if not self.c.STACK_MIN_TOOL_Z <= xyz[2] <= self.c.STACK_MAX_TOOL_Z:
            raise StackPickupError("Measured tool height is outside stack motion limits")
        return xyz, wrench

    def _guard(self, wrench):
        delta = wrench if self.baseline is None else wrench-self.baseline
        if (np.linalg.norm(delta[:3]) > self.c.STACK_MAX_FORCE
                or np.linalg.norm(delta[3:]) > self.c.STACK_MAX_TORQUE):
            raise StackPickupError("Excess force/torque during stack pickup; stopped without opening gripper")

    def _stationary_wrench(self):
        # Used only with no skill active, after settling. Median rejects isolated spikes.
        self.clock.sleep(.5)
        readings = []
        for _ in range(20):
            _, wrench = self._read()
            self._guard(wrench)
            readings.append(wrench)
            self.clock.sleep(.025)
        readings = np.array(readings)
        self.last_std = np.std(readings[:, :3], axis=0)
        if np.max(self.last_std) > self.c.STACK_FORCE_NOISE:
            raise StackPickupError("Force readings are too noisy for stack sensing")
        return np.median(readings, axis=0)

    def _move(self, xy, z, duration, yaw=0., probe=False):
        if not self.c.STACK_MIN_TOOL_Z <= z <= self.c.STACK_MAX_TOOL_Z:
            raise StackPickupError("Stack motion target exceeds tool-height limits")
        for axis, value in zip(("x", "y"), xy):
            if not self.c.WORKSPACE[axis][0] <= value <= self.c.WORKSPACE[axis][1]:
                raise StackPickupError("Stack motion target exceeds WORKSPACE")
        cs, sn = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[cs, -sn, 0], [sn, cs, 0], [0, 0, 1]]) @ self.c.GRIPPER_ROTATION
        kwargs = dict(block=False, ignore_errors=False)
        if probe:
            kwargs["cartesian_impedances"] = [600, 600, self.c.STACK_PROBE_STIFFNESS, 50, 50, 50]
        self.robot.goto(xy, z, duration, rotation=rotation, **kwargs)
        started = self.clock.monotonic()
        contacts = []
        try:
            while not self.fa.is_skill_done(ignore_errors=False):
                xyz, wrench = self._read()
                self._guard(wrench)
                if probe:
                    if np.linalg.norm(xyz[:2]-self.xy) > .005:
                        raise StackPickupError("Probe drifted more than 5 mm from stack centre")
                    # Downward probing receives an upward (+base Z) contact force.
                    if wrench[2]-self.baseline[2] >= self.c.STACK_CONTACT_FORCE:
                        contacts.append(xyz[2])
                        if len(contacts) >= self.c.STACK_CONTACT_SAMPLES:
                            self.fa.stop_skill()
                            return float(contacts[0])  # actual tool Z at first confirmed contact
                    else:
                        contacts.clear()
                if self.clock.monotonic()-started > duration+2:
                    raise StackPickupError("Stack motion timed out")
                self.clock.sleep(.02)
            # FrankaPy wait_for_skill does not itself verify the action status.
            if self.fa._client.get_state() != 3:
                raise StackPickupError("Stack motion action did not succeed")
            if not probe:
                xyz, wrench = self._read()
                self._guard(wrench)
                if np.linalg.norm(xyz-np.r_[xy, z]) > .005:
                    raise StackPickupError("Stack motion ended more than 5 mm from its target")
        except BaseException:
            self.fa.stop_skill()
            raise
        return None

    def _approach(self):
        # Lift vertically before moving laterally over the maximum possible stack.
        xyz, _ = self._read()
        self._move(xyz[:2], self.clear_z, self.c.TRAVEL_DURATION)
        self._move(self.xy, self.clear_z, self.c.TRAVEL_DURATION)

    def find_top(self):
        self.fa.goto_gripper(0., ignore_errors=False)
        self._approach()
        self._move(self.xy, self.c.STACK_SEARCH_TOP_Z, self.c.VERTICAL_DURATION)
        self.baseline = None
        self.baseline = self._stationary_wrench()
        duration = max(2., (self.c.STACK_SEARCH_TOP_Z-self.c.STACK_SEARCH_BOTTOM_Z)
                       / self.c.STACK_PROBE_SPEED)
        top = self._move(self.xy, self.c.STACK_SEARCH_BOTTOM_Z, duration, probe=True)
        if top is None:
            # A missed/empty stack is never turned into a guessed grasp height.
            self._move(self.xy, self.clear_z, self.c.VERTICAL_DURATION)
            raise StackPickupError("No confirmed contact before STACK_SEARCH_BOTTOM_Z; no grasp attempted")
        if not self.c.STACK_SEARCH_BOTTOM_Z+.002 < top < self.c.STACK_SEARCH_TOP_Z-.003:
            raise StackPickupError("Contact is too close to a search boundary; check the stack search bounds")
        if self.previous_top is not None and top > self.previous_top+.003:
            raise StackPickupError("Stack top rose unexpectedly; source may have moved")
        print(f"    Stack contact: tool Z={top:.4f} m; baseline-relative Fz >= {self.c.STACK_CONTACT_FORCE:g} N")
        return top

    def _grasp(self, z):
        self.fa.goto_gripper(self.c.OPEN_WIDTH, ignore_errors=False)
        self._move(self.xy, z, self.c.VERTICAL_DURATION)
        self.fa.goto_gripper(self.c.GRASP_WIDTH, grasp=True, force=self.c.GRASP_FORCE,
                             epsilon_inner=.002, epsilon_outer=.002, ignore_errors=False)
        # Standard FrankaPy discards the ROS grasp result in goto_gripper().
        result = self.fa._gripper_grasp_client.get_result()
        if result is None or not result.success:
            raise StackPickupError("Stack grasp failed: " + str(getattr(result, "error", "no action result")))

    def pick(self):
        # Empty-hand reference and loaded measurement use the same pose above the stack.
        self.baseline = None
        self._approach()
        self.fa.goto_gripper(self.c.GRASP_WIDTH, ignore_errors=False)
        empty = self._stationary_wrench()
        empty_noise = self.last_std[2]
        top = self.find_top()
        grasp_z = top + self.c.STACK_GRASP_OFFSET_Z
        self._move(self.xy, top + self.c.CLEARANCE, self.c.VERTICAL_DURATION)
        self._grasp(grasp_z)
        peel_z = grasp_z + self.c.STACK_INITIAL_LIFT
        self._move(self.xy, peel_z, self.c.VERTICAL_DURATION)
        for xy, z, yaw in shimmy_waypoints(self.xy, peel_z, self.c):
            self._move(xy, z, self.c.STACK_SHIMMY_DURATION, yaw=yaw)
        self._move(self.xy, self.clear_z, self.c.VERTICAL_DURATION)
        loaded = self._stationary_wrench()
        weight = float(empty[2]-loaded[2])
        uncertainty = 3*math.hypot(empty_noise, self.last_std[2])
        verified = False
        record = dict(contact_z=top, grasp_z=grasp_z, payload_n=weight,
                      payload_uncertainty_n=uncertainty, single_cup_band=False)
        self.records.append(record)
        print(f"    Payload estimate: {weight:.3f} N; stationary noise allowance: ±{uncertainty:.3f} N")
        if self.c.STACK_SINGLE_WEIGHT is not None:
            if uncertainty >= self.c.STACK_WEIGHT_TOLERANCE:
                raise StackPickupError("Force noise cannot resolve the configured single-cup weight band")
            if abs(weight-self.c.STACK_SINGLE_WEIGHT)+uncertainty > self.c.STACK_WEIGHT_TOLERANCE:
                raise StackPickupError(f"Payload {weight:.3f} N is outside the calibrated single-cup band; "
                                       "gripper remains closed for inspection")
            verified = True
        print("    Separation complete; " + (f"payload {weight:.3f} N is in single-cup band"
                                               if verified else "cup count UNVERIFIED (no calibrated payload band)"))
        self.previous_top = top
        record["single_cup_band"] = verified
        return record
