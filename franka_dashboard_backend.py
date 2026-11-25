"""Hardware adapter and guarded command runner for the Franka dashboard.

ROS/FrankaPy imports are deferred so validation can be tested without hardware.
"""
import csv
import queue
import threading
import time
from dataclasses import dataclass, field

import numpy as np

import config as C


JOINT_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
STALE_SECONDS = 1.0

# key: (display label, unit, channel labels). Missing data stays NaN.
GROUPS = {
    "force_base": ("External force · base O", "N", ("Fx", "Fy", "Fz")),
    "torque_base": ("External moment · base O", "Nm", ("Tx", "Ty", "Tz")),
    "joint_torque": ("Measured joint torque", "Nm", tuple(f"J{i}" for i in range(1, 8))),
    "joint_torque_rate": ("Joint torque derivative", "Nm/s", tuple(f"J{i}" for i in range(1, 8))),
    "joint_external": ("External joint torque · ROS", "Nm", tuple(f"J{i}" for i in range(1, 8))),
    "joint_desired": ("Desired joint torque · ROS", "Nm", tuple(f"J{i}" for i in range(1, 8))),
    "force_stiffness": ("External force · stiffness K · ROS", "N", ("Fx", "Fy", "Fz")),
    "torque_stiffness": ("External moment · stiffness K · ROS", "Nm", ("Tx", "Ty", "Tz")),
    "gripper_width": ("Measured gripper width", "mm", ("width",)),
    "grasp_command": ("Last grasp force command · NOT measured", "N", ("command",)),
}


def vector(value, size, label):
    out = np.asarray(value, dtype=float)
    if out.shape != (size,) or not np.all(np.isfinite(out)):
        raise ValueError(f"{label} must contain {size} finite numbers")
    return out


def bounded(value, low, high, label):
    value = float(value)
    if not np.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{label} must be between {low:g} and {high:g}")
    return value


def rotation_from_rpy(degrees):
    r, p, y = np.deg2rad(vector(degrees, 3, "Roll/pitch/yaw"))
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def rpy_from_rotation(rotation):
    m = np.asarray(rotation)
    pitch = np.arctan2(-m[2, 0], np.hypot(m[0, 0], m[1, 0]))
    if np.hypot(m[0, 0], m[1, 0]) > 1e-8:
        roll, yaw = np.arctan2(m[2, 1], m[2, 2]), np.arctan2(m[1, 0], m[0, 0])
    else:
        roll, yaw = np.arctan2(-m[1, 2], m[1, 1]), 0.0
    return np.rad2deg([roll, pitch, yaw])


@dataclass
class Sample:
    received: float
    wall_time: float
    xyz: np.ndarray
    rotation: np.ndarray
    joints: np.ndarray
    values: dict


@dataclass
class Limits:
    xyz_min: np.ndarray = field(default_factory=lambda: np.array([
        C.WORKSPACE["x"][0], C.WORKSPACE["y"][0], C.GUI_Z_MIN]))
    xyz_max: np.ndarray = field(default_factory=lambda: np.array([
        C.WORKSPACE["x"][1], C.WORKSPACE["y"][1], C.GUI_Z_MAX]))

    def workspace(self, xyz, margin=0.0):
        xyz = vector(xyz, 3, "XYZ")
        if np.any(xyz - margin < self.xyz_min) or np.any(xyz + margin > self.xyz_max):
            raise ValueError("Target/force travel envelope exceeds GUI workspace in config.py")

    def validate(self, kind, args, sample):
        """Validate again immediately before dispatch, against current telemetry."""
        if sample is None or time.monotonic() - sample.received > STALE_SECONDS:
            raise ValueError("Robot telemetry is missing or stale")
        a = dict(args)
        if kind in ("pose", "jog", "joints"):
            a["duration"] = bounded(a["duration"], 2.0, 30.0, "Motion duration (s)")
        if kind == "jog":
            delta = vector(a.pop("delta"), 3, "Jog displacement")
            if np.linalg.norm(delta) > C.GUI_MAX_TRANSLATION:
                raise ValueError("Jog exceeds GUI_MAX_TRANSLATION")
            a["xyz"] = sample.xyz + delta
            a["rotation"] = sample.rotation.copy()
            kind = "pose"
        if kind == "pose":
            a["xyz"] = vector(a["xyz"], 3, "XYZ")
            rotation = np.asarray(a["rotation"], dtype=float)
            if (rotation.shape != (3, 3) or not np.all(np.isfinite(rotation))
                    or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
                    or not np.isclose(np.linalg.det(rotation), 1.0)):
                raise ValueError("Invalid rotation matrix")
            a["rotation"] = rotation
            self.workspace(sample.xyz)
            self.workspace(a["xyz"])
            if np.linalg.norm(a["xyz"] - sample.xyz) > C.GUI_MAX_TRANSLATION:
                raise ValueError("Move exceeds GUI_MAX_TRANSLATION; use smaller steps")
            angle = np.arccos(np.clip((np.trace(sample.rotation.T @ rotation)-1)/2, -1, 1))
            if angle > np.deg2rad(C.GUI_MAX_ROTATION_DEG) + 1e-8:
                raise ValueError("Move exceeds GUI_MAX_ROTATION_DEG")
        elif kind == "joints":
            a["joints"] = vector(a["joints"], 7, "Joint targets")
            if np.any(a["joints"] < JOINT_MIN) or np.any(a["joints"] > JOINT_MAX):
                raise ValueError("Joint target exceeds Panda joint limits")
            if np.max(np.abs(a["joints"] - sample.joints)) > np.deg2rad(C.GUI_MAX_JOINT_STEP_DEG):
                raise ValueError("Move exceeds GUI_MAX_JOINT_STEP_DEG; use smaller steps")
        elif kind == "gripper":
            a["width"] = bounded(a["width"], 0, 0.08, "Width (m)")
            a["speed"] = bounded(a["speed"], 0.001, 0.05, "Gripper speed (m/s)")
            a["force"] = bounded(a["force"], 0.1, C.GUI_MAX_GRASP_FORCE, "Grasp force (N)")
            a["epsilon"] = bounded(a["epsilon"], 0.0001, 0.01, "Grasp tolerance (m)")
            a["grasp"] = bool(a["grasp"])
            if not np.isfinite(sample.values["gripper_width"][0]):
                raise ValueError("Gripper joint-state feedback is missing or stale")
        elif kind == "force":
            a["forces"] = vector(a["forces"], 3, "Force")
            a["torques"] = vector(a["torques"], 3, "Torque")
            a["duration"] = bounded(a["duration"], 0.2, 5, "Force duration (s)")
            a["ramp"] = bounded(a["ramp"], 0.05, a["duration"]/2, "Ramp (s)")
            a["translation"] = bounded(a["translation"], 0.001, 0.03, "Travel cap (m)")
            a["rotation"] = bounded(a["rotation"], np.deg2rad(0.1), np.deg2rad(10), "Rotation cap (rad)")
            force, torque = np.linalg.norm(a["forces"]), np.linalg.norm(a["torques"])
            if force == 0 and torque == 0:
                raise ValueError("Enter a nonzero force or torque")
            if force > C.GUI_MAX_FORCE or torque > C.GUI_MAX_TORQUE:
                raise ValueError("Wrench magnitude exceeds GUI_MAX_FORCE / GUI_MAX_TORQUE")
            if force*a["duration"] > 20 or torque*a["duration"] > 2:
                raise ValueError("Force × duration > 20 Ns or torque × duration > 2 Nms")
            self.workspace(sample.xyz, margin=a["translation"])
        elif kind != "pose":
            raise ValueError(f"Unknown command: {kind}")
        return kind, a


def export_csv(path, samples):
    columns = [(key, i, f"{key}_{label}_{unit}")
               for key, (_, unit, labels) in GROUPS.items() for i, label in enumerate(labels)]
    with open(path, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["unix_time_s", "elapsed_s"] + [c[2] for c in columns])
        for s in samples:
            writer.writerow([s.wall_time, s.received - samples[0].received] +
                            [s.values.get(key, [np.nan]*len(GROUPS[key][2]))[i]
                             for key, i, _ in columns])


class FrankaBackend:
    """ROS1 FrankaPy arm plus franka_ros gripper actions with checked results."""

    def __init__(self, robot_num=1, state_topic=None, gripper_namespace=None):
        # Construct on the main thread: FrankaArm installs a SIGINT handler.
        import rospy
        from actionlib import SimpleActionClient
        from autolab_core import RigidTransform
        from frankapy import FrankaArm
        from franka_gripper.msg import MoveAction, GraspAction, StopAction
        from sensor_msgs.msg import JointState

        self.fa = FrankaArm(robot_num=robot_num, with_gripper=False)
        self.RT = RigidTransform
        self.cache_lock = threading.Lock()
        self.ros_state = None
        self.gripper_state = None
        self.last_grasp_command = np.nan
        self.active_gripper = None
        self.state_topic = state_topic
        self.topic_note = "Extra ROS torque channels unavailable (no FrankaState topic)"
        namespace = (gripper_namespace or f"/franka_gripper_{robot_num}").rstrip("/")
        self.grip_move = SimpleActionClient(namespace + "/move", MoveAction)
        self.grip_grasp = SimpleActionClient(namespace + "/grasp", GraspAction)
        self.grip_stop = SimpleActionClient(namespace + "/stop", StopAction)
        self.has_gripper = all([client.wait_for_server(rospy.Duration(2.0))
                                for client in (self.grip_move, self.grip_grasp, self.grip_stop)])
        self.gripper_sub = rospy.Subscriber(namespace + "/joint_states", JointState,
                                            self._gripper_callback, queue_size=1)
        try:
            from franka_msgs.msg import FrankaState
            if not state_topic:
                topics = [name for name, typ in rospy.get_published_topics()
                          if typ == "franka_msgs/FrankaState"]
                # Never silently attach another arm's topic in a multi-robot system.
                state_topic = topics[0] if robot_num == 1 and len(topics) == 1 else None
            if state_topic:
                self.state_topic = state_topic
                self.state_sub = rospy.Subscriber(state_topic, FrankaState,
                                                  self._state_callback, queue_size=1)
                self.topic_note = f"Extra ROS channels: {state_topic} (NaN until received)"
        except ImportError:
            self.topic_note = "franka_msgs unavailable; showing FrankaPy force/torque channels"

    def _state_callback(self, msg):
        with self.cache_lock:
            self.ros_state = (time.monotonic(), msg)

    def _gripper_callback(self, msg):
        if len(msg.position) >= 2:
            with self.cache_lock:
                self.gripper_state = (time.monotonic(), sum(msg.position[:2]))

    def read(self):
        # One state RPC per sample; get_gripper_width() would block on a ROS message.
        requested = time.monotonic()
        state = self.fa.get_robot_state()
        now = time.monotonic()
        if now-requested > STALE_SECONDS:
            raise TimeoutError("Robot state RPC took over one second")
        pose = state["pose"] * self.fa.get_tool_base_pose()
        wrench = vector(state["ee_force_torque"], 6, "External wrench")
        values = {key: np.full(len(labels), np.nan) for key, (_, _, labels) in GROUPS.items()}
        values.update(force_base=wrench[:3], torque_base=wrench[3:],
                      joint_torque=vector(state["joint_torques"], 7, "Joint torques"),
                      joint_torque_rate=np.asarray(state.get("joint_torques_derivative", [np.nan]*7)),
                      grasp_command=np.array([self.last_grasp_command]))
        with self.cache_lock:
            extra, gripper = self.ros_state, self.gripper_state
        if extra and now-extra[0] < STALE_SECONDS:
            msg = extra[1]
            values.update(joint_external=np.array(msg.tau_ext_hat_filtered),
                          joint_desired=np.array(msg.tau_J_d),
                          force_stiffness=np.array(msg.K_F_ext_hat_K[:3]),
                          torque_stiffness=np.array(msg.K_F_ext_hat_K[3:]))
        if gripper and now-gripper[0] < STALE_SECONDS:
            values["gripper_width"] = np.array([1000*gripper[1]])
        return Sample(now, time.time(), vector(pose.translation, 3, "Pose"),
                      pose.rotation.copy(), vector(state["joints"], 7, "Joints"), values)

    def start(self, kind, a):
        common = dict(block=False, ignore_errors=False,
                      force_thresholds=[C.GUI_CONTACT_FORCE]*3 + [C.GUI_CONTACT_TORQUE]*3)
        if kind == "pose":
            pose = self.RT(translation=a["xyz"], rotation=a["rotation"],
                           from_frame="franka_tool", to_frame="world")
            self.fa.goto_pose(pose, duration=a["duration"], **common)
        elif kind == "joints":
            self.fa.goto_joints(a["joints"].tolist(), duration=a["duration"], **common)
        elif kind == "force":
            self.fa.apply_effector_forces_torques(
                a["duration"], a["ramp"], a["translation"], a["rotation"],
                forces=a["forces"].tolist(), torques=a["torques"].tolist(), **common)
        elif kind == "gripper":
            if not self.has_gripper:
                raise RuntimeError("Gripper action servers unavailable; check --gripper-namespace")
            from franka_gripper.msg import GraspGoal, MoveGoal
            if a["grasp"]:
                goal = GraspGoal(width=a["width"], speed=a["speed"], force=a["force"])
                goal.epsilon.inner = goal.epsilon.outer = a["epsilon"]
                self.active_gripper = self.grip_grasp
            else:
                goal = MoveGoal(width=a["width"], speed=a["speed"])
                self.active_gripper = self.grip_move
            self.active_gripper.send_goal(goal)
            if a["grasp"]:
                self.last_grasp_command = a["force"]

    def done(self, kind):
        if kind != "gripper":
            done = self.fa.is_skill_done(ignore_errors=False)
            if done:
                # FrankaPy has no public result getter; inspect its action result
                # to distinguish aborted/preempted skills from successful completion.
                state = self.fa._client.get_state()
                if state != 3:
                    raise RuntimeError(f"Arm action ended with ROS status {state}")
            return done
        state = self.active_gripper.get_state()
        if state in (0, 1, 6, 7):
            return False
        result = self.active_gripper.get_result()
        if state != 3 or result is None or not result.success:
            raise RuntimeError(f"Gripper failed: {getattr(result, 'error', state)}")
        return True

    def stop(self):
        # Send both cancellations before waiting for anything. No wait_for_skill()
        # here: that can wait indefinitely on an unhealthy ROS connection.
        from franka_gripper.msg import StopGoal
        errors = []
        for client in (self.fa._client, self.grip_move, self.grip_grasp):
            try:
                client.cancel_goal()
            except Exception as exc:
                errors.append(str(exc))
        try:
            if self.has_gripper:
                self.grip_stop.send_goal(StopGoal())
        except Exception as exc:
            errors.append(str(exc))
        if errors:
            raise RuntimeError("; ".join(errors))
        deadline = time.monotonic() + 3.0
        while self.fa._client.get_state() in (0, 1, 6, 7):
            if time.monotonic() > deadline:
                raise TimeoutError("Arm cancellation was not acknowledged")
            time.sleep(0.02)
        if self.has_gripper:
            import rospy
            if not self.grip_stop.wait_for_result(rospy.Duration(3.0)):
                raise TimeoutError("Gripper stop was not acknowledged")
            result = self.grip_stop.get_result()
            if self.grip_stop.get_state() != 3 or result is None or not result.success:
                raise RuntimeError("Gripper stop action failed")


class Controller:
    """Keep ROS off the UI thread; serialize commands, invalidate on stop."""

    def __init__(self, backend, rate=20):
        self.backend, self.rate = backend, rate
        self.limits = Limits()
        self.lock = threading.RLock()
        self.dispatch_lock = threading.Lock()
        self.latest = None
        self.error = ""
        self.armed = self.busy = self.stopping = False
        self.stop_failed = False
        self.epoch = 0
        self.commands = queue.Queue()
        self.events = queue.Queue()
        self.samples = queue.Queue(maxsize=500)
        self.closed = threading.Event()

    def start(self):
        for target in (self._read_loop, self._command_loop, self._watchdog_loop):
            threading.Thread(target=target, daemon=True).start()

    def fresh(self):
        return self.latest is not None and time.monotonic()-self.latest.received < STALE_SECONDS

    def arm(self):
        with self.lock:
            if self.stop_failed:
                raise ValueError("Previous STOP failed. Press STOP again and check the physical robot")
            if self.stopping or self.busy or not self.fresh():
                raise ValueError("Wait for fresh telemetry and the current command/stop to finish")
            self.armed = True
            self.events.put("Controls enabled")

    def submit(self, kind, args):
        with self.lock:
            if not self.armed or self.stopping:
                raise ValueError("Enable controls before sending a command")
            if self.busy:
                raise ValueError("A command is active; wait or press STOP")
            self.limits.validate(kind, args, self.latest)
            self.busy = True
            self.commands.put((self.epoch, kind, args))

    def stop(self, reason="Stop requested"):
        with self.lock:
            self.armed = False
            self.epoch += 1
            if self.stopping:
                return
            self.stopping = True
            self.events.put(reason + "; controls disabled")

        def cancel():
            try:
                with self.dispatch_lock:
                    self.backend.stop()
                with self.lock:
                    self.stop_failed = False
                self.events.put("Stop actions acknowledged; controls disabled")
            except Exception as exc:
                with self.lock:
                    self.stop_failed = True
                self.events.put(f"STOP ERROR: {exc}. Use the physical stop.")
            finally:
                with self.lock:
                    self.stopping = False
        threading.Thread(target=cancel, daemon=True).start()

    def _read_loop(self):
        while not self.closed.is_set():
            started = time.monotonic()
            try:
                sample = self.backend.read()
                with self.lock:
                    self.latest, self.error = sample, ""
                if self.samples.full():
                    self.samples.get_nowait()
                self.samples.put_nowait(sample)
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
            self.closed.wait(max(0.001, 1/self.rate - (time.monotonic()-started)))

    def _watchdog_loop(self):
        while not self.closed.wait(0.1):
            with self.lock:
                stale = self.armed and not self.fresh()
            if stale:
                self.stop("Telemetry stale")

    def _command_loop(self):
        while not self.closed.is_set():
            try:
                epoch, kind, args = self.commands.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                with self.dispatch_lock:
                    with self.lock:
                        if epoch != self.epoch or not self.armed:
                            continue
                        kind, args = self.limits.validate(kind, args, self.latest)
                    self.backend.start(kind, args)
                self.events.put(f"Running {kind}")
                timeout = args.get("duration", 15.0) + 8.0
                deadline = time.monotonic() + timeout
                while not self.closed.wait(0.03):
                    with self.lock:
                        if epoch != self.epoch:
                            break
                    if self.backend.done(kind):
                        self.events.put(f"{kind.capitalize()} action finished; inspect feedback")
                        break
                    if time.monotonic() > deadline:
                        raise TimeoutError("Command timed out")
            except Exception as exc:
                self.events.put(f"ERROR: {exc}")
                self.stop("Command error")
            finally:
                with self.lock:
                    self.busy = False

    def close(self):
        self.stop("Dashboard closing")
        self.closed.set()
