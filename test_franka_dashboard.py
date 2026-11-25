"""Hardware-free checks of the hardware dashboard's validation and dispatch.

    python -m unittest test_franka_dashboard -v
"""
import csv
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock

import numpy as np

from franka_dashboard_backend import (
    Controller, FrankaBackend, GROUPS, Limits, Sample, export_csv,
    rotation_from_rpy, rpy_from_rotation,
)


def sample():
    values = {key: np.zeros(len(labels)) for key, (_, _, labels) in GROUPS.items()}
    values["joint_external"][:] = np.nan
    return Sample(time.monotonic(), time.time(), np.array([0.5, 0., 0.3]), np.eye(3),
                  np.array([0., -0.5, 0., -2., 0., 1.5, 0.5]), values)


def force(**kwargs):
    out = dict(forces=[0, 0, -2], torques=[0, 0, 0], duration=1., ramp=0.2,
               translation=0.005, rotation=0.03)
    out.update(kwargs)
    return out


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("Timed out waiting for worker")
        time.sleep(0.005)


class ValidationTests(unittest.TestCase):
    def setUp(self):
        self.s, self.limits = sample(), Limits()

    def test_stale_feedback_rejects_motion(self):
        self.s.received -= 2
        with self.assertRaisesRegex(ValueError, "stale"):
            self.limits.validate("force", force(), self.s)

    def test_jog_uses_current_pose_and_preserves_rotation(self):
        self.s.rotation = rotation_from_rpy([180, 0, 35])
        kind, a = self.limits.validate("jog", dict(delta=[0, 0.005, 0], duration=3), self.s)
        self.assertEqual(kind, "pose")
        np.testing.assert_allclose(a["xyz"], [0.5, .005, .3])
        np.testing.assert_allclose(a["rotation"], self.s.rotation)

    def test_pose_rejects_bad_rotation_distance_and_workspace(self):
        for xyz, rot in (([.5, 0, .3], np.zeros((3, 3))),
                         ([.7, 0, .3], np.eye(3)), ([.5, 0, .02], np.eye(3)),
                         ([.5, 0, .3], rotation_from_rpy([20, 0, 0]))):
            with self.subTest(xyz=xyz, rot=rot), self.assertRaises(ValueError):
                self.limits.validate("pose", dict(xyz=xyz, rotation=rot, duration=3), self.s)

    def test_force_limits_and_nonfinite_values(self):
        self.limits.validate("force", force(), self.s)
        bad = [dict(forces=[np.nan, 0, 0]), dict(forces=[np.inf, 0, 0]),
               dict(forces=[8, 8, 0]), dict(torques=[0, 0, 2]), dict(duration=-1),
               dict(forces=[0, 0, 0]), dict(ramp=0), dict(ramp=.6),
               dict(translation=0), dict(rotation=0), dict(duration=5, forces=[5, 0, 0]),
               dict(duration=3, torques=[1, 0, 0])]
        for changes in bad:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.limits.validate("force", force(**changes), self.s)

    def test_force_envelope_stays_in_workspace(self):
        self.s.xyz[0] = self.limits.xyz_min[0]+.001
        with self.assertRaisesRegex(ValueError, "envelope"):
            self.limits.validate("force", force(), self.s)

    def test_joint_step_and_panda_limits(self):
        target = self.s.joints.copy()
        target[0] += .1
        self.limits.validate("joints", dict(joints=target, duration=3), self.s)
        target[0] += .2
        with self.assertRaisesRegex(ValueError, "step|STEP"):
            self.limits.validate("joints", dict(joints=target, duration=3), self.s)
        target = self.s.joints.copy()
        target[3] = 0
        with self.assertRaisesRegex(ValueError, "joint limits"):
            self.limits.validate("joints", dict(joints=target, duration=3), self.s)

    def test_gripper_requires_feedback_and_positive_bounded_force(self):
        args = dict(width=.07, speed=.02, force=5, epsilon=.001, grasp=True)
        self.limits.validate("gripper", args, self.s)
        with self.assertRaises(ValueError):
            self.limits.validate("gripper", dict(args, force=0), self.s)
        self.s.values["gripper_width"][:] = np.nan
        with self.assertRaisesRegex(ValueError, "Gripper"):
            self.limits.validate("gripper", args, self.s)

    def test_orientation_roundtrip_including_gimbal_lock(self):
        for angles in ([180, 0, 90], [12, 34, 56], [10, 90, 30], [-30, -90, 25]):
            rot = rotation_from_rpy(angles)
            np.testing.assert_allclose(rotation_from_rpy(rpy_from_rotation(rot)), rot, atol=1e-7)

    def test_csv_covers_every_force_channel_and_preserves_unavailable(self):
        s = sample()
        with tempfile.NamedTemporaryFile(suffix=".csv") as f:
            export_csv(f.name, [s])
            with open(f.name) as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows[0]), 2 + sum(len(v[2]) for v in GROUPS.values()))
        self.assertEqual(rows[0]["joint_external_J1_Nm"], "nan")
        self.assertIn("grasp_command_command_N", rows[0])


class DispatchTests(unittest.TestCase):
    def setUp(self):
        self.backend = Mock()
        self.backend.done.return_value = False
        self.c = Controller(self.backend)
        self.c.latest = sample()
        self.worker = threading.Thread(target=self.c._command_loop, daemon=True)
        self.worker.start()

    def tearDown(self):
        self.c.closed.set()
        self.worker.join(timeout=1)

    def test_disarmed_and_busy_commands_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "Enable"):
            self.c.submit("force", force())
        self.c.arm()
        self.c.submit("force", force())
        with self.assertRaisesRegex(ValueError, "active"):
            self.c.submit("force", force())
        wait_for(lambda: self.backend.start.called)
        self.backend.done.return_value = True
        wait_for(lambda: not self.c.busy)

    def test_stop_invalidates_a_command_waiting_for_dispatch(self):
        with self.c.dispatch_lock:
            self.c.arm()
            self.c.submit("force", force())
            self.c.stop()
        wait_for(lambda: not self.c.stopping and not self.c.busy)
        self.backend.start.assert_not_called()
        self.backend.stop.assert_called_once()
        self.assertFalse(self.c.armed)

    def test_dispatch_revalidates_feedback_after_queueing(self):
        with self.c.dispatch_lock:
            self.c.arm()
            self.c.submit("force", force())
            self.c.latest.received -= 2
        wait_for(lambda: not self.c.busy and self.backend.stop.called)
        self.backend.start.assert_not_called()

    def test_watchdog_stops_even_if_reader_is_blocked(self):
        self.c.arm()
        watchdog = threading.Thread(target=self.c._watchdog_loop, daemon=True)
        watchdog.start()
        self.c.latest.received -= 2
        wait_for(lambda: self.backend.stop.called)
        self.assertFalse(self.c.armed)

    def test_stop_failure_prevents_rearming(self):
        self.backend.stop.side_effect = RuntimeError("network down")
        self.c.stop()
        wait_for(lambda: not self.c.stopping)
        with self.assertRaisesRegex(ValueError, "STOP failed"):
            self.c.arm()

    def test_backend_fault_cancels_and_disables_controls(self):
        self.backend.start.side_effect = RuntimeError("robot reflex")
        self.c.arm()
        self.c.submit("force", force())
        wait_for(lambda: self.backend.stop.called and not self.c.busy)
        self.assertFalse(self.c.armed)


class HardwareMappingTests(unittest.TestCase):
    def test_read_maps_one_state_rpc_and_uses_public_tool_offset_accessor(self):
        backend = FrankaBackend.__new__(FrankaBackend)
        s = sample()
        pose = Mock()
        pose.translation, pose.rotation = s.xyz, s.rotation
        raw_pose = Mock()
        raw_pose.__mul__ = Mock(return_value=pose)
        backend.fa = Mock(spec=["get_robot_state", "get_tool_base_pose"])
        backend.fa.get_robot_state.return_value = dict(
            pose=raw_pose, joints=s.joints, ee_force_torque=np.arange(6),
            joint_torques=np.arange(7), joint_torques_derivative=np.arange(7)*2)
        backend.cache_lock = threading.Lock()
        backend.ros_state = None
        backend.gripper_state = (time.monotonic(), .07)
        backend.last_grasp_command = 5.
        read = backend.read()
        backend.fa.get_robot_state.assert_called_once()
        backend.fa.get_tool_base_pose.assert_called_once()
        np.testing.assert_allclose(read.values["torque_base"], [3, 4, 5])
        np.testing.assert_allclose(read.values["gripper_width"], [70])
        self.assertTrue(np.all(np.isnan(read.values["joint_external"])))

    def test_force_dispatch_uses_nonblocking_api_and_thresholds(self):
        backend = FrankaBackend.__new__(FrankaBackend)
        backend.fa = Mock()
        _, args = Limits().validate("force", force(), sample())
        backend.start("force", args)
        call = backend.fa.apply_effector_forces_torques.call_args
        self.assertEqual(call.args, (1., .2, .005, .03))
        self.assertEqual(call.kwargs["forces"], [0., 0., -2.])
        self.assertFalse(call.kwargs["block"])
        self.assertFalse(call.kwargs["ignore_errors"])
        self.assertEqual(len(call.kwargs["force_thresholds"]), 6)

    def test_aborted_arm_action_is_an_error(self):
        backend = FrankaBackend.__new__(FrankaBackend)
        backend.fa = Mock()
        backend.fa.is_skill_done.return_value = True
        backend.fa._client.get_state.return_value = 4
        with self.assertRaisesRegex(RuntimeError, "status 4"):
            backend.done("pose")


if __name__ == "__main__":
    unittest.main()
