"""Hardware-free regression tests; no simulator mode is exposed by the app."""
import contextlib
import io
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

import config as C
import main
from planner import Cup, InfeasibleError, build_slots, plan
from stack_pickup import StackPicker, StackPickupError, shimmy_waypoints, validate_stack_config


def settings(**changes):
    data = {key: value for key, value in vars(C).items() if key.isupper()}
    data.update(STACK_SEARCH_TOP_Z=.25, STACK_SEARCH_BOTTOM_Z=.08,
                STACK_GRASP_OFFSET_Z=-.012, STACK_XY=[.5, -.2])
    data.update(changes)
    return SimpleNamespace(**data)


class Clock:
    def __init__(self):
        self.now = 0.

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Pose:
    def __init__(self, xyz):
        self.translation = np.array(xyz)

    def __mul__(self, offset):
        return self


class HardwareDouble:
    """Minimal state/action contract for testing force-triggered contact logic."""
    def __init__(self, clock, top=.17, weight=.8):
        self.clock, self.top, self.weight = clock, top, weight
        self.xyz = np.array([.4, 0., .4])
        self.motion = None
        self.grasped = False
        self.stop_count = 0
        self.grasps = 0
        self.extra_force = None
        self.read_count = 0
        self.moves = []
        self._client = SimpleNamespace(get_state=lambda: 3)
        self._gripper_grasp_client = SimpleNamespace(
            get_result=lambda: SimpleNamespace(success=True))

    def goto(self, xy, z, duration, rotation=None, **kwargs):
        self.moves.append((tuple(xy), z, duration, kwargs))
        self.motion = (self.clock.now, self.xyz.copy(), np.r_[xy, z], duration,
                       "cartesian_impedances" in kwargs)

    def _state(self):
        force = np.zeros(6)
        if self.motion:
            start, initial, target, duration, probing = self.motion
            progress = min(1., (self.clock.now-start)/duration)
            self.xyz = initial + progress*(target-initial)
            if probing and self.top is not None and self.xyz[2] < self.top:
                force[2] = 150*(self.top-self.xyz[2])
                self.xyz[2] = self.top
        if self.grasped:
            force[2] -= self.weight
        self.read_count += 1
        if self.extra_force:
            force += self.extra_force(self)
        return force

    def is_skill_done(self, **kwargs):
        self._state()
        if self.motion and self.clock.now-self.motion[0] >= self.motion[3]:
            self.motion = None
        return self.motion is None

    def stop_skill(self):
        self._state()
        self.motion = None
        self.stop_count += 1

    def get_robot_state(self):
        wrench = self._state()
        return dict(pose=Pose(self.xyz), ee_force_torque=wrench)

    def get_tool_base_pose(self):
        return None

    def goto_gripper(self, width, grasp=False, **kwargs):
        if grasp:
            self.grasped = True
            self.grasps += 1
        elif width >= .079:
            self.grasped = False


class StackTests(unittest.TestCase):
    def make_picker(self, cfg=None, top=.17, weight=.8):
        clock = Clock()
        hw = HardwareDouble(clock, top, weight)
        robot = SimpleNamespace(fa=hw, goto=hw.goto, travel_z=.35)
        return StackPicker(robot, cfg or settings(), clock), hw

    def test_missing_measurements_and_unsafe_settings_rejected(self):
        bad = [dict(STACK_SEARCH_TOP_Z=None), dict(STACK_SEARCH_BOTTOM_Z=.26),
               dict(STACK_GRASP_OFFSET_Z=.01), dict(STACK_SHIMMY_XY=.02),
               dict(STACK_SHIMMY_CYCLES=1.5), dict(STACK_PROBE_SPEED=.1),
               dict(STACK_SINGLE_WEIGHT=.1, STACK_WEIGHT_TOLERANCE=.06),
               dict(STACK_SINGLE_WEIGHT=.1), dict(STACK_XY=[.25, 0.])]
        for changes in bad:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_stack_config(settings(**changes))

    def test_waypoints_are_bounded_rise_gradually_and_return_to_centre(self):
        c = settings()
        points = list(shimmy_waypoints(c.STACK_XY, .2, c))
        self.assertEqual(len(points), c.STACK_SHIMMY_CYCLES*5)
        for xy, z, yaw in points:
            self.assertLessEqual(np.linalg.norm(np.array(xy)-c.STACK_XY), c.STACK_SHIMMY_XY+1e-12)
            self.assertLessEqual(abs(yaw), math.radians(c.STACK_SHIMMY_YAW_DEG))
        np.testing.assert_allclose(points[-1][0], c.STACK_XY)
        self.assertEqual(points[-1][2], 0.)
        self.assertAlmostEqual(points[-1][1], .2+c.STACK_SHIMMY_CYCLES*c.STACK_SHIMMY_RISE)
        self.assertTrue(all(a[1] < b[1] for a, b in zip(points, points[1:])))

    def test_contact_returns_measured_height_and_stops_descent(self):
        picker, hw = self.make_picker()
        with contextlib.redirect_stdout(io.StringIO()):
            top = picker.find_top()
        self.assertAlmostEqual(top, .17)
        self.assertEqual(hw.stop_count, 1)
        self.assertGreater(top, picker.c.STACK_SEARCH_BOTTOM_Z)
        self.assertEqual(hw.grasps, 0)

    def test_missing_stack_never_attempts_grasp(self):
        picker, hw = self.make_picker(top=None)
        with self.assertRaisesRegex(StackPickupError, "No confirmed contact"):
            picker.pick()
        self.assertEqual(hw.grasps, 0)
        self.assertAlmostEqual(hw.xyz[2], picker.clear_z)

    def test_negative_contact_force_does_not_count_as_top_contact(self):
        picker, hw = self.make_picker(top=None)
        hw.extra_force = lambda h: np.array([0, 0, -2., 0, 0, 0]) if h.motion and h.motion[4] else np.zeros(6)
        with self.assertRaisesRegex(StackPickupError, "No confirmed contact"):
            picker.find_top()
        self.assertEqual(hw.grasps, 0)

    def test_excess_force_stops_motion_without_grasping(self):
        picker, hw = self.make_picker(top=None)
        hw.extra_force = lambda h: np.array([0, 0, 7., 0, 0, 0]) if h.motion and h.motion[4] else np.zeros(6)
        with self.assertRaisesRegex(StackPickupError, "Excess force"):
            picker.find_top()
        self.assertEqual(hw.grasps, 0)
        self.assertGreater(hw.stop_count, 0)

    def test_noisy_stationary_baseline_aborts_before_search(self):
        picker, hw = self.make_picker()
        hw.extra_force = lambda h: np.array([0, 0, .4*(-1)**h.read_count, 0, 0, 0])
        with self.assertRaisesRegex(StackPickupError, "too noisy"):
            picker.find_top()
        self.assertFalse(any(move[3].get("cartesian_impedances") for move in hw.moves))

    def test_single_force_spikes_do_not_pass_contact_debounce(self):
        picker, hw = self.make_picker(top=None)
        hw.extra_force = lambda h: np.array([0, 0, 2., 0, 0, 0]) if (
            h.motion and h.motion[4] and h.read_count % 4 == 0) else np.zeros(6)
        with self.assertRaisesRegex(StackPickupError, "No confirmed contact"):
            picker.find_top()
        self.assertEqual(hw.grasps, 0)

    def test_failed_grasp_does_not_start_separation(self):
        picker, hw = self.make_picker()
        hw._gripper_grasp_client.get_result = lambda: SimpleNamespace(success=False, error="no object")
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(StackPickupError, "grasp failed"):
            picker.pick()
        self.assertEqual(hw.grasps, 1)
        self.assertFalse(any(move[2] == picker.c.STACK_SHIMMY_DURATION for move in hw.moves))

    def test_separation_uses_contact_height_and_reports_unverified_count(self):
        picker, hw = self.make_picker()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            record = picker.pick()
        self.assertAlmostEqual(record["contact_z"], .17)
        self.assertAlmostEqual(record["grasp_z"], .158)
        self.assertFalse(record["single_cup_band"])
        self.assertIn("UNVERIFIED", output.getvalue())
        self.assertTrue(hw.grasped)
        np.testing.assert_allclose(hw.xyz, [*picker.xy, picker.clear_z])

    def test_new_pick_probes_new_height_instead_of_subtracting_cup_height(self):
        picker, hw = self.make_picker()
        with contextlib.redirect_stdout(io.StringIO()):
            first = picker.pick()
            hw.grasped = False  # cup placed by the main program
            hw.top = .164      # nested pitch, NOT CUP_HEIGHT
            second = picker.pick()
        self.assertAlmostEqual(first["contact_z"]-second["contact_z"], .006)
        self.assertEqual(hw.grasps, 2)

    def test_calibrated_payload_band_accepts_one_cup(self):
        picker, hw = self.make_picker(settings(STACK_SINGLE_WEIGHT=.8, STACK_WEIGHT_TOLERANCE=.15))
        with contextlib.redirect_stdout(io.StringIO()):
            record = picker.pick()
        self.assertTrue(record["single_cup_band"])
        self.assertAlmostEqual(record["payload_n"], .8)

    def test_zero_or_two_cups_fail_payload_check_while_gripper_stays_closed(self):
        for weight in (0., 1.6):
            with self.subTest(weight=weight):
                picker, hw = self.make_picker(settings(STACK_SINGLE_WEIGHT=.8, STACK_WEIGHT_TOLERANCE=.15), weight=weight)
                with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(StackPickupError, "outside.*single-cup band"):
                    picker.pick()
                self.assertTrue(hw.grasped)

    def test_light_cup_is_not_verified_when_force_noise_hides_count(self):
        picker, hw = self.make_picker(settings(STACK_SINGLE_WEIGHT=.8, STACK_WEIGHT_TOLERANCE=.15))
        hw.extra_force = lambda h: np.array([0, 0, .1*(-1)**h.read_count, 0, 0, 0]) if h.grasped else np.zeros(6)
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(StackPickupError, "cannot resolve"):
            picker.pick()
        self.assertTrue(hw.grasped)


class StackPlanningTests(unittest.TestCase):
    def test_planner_preserves_physical_order(self):
        cups = [Cup(str(i), col, (.5, -.2)) for i, col in enumerate(["yellow"]*3+["orange"]*2+["red", "blue"])]
        slots = build_slots([["yellow"]*3, ["orange"]*2, ["red"]], (.5, .1), .085)
        moves, _ = plan(cups, slots, (.31, 0), source_order=True)
        self.assertEqual([cup.id for cup, _ in moves], [str(i) for i in range(6)])

    def test_buried_needed_cup_cannot_be_used_out_of_order(self):
        cups = [Cup(str(i), col, (.5, -.2)) for i, col in enumerate(["red", "yellow", "yellow"])]
        slots = build_slots([["yellow"]*2, ["red"]], (.5, .1), .085)
        plan(cups, slots, (.31, 0))  # the same inventory works when individually accessible
        with self.assertRaisesRegex(InfeasibleError, "top-to-bottom"):
            plan(cups, slots, (.31, 0), source_order=True)

    def test_stack_cli_dry_run_uses_fixed_xy_and_never_connects(self):
        cups = [Cup("y1", "yellow", (.7, .3), "up")]  # old table-layout metadata is irrelevant
        output = io.StringIO()
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch("sys.argv", ["main.py", "--from-stack", "--goal", "base yellow", "--dry-run"]))
            stack.enter_context(patch.object(main, "load_cups", return_value=cups))
            robot = stack.enter_context(patch.object(main, "Robot"))
            stack.enter_context(contextlib.redirect_stdout(output))
            main.main()
        robot.assert_not_called()
        self.assertIn("top to bottom", output.getvalue())
        self.assertNotIn("(0.7, 0.3)", output.getvalue())

    def test_stack_failure_never_places_opens_or_homes(self):
        args = SimpleNamespace(step=False, log="unused.csv", goal="base yellow", layer_by_layer=False)
        cup = Cup("y1", "yellow", (.5, -.2))
        slot = build_slots([["yellow"]], (.5, .1), .085)[0]
        robot = Mock()
        picker = Mock()
        picker.records = []
        picker.pick.side_effect = StackPickupError("multiple cups suspected")
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(main, "Robot", return_value=robot))
            stack.enter_context(patch.object(main, "StackPicker", return_value=picker))
            log = stack.enter_context(patch.object(main, "log_trial"))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            with self.assertRaisesRegex(StackPickupError, "multiple cups"):
                main.run_from_stack(args, [["yellow"]], [cup], [(cup, slot)], .1, 1.)
        robot.place.assert_not_called()
        robot.home.assert_not_called()
        robot.fa.goto_gripper.assert_not_called()
        self.assertEqual(log.call_args.args[1]["success"], "n")


if __name__ == "__main__":
    unittest.main()
