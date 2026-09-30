import sys
import tomllib
import types
import unittest
from pathlib import Path
from types import SimpleNamespace


if "tomlkit" not in sys.modules:
    sys.modules["tomlkit"] = types.ModuleType("tomlkit")

from model.formats.frame_by_frame.AxisFrameDataFormat import AxisData as FrameRow, AxisFrameData
from model.motionplan.MotionXNUpdown2FramePlanning import MotionXNUpdown2FramePlanning
from model.plc.MovingFrameData import AxisData


class ConfigurableTopChainHoldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).parents[1] / "model" / "tomls" / "MachineConfig1.toml"
        with path.open("rb") as config_file:
            cls.config = tomllib.load(config_file)

    def setUp(self):
        self.planner = MotionXNUpdown2FramePlanning(
            process_cfg={"z_threshold": 10, "x_range": 300},
            spray_cfg={"spray_pos_tolerance": 20},
        )
        self.plc = SimpleNamespace(
            AxisList=[AxisData() for _ in range(12)], ChainStatus="moving_forward"
        )
        self.frames = SimpleNamespace(frame_stack={
            "left": [AxisFrameData(FrameData=[]) for _ in range(525)],
            "right": [AxisFrameData(FrameData=[]) for _ in range(525)],
        })

    def machine(self, sn):
        return dict(self.config[str(sn)])

    def points(self, side, *rows):
        index = 503 if side == "left" else 462
        self.frames.frame_stack[side][index] = AxisFrameData(FrameData=[
            FrameRow(H_Axis=y, V_Axis_Min=xmin, V_Axis_Max=xmax)
            for y, xmin, xmax in rows
        ])

    def feedback(self, side, *, x2=None, y2=None):
        x_index, y_index = (3, 2) if side == "left" else (9, 8)
        if x2 is not None:
            self.plc.AxisList[x_index].Pos = x2
        if y2 is not None:
            self.plc.AxisList[y_index].Pos = y2

    def step(self, machine):
        return self.planner.auto_xn_updown2_move(machine, machine, self.plc, self.frames)

    def start_right_spraying(self, machine):
        self.points("right", (1200, 1300, 2000), (2400, 1400, 1900))
        self.step(machine)
        self.feedback("right", x2=135, y2=530)
        self.step(machine)
        return self.step(machine)

    def test_right_config_uses_top_band_xmin_in_normal_positioning(self):
        machine = self.machine(2)
        self.assertTrue(machine["top_chain_hold_enabled"])
        self.points("right", (1200, 1300, 2000), (2400, 1400, 1900))

        commands = self.step(machine)

        self.assertEqual((135, 500, 0), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))
        self.assertEqual((530, 500, 0), (commands["y2"].Pos, commands["y2"].Speed, commands["y2"].Status))
        self.assertEqual(35, commands["x1"].Pos)

    def test_right_top_sprays_at_latest_xmin_without_reciprocating(self):
        machine = self.machine(2)

        commands = self.start_right_spraying(machine)
        self.assertEqual((135, 300, 1), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))

        self.feedback("right", x2=135)
        commands = self.step(machine)
        self.assertEqual((135, 300, 1), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))

        self.points("right", (1200, 1300, 2000), (2400, 1450, 1900))
        commands = self.step(machine)
        self.assertEqual((185, 300, 1), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))

        self.feedback("right", x2=185)
        commands = self.step(machine)
        self.assertEqual((185, 300, 1), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))

    def test_right_top_does_not_resume_reciprocating_after_chain(self):
        machine = self.machine(2)
        self.start_right_spraying(machine)
        self.feedback("right", x2=600, y2=525)
        self.points("right", (1200, 1300, 2000), (2400, 1400, 1900), (2950, 1500, 1800))
        self.step(machine)

        self.feedback("right", x2=135)
        self.points("right", (1200, 1300, 2000), (2400, 1400, 1900))
        commands = self.step(machine)

        self.assertEqual((135, 300, 1), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))

    def test_right_top_holds_feedback_y_and_retracts_to_band_xmin_with_powder(self):
        machine = self.machine(2)
        self.assertEqual(1, self.start_right_spraying(machine)["x2"].Status)
        self.feedback("right", x2=600, y2=525)
        self.points("right", (1200, 1300, 2000), (2400, 1400, 1900), (2950, 1500, 1800))

        commands = self.step(machine)

        self.assertEqual((135, 300, 1), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))
        self.assertEqual((525, 0, 0), (commands["y2"].Pos, commands["y2"].Speed, commands["y2"].Status))
        self.feedback("right", x2=135)
        commands = self.step(machine)
        self.assertEqual((135, 0, 1), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))
        self.plc.ChainStatus = "stopped"
        self.assertEqual(0, self.step(machine)["x2"].Status)

    def test_right_top_high_points_at_start_stay_safe_and_powder_off(self):
        machine = self.machine(2)
        self.points("right", (1200, 1300, 2000), (2950, 1500, 1800))

        commands = self.step(machine)

        self.assertEqual((0, 0, 0), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))
        self.assertEqual((0, 0, 0), (commands["y2"].Pos, commands["y2"].Speed, commands["y2"].Status))

    def test_high_point_immediately_after_positioning_cannot_start_powder(self):
        machine = self.machine(2)
        self.points("right", (1200, 1300, 2000), (2400, 1400, 1900))
        self.step(machine)
        self.feedback("right", x2=135, y2=530)
        positioning_commands = self.step(machine)
        self.assertEqual(0, positioning_commands["x2"].Status)
        self.points("right", (1200, 1300, 2000), (2950, 1500, 1800))

        commands = self.step(machine)

        self.assertEqual(0, commands["x2"].Status)
        self.assertEqual(0, commands["x2"].Pos)

    def test_retraction_cannot_start_powder_before_first_normal_spray(self):
        machine = self.machine(2)
        self.plc.ChainStatus = "stopped"
        self.points("right", (1200, 1300, 2000), (2400, 1400, 1900))
        self.step(machine)
        self.feedback("right", x2=135, y2=530)
        self.step(machine)
        self.step(machine)
        self.feedback("right", x2=600)
        self.plc.ChainStatus = "moving_forward"
        self.points("right", (1200, 1300, 2000), (2500, 1400, 1900))

        commands = self.step(machine)

        self.assertEqual(0, commands["x2"].Status)
        self.points("right", (1200, 1300, 2000), (2950, 1500, 1800))
        self.assertEqual(0, self.step(machine)["x2"].Status)

    def test_chain_exit_checks_actual_y_feedback_before_resuming_spray(self):
        machine = self.machine(2)
        self.start_right_spraying(machine)
        self.feedback("right", x2=600, y2=525)
        self.points("right", (1200, 1300, 2000), (2400, 1400, 1900), (2950, 1500, 1800))
        self.step(machine)
        self.feedback("right", x2=135, y2=480)
        self.points("right", (1200, 1300, 2000), (2400, 1400, 1900))

        commands = self.step(machine)

        self.assertEqual(0, commands["y2"].Status)
        self.assertEqual(0, commands["y2"].Speed)
        self.assertEqual(0, commands["x2"].Status)

    def test_only_points_above_top_band_return_top_safe(self):
        machine = self.machine(2)
        self.start_right_spraying(machine)
        self.feedback("right", x2=600, y2=530)
        self.points("right", (1200, 1300, 2000), (3100, 1500, 1800))

        commands = self.step(machine)

        self.assertEqual((0, 500, 0), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))
        self.assertEqual((530, 0, 0), (commands["y2"].Pos, commands["y2"].Speed, commands["y2"].Status))

    def test_flat_width_still_returns_both_groups_safe(self):
        machine = self.machine(2)
        self.start_right_spraying(machine)
        self.feedback("right", x2=600, y2=530)
        self.points("right", (1200, 1400, 1700), (2950, 1500, 1600))

        commands = self.step(machine)

        self.assertEqual((0, 500, 0), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))
        self.assertEqual(0, commands["x1"].Status)

    def test_left_config_can_enable_same_behavior_without_code_change(self):
        machine = self.machine(0)
        self.assertFalse(machine.get("top_chain_hold_enabled", False))
        machine["top_chain_hold_enabled"] = True
        self.points("left", (1200, 1300, 2000), (2400, 1400, 1900))
        self.step(machine)
        self.feedback("left", x2=135, y2=530)
        self.step(machine)
        self.step(machine)
        self.feedback("left", x2=600, y2=520)
        self.points("left", (1200, 1300, 2000), (2950, 1500, 1800))

        commands = self.step(machine)

        self.assertEqual((235, 300, 1), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))
        self.assertEqual((520, 0, 0), (commands["y2"].Pos, commands["y2"].Speed, commands["y2"].Status))

    def test_left_top_without_config_keeps_existing_overtravel_return(self):
        machine = self.machine(0)
        self.points("left", (1200, 1300, 2000), (2400, 1400, 1900))
        self.step(machine)
        self.feedback("left", x2=35, y2=530)
        self.step(machine)
        self.step(machine)
        self.feedback("left", x2=600)
        self.points("left", (1200, 1300, 2000), (2950, 1500, 1800))

        commands = self.step(machine)

        self.assertEqual((0, 500, 0), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))
        self.assertEqual((530, 0, 0), (commands["y2"].Pos, commands["y2"].Speed, commands["y2"].Status))

    def test_chain_exit_waits_for_latest_xmin_before_moving_y(self):
        machine = self.machine(2)
        self.start_right_spraying(machine)
        self.feedback("right", x2=600, y2=530)
        self.points("right", (1200, 1300, 2000), (2950, 1500, 1800))
        self.step(machine)
        self.points("right", (1200, 1300, 2000), (2500, 1400, 1900))

        commands = self.step(machine)
        self.assertEqual((135, 300, 1), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))
        self.assertEqual((530, 0, 0), (commands["y2"].Pos, commands["y2"].Speed, commands["y2"].Status))
        self.feedback("right", x2=135)
        self.step(machine)
        self.points("right", (1200, 1300, 2000), (2500, 1450, 1900))

        commands = self.step(machine)
        self.assertEqual((185, 500, 0), (commands["x2"].Pos, commands["x2"].Speed, commands["x2"].Status))
        self.assertEqual((530, 0, 0), (commands["y2"].Pos, commands["y2"].Speed, commands["y2"].Status))
        self.feedback("right", x2=185)
        commands = self.step(machine)
        self.assertEqual((630, 500, 0), (commands["y2"].Pos, commands["y2"].Speed, commands["y2"].Status))


if __name__ == "__main__":
    unittest.main()
