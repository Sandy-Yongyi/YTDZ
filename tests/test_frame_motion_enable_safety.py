import contextlib
import io
import sys
import tomllib
import types
import unittest
from pathlib import Path
from types import SimpleNamespace


if "tomlkit" not in sys.modules:
    sys.modules["tomlkit"] = types.ModuleType("tomlkit")

from model.motionplan.MotionFrameByFramePlanning import MotionFrameByFramePlanning
from model.plc.MovingFrameData import AxisData


class FrameMotionEnableSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config_path = Path(__file__).parents[1] / "model" / "tomls" / "MachineConfig1.toml"
        with config_path.open("rb") as config_file:
            cls.machine_config = tomllib.load(config_file)

    def setUp(self):
        self.planner = MotionFrameByFramePlanning()

    def _build_process(self, operate, *, lidar_status=0, raw_data_timeout=False, last_operate=0):
        positions = [306, -9, -132, -9, 777, 188, 58, 33, -53, -13, 888, 56]
        plc_data = SimpleNamespace(
            Operate=operate,
            Status=1,
            HeartBeat=99,
            AxisList=[AxisData(Pos=position, Speed=123, Status=1) for position in positions],
        )
        return SimpleNamespace(
            plc_data=plc_data,
            lidar_status=lidar_status,
            raw_data_timeout_active=raw_data_timeout,
            mode_config={"spray_mode": 0},
            machine_config=self.machine_config,
            runtime_machine_config={int(sn): dict(config) for sn, config in self.machine_config.items()},
            frame_queue_manager=SimpleNamespace(frame_stack={}),
            num_devices=4,
            device_origin_complete={sn: True for sn in range(4)},
            device_returning_to_origin={sn: False for sn in range(4)},
            last_operate_state=last_operate,
        )

    def test_master_disable_holds_axes_and_clears_all_enable_bits(self):
        proc = self._build_process(operate=0)

        frame = self.planner.build_moving_frame(proc)

        self.assertEqual(0, frame.Enable)
        self.assertEqual(2, frame.Operate)
        self.assertEqual(
            [306, -9, -132, -9, 0, 188, 58, 33, -53, -13, 0, 56],
            [axis.Pos for axis in frame.AxisList],
        )
        self.assertTrue(all(axis.Speed == 0 and axis.Status == 0 for axis in frame.AxisList))
        self.assertEqual({}, self.planner.xn_updown2_planner._states)

    def test_devices_that_were_never_enabled_hold_without_device_enable_bits(self):
        proc = self._build_process(operate=0x01)

        frame = self.planner.build_moving_frame(proc)

        self.assertEqual(0x01, frame.Enable)
        self.assertEqual(-9, frame.AxisList[1].Pos)
        self.assertEqual(0, frame.AxisList[4].Pos)
        self.assertEqual(188, frame.AxisList[5].Pos)
        self.assertTrue(all(axis.Speed == 0 and axis.Status == 0 for axis in frame.AxisList))

    def test_manual_mode_devices_that_were_never_enabled_hold_position(self):
        proc = self._build_process(operate=0x01)
        proc.mode_config["spray_mode"] = 1

        frame = self.planner.build_moving_frame(proc)

        self.assertEqual(0x01, frame.Enable)
        self.assertEqual(-9, frame.AxisList[1].Pos)
        self.assertEqual(0, frame.AxisList[4].Pos)
        self.assertEqual(188, frame.AxisList[5].Pos)
        self.assertTrue(all(axis.Speed == 0 and axis.Status == 0 for axis in frame.AxisList))

    def test_device_enabled_then_disabled_returns_to_safe_position(self):
        proc = self._build_process(operate=0x01, last_operate=0x03)
        proc.plc_data.AxisList[1].Pos = 100

        frame = self.planner.build_moving_frame(proc)

        self.assertEqual(0x03, frame.Enable)
        self.assertEqual(0, frame.AxisList[1].Pos)
        self.assertGreater(frame.AxisList[1].Speed, 0)
        self.assertTrue(proc.device_returning_to_origin[0])

    def test_safety_fault_prints_reason_and_forces_all_devices_to_safe_position(self):
        proc = self._build_process(operate=0x01, lidar_status=1)
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            frame = self.planner.build_moving_frame(proc)

        self.assertEqual(0x1E, frame.Enable)
        self.assertEqual(0, frame.Operate)
        self.assertIn("雷达异常", output.getvalue())
        self.assertIn("所有设备安全回零", output.getvalue())

    def test_sustained_safety_fault_prints_only_once(self):
        proc = self._build_process(operate=0x01, lidar_status=1)
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            self.planner.build_moving_frame(proc)
            self.planner.build_moving_frame(proc)

        self.assertEqual(1, output.getvalue().count("所有设备安全回零"))

    def test_master_disable_still_sends_hard_stop_when_one_device_config_is_invalid(self):
        proc = self._build_process(operate=0)
        proc.machine_config = {sn: dict(config) for sn, config in proc.machine_config.items()}
        proc.machine_config["0"]["install_orietation"] = "invalid"

        frame = self.planner.build_moving_frame(proc)

        self.assertEqual(0, frame.Enable)
        self.assertEqual([0, 0, 0, 0], [axis.Pos for axis in frame.AxisList[:4]])
        self.assertEqual(188, frame.AxisList[5].Pos)

    def test_master_disable_takes_priority_over_safety_fault(self):
        proc = self._build_process(operate=0, lidar_status=1)

        frame = self.planner.build_moving_frame(proc)

        self.assertEqual(0, frame.Enable)
        self.assertEqual(-9, frame.AxisList[1].Pos)
        self.assertEqual(0, frame.AxisList[4].Pos)
        self.assertEqual(188, frame.AxisList[5].Pos)
        self.assertEqual({}, self.planner.xn_updown2_planner._states)


if __name__ == "__main__":
    unittest.main()
