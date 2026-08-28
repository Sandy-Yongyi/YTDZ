import queue
import unittest
from types import SimpleNamespace

import numpy as np

from control.LidarAcquisitionStrategies import CompleteWorkpieceStrategy
from control.PlcCommunicationProcess import PlcCommunicationProcess
from model.utils.FrameQueueManager import FrameQueueManager


class PulseRolloverTests(unittest.TestCase):
    @staticmethod
    def _build_process():
        proc = object.__new__(PlcCommunicationProcess)
        proc.pulse_history = []
        proc.diff_start_pulse = 1
        proc.max_pulse = 160000
        proc.chain_motion_status = "stopped"
        proc.pulse_queue = queue.Queue()
        proc.plc_data = None
        proc._reset_raw_data_timeout_timer = lambda fifo=None: None
        return proc

    def test_forward_rollover_is_not_reported_as_reverse(self):
        proc = self._build_process()

        for pulse in (159990, 159995, 159999, 2, 6):
            proc._update_chain_status(0, pulse)

        self.assertEqual("moving_forward", proc.chain_motion_status)

    def test_reverse_rollover_is_reported_as_reverse(self):
        proc = self._build_process()

        for pulse in (8, 4, 0, 159997, 159993):
            proc._update_chain_status(0, pulse)

        self.assertEqual("moving_reverse", proc.chain_motion_status)


class FullWorkpieceQueueTests(unittest.TestCase):
    @staticmethod
    def _workpiece(name):
        return {"stop_pulse": 100, "data": SimpleNamespace(name=name, is_empty=False)}

    def test_full_queue_shift_keeps_position_tracking_with_remaining_workpiece(self):
        manager = FrameQueueManager(
            system_config_path="",
            stack_size=2,
            strategy_name="complete_workpiece",
            machine_config={"0": {"install_orietation": "right"}},
        )
        manager.push_workpiece("right", 0, self._workpiece("first"))
        manager.push_workpiece("right", 0, self._workpiece("second"))

        proc = object.__new__(PlcCommunicationProcess)
        proc.frame_queue_manager = manager
        proc.machine_data_queue = queue.Queue()
        proc.raw_data_queue = queue.Queue()
        proc.machine_data_queue.put({"lidar_status": 0, "right": {"stop_pulse": 300, "data": self._workpiece("new")["data"]}})
        proc.lidar_status = 0
        proc.machine_config = {}
        proc.gun_distributor = None
        proc.last_workpiece_chain_mm = {"right": {0: {0: 10.0, 1: 20.0}}}
        proc.last_workpiece_chain_mm_residual = {"right": {0: {0: 1.0, 1: 2.0}}}

        proc._process_workpiece_data()

        self.assertEqual({0: 20.0}, proc.last_workpiece_chain_mm["right"][0])
        self.assertEqual({0: 2.0}, proc.last_workpiece_chain_mm_residual["right"][0])


class CompleteWorkpieceStopPulseTests(unittest.TestCase):
    @staticmethod
    def _build_process(translate_data_origin, left_stop_pulse, right_stop_pulse):
        return SimpleNamespace(
            read_data_config={"translate_data_origin": translate_data_origin},
            lidar_status=1,
            lidar_config={"left": ["1"], "right": ["4"]},
            direction_states={
                "left": SimpleNamespace(stop_pulse=left_stop_pulse, xyz_data=np.empty((0, 3))),
                "right": SimpleNamespace(stop_pulse=right_stop_pulse, xyz_data=np.array([[1.0, 2.0, 3.0]])),
            },
            all_xyz_data=np.array([[1.0, 2.0, 3.0]]),
            raw_data_queue=queue.Queue(),
        )

    def test_same_origin_missing_direction_uses_all_data_stop_pulse(self):
        proc = self._build_process(translate_data_origin=1, left_stop_pulse=None, right_stop_pulse=12345)

        CompleteWorkpieceStrategy({}).send_raw_data(proc)
        raw_data = proc.raw_data_queue.get_nowait()

        self.assertEqual(12345, raw_data["left_stop_pulse"])
        self.assertEqual(12345, raw_data["all_stop_pulse"])

    def test_independent_origin_missing_direction_uses_common_stop_pulse(self):
        proc = self._build_process(translate_data_origin=2, left_stop_pulse=None, right_stop_pulse=12345)

        CompleteWorkpieceStrategy({}).send_raw_data(proc)
        raw_data = proc.raw_data_queue.get_nowait()

        self.assertEqual(12345, raw_data["left_stop_pulse"])

    def test_real_zero_stop_pulse_is_preserved(self):
        proc = self._build_process(translate_data_origin=1, left_stop_pulse=0, right_stop_pulse=12345)

        CompleteWorkpieceStrategy({}).send_raw_data(proc)
        raw_data = proc.raw_data_queue.get_nowait()

        self.assertEqual(0, raw_data["left_stop_pulse"])


if __name__ == "__main__":
    unittest.main()
