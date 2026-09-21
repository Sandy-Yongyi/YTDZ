import queue
import sys
import types
import unittest
from types import SimpleNamespace
from unittest.mock import patch


if "tomlkit" not in sys.modules:
    sys.modules["tomlkit"] = types.ModuleType("tomlkit")
if "open3d" not in sys.modules:
    sys.modules["open3d"] = types.ModuleType("open3d")

from control.PlcCommunicationProcess import PlcCommunicationProcess
from control.LidarAcquisitionProcess import LidarAcquisitionProcess
from model.formats.frame_by_frame.AxisFrameDataFormat import AxisData, AxisFrameData
from model.utils.FrameQueueManager import FrameQueueManager


class PlcReconnectFrameSyncTests(unittest.TestCase):
    def test_disconnect_immediately_pauses_motion_and_notifies_lidar(self):
        proc = PlcCommunicationProcess.__new__(PlcCommunicationProcess)
        proc.pulse_queue = queue.Queue()
        proc.plc_connected = True
        proc.chain_motion_status = "moving_forward"
        proc.raw_data_timeout_active = True
        proc.plc_reconnect_pending = False
        proc.plc_connection_generation = 0
        proc.pulse_history = [100, 110, 120, 130, 140]
        proc.plc_data = SimpleNamespace(ChainCountCM=120, ChainStatus="moving_forward")

        proc._mark_plc_disconnected()

        self.assertFalse(proc.plc_connected)
        self.assertEqual("stopped", proc.chain_motion_status)
        self.assertFalse(proc.raw_data_timeout_active)
        self.assertTrue(proc.plc_reconnect_pending)
        self.assertEqual([], proc.pulse_history)
        self.assertEqual("stopped", proc.plc_data.ChainStatus)
        self.assertEqual(
            {"pulse": -999, "fifo": 120, "status": "stopped", "plc_generation": 1},
            proc.pulse_queue.get_nowait(),
        )

    def test_lidar_reconnect_uses_fresh_fifo_as_new_send_baseline(self):
        proc = LidarAcquisitionProcess.__new__(LidarAcquisitionProcess)
        proc.pulse_queue = queue.Queue()
        proc.pulse_queue.put({"pulse": 2000, "fifo": 200, "status": "moving_forward"})
        proc.plc_disconnected = True
        proc.plc_reconnected = False
        proc.current_pulse = 1000
        proc.current_fifo = 100
        proc.current_status = "moving_forward"
        proc.strategy_name = "frame_by_frame"
        proc.cm_accum = {100: {"left": [object()]}}
        proc.last_sent_fifo = 100
        proc.max_fifo = 2000

        _, fifo, status = proc._update_pulse_data()

        self.assertEqual(200, fifo)
        self.assertEqual("moving_forward", status)
        self.assertTrue(proc.plc_reconnected)
        self.assertEqual({100: {"left": [proc.cm_accum[100]["left"][0]]}}, proc.cm_accum)
        self.assertEqual(200, proc.last_sent_fifo)

    def test_disconnect_sentinel_is_observed_before_queued_reconnect_sample(self):
        proc = LidarAcquisitionProcess.__new__(LidarAcquisitionProcess)
        proc.pulse_queue = queue.Queue()
        proc.pulse_queue.put({"pulse": -999, "fifo": 100, "status": "stopped", "plc_generation": 1})
        proc.pulse_queue.put({"pulse": 1030, "fifo": 103, "status": "moving_forward", "plc_generation": 1})
        proc.plc_disconnected = False
        proc.plc_reconnected = False
        proc.current_pulse = 1000
        proc.current_fifo = 100
        proc.current_status = "moving_forward"
        proc.strategy_name = "frame_by_frame"
        proc.cm_accum = {}
        proc.last_sent_fifo = 100
        proc.max_fifo = 1000

        _, first_fifo, first_status = proc._update_pulse_data()
        _, second_fifo, second_status = proc._update_pulse_data()

        self.assertEqual((100, "stopped"), (first_fifo, first_status))
        self.assertEqual((103, "moving_forward"), (second_fifo, second_status))
        self.assertFalse(proc.plc_disconnected)
        self.assertTrue(proc.plc_reconnected)
        self.assertEqual(103, proc.last_sent_fifo)
        self.assertEqual(1, proc.plc_connection_generation)

    def test_plc_reconnect_gap_advances_existing_frames_with_empty_frames(self):
        old_head = AxisFrameData(FrameData=[AxisData(H_Axis=999)])
        manager = FrameQueueManager.__new__(FrameQueueManager)
        manager.stack_size = 6
        manager.y_min = 0
        manager.y_max = 1
        manager.y_threshold = 1
        manager.x_min = 0
        manager.x_max = 1
        manager.x_threshold = 1
        manager.frame_stack = {"left": [old_head] + [manager.create_empty_frame_y() for _ in range(5)]}

        proc = PlcCommunicationProcess.__new__(PlcCommunicationProcess)
        proc.strategy_name = "frame_by_frame"
        proc.frame_queue_manager = manager
        proc.max_fifo = 1000
        proc.last_synced_chain_fifo = 100
        proc.current_cycle_raw_shift_steps = 9
        proc.plc_reconnect_pending = True
        proc.plc_data = SimpleNamespace(ChainCountCM=103)

        proc._resync_frame_queue_after_plc_reconnect()

        frames = manager.frame_stack["left"]
        self.assertIs(old_head, frames[3])
        self.assertTrue(all(frame.FrameData[0].H_Axis == 0 for frame in frames[:3]))
        self.assertEqual(103, proc.last_synced_chain_fifo)
        self.assertEqual(103, proc.reconnect_raw_packet_floor_fifo)
        self.assertFalse(proc.plc_reconnect_pending)
        self.assertEqual(0, proc.current_cycle_raw_shift_steps)

    def test_stale_raw_packet_cannot_shift_frames_again_after_reconnect_resync(self):
        old_head = AxisFrameData(FrameData=[AxisData(H_Axis=111)])
        stale_frame = AxisFrameData(FrameData=[AxisData(H_Axis=999)])
        manager = FrameQueueManager.__new__(FrameQueueManager)
        manager.stack_size = 4
        manager.frame_stack = {"left": [old_head, stale_frame, stale_frame, stale_frame]}
        manager.active_directions = ("left",)

        proc = PlcCommunicationProcess.__new__(PlcCommunicationProcess)
        proc.raw_data_queue = queue.Queue()
        proc.raw_data_queue.put({"fifo": 103, "repeat_count": 62, "left": stale_frame})
        proc.frame_queue_manager = manager
        proc.max_fifo = 1000
        proc.reconnect_raw_packet_floor_fifo = 103
        proc.current_cycle_raw_shift_steps = 0
        proc.lidar_status = 0

        proc._process_frame_data()

        self.assertIs(old_head, manager.frame_stack["left"][0])
        self.assertEqual(0, proc.current_cycle_raw_shift_steps)
        self.assertEqual(103, proc.reconnect_raw_packet_floor_fifo)

    def test_normal_fifo_jump_after_reconnect_still_repeats_current_frame(self):
        proc = LidarAcquisitionProcess.__new__(LidarAcquisitionProcess)
        proc.current_fifo = 203
        proc.last_sent_fifo = 200
        proc.max_fifo = 1000
        proc.cm_accum = {}
        proc.active_directions = []
        proc.lidar_status = 0
        proc.raw_data_queue = queue.Queue()
        proc.read_data_config = {"fifo_reverse_tolerance": 10}

        proc.send_data_to_queue({})

        packet = proc.raw_data_queue.get_nowait()
        self.assertEqual(203, packet["fifo"])
        self.assertEqual(3, packet["repeat_count"])


if __name__ == "__main__":
    unittest.main()
