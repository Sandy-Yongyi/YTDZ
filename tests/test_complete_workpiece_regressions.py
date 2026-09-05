import queue
import unittest

from control.PlcCommunicationProcess import PlcCommunicationProcess


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


if __name__ == "__main__":
    unittest.main()
