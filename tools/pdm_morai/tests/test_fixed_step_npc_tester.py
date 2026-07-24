from __future__ import annotations

import unittest
from unittest.mock import Mock, patch

from fixed_step_npc_tester import build_parser, run
from pdm_morai.morai_tcp import SCENARIO_PLAY, SCENARIO_STOP, TIME_MODE_FIXED


class FixedStepNpcTesterTest(unittest.TestCase):
    def test_rejects_non_positive_steps(self) -> None:
        args = build_parser().parse_args(["--suite", "case.msuite", "--steps", "0"])

        self.assertEqual(run(args), 2)

    @patch("fixed_step_npc_tester.MoraiTcpClient")
    def test_runs_tcp_only_fixed_steps(self, client_class: Mock) -> None:
        tcp = client_class.return_value
        tcp.scenario_control.return_value = {"result_code": 0}
        tcp.load_suite.return_value = {"result_code": 0}
        tcp.set_simulation_time_mode.return_value = {"result_code": 0}
        tcp.fixed_step.return_value = {"result_code": 0}
        tcp.save_data.return_value = {"result_code": 0}
        tcp.get_simulation_time_status.return_value = {"step_index": 3}
        args = build_parser().parse_args(
            [
                "--suite",
                "case.msuite",
                "--steps",
                "3",
                "--settle",
                "0",
            ]
        )

        result = run(args)

        self.assertEqual(result, 0)
        tcp.load_suite.assert_called_once()
        tcp.set_simulation_time_mode.assert_called_once_with(
            TIME_MODE_FIXED,
            50,
            10,
            1,
            1,
        )
        self.assertEqual(tcp.fixed_step.call_count, 3)
        self.assertEqual(tcp.save_data.call_count, 3)
        self.assertEqual(
            tcp.scenario_control.call_args_list[0].args,
            (SCENARIO_STOP, ""),
        )
        self.assertEqual(
            tcp.scenario_control.call_args_list[1].args,
            (SCENARIO_PLAY, "case"),
        )


if __name__ == "__main__":
    unittest.main()
