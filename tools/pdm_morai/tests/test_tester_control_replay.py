from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
import struct

from pdm_lite.team_code.ego_manual_control import EgoManualControl
from tools.pdm_morai.morai_udp import pack_control
from tools.pdm_morai_control import build_parser, load_tester_controls


class TesterControlReplayTest(unittest.TestCase):
    def test_parser_accepts_temporary_tester_csv(self) -> None:
        args = build_parser().parse_args(
            [
                "--tester-control-csv",
                "logs/deterministic.csv",
                "--tester-udp-settle-ms",
                "20",
            ]
        )

        self.assertEqual(args.tester_control_csv, "logs/deterministic.csv")
        self.assertEqual(args.tester_udp_settle_ms, 20.0)

    def test_loads_only_ego_control_columns(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "controls.csv"
            path.write_text(
                "time (sec),Entry,ThrottleInput (-),BrakeInput (-),SteerInput (-)\n"
                "0.01,Ego,0.1,0.2,-0.3\n"
                "0.02,NPC_1,0.9,0.0,0.8\n"
                "0.03,Ego,0.4,0.0,0.5\n",
                encoding="utf-8",
            )

            controls = load_tester_controls(path, "Ego")

        self.assertEqual(len(controls), 2)
        self.assertEqual(controls[0].as_morai_udp_tuple(), (0.1, 0.2, -0.3))
        self.assertEqual(controls[1].as_morai_udp_tuple(), (0.4, 0.0, 0.5))

    def test_missing_control_column_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "controls.csv"
            path.write_text(
                "Entry,ThrottleInput (-),BrakeInput (-)\n"
                "Ego,0.1,0.0\n",
                encoding="utf-8",
            )

            with self.assertRaises(ValueError):
                load_tester_controls(path, "Ego")

    def test_udp_payload_preserves_throttle_brake_steer_order_and_values(self) -> None:
        packet = pack_control(
            EgoManualControl(throttle=0.125, brake=0.25, steer=-0.375)
        )

        self.assertEqual(struct.unpack("<ddd", packet), (0.125, 0.25, -0.375))


if __name__ == "__main__":
    unittest.main()
