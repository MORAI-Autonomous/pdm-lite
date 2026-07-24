# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from tools.pdm_morai_control import RuntimeTraceLog, build_parser, build_runtime_trace_record


class RuntimeTraceLogTest(unittest.TestCase):
    def test_parser_accepts_trace_log(self) -> None:
        args = build_parser().parse_args(["--trace-log", "logs/trace.csv"])
        self.assertEqual(args.trace_log, "logs/trace.csv")

    def test_csv_record_matches_sim_statelog_fields(self) -> None:
        ego = SimpleNamespace(
            stamp_seconds=0,
            stamp_nanos=680_000_000,
            id="Ego",
            x=-2741.725126,
            y=1609.277937,
            z=28.357336,
            velocity_x=2.940184 / 3.6,
            velocity_y=-0.024575 / 3.6,
            velocity_z=0.512644 / 3.6,
            acceleration_x=4.037546,
            acceleration_y=-0.003481,
            acceleration_z=0.581151,
            roll_deg=0.044688,
            pitch_deg=1.166263,
            yaw_deg=-65.989612,
            throttle=0.864,
            brake=0.0,
            steer=-0.028697,
        )
        record = build_runtime_trace_record(ego=ego)

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "nested" / "trace.csv"
            trace = RuntimeTraceLog(str(path))
            trace.write(record)
            trace.close()
            with path.open("r", encoding="utf-8", newline="") as file:
                saved = next(csv.DictReader(file))

        self.assertEqual(saved["time (sec)"], "0.680000")
        self.assertEqual(saved["Entry"], "Ego")
        self.assertEqual(saved["PositionX (m)"], "-2741.725126")
        self.assertEqual(saved["VelocityX (km/h)"], "2.940184")
        self.assertEqual(saved["VelocityY (km/h)"], "-0.024575")
        self.assertEqual(saved["AccelerationX (m/s2)"], "4.037546")
        self.assertEqual(saved["RotationZ (deg)"], "-65.989612")
        self.assertEqual(saved["ThrottleInput (-)"], "0.864000")
        self.assertEqual(saved["SteerInput (-)"], "-0.028697")

    def test_open_failure_does_not_raise(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            blocker = Path(temp_dir) / "not-a-directory"
            blocker.write_text("blocked", encoding="utf-8")
            trace = RuntimeTraceLog(str(blocker / "trace.csv"))
            self.assertFalse(trace.enabled)


if __name__ == "__main__":
    unittest.main()
