from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


COMMAND_COLUMNS = ("ThrottleInput (-)", "BrakeInput (-)", "SteerInput (-)")
POSITION_COLUMNS = ("PositionX (m)", "PositionY (m)")


def ego_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as file:
        return [row for row in csv.DictReader(file) if row.get("Entry") == "Ego"]


def compare(base: list[dict[str, str]], rows: list[dict[str, str]]) -> dict[str, object]:
    count = min(len(base), len(rows))
    command_mismatches = 0
    first_command_mismatch: str | None = None
    max_position_error = 0.0
    end_position_error = 0.0

    for index in range(count):
        command_diff = [
            abs(float(rows[index][column]) - float(base[index][column]))
            for column in COMMAND_COLUMNS
        ]
        if any(value > 5e-7 for value in command_diff):
            command_mismatches += 1
            if first_command_mismatch is None:
                first_command_mismatch = rows[index]["time (sec)"]

        dx = float(rows[index][POSITION_COLUMNS[0]]) - float(base[index][POSITION_COLUMNS[0]])
        dy = float(rows[index][POSITION_COLUMNS[1]]) - float(base[index][POSITION_COLUMNS[1]])
        end_position_error = math.hypot(dx, dy)
        max_position_error = max(max_position_error, end_position_error)

    return {
        "rows": len(rows),
        "command_mismatches": command_mismatches,
        "first_command_mismatch": first_command_mismatch or "-",
        "max_position_error_m": max_position_error,
        "end_position_error_m": end_position_error,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compare Ego rows across MORAI statelog trials.")
    parser.add_argument("statelogs", nargs="+")
    args = parser.parse_args(argv)

    paths = [Path(value).expanduser().resolve(strict=False) for value in args.statelogs]
    if len(paths) < 2:
        print("ERROR: at least two statelogs are required.")
        return 2

    base = ego_rows(paths[0])
    failed = False
    print(f"statelog_compare_base={paths[0]} rows={len(base)}")
    for path in paths[1:]:
        result = compare(base, ego_rows(path))
        print(f"statelog_compare path={path} result={result}")
        failed = failed or result["command_mismatches"] != 0
        failed = failed or result["max_position_error_m"] > 1e-6
        failed = failed or result["rows"] != len(base)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
