from __future__ import annotations

import argparse
import csv
from pathlib import Path


# TESTER_CONTROL_REPLAY_BEGIN
# Temporary validator for the deterministic UDP control replay tester.
# Remove with --tester-control-csv after UDP sender validation is complete.
CONTROL_COLUMNS = (
    "ThrottleInput (-)",
    "BrakeInput (-)",
    "SteerInput (-)",
)


def control_rows(path: Path, ego_id: str) -> list[tuple[float, float, float]]:
    with path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        missing = [
            column for column in CONTROL_COLUMNS if column not in (reader.fieldnames or [])
        ]
        if missing:
            raise ValueError(f"{path} is missing columns: {missing}")

        rows: list[tuple[float, float, float]] = []
        for row in reader:
            entry = str(row.get("Entry") or "").strip()
            if entry and entry != ego_id:
                continue
            rows.append(tuple(float(row[column]) for column in CONTROL_COLUMNS))
        return rows


def compare_at_offset(
    expected: list[tuple[float, float, float]],
    actual: list[tuple[float, float, float]],
    offset: int,
    tolerance: float,
) -> dict[str, object]:
    expected_start = max(0, -offset)
    actual_start = max(0, offset)
    count = min(len(expected) - expected_start, len(actual) - actual_start)
    mismatches = 0
    first_mismatch: int | None = None
    max_error = [0.0, 0.0, 0.0]

    for relative_index in range(max(0, count)):
        expected_index = expected_start + relative_index
        actual_index = actual_start + relative_index
        errors = [
            abs(actual[actual_index][column] - expected[expected_index][column])
            for column in range(3)
        ]
        max_error = [max(current, error) for current, error in zip(max_error, errors)]
        if any(error > tolerance for error in errors):
            mismatches += 1
            if first_mismatch is None:
                first_mismatch = relative_index

    return {
        "offset": offset,
        "overlap": max(0, count),
        "mismatches": mismatches,
        "first_mismatch": first_mismatch,
        "max_error": tuple(max_error),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Compare tester CSV throttle/brake/steer rows with MORAI Ego statelog "
            "and identify a fixed command-application step offset."
        )
    )
    parser.add_argument("tester_csv")
    parser.add_argument("statelog_csv")
    parser.add_argument("--ego-id", default="Ego")
    parser.add_argument("--max-offset", type=int, default=5)
    parser.add_argument("--tolerance", type=float, default=5e-7)
    args = parser.parse_args(argv)

    expected = control_rows(Path(args.tester_csv), args.ego_id)
    actual = control_rows(Path(args.statelog_csv), args.ego_id)
    if not expected or not actual:
        print(
            f"ERROR: empty control rows expected={len(expected)} actual={len(actual)}"
        )
        return 2

    results = [
        compare_at_offset(expected, actual, offset, args.tolerance)
        for offset in range(-args.max_offset, args.max_offset + 1)
    ]
    best = min(
        results,
        key=lambda result: (
            int(result["mismatches"]),
            -int(result["overlap"]),
            abs(int(result["offset"])),
        ),
    )
    max_error = best["max_error"]
    print(
        "tester_control_compare "
        f"expected_rows={len(expected)} actual_rows={len(actual)} "
        f"best_offset={best['offset']} overlap={best['overlap']} "
        f"mismatches={best['mismatches']} "
        f"first_mismatch={best['first_mismatch']} "
        f"max_error=(thr={max_error[0]:.9g},brk={max_error[1]:.9g},"
        f"steer={max_error[2]:.9g})"
    )

    if best["mismatches"] != 0:
        return 1
    if best["offset"] == 0:
        print("PASS: statelog command values match the UDP tester rows at the same index.")
    else:
        print(
            "PASS_WITH_OFFSET: values are unchanged, but SIM records them at "
            f"expected_index+{best['offset']}."
        )
    return 0
# TESTER_CONTROL_REPLAY_END


if __name__ == "__main__":
    raise SystemExit(main())
