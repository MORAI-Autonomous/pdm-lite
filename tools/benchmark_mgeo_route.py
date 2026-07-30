# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from tools.pdm_morai.mgeo_route import MGeoRoutePlanner


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Measure MGeo load and global-route generation time."
    )
    parser.add_argument(
        "--mgeo-dir",
        action="append",
        required=True,
        help="MGeo directory to benchmark; repeat to compare paths",
    )
    parser.add_argument("--repeats", type=int, default=7)
    parser.add_argument("--route-length", type=float, default=350.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.repeats < 1:
        print("ERROR: --repeats must be at least 1")
        return 2

    paths = [Path(value).expanduser().resolve(strict=True) for value in args.mgeo_dir]
    results: dict[Path, list[tuple[float, float, int]]] = {
        path: [] for path in paths
    }
    for index in range(args.repeats * len(paths)):
        path = paths[index % len(paths)]
        started = time.perf_counter()
        planner = MGeoRoutePlanner(path, spacing=0.1)
        loaded = time.perf_counter()
        link = next(link for link in planner.links.values() if link.road_id)
        delta = link.points[-1, :2] - link.points[0, :2]
        heading = math.atan2(float(delta[1]), float(delta[0]))
        planner.rebuild_from_pose(
            float(link.points[0, 0]),
            float(link.points[0, 1]),
            heading,
            min_route_length=args.route_length,
        )
        finished = time.perf_counter()
        results[path].append(
            (
                (loaded - started) * 1000.0,
                (finished - loaded) * 1000.0,
                len(planner.route_points),
            )
        )

    for path in paths:
        samples = results[path]
        print(
            f"mgeo_dir={path} "
            f"samples={len(samples)} "
            f"load_ms_median={statistics.median(item[0] for item in samples):.3f} "
            f"route_ms_median={statistics.median(item[1] for item in samples):.3f} "
            f"route_points={samples[-1][2]}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
