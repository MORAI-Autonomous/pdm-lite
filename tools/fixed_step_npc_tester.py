# TESTER_NPC_FIXED_STEP_BEGIN
# Temporary TCP-only tester for NPC deterministic validation.
# Remove this file after validation is complete.
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from pdm_morai.morai_tcp import (
    SCENARIO_PLAY,
    SCENARIO_STOP,
    TIME_MODE_FIXED,
    MoraiTcpClient,
    MoraiTcpError,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run XOSC NPC entities with TCP FixedStep only; no Ego UDP control."
    )
    parser.add_argument("--suite", required=True)
    parser.add_argument("--scenario-name", default=None)
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=20000)
    parser.add_argument("--tcp-timeout", type=float, default=30.0)
    parser.add_argument("--steps", type=int, default=1500)
    parser.add_argument("--target-fps", type=int, default=50)
    parser.add_argument("--physics-delta-time", type=int, default=10)
    parser.add_argument("--rtf", type=int, default=1)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--report-every", type=int, default=100)
    return parser


def require_success(action: str, result: dict[str, int]) -> None:
    if result.get("result_code", 0) != 0:
        raise MoraiTcpError(f"{action} failed: {result}")


def run(args: argparse.Namespace) -> int:
    if args.steps < 1:
        print("ERROR: --steps must be greater than 0.")
        return 2

    suite = Path(args.suite).expanduser().resolve(strict=False)
    scenario_name = args.scenario_name or suite.stem
    tcp = MoraiTcpClient(args.tcp_host, args.tcp_port, timeout=args.tcp_timeout)
    try:
        tcp.open()
        print(
            "TESTER_NPC_FIXED_STEP active: "
            f"suite={suite} scenario={scenario_name} steps={args.steps} "
            f"target_fps={args.target_fps} physics_delta_ms={args.physics_delta_time} "
            "ego_udp_control=disabled"
        )

        stop_result = tcp.scenario_control(SCENARIO_STOP, "")
        require_success("Scenario Stop", stop_result)

        load_result = tcp.load_suite(suite)
        require_success("LoadSuite", load_result)

        time_result = tcp.set_simulation_time_mode(
            TIME_MODE_FIXED,
            args.target_fps,
            args.physics_delta_time,
            args.rtf,
            1,
        )
        require_success("SetSimulationTimeMode", time_result)

        play_result = tcp.scenario_control(SCENARIO_PLAY, scenario_name)
        require_success("Scenario Play", play_result)
        if args.settle > 0:
            time.sleep(args.settle)

        for step_index in range(1, args.steps + 1):
            require_success("FixedStep", tcp.fixed_step(1))
            require_success("SaveData", tcp.save_data())
            if (
                step_index == 1
                or step_index == args.steps
                or step_index % max(1, args.report_every) == 0
            ):
                print(f"npc_fixed_step={step_index}/{args.steps}")

        status = tcp.get_simulation_time_status()
        print(f"npc_fixed_step_complete status={status}")
        return 0
    except (OSError, MoraiTcpError) as exc:
        print(f"ERROR: NPC fixed-step tester failed: {exc}")
        return 2
    finally:
        try:
            tcp.scenario_control(SCENARIO_STOP, "")
        except (OSError, MoraiTcpError):
            pass
        tcp.close()


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
# TESTER_NPC_FIXED_STEP_END
