# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
TOOLS_DIR = ROOT_DIR / "tools"
for path in (ROOT_DIR, TOOLS_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from pdm_lite.team_code.ego_manual_control import EgoManualControl
from pdm_morai.mgeo_route import MGeoRoutePlanner
from pdm_morai.morai_tcp import (
    SCENARIO_PAUSE,
    SCENARIO_PLAY,
    SCENARIO_STOP,
    TIME_MODE_FIXED,
    TIME_MODE_VARIABLE,
    MoraiTcpClient,
    MoraiTcpError,
)
from pdm_morai.morai_udp import MoraiUdpClient, pack_control
from pdm_morai.pdm_config import (
    PDM_PROFILE_CHOICES,
    PdmLiteMoraiConfig,
    apply_pdm_profile,
    pdm_profile_target_speed_scale,
)
from pdm_morai.pdm_controller import PdmLeadingVehicle, PdmMoraiController
from pdm_morai.scenario import DEFAULT_TARGET_SPEED_MPS, load_scenario


class ScenarioBoundaryDetected(RuntimeError):
    pass


class FixedAckUdpWaitTimeout(RuntimeError):
    pass


class ActiveScenarioChanged(RuntimeError):
    def __init__(
        self,
        previous_name: str,
        active_name: str,
        active_status: dict[str, Any],
        scenario_status: dict[str, Any],
    ):
        self.previous_name = previous_name
        self.active_name = active_name
        self.active_status = active_status
        self.scenario_status = scenario_status
        super().__init__(
            "SIM active scenario changed: "
            f"{previous_name!r} -> {active_name!r}; "
            f"scenario_status={scenario_status}"
        )


DEFAULT_ATTACH_RX_IP = "127.0.0.1"
DEFAULT_ATTACH_RX_PORT = 9097
DEFAULT_ATTACH_TX_IP = "127.0.0.1"
DEFAULT_ATTACH_TX_PORT = 9091
DEFAULT_ATTACH_CONTROL_HZ = 60
DEFAULT_ATTACH_EGO_CANDIDATES = ("Ego", "Car_1")
DEFAULT_ATTACH_MGEO_DIR = ROOT_DIR / "data" / "morai" / "mgeo" / "KATRI"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run PDM Lite controller output into MORAI Manual Control UDP."
    )
    parser.add_argument(
        "--run-preset",
        choices=["headless-tcp-lockstep-20ms", "headless-tcp-lockstep-10ms"],
        default=None,
        help=(
            "Apply a compact run preset. Keeps --suite, --tcp-port, --scenario-name, "
            "and --trace-log explicit."
        ),
    )
    parser.add_argument(
        "--suite",
        "--path",
        dest="source",
        default=None,
        help="Optional suite metadata source; required only for --load-suite or --headless",
    )
    parser.add_argument(
        "--execution-mode",
        choices=["gui", "headless"],
        default="gui",
        help=(
            "Who loads the suite. gui means the user opened it in MORAI; "
            "headless means a wrapper may load it explicitly."
        ),
    )
    parser.add_argument(
        "--control-mode",
        choices=[
            "variable-udp",
            "fixed-auto-udp",
            "fixed-udp",
            "fixed-tcp-lockstep",
        ],
        default="variable-udp",
        help=(
            "Runtime control behavior. GUI/headless are not control modes."
        ),
    )
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="Print execution/control plan and preflight status, then exit without control.",
    )
    parser.add_argument(
        "--check-suite",
        action="store_true",
        help="Require TCP active suite/scenario/time status preflight before control.",
    )
    parser.add_argument(
        "--check-udp",
        action="store_true",
        help="Require UDP Vehicle Info timestamp preflight before control.",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; 0 means until Ctrl+C")
    parser.add_argument(
        "--max-control-steps",
        type=int,
        default=0,
        help="Stop after this many timestamp-driven Ego control steps; 0 disables the limit.",
    )
    parser.add_argument("--target-speed", type=float, default=None, help="Override target speed in m/s")
    parser.add_argument(
        "--pdm-profile",
        choices=PDM_PROFILE_CHOICES,
        default="normal",
        help=(
            "PDM behavior preset. normal is the current baseline; mad-max is aggressive; "
            "beginner is slow/overcautious; idiot intentionally reacts late."
        ),
    )
    parser.add_argument("--ego-id", default=None, help="Override logical ego id")
    parser.add_argument("--entity-ids", default=None, help="Comma-separated logical ids for received Vehicle Info records")
    parser.add_argument("--rx-ip", default=None, help="Override Vehicle Info receive IP")
    parser.add_argument("--rx-port", type=int, default=None, help="Override Vehicle Info receive UDP port")
    parser.add_argument("--tx-ip", default=None, help="Override Manual Control target IP")
    parser.add_argument("--tx-port", type=int, default=None, help="Override Manual Control target UDP port")
    parser.add_argument("--no-send", action="store_true", help="Compute controls without sending UDP")
    parser.add_argument("--manual-test", action="store_true", help="Send fixed Manual Control UDP without waiting for Vehicle Info")
    parser.add_argument("--manual-throttle", type=float, default=0.15)
    parser.add_argument("--manual-brake", type=float, default=0.0)
    parser.add_argument("--manual-steer", type=float, default=0.0)
    # TESTER_CONTROL_REPLAY_BEGIN: remove with the temporary deterministic tester.
    parser.add_argument(
        "--tester-control-csv",
        default=None,
        help=(
            "TEMP TESTER: replay Ego throttle/brake/steer rows from a trace CSV; "
            "bypasses PDM planner/controller calculations"
        ),
    )
    parser.add_argument(
        "--tester-udp-settle-ms",
        type=float,
        default=0.0,
        help=(
            "TEMP TESTER: wait after each UDP control send before FixedStep "
            "to diagnose UDP receive/step ordering"
        ),
    )
    # TESTER_CONTROL_REPLAY_END
    parser.add_argument(
        "--hold-until-play",
        action="store_true",
        help="Do not send control until UDP simulation timestamp advances after the first ego state",
    )
    parser.add_argument("--dry-run", action="store_true", help="Load scenario and route only")
    parser.add_argument("--fixed-step", action="store_true", help="Drive ticks with TCP FixedStep(1); UDP remains the realtime control path")
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=0)
    parser.add_argument("--tcp-timeout", type=float, default=5.0)
    parser.add_argument("--tcp-settle", type=float, default=1.0, help="Seconds to wait after TCP setup before first FixedStep")
    parser.add_argument("--fixed-target-fps", type=int, default=50)
    parser.add_argument(
        "--fixed-control-transport",
        choices=["tcp", "udp"],
        default="tcp",
        help=(
            "Ego command transport in fixed-step mode. tcp waits for "
            "ManualControlById ACK before FixedStep; udp keeps legacy behavior."
        ),
    )
    parser.add_argument(
        "--lockstep-state-timeout",
        type=float,
        default=5.0,
        help="Fail if the next Ego timestamp does not arrive within this many seconds.",
    )
    parser.add_argument(
        "--pdm-step-delta-time",
        type=float,
        default=None,
        help=(
            "PDM/SUT external step delta in ms. When set, its reciprocal determines "
            "the fixed target FPS; SIM physics delta remains independent."
        ),
    )
    parser.add_argument(
        "--physics-delta-time",
        type=int,
        default=20,
        help="SIM internal fixed-step physics delta in ms; does not determine PDM dt",
    )
    parser.add_argument("--rtf", type=int, default=1)
    parser.add_argument("--user-control", type=int, default=1)
    parser.add_argument("--save-data-each-step", action="store_true", help="Call TCP SaveData after FixedStep to flush simulator-side frame output")
    parser.add_argument("--load-suite", action="store_true", help="Load the suite over TCP before running")
    parser.add_argument("--scenario-play", action="store_true", help="Send ScenarioControl Play over TCP before running")
    parser.add_argument("--no-scenario-play", dest="scenario_play", action="store_false", help="Do not send ScenarioControl Play")
    parser.add_argument("--scenario-stop", action="store_true", help="Send ScenarioControl Stop before load/play/run")
    parser.add_argument("--no-scenario-stop", dest="scenario_stop", action="store_false", help="Do not send ScenarioControl Stop")
    parser.add_argument("--scenario-stop-only", action="store_true", help="Send ScenarioControl Stop over TCP and exit")
    parser.add_argument(
        "--scenario-control-only",
        choices=["status", "play", "pause", "stop"],
        default=None,
        help="Run one TCP scenario menu command and exit",
    )
    parser.add_argument("--scenario-name", default=None, help="Scenario name used for TCP ScenarioControl Play")
    parser.add_argument(
        "--no-stop-before-start",
        action="store_true",
        help="Do not send TCP ScenarioControl Stop before load/play",
    )
    parser.add_argument(
        "--udp-ready-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for valid UDP Ego Vehicle Info before starting PDM control; 0 disables the preflight gate",
    )
    parser.add_argument(
        "--udp-ready-steps",
        type=int,
        default=0,
        help="Deprecated; UDP readiness now waits by --udp-ready-timeout instead of a fixed step cap",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Deprecated alias for --execution-mode headless; does not select a control mode",
    )
    parser.add_argument("--headless-timeout", type=float, default=30.0, help="Seconds to wait for headless suite activation")
    parser.add_argument(
        "--auto-next-scenario",
        action="store_true",
        help="Opt-in continuous validation: advance to the next suite scenario at a detected scenario boundary.",
    )
    parser.add_argument(
        "--exit-on-scenario-end",
        action="store_true",
        help="Treat stopped/completed scenario state as normal completion instead of a failure.",
    )
    parser.add_argument(
        "--scenario-transition-wait",
        type=float,
        default=3.0,
        help=(
            "Seconds to hold control/FixedStep while a SIM-owned scenario "
            "transition settles, or before runner-owned --auto-next-scenario play."
        ),
    )
    parser.add_argument("--status-hz", type=float, default=2.0)
    parser.add_argument(
        "--trace-log",
        default=None,
        help="Write SIM-statelog-compatible Ego runtime records as UTF-8 CSV (overwrites the file)",
    )
    parser.add_argument("--route-length", type=float, default=350.0)
    parser.add_argument("--route-points", type=int, default=360)
    parser.add_argument("--rebuild-distance", type=float, default=8.0)
    parser.add_argument("--route-max-error", type=float, default=6.0)
    parser.add_argument(
        "--route-from-runtime-pose",
        action="store_true",
        help=(
            "Build the MGeo route from the first runtime Ego VehicleInfo pose, "
            "even when a suite/xosc start pose is available."
        ),
    )
    parser.add_argument(
        "--mgeo-dir",
        default=str(DEFAULT_ATTACH_MGEO_DIR),
        help="MGeo fallback used to build a route from the first runtime Ego pose",
    )
    parser.add_argument(
        "--route-entity",
        default=None,
        help="Use Ego AdvancedRouting only when explicitly set to the ego id. NPC routes are not valid ego routes.",
    )
    return parser


def _set_if_default(args: argparse.Namespace, parser: argparse.ArgumentParser, name: str, value) -> None:
    if getattr(args, name) == parser.get_default(name):
        setattr(args, name, value)


def apply_run_preset(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if args.run_preset is None:
        return
    if args.run_preset == "headless-tcp-lockstep-20ms":
        step_ms = 20.0
    elif args.run_preset == "headless-tcp-lockstep-10ms":
        step_ms = 10.0
    else:
        raise ValueError(f"Unknown run preset: {args.run_preset}")

    args.execution_mode = "headless"
    args.control_mode = "fixed-tcp-lockstep"
    args.headless = True
    args.fixed_step = True
    args.fixed_control_transport = "tcp"
    args.load_suite = True
    args.scenario_play = True
    args.save_data_each_step = True
    _set_if_default(args, parser, "tcp_timeout", 30.0)
    _set_if_default(args, parser, "tcp_settle", 2.0)
    _set_if_default(args, parser, "pdm_step_delta_time", step_ms)
    _set_if_default(args, parser, "physics_delta_time", int(step_ms))
    _set_if_default(args, parser, "rtf", 1)
    _set_if_default(args, parser, "user_control", 1)
    _set_if_default(args, parser, "status_hz", 2.0)


def _option_present(argv: list[str] | None, *names: str) -> bool:
    values = sys.argv[1:] if argv is None else argv
    return any(item == name or item.startswith(f"{name}=") for item in values for name in names)


def normalize_runtime_modes(
    args: argparse.Namespace,
    *,
    control_mode_explicit: bool,
) -> None:
    if args.headless:
        args.execution_mode = "headless"

    if not control_mode_explicit and args.fixed_step:
        args.control_mode = (
            "fixed-tcp-lockstep"
            if args.fixed_control_transport == "tcp"
            else "fixed-udp"
        )

    if args.control_mode == "variable-udp":
        args.fixed_step = False
        args.fixed_control_transport = "udp"
    elif args.control_mode == "fixed-auto-udp":
        args.fixed_step = False
        args.fixed_control_transport = "udp"
        args.rtf = 1
        args.user_control = 0
    elif args.control_mode == "fixed-udp":
        args.fixed_step = True
        args.fixed_control_transport = "udp"
        # MORAI publishes the UDP Vehicle Info frame only after SaveData has
        # committed the preceding FixedStep output.
        args.save_data_each_step = True
    elif args.control_mode == "fixed-tcp-lockstep":
        args.fixed_step = True
        args.fixed_control_transport = "tcp"


def parse_runtime_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    apply_run_preset(args, parser)
    normalize_runtime_modes(
        args,
        control_mode_explicit=_option_present(argv, "--control-mode"),
    )
    return args


def uses_fixed_time_mode(args: argparse.Namespace) -> bool:
    return args.fixed_step or args.control_mode != "variable-udp"


@dataclass(frozen=True)
class RuntimeSettings:
    mode: str
    ego_id: str
    ego_candidates: tuple[str, ...]
    entity_ids: list[str]
    rx_ip: str
    rx_port: int
    tx_ip: str
    tx_port: int
    control_hz: int
    target_speed: float
    map_dir: Path
    actor_metadata: dict[str, Any]


def select_runtime_mode(args: argparse.Namespace) -> str:
    return f"{args.execution_mode}:{args.control_mode}"


def should_stop_before_start(args: argparse.Namespace) -> bool:
    return not args.no_stop_before_start and bool(args.scenario_stop)


def build_runtime_settings(args: argparse.Namespace, scenario) -> RuntimeSettings:
    mode = select_runtime_mode(args)
    explicit_entity_ids = parse_entity_ids(args.entity_ids)
    if scenario is not None:
        binding = scenario.binding
        ego_id = args.ego_id or binding.ego_id
        if args.ego_id:
            ego_candidates = (ego_id,)
        else:
            ego_candidates = tuple(
                dict.fromkeys((ego_id, *DEFAULT_ATTACH_EGO_CANDIDATES))
            )
        entity_ids = explicit_entity_ids or binding.entity_ids
        rx_ip = args.rx_ip or binding.ego_info_ip
        rx_port = args.rx_port or binding.ego_info_port
        tx_ip = args.tx_ip or binding.ctrl_ip
        tx_port = args.tx_port or binding.ctrl_port
        control_hz = binding.control_hz
        target_speed = args.target_speed if args.target_speed is not None else scenario.target_speed
        map_dir = scenario.map_dir
        actor_metadata = scenario.actor_metadata
    else:
        ego_id = args.ego_id or DEFAULT_ATTACH_EGO_CANDIDATES[0]
        ego_candidates = (ego_id,) if args.ego_id else DEFAULT_ATTACH_EGO_CANDIDATES
        entity_ids = explicit_entity_ids or []
        rx_ip = args.rx_ip or DEFAULT_ATTACH_RX_IP
        rx_port = args.rx_port or DEFAULT_ATTACH_RX_PORT
        tx_ip = args.tx_ip or DEFAULT_ATTACH_TX_IP
        tx_port = args.tx_port or DEFAULT_ATTACH_TX_PORT
        control_hz = DEFAULT_ATTACH_CONTROL_HZ
        target_speed = (
            args.target_speed
            if args.target_speed is not None
            else DEFAULT_TARGET_SPEED_MPS
        )
        map_dir = Path(args.mgeo_dir).expanduser().resolve(strict=False)
        actor_metadata = {}

    return RuntimeSettings(
        mode=mode,
        ego_id=ego_id,
        ego_candidates=tuple(ego_candidates),
        entity_ids=list(entity_ids),
        rx_ip=rx_ip,
        rx_port=int(rx_port),
        tx_ip=tx_ip,
        tx_port=int(tx_port),
        control_hz=int(control_hz),
        target_speed=float(target_speed),
        map_dir=Path(map_dir),
        actor_metadata=actor_metadata,
    )


class RuntimeTraceLog:
    FIELDNAMES = [
        "time (sec)",
        "Entry",
        "PositionX (m)",
        "PositionY (m)",
        "PositionZ (m)",
        "VelocityX (km/h)",
        "VelocityY (km/h)",
        "VelocityZ (km/h)",
        "AccelerationX (m/s2)",
        "AccelerationY (m/s2)",
        "AccelerationZ (m/s2)",
        "RotationX (deg)",
        "RotationY (deg)",
        "RotationZ (deg)",
        "ThrottleInput (-)",
        "BrakeInput (-)",
        "SteerInput (-)",
    ]

    def __init__(self, path: str | None):
        self.path: Path | None = None
        self._file = None
        self._writer: csv.DictWriter | None = None
        if path is None:
            return
        try:
            self.path = Path(path).expanduser().resolve(strict=False)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self.path.open("w", encoding="utf-8", newline="\n")
            self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
            self._writer.writeheader()
            self._file.flush()
            print(f"trace_log: {self.path} (CSV, overwrite)")
        except (OSError, RuntimeError) as exc:
            print(f"WARNING: trace logging disabled: {exc}")

    @property
    def enabled(self) -> bool:
        return self._file is not None

    def write(self, record: dict[str, str]) -> None:
        if self._file is None or self._writer is None:
            return
        try:
            self._writer.writerow(record)
            self._file.flush()
        except (OSError, csv.Error, ValueError) as exc:
            print(f"WARNING: trace logging disabled after write failure: {exc}")
            self.close()

    def close(self) -> None:
        if self._file is None:
            return
        try:
            self._file.close()
        except OSError:
            pass
        self._file = None
        self._writer = None


def build_runtime_trace_record(
    *,
    ego,
) -> dict[str, str]:
    sim_time = float(ego.stamp_seconds) + float(ego.stamp_nanos) / 1_000_000_000.0
    return {
        "time (sec)": f"{sim_time:.6f}",
        "Entry": ego.id,
        "PositionX (m)": f"{ego.x:.6f}",
        "PositionY (m)": f"{ego.y:.6f}",
        "PositionZ (m)": f"{ego.z:.6f}",
        "VelocityX (km/h)": f"{ego.velocity_x * 3.6:.6f}",
        "VelocityY (km/h)": f"{ego.velocity_y * 3.6:.6f}",
        "VelocityZ (km/h)": f"{ego.velocity_z * 3.6:.6f}",
        "AccelerationX (m/s2)": f"{ego.acceleration_x:.6f}",
        "AccelerationY (m/s2)": f"{ego.acceleration_y:.6f}",
        "AccelerationZ (m/s2)": f"{ego.acceleration_z:.6f}",
        "RotationX (deg)": f"{ego.roll_deg:.6f}",
        "RotationY (deg)": f"{ego.pitch_deg:.6f}",
        "RotationZ (deg)": f"{ego.yaw_deg:.6f}",
        "ThrottleInput (-)": f"{ego.throttle:.6f}",
        "BrakeInput (-)": f"{ego.brake:.6f}",
        "SteerInput (-)": f"{ego.steer:.6f}",
    }


# TESTER_CONTROL_REPLAY_BEGIN
# Temporary deterministic-input tester. Remove this block and
# --tester-control-csv after validation is complete.
TESTER_CONTROL_COLUMNS = (
    "ThrottleInput (-)",
    "BrakeInput (-)",
    "SteerInput (-)",
)


def load_tester_controls(path: str | Path, ego_id: str) -> list[EgoManualControl]:
    csv_path = Path(path).expanduser().resolve(strict=False)
    controls: list[EgoManualControl] = []
    with csv_path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        missing = [
            column
            for column in TESTER_CONTROL_COLUMNS
            if column not in (reader.fieldnames or [])
        ]
        if missing:
            raise ValueError(
                f"tester control CSV is missing columns {missing}: {csv_path}"
            )
        for line_number, row in enumerate(reader, start=2):
            entry = str(row.get("Entry") or "").strip()
            if entry and entry != ego_id:
                continue
            try:
                controls.append(
                    EgoManualControl(
                        throttle=float(row[TESTER_CONTROL_COLUMNS[0]]),
                        brake=float(row[TESTER_CONTROL_COLUMNS[1]]),
                        steer=float(row[TESTER_CONTROL_COLUMNS[2]]),
                    ).clamped()
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"invalid tester control at {csv_path}:{line_number}"
                ) from exc
    if not controls:
        raise ValueError(f"tester control CSV has no rows for Ego '{ego_id}': {csv_path}")
    return controls


def run_tester_control_replay(
    args: argparse.Namespace,
    binding,
    controls: list[EgoManualControl],
    tx_ip: str,
    tx_port: int,
    control_hz: int,
) -> int:
    tcp = MoraiTcpClient(args.tcp_host, args.tcp_port, timeout=args.tcp_timeout)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    report_every = max(1, control_hz)
    try:
        tcp.open()
        if should_stop_before_start(args):
            stop_before_start(tcp, binding, args.scenario_name)
        if args.load_suite:
            load_result = tcp.load_suite(binding.suite_path or binding.source_path)
            print(f"tester_load_suite={load_result}")
            if load_result.get("result_code") != 0:
                raise MoraiTcpError(f"Tester LoadSuite failed: {load_result}")
        configure_fixed_step(tcp, args, control_hz)
        if args.scenario_play:
            play_name = scenario_control_name(tcp, binding, args.scenario_name)
            print(
                f"tester_scenario_play name={play_name!r} "
                f"result={tcp.scenario_control(SCENARIO_PLAY, play_name)}"
            )
        if args.tcp_settle > 0:
            time.sleep(args.tcp_settle)

        print(
            "TESTER_CONTROL_REPLAY active: "
            f"rows={len(controls)} target={tx_ip}:{tx_port} fixed_step=1"
        )
        for index, control in enumerate(controls, start=1):
            sender.sendto(pack_control(control), (tx_ip, tx_port))
            if args.tester_udp_settle_ms > 0:
                time.sleep(args.tester_udp_settle_ms / 1000.0)
            fixed_result = tcp.fixed_step(1)
            save_result = tcp.save_data() if args.save_data_each_step else None
            if index == 1 or index == len(controls) or index % report_every == 0:
                print(
                    "tester_replay "
                    f"row={index}/{len(controls)} "
                    f"cmd=(thr={control.throttle:.6f}, brk={control.brake:.6f}, "
                    f"steer={control.steer:.6f}) "
                    f"fixed={fixed_result} save={save_result}"
                )
    except (OSError, MoraiTcpError) as exc:
        print(f"ERROR: tester replay failed: {exc}")
        return 2
    finally:
        try:
            sender.sendto(pack_control(EgoManualControl()), (tx_ip, tx_port))
        except OSError:
            pass
        sender.close()
        tcp.close()
    return 0
# TESTER_CONTROL_REPLAY_END


def parse_entity_ids(value: str | None) -> list[str] | None:
    if value is None:
        return None
    entity_ids = [item.strip() for item in value.split(",") if item.strip()]
    return entity_ids or None


def resolve_runtime_ego(
    udp: MoraiUdpClient,
    ego_candidates: tuple[str, ...],
):
    for candidate in ego_candidates:
        state = udp.states.get(candidate)
        if state is not None:
            return state
    return None


def run_manual_test(args, tx_ip: str, tx_port: int, control_hz: int) -> int:
    duration = args.duration if args.duration > 0 else 3.0
    period = 1.0 / max(1, control_hz)
    status_period = 1.0 / max(0.1, args.status_hz)
    end_time = time.monotonic() + duration
    next_status = 0.0
    control = EgoManualControl(
        steer=args.manual_steer,
        throttle=args.manual_throttle,
        brake=args.manual_brake,
    ).clamped()
    neutral = EgoManualControl()
    sent = 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        packet = pack_control(control)
        print(
            "manual_test start: "
            f"tx={tx_ip}:{tx_port}, duration={duration:.2f}s, hz={control_hz}, "
            f"cmd=(thr={control.throttle:.2f}, brk={control.brake:.2f}, steer={control.steer:.2f}), "
            f"packet_bytes={len(packet)}"
        )
        while time.monotonic() < end_time:
            loop_start = time.monotonic()
            sock.sendto(packet, (tx_ip, tx_port))
            sent += 1
            if loop_start >= next_status:
                print(f"manual_test tx={sent} cmd=(thr={control.throttle:.2f}, brk={control.brake:.2f}, steer={control.steer:.2f})")
                next_status = loop_start + status_period
            elapsed = time.monotonic() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        try:
            sock.sendto(pack_control(neutral), (tx_ip, tx_port))
        finally:
            sock.close()
    print(f"manual_test done: tx={sent}, neutral_sent=1")
    return 0


def print_dry_run(
    scenario,
    route: MGeoRoutePlanner,
    ego_id: str,
    entity_ids: list[str],
    rx_ip: str,
    rx_port: int,
    tx_ip: str,
    tx_port: int,
    route_source: str,
    route_max_error: float,
) -> None:
    binding = scenario.binding
    print(f"source       : {binding.source_path}")
    print(f"scenario     : {binding.scenario_path}")
    print(f"map_dir      : {scenario.map_dir}")
    print(f"ego_id       : {ego_id}")
    print(f"target_speed : {scenario.target_speed:.3f} m/s")
    print(f"rx           : {rx_ip}:{rx_port}")
    print(f"tx           : {tx_ip}:{tx_port}")
    print(f"runtime_ids  : {', '.join(entity_ids) if entity_ids else '-'}")
    if scenario.advanced_routes:
        print(f"advanced_routes: {', '.join(sorted(scenario.advanced_routes))}")
    else:
        print("advanced_routes: -")
    print(f"route_source : {route_source}")
    print(f"route_links  : {', '.join(route.current_link_ids[:8])}")
    print(f"route_points : {len(route.route_points)}")
    print(f"actor_meta   : {len(scenario.actor_metadata)} actors from xosc")
    speed_limits = [
        link.max_speed_kph
        for link in route.current_links
        if link.max_speed_kph is not None and link.max_speed_kph > 0
    ]
    if speed_limits:
        print(f"speed_limits : {min(speed_limits):.1f}..{max(speed_limits):.1f} km/h")
    if scenario.start_pose is not None:
        start = scenario.start_pose
        print(f"start_pose   : x={start.x:.3f}, y={start.y:.3f}, h={start.heading:.6f}")
        projection = route.project(start.x, start.y)
        if projection is not None:
            heading_error = math.degrees(
                (projection.heading - start.heading + math.pi) % (2.0 * math.pi) - math.pi
            )
            print(
                "start_route  : "
                f"distance={projection.distance:.3f}m, "
                f"lateral={projection.lateral_offset:.3f}m, "
                f"heading_error={heading_error:.1f}deg"
            )
            if projection.distance > route_max_error:
                print(
                    "route_warning: "
                    f"start pose is {projection.distance:.3f}m from the generated MGeo route "
                    f"(max={route_max_error:.3f}m). Check XOSC coordinates against the selected map."
                )


def estimate_leading_vehicle(
    ego,
    states,
    route: MGeoRoutePlanner,
    config: PdmLiteMoraiConfig,
    actor_lengths: dict[str, float],
) -> PdmLeadingVehicle | None:
    ego_projection = route.project(ego.x, ego.y)
    if ego_projection is None:
        return None
    ego_length = actor_lengths.get(ego.id, config.ego_length)
    best: PdmLeadingVehicle | None = None
    for actor in states.values():
        if actor.id == ego.id:
            continue
        actor_projection = route.project(actor.x, actor.y)
        if actor_projection is None:
            continue
        center_distance = actor_projection.s - ego_projection.s
        if center_distance <= 0.0 or center_distance > config.leading_vehicle_max_distance:
            continue
        lateral = actor_projection.lateral_offset - ego_projection.lateral_offset
        if abs(lateral) > config.leading_vehicle_lateral_window:
            continue
        heading_delta = abs((actor.heading_rad - actor_projection.heading + np.pi) % (2.0 * np.pi) - np.pi)
        if np.degrees(heading_delta) > config.leading_vehicle_heading_window_deg:
            continue
        actor_length = actor_lengths.get(actor.id, config.ego_length)
        bumper_gap = max(0.0, center_distance - 0.5 * (ego_length + actor_length))
        candidate = PdmLeadingVehicle(
            entity_id=actor.id,
            distance=float(bumper_gap),
            lateral_offset=float(lateral),
            speed=actor.speed,
            relative_speed=float(ego.speed - actor.speed),
            center_distance=float(center_distance),
        )
        if best is None or candidate.distance < best.distance:
            best = candidate
    return best


def actor_lengths_from_metadata(
    actor_metadata: dict[str, Any],
    fallback: float,
    ego_candidates: tuple[str, ...],
) -> dict[str, float]:
    lengths: dict[str, float] = {}
    for actor_id, metadata in actor_metadata.items():
        if metadata.dimensions is not None and metadata.dimensions[0] > 0:
            lengths[actor_id] = float(metadata.dimensions[0])
    for ego_id in ego_candidates:
        if ego_id not in lengths:
            lengths[ego_id] = fallback
    return lengths


def state_stamp(state) -> tuple[int, int]:
    return state.stamp_seconds, state.stamp_nanos


class FixedUdpDashboard:
    def __init__(self) -> None:
        self.line_count = 0
        self.cycle: int | str = "-"
        self.fixed = "-"
        self.save = "-"
        self.vehicle = "-"
        self.manual = "-"
        self.ego = "-"
        self.position = "-"
        self.yaw = "-"
        self.speed = "-"
        self.speed_cap = "-"
        self.control = "-"
        self.applied = "-"
        self.leading = "-"

    def _render(self) -> None:
        lines = [
            "[ fixed-udp lockstep ]",
            f"Cycle           : {self.cycle}",
            f"FixedStep       : {self.fixed}",
            f"SaveData        : {self.save}",
            f"VehicleInfo     : {self.vehicle}",
            f"ManualControl   : {self.manual}",
            "",
            f"Ego             : {self.ego}",
            f"Position        : {self.position}",
            f"Yaw             : {self.yaw}",
            f"Speed           : {self.speed}",
            f"Speed cap       : {self.speed_cap}",
            f"Control command : {self.control}",
            f"Applied control : {self.applied}",
            f"Leading vehicle : {self.leading}",
        ]
        if self.line_count:
            sys.stdout.write(f"\x1b[{self.line_count - 1}F")
        sys.stdout.write("\n".join(f"\x1b[2K\r{line}" for line in lines))
        sys.stdout.flush()
        self.line_count = len(lines)

    def begin(self, cycle: int | str) -> None:
        self.cycle = cycle
        self.fixed = "WAIT  TCP 0x1201"
        self.save = "-"
        self.vehicle = "-"
        self.manual = "-"
        self._render()

    def fixed_ok(self) -> None:
        self.fixed = "OK    TCP 0x1201"
        self.save = "WAIT  TCP 0x1202"
        self._render()

    def save_ok(self, called: bool) -> None:
        self.save = "OK    TCP 0x1202" if called else "SKIP  TCP 0x1202"
        self.vehicle = "WAIT  UDP"
        self._render()

    def vehicle_ok(self, ego, udp: MoraiUdpClient) -> None:
        stamp = state_stamp(ego)
        self.vehicle = (
            f"OK    UDP rx={udp.rx_records} stamp={format_stamp(stamp)}"
        )
        self.manual = "WAIT  UDP"
        self.ego = str(getattr(ego, "id", "-"))
        self.position = (
            f"({getattr(ego, 'x', 0.0):.2f}, {getattr(ego, 'y', 0.0):.2f})"
        )
        self.yaw = f"{getattr(ego, 'yaw_deg', 0.0):.1f} deg"
        self.speed = f"{getattr(ego, 'speed', 0.0) * 3.6:.1f} kph"
        self.applied = (
            f"T={getattr(ego, 'throttle', 0.0):.2f} "
            f"B={getattr(ego, 'brake', 0.0):.2f} "
            f"S={getattr(ego, 'steer', 0.0):.2f}"
        )
        self._render()

    def prepare_control(
        self,
        *,
        control: EgoManualControl,
        current_speed: float,
        target_speed: float,
        speed_cap: float | None,
        leading: str,
    ) -> None:
        self.speed = f"{current_speed * 3.6:.1f} -> {target_speed * 3.6:.1f} kph"
        self.speed_cap = f"{speed_cap * 3.6:.1f} kph" if speed_cap is not None else "-"
        self.control = (
            f"T={control.throttle:.2f} B={control.brake:.2f} "
            f"S={control.steer:.2f}"
        )
        self.leading = leading

    def manual_ok(
        self, control: EgoManualControl, udp: MoraiUdpClient
    ) -> None:
        self.manual = f"OK    UDP tx={udp.tx_packets}"
        if self.control == "-":
            self.control = (
                f"T={control.throttle:.2f} B={control.brake:.2f} "
                f"S={control.steer:.2f}"
            )
        self._render()

    def finish(self) -> None:
        if self.line_count:
            sys.stdout.write("\n")
            sys.stdout.flush()
            self.line_count = 0


def send_ego_control(
    *,
    udp: MoraiUdpClient,
    tcp: MoraiTcpClient | None,
    fixed_step: bool,
    fixed_control_transport: str,
    ego_id: str,
    control: EgoManualControl,
) -> dict[str, int] | None:
    if fixed_step and fixed_control_transport == "tcp":
        if tcp is None:
            raise MoraiTcpError("TCP control transport requires an active TCP connection")
        result = tcp.manual_control_by_id(ego_id, control)
        if result.get("result_code") != 0:
            raise MoraiTcpError(f"ManualControlById failed: {result}")
        return result
    udp.send_control(control)
    return None


def print_fixed_udp_control_event(
    *,
    tcp: MoraiTcpClient,
    udp: MoraiUdpClient,
    control: EgoManualControl,
) -> None:
    dashboard = getattr(tcp, "_fixed_udp_dashboard", None)
    if isinstance(dashboard, FixedUdpDashboard):
        dashboard.manual_ok(control, udp)
        return
    cycle_value = getattr(tcp, "fixed_step_request_count", "-")
    cycle = cycle_value if isinstance(cycle_value, int) else "-"
    print(
        f"lockstep_event cycle={cycle} "
        "transport=UDP api=ManualControl sent=yes "
        f"cmd=(thr={control.throttle:.2f}, brk={control.brake:.2f}, "
        f"steer={control.steer:.2f}) control_udp_tx={udp.tx_packets}"
    )


def is_new_state_sample(
    state,
    last_processed_stamp: tuple[int, int] | None,
) -> bool:
    return last_processed_stamp is None or state_stamp(state) > last_processed_stamp


def format_stamp(stamp: tuple[int, int] | None) -> str:
    if stamp is None:
        return "-"
    seconds, nanos = stamp
    return f"{seconds}.{nanos:09d}"


def scenario_control_name(tcp: MoraiTcpClient, binding, override: str | None) -> str:
    if override:
        return override
    if binding is not None and binding.scenario_path is not None:
        return binding.scenario_path.stem
    try:
        status = tcp.active_suite_status()
    except MoraiTcpError:
        status = {}
    active_name = str(status.get("active_scenario_name") or "").strip()
    if active_name:
        return active_name
    return ""


def stop_before_start(tcp: MoraiTcpClient, binding, override: str | None) -> bool:
    # The MORAI TCP helper only asks for scenario_name on Play. Stop should
    # target the currently active scenario unless the caller explicitly
    # overrides it; using the local suite/xosc stem here can miss the active
    # SIM scenario after a previous run.
    scenario_name = override or ""
    try:
        active_status = tcp.active_suite_status()
    except MoraiTcpError as exc:
        active_status = {"error": str(exc)}
    try:
        status = tcp.scenario_status()
    except MoraiTcpError as exc:
        status = {"error": str(exc)}
    try:
        result = tcp.scenario_control(SCENARIO_STOP, scenario_name)
        print(
            "scenario_stop_before_start "
            f"name={scenario_name!r} active_before={active_status} "
            f"status_before={status} result={result}"
        )
        return result.get("result_code") == 0
    except MoraiTcpError as exc:
        print(
            "scenario_stop_before_start warning: "
            f"name={scenario_name!r} active_before={active_status} "
            f"status_before={status} error={exc}"
        )
        return False


def run_scenario_control_once(
    tcp: MoraiTcpClient,
    binding,
    menu: str,
    override: str | None,
) -> bool:
    try:
        active_before = tcp.active_suite_status()
    except MoraiTcpError as exc:
        active_before = {"error": str(exc)}
    try:
        status_before = tcp.scenario_status()
    except MoraiTcpError as exc:
        status_before = {"error": str(exc)}
    try:
        time_before = tcp.get_simulation_time_status()
    except MoraiTcpError as exc:
        time_before = {"error": str(exc)}

    if menu == "status":
        print(f"scenario_status active={active_before} status={status_before} time={time_before}")
        return True

    command_by_menu = {
        "play": SCENARIO_PLAY,
        "pause": SCENARIO_PAUSE,
        "stop": SCENARIO_STOP,
    }
    command = command_by_menu[menu]
    if menu == "play":
        if override:
            scenario_name = override
        else:
            active_name = str(active_before.get("active_scenario_name") or "").strip()
            scenario_list = [str(item).strip() for item in active_before.get("scenario_list") or []]
            scenario_name = active_name or (scenario_list[0] if scenario_list else "")
    else:
        # MORAI's own CLI asks for scenario_name only on Play. Pause/Stop
        # should apply to the active scenario unless explicitly overridden.
        scenario_name = override or ""

    result = tcp.scenario_control(command, scenario_name)
    time.sleep(0.2)
    try:
        active_after = tcp.active_suite_status()
    except MoraiTcpError as exc:
        active_after = {"error": str(exc)}
    try:
        status_after = tcp.scenario_status()
    except MoraiTcpError as exc:
        status_after = {"error": str(exc)}
    try:
        time_after = tcp.get_simulation_time_status()
    except MoraiTcpError as exc:
        time_after = {"error": str(exc)}
    print(
        "scenario_control_once "
        f"menu={menu!r} command={command} name={scenario_name!r} "
        f"active_before={active_before} status_before={status_before} time_before={time_before} "
        f"result={result} active_after={active_after} status_after={status_after} time_after={time_after}"
    )
    return result.get("result_code") == 0


def expected_suite_name(binding) -> str:
    suite_path = binding.suite_path or binding.source_path
    if suite_path.suffix.lower() == ".msuite" and suite_path.exists():
        try:
            with suite_path.open("r", encoding="utf-8-sig") as file:
                suite = json.load(file)
            name = str(suite.get("name") or "").strip() if isinstance(suite, dict) else ""
            if name:
                return name
        except (OSError, json.JSONDecodeError):
            pass
    return suite_path.stem


def suite_status_matches(status: dict, expected_name: str) -> bool:
    active_suite_name = str(status.get("active_suite_name") or "").strip()
    if active_suite_name == expected_name:
        return True
    scenarios = [str(item).strip() for item in status.get("scenario_list") or []]
    return expected_name in scenarios


def wait_for_headless_suite(
    tcp: MoraiTcpClient,
    binding,
    timeout: float,
    status_period: float,
) -> dict:
    expected_name = expected_suite_name(binding)
    deadline = time.monotonic() + max(0.0, timeout)
    last_status: dict = {}
    while True:
        last_status = tcp.active_suite_status()
        if suite_status_matches(last_status, expected_name):
            return last_status
        if time.monotonic() >= deadline:
            raise MoraiTcpError(
                "headless suite did not become active: "
                f"expected={expected_name!r}, last_status={last_status}"
            )
        time.sleep(max(0.1, status_period))


def udp_status_text(
    udp: MoraiUdpClient,
    ego_candidates: tuple[str, ...],
) -> str:
    ids = ", ".join(sorted(udp.states)[:8]) if udp.states else "-"
    if len(udp.states) > 8:
        ids += ", ..."
    packet_sizes = getattr(udp, "packet_sizes", [])
    packet_text = ",".join(str(size) for size in packet_sizes[-4:]) if packet_sizes else "-"
    datagrams = getattr(udp, "rx_datagrams", 0)
    error_text = f" error={udp.last_error}" if udp.last_error else ""
    ego_text = "yes" if resolve_runtime_ego(udp, ego_candidates) is not None else "no"
    return (
        f"ego={ego_text} rx={udp.rx_records} chunks={udp.rx_chunks} "
        f"datagrams={datagrams} packet_sizes=[{packet_text}] "
        f"actors={len(udp.states)} ids=[{ids}] invalid={udp.invalid_packets}{error_text}"
    )


def scenario_state_allows_next(status: dict[str, Any]) -> bool:
    try:
        state = int(status.get("state", 0))
    except (TypeError, ValueError):
        state = 0
    return state in (0, 3, 4)


def scenario_state_is_running(status: dict[str, Any]) -> bool:
    try:
        return int(status.get("state", 0)) == 1
    except (TypeError, ValueError):
        return False


def next_scenario_from_status(status: dict[str, Any], current: str | None) -> str | None:
    scenarios = [str(item).strip() for item in status.get("scenario_list") or [] if str(item).strip()]
    if not scenarios:
        return None
    active_name = str(status.get("active_scenario_name") or "").strip()
    current_name = str(current or active_name or "").strip()
    if current_name in scenarios:
        index = scenarios.index(current_name)
        if index + 1 < len(scenarios):
            return scenarios[index + 1]
        return None
    return scenarios[0]


def read_tcp_preflight(tcp: MoraiTcpClient) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    return tcp.active_suite_status(), tcp.scenario_status(), tcp.get_simulation_time_status()


def print_preflight_status(
    *,
    args: argparse.Namespace,
    active_status: dict[str, Any],
    scenario_status: dict[str, Any],
    time_status: dict[str, Any],
    udp: MoraiUdpClient,
    ego_candidates: tuple[str, ...],
) -> None:
    print(
        "plan_check: "
        f"execution_mode={args.execution_mode} control_mode={args.control_mode} "
        f"auto_load={bool(args.load_suite)} auto_play={bool(args.scenario_play)} "
        f"auto_stop={bool(args.scenario_stop)}"
    )
    print(
        "plan_check_tcp: "
        f"active_suite={active_status.get('active_suite_name')!r} "
        f"active_scenario={active_status.get('active_scenario_name')!r} "
        f"scenario_list={active_status.get('scenario_list')} "
        f"scenario_state={scenario_status} "
        f"time_mode={time_status.get('mode')} "
        f"target_fps={time_status.get('target_fps')} "
        f"physics_delta_time={time_status.get('physics_delta_time')} "
        f"rtf={time_status.get('rtf')} "
        f"user_control={time_status.get('user_control')} "
        f"step_index={time_status.get('step_index')} "
        f"sim_time={time_status.get('seconds')}.{int(time_status.get('nanos') or 0):09d}"
    )
    print(
        "plan_check_udp: "
        f"bind={udp.ego_info_ip}:{udp.ego_info_port} "
        f"{udp_status_text(udp, ego_candidates)}"
    )


def verify_suite_preflight(
    *,
    args: argparse.Namespace,
    binding,
    active_status: dict[str, Any],
    scenario_status: dict[str, Any],
) -> bool:
    active_suite = str(active_status.get("active_suite_name") or "").strip()
    active_scenario = str(active_status.get("active_scenario_name") or "").strip()
    if not active_suite:
        print("ERROR: suite preflight failed: no active suite")
        return False
    if not active_scenario:
        print("ERROR: suite preflight failed: no active scenario")
        return False
    if binding is not None:
        expected = expected_suite_name(binding)
        if not suite_status_matches(active_status, expected):
            print(
                "ERROR: suite preflight failed: "
                f"expected={expected!r} active={active_status}"
            )
            return False
    if args.control_mode != "variable-udp" and scenario_state_allows_next(scenario_status):
        if args.scenario_play:
            print(
                "suite_preflight: scenario Play was acknowledged but state is still "
                f"stopped/completed; allowing FixedStep bootstrap status={scenario_status}"
            )
            return True
        print(
            "ERROR: suite preflight failed: scenario is stopped/completed "
            f"status={scenario_status}"
        )
        return False
    return True


def wait_for_udp_ready(
    udp: MoraiUdpClient,
    tcp: MoraiTcpClient | None,
    *,
    timeout: float,
    status_period: float,
    fixed_step: bool,
    save_data_each_step: bool,
    max_fixed_steps: int,
    require_stamp_advance: bool,
    ego_candidates: tuple[str, ...],
) -> bool:
    if timeout <= 0.0:
        print("udp_ready: disabled")
        return True

    deadline = time.monotonic() + timeout
    next_status = 0.0
    fixed_steps = 0
    first_stamp: tuple[int, int] | None = None
    last_fixed_result = None
    last_save_result = None

    while True:
        now = time.monotonic()
        udp.poll()
        ego = resolve_runtime_ego(udp, ego_candidates)
        if ego is not None:
            stamp = state_stamp(ego)
            if first_stamp is None:
                first_stamp = stamp
                if not require_stamp_advance:
                    print(
                        "udp_ready: "
                        f"stamp={format_stamp(stamp)} {udp_status_text(udp, ego_candidates)} "
                        f"warmup_steps={fixed_steps}"
                    )
                    return True
            elif stamp > first_stamp:
                print(
                    "udp_ready: "
                    f"stamp={format_stamp(first_stamp)}->{format_stamp(stamp)} "
                    f"{udp_status_text(udp, ego_candidates)} warmup_steps={fixed_steps}"
                )
                return True

        if now >= deadline:
            break
        if tcp is not None and fixed_step:
            last_fixed_result = tcp.fixed_step(1)
            fixed_steps += 1
            if save_data_each_step:
                last_save_result = tcp.save_data()

        if now >= next_status:
            fixed_text = (
                f" fixed={last_fixed_result} save={last_save_result} warmup_steps={fixed_steps}"
                if last_fixed_result is not None
                else f" warmup_steps={fixed_steps}"
            )
            stamp_text = f" first_stamp={format_stamp(first_stamp)}" if first_stamp else ""
            print(
                f"udp_ready_wait: "
                f"{udp_status_text(udp, ego_candidates)}{stamp_text}{fixed_text}"
            )
            next_status = now + status_period
        time.sleep(0.005 if tcp is not None else 0.02)

    reason = "timestamp did not advance" if first_stamp is not None and require_stamp_advance else "ego state not received"
    print(
        "ERROR: UDP Vehicle Info readiness failed: "
        f"{reason}; timeout={timeout:.1f}s, fixed_warmup_steps={fixed_steps}, "
        f"{udp_status_text(udp, ego_candidates)}. Check that SIM NetworkInterface "
        f"Vehicle Info SEND is active on {udp.ego_info_ip}:{udp.ego_info_port} "
        f"and includes one of {', '.join(ego_candidates)}."
    )
    return False


def fixed_step_and_wait_for_ego(
    *,
    tcp: MoraiTcpClient,
    udp: MoraiUdpClient,
    previous_stamp: tuple[int, int] | None,
    timeout: float,
    status_period: float,
    save_data_each_step: bool,
    ego_candidates: tuple[str, ...],
    expected_scenario_name: str | None = None,
) -> tuple[object, dict[str, int], dict[str, int] | None]:
    dashboard = getattr(tcp, "_fixed_udp_dashboard", None)
    if isinstance(dashboard, FixedUdpDashboard):
        next_cycle = getattr(tcp, "fixed_step_request_count", 0) + 1
        dashboard.begin(next_cycle)

    def recover_after_tcp_failure(operation: str, exc: Exception) -> None:
        if not expected_scenario_name:
            raise MoraiTcpError(f"{operation} failed: {exc}") from exc
        print(
            "scenario_transition_tcp_recovery: "
            f"operation={operation} expected={expected_scenario_name!r} error={exc}"
        )
        try:
            tcp.close()
            tcp.open()
            active_status = tcp.active_suite_status()
            scenario_status = tcp.scenario_status()
        except (OSError, MoraiTcpError) as recovery_exc:
            raise MoraiTcpError(
                f"{operation} failed and TCP transition recovery failed: {recovery_exc}"
            ) from recovery_exc
        if active_status.get("result_code") != 0:
            raise MoraiTcpError(
                f"{operation} failed; recovery ActiveSuiteStatus failed: {active_status}"
            ) from exc
        if scenario_status.get("result_code") != 0:
            raise MoraiTcpError(
                f"{operation} failed; recovery ScenarioStatus failed: {scenario_status}"
            ) from exc
        active_name = str(
            active_status.get("active_scenario_name") or ""
        ).strip()
        print(
            "scenario_transition_tcp_recovery_status: "
            f"operation={operation} active_scenario={active_name or '-'} "
            f"scenario_status={scenario_status}"
        )
        if active_name and active_name != expected_scenario_name:
            raise ActiveScenarioChanged(
                expected_scenario_name,
                active_name,
                active_status,
                scenario_status,
            ) from exc
        if scenario_state_allows_next(scenario_status):
            raise ScenarioBoundaryDetected(
                f"{operation} failed at scenario boundary: "
                f"expected={expected_scenario_name!r} status={scenario_status}"
            ) from exc
        raise MoraiTcpError(
            f"{operation} failed while active scenario remained "
            f"{expected_scenario_name!r}: {exc}"
        ) from exc

    try:
        fixed_result = tcp.fixed_step(1)
    except (OSError, MoraiTcpError) as exc:
        recover_after_tcp_failure("FixedStep(1)", exc)
        raise AssertionError("unreachable")
    if fixed_result.get("result_code") != 0:
        raise MoraiTcpError(f"FixedStep(1) failed: {fixed_result}")
    if isinstance(dashboard, FixedUdpDashboard):
        dashboard.fixed_ok()
    try:
        save_result = tcp.save_data() if save_data_each_step else None
    except (OSError, MoraiTcpError) as exc:
        recover_after_tcp_failure("SaveData", exc)
        raise AssertionError("unreachable")
    if isinstance(dashboard, FixedUdpDashboard):
        dashboard.save_ok(save_result is not None)
    def check_scenario_progress(phase: str) -> None:
        try:
            active_status = tcp.active_suite_status() if expected_scenario_name else None
        except (OSError, MoraiTcpError) as exc:
            recover_after_tcp_failure("ActiveSuiteStatus", exc)
            raise AssertionError("unreachable")
        if active_status is not None and active_status.get("result_code") != 0:
            raise MoraiTcpError(f"ActiveSuiteStatus failed: {active_status}")
        try:
            scenario_status = tcp.scenario_status()
        except (OSError, MoraiTcpError) as exc:
            recover_after_tcp_failure("ScenarioStatus", exc)
            raise AssertionError("unreachable")
        if scenario_status.get("result_code") != 0:
            raise MoraiTcpError(f"ScenarioStatus failed: {scenario_status}")
        active_name = (
            str(active_status.get("active_scenario_name") or "").strip()
            if active_status is not None
            else ""
        )
        cycle_value = getattr(tcp, "fixed_step_request_count", "-")
        cycle = cycle_value if isinstance(cycle_value, int) else "-"
        if phase == "ack":
            if not isinstance(dashboard, FixedUdpDashboard):
                print(
                    f"lockstep_event cycle={cycle} "
                    f"transport=TCP api=FixedStep msg=0x1201 result={fixed_result}"
                )
                print(
                    f"lockstep_event cycle={cycle} "
                    f"transport=TCP api=SaveData msg=0x1202 "
                    f"result={save_result if save_result is not None else 'not_called'}"
                )
            last_reported_active_name = getattr(
                tcp, "_last_reported_active_scenario_name", None
            )
            if not isinstance(last_reported_active_name, str):
                last_reported_active_name = None
            if last_reported_active_name is None:
                setattr(tcp, "_last_reported_active_scenario_name", active_name)
            elif active_name != last_reported_active_name:
                if isinstance(dashboard, FixedUdpDashboard):
                    dashboard.finish()
                print(
                    f"tcp_response cycle={cycle} api=ActiveSuiteStatus msg=0x1401 "
                    f"event=changed previous={last_reported_active_name or '-'} "
                    f"active_scenario={active_name or '-'}"
                )
                setattr(tcp, "_last_reported_active_scenario_name", active_name)
            scenario_state = scenario_status.get("state")
            last_reported_scenario_state = getattr(
                tcp, "_last_reported_scenario_state", None
            )
            if not isinstance(last_reported_scenario_state, int):
                last_reported_scenario_state = None
            if last_reported_scenario_state is None:
                setattr(tcp, "_last_reported_scenario_state", scenario_state)
            elif scenario_state != last_reported_scenario_state:
                if isinstance(dashboard, FixedUdpDashboard):
                    dashboard.finish()
                print(
                    f"tcp_response cycle={cycle} api=ScenarioStatus msg=0x1504 "
                    f"event=changed previous={last_reported_scenario_state!r} "
                    f"result={scenario_status}"
                )
                setattr(tcp, "_last_reported_scenario_state", scenario_state)
        else:
            if isinstance(dashboard, FixedUdpDashboard):
                dashboard.finish()
            print(
                f"lockstep_progress cycle={cycle} phase={phase} "
                f"active_scenario={active_name or '-'} "
                f"scenario_status={scenario_status}"
            )
        if (
            expected_scenario_name
            and active_name
            and active_name != expected_scenario_name
        ):
            raise ActiveScenarioChanged(
                expected_scenario_name,
                active_name,
                active_status or {},
                scenario_status,
            )
        if scenario_state_allows_next(scenario_status):
            raise ScenarioBoundaryDetected(
                "scenario boundary after FixedStep ACK: "
                f"expected={expected_scenario_name!r} status={scenario_status} "
                f"fixed={fixed_result} save={save_result}"
            )

    check_scenario_progress("ack")
    wait_started = time.monotonic()
    deadline = None if timeout <= 0 else wait_started + timeout
    next_status = wait_started + status_period
    polls = 0

    while True:
        now = time.monotonic()
        udp.poll()
        polls += 1
        ego = resolve_runtime_ego(udp, ego_candidates)
        current_stamp = state_stamp(ego) if ego is not None else None
        if ego is not None and (previous_stamp is None or current_stamp > previous_stamp):
            if isinstance(dashboard, FixedUdpDashboard):
                dashboard.vehicle_ok(ego, udp)
                return ego, fixed_result, save_result
            cycle_value = getattr(tcp, "fixed_step_request_count", "-")
            cycle = cycle_value if isinstance(cycle_value, int) else "-"
            print(
                f"lockstep_event cycle={cycle} "
                f"source=UDP api=VehicleInfo received=yes "
                f"stamp={format_stamp(current_stamp)} "
                f"ego={getattr(ego, 'id', '-')} raw={getattr(ego, 'raw_id', '-')} "
                f"pos=({getattr(ego, 'x', 0.0):.2f},"
                f"{getattr(ego, 'y', 0.0):.2f}) udp_rx={udp.rx_records}"
            )
            return ego, fixed_result, save_result

        if now >= next_status:
            check_scenario_progress("state_wait")
            print(
                "fixed_step_state_wait: "
                f"previous={format_stamp(previous_stamp)} "
                f"current={format_stamp(current_stamp)} "
                f"polls={polls} rx={udp.rx_records} chunks={udp.rx_chunks} "
                f"invalid={udp.invalid_packets} fixed={fixed_result} save={save_result}"
            )
            next_status = now + status_period

        if deadline is not None and now >= deadline:
            check_scenario_progress("timeout")
            reason = "ego state not received" if ego is None else "ego timestamp did not advance"
            raise FixedAckUdpWaitTimeout(
                "fixed_ack_udp_wait_timeout after FixedStep(1): "
                f"{reason}; previous={format_stamp(previous_stamp)} "
                f"current={format_stamp(current_stamp)} rx={udp.rx_records} "
                f"chunks={udp.rx_chunks} invalid={udp.invalid_packets} "
                f"polls={polls} fixed={fixed_result} save={save_result}"
            )
        time.sleep(0.005)


def bootstrap_fixed_tcp_ego(
    *,
    tcp: MoraiTcpClient,
    udp: MoraiUdpClient,
    ego_id: str,
    timeout: float,
    status_period: float,
    save_data_each_step: bool,
    ego_candidates: tuple[str, ...],
    send_control: bool,
    force_step: bool = False,
    expected_scenario_name: str | None = None,
) -> tuple[object, dict[str, int] | None, dict[str, int] | None]:
    udp.poll()
    ego = resolve_runtime_ego(udp, ego_candidates)
    if ego is not None and not force_step:
        print(
            "udp_ready: "
            f"stamp={format_stamp(state_stamp(ego))} "
            f"{udp_status_text(udp, ego_candidates)} bootstrap_step=0"
        )
        return ego, None, None
    if force_step:
        udp.states.clear()
        ego = None
        print(
            "scenario_transition_udp_reset: "
            "drained cached VehicleInfo and forcing FixedStep(1) bootstrap"
        )

    print(
        "lockstep_bootstrap: initial Ego state unavailable; "
        f"manual_control={'deferred' if send_control else 'disabled'} "
        "sending wake-up FixedStep(1)"
    )
    ego, fixed_result, save_result = fixed_step_and_wait_for_ego(
        tcp=tcp,
        udp=udp,
        previous_stamp=None,
        timeout=timeout,
        status_period=status_period,
        save_data_each_step=save_data_each_step,
        ego_candidates=ego_candidates,
        expected_scenario_name=expected_scenario_name,
    )
    print(
        "lockstep_bootstrap_ready: "
        f"stamp={format_stamp(state_stamp(ego))} fixed={fixed_result} save={save_result}"
    )
    return ego, fixed_result, save_result


def hold_for_sim_scenario_transition(
    *,
    tcp: MoraiTcpClient,
    udp: MoraiUdpClient,
    previous_name: str | None,
    timeout: float,
    status_period: float,
    initial_change: ActiveScenarioChanged | None = None,
) -> ActiveScenarioChanged | None:
    previous = str(previous_name or "").strip()
    latest_name = initial_change.active_name if initial_change is not None else ""
    latest_active = initial_change.active_status if initial_change is not None else {}
    latest_scenario = initial_change.scenario_status if initial_change is not None else {}
    hold_seconds = max(0.0, timeout)
    started = time.monotonic()
    deadline = started + hold_seconds
    check_period = min(0.25, max(0.05, status_period))
    next_check = started
    checks = 0
    print(
        "scenario_transition_hold: "
        f"previous={previous or '-'} seconds={hold_seconds:.1f} "
        "control=hold fixed_step=hold udp_receive=replace"
    )
    # Discard the previous scenario's cached state before the SIM starts
    # publishing the next scenario. Samples received during the hold belong
    # to the replacement scenario and must remain available as its initial
    # PDM input state.
    udp.states.clear()

    if latest_name:
        print(
            'scenario_transition_confirmed: '
            f'previous={previous!r} active={latest_name!r} '
            f'checks={checks} status={latest_scenario}'
        )
        return ActiveScenarioChanged(
            previous,
            latest_name,
            latest_active,
            latest_scenario,
        )

    while True:
        udp.poll()
        now = time.monotonic()
        if now >= next_check or now >= deadline:
            checks += 1
            try:
                active_status = tcp.active_suite_status()
                scenario_status = tcp.scenario_status()
            except (OSError, MoraiTcpError) as exc:
                print(f"scenario_transition_status_retry: error={exc}")
            else:
                active_name = str(
                    active_status.get("active_scenario_name") or ""
                ).strip()
                if active_name and active_name != previous:
                    latest_name = active_name
                    latest_active = active_status
                    latest_scenario = scenario_status
                    print(
                        'scenario_transition_confirmed: '
                        f'previous={previous!r} active={latest_name!r} '
                        f'checks={checks} status={latest_scenario}'
                    )
                    return ActiveScenarioChanged(
                        previous,
                        latest_name,
                        latest_active,
                        latest_scenario,
                    )
            next_check = now + check_period
        if now >= deadline:
            break
        time.sleep(min(0.02, max(0.0, deadline - now)))

    udp.poll()
    if not latest_name:
        print(
            "scenario_transition_not_observed: "
            f"previous={previous or '-'} checks={checks}"
        )
        return None
    print(
        "scenario_transition_confirmed: "
        f"previous={previous or '-'} active={latest_name!r} "
        f"checks={checks} status={latest_scenario}"
    )
    return ActiveScenarioChanged(
        previous,
        latest_name,
        latest_active,
        latest_scenario,
    )


def select_advanced_route(scenario, ego_id: str, route_entity: str | None):
    if not route_entity:
        return None
    if route_entity != ego_id:
        raise RuntimeError(
            f"NPC AdvancedRouting '{route_entity}' cannot be used as the Ego route. "
            "Leave --route-entity unset to build a route from Ego pose and MGeo nearest-link successors."
        )
    return scenario.advanced_routes.get(route_entity)


def apply_advanced_route_if_available(
    route: MGeoRoutePlanner,
    scenario,
    ego_id: str,
    route_entity: str | None,
) -> tuple[bool, str]:
    advanced_route = select_advanced_route(scenario, ego_id, route_entity)
    if advanced_route is None:
        if route_entity:
            available = ", ".join(sorted(scenario.advanced_routes)) or "-"
            raise RuntimeError(
                f"AdvancedRouting for route entity '{route_entity}' was not found. "
                f"Available routes: {available}"
            )
        return False, "MGeo nearest-link route"
    route.set_route_points(
        np.asarray(advanced_route.points, dtype=float),
        f"AdvancedRouting:{advanced_route.entity_id}:{advanced_route.name}",
    )
    return True, f"AdvancedRouting:{advanced_route.entity_id}:{advanced_route.name}"


def rebuild_route_from_runtime_pose(
    route: MGeoRoutePlanner,
    ego,
    route_length: float,
) -> str:
    try:
        route.rebuild_from_pose(
            ego.x,
            ego.y,
            ego.heading_rad,
            min_route_length=route_length,
        )
    except (OSError, RuntimeError, ValueError) as exc:
        raise RuntimeError(
            "Route context unavailable. Provide --suite or configure MGeo fallback route."
        ) from exc
    if len(route.route_points) < 2:
        raise RuntimeError(
            "Route context unavailable. Provide --suite or configure MGeo fallback route."
        )
    return "MGeo runtime Ego pose route"


def is_inside_drivable_area(
    route: MGeoRoutePlanner,
    x: float,
    y: float,
) -> bool:
    drivable_area = getattr(route, "drivable_area", None)
    if drivable_area is None:
        # Older MGeoRoutePlanner data has no polygon index. In that case retain
        # the established route-distance guard instead of failing at runtime.
        return True
    return bool(drivable_area.contains(x, y))


def fixed_time_status_matches(
    status: dict[str, Any],
    *,
    control_hz: int,
    physics_delta_time: int,
    rtf: int,
    user_control: int,
) -> bool:
    return (
        status.get("mode") == TIME_MODE_FIXED
        and status.get("target_fps") == control_hz
        and status.get("physics_delta_time") == physics_delta_time
        and status.get("rtf") == rtf
        and status.get("user_control") == user_control
    )


def variable_time_status_matches(
    status: dict[str, Any],
    *,
    control_hz: int,
    physics_delta_time: int,
) -> bool:
    return (
        status.get("mode") == TIME_MODE_VARIABLE
        and status.get("target_fps") == control_hz
        and status.get("physics_delta_time") == physics_delta_time
        and status.get("user_control") == 0
    )


def resolve_sut_timing(
    args: argparse.Namespace,
    default_control_hz: int,
) -> tuple[int, float]:
    if args.pdm_step_delta_time is not None:
        time_step = float(args.pdm_step_delta_time) / 1000.0
        if time_step <= 0.0:
            raise ValueError("--pdm-step-delta-time must be greater than 0 ms")
        exact_hz = 1.0 / time_step
        control_hz = int(round(exact_hz))
        if control_hz < 1 or not math.isclose(exact_hz, control_hz, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError(
                "--pdm-step-delta-time must map to an integer target FPS; "
                f"got {args.pdm_step_delta_time:g} ms ({exact_hz:.9g} Hz)"
            )
        return control_hz, time_step

    control_hz = max(
        1,
        (
            int(args.fixed_target_fps)
            if uses_fixed_time_mode(args)
            else int(default_control_hz)
        ),
    )
    return control_hz, 1.0 / float(control_hz)


def configure_fixed_step(
    tcp: MoraiTcpClient,
    args: argparse.Namespace,
    control_hz: int,
    user_control_override: int | None = None,
    force_reapply: bool = False,
) -> None:
    requested_user_control = (
        args.user_control
        if user_control_override is None
        else int(user_control_override)
    )
    time_status_before = tcp.get_simulation_time_status()
    print(f"time_status_before={time_status_before}")
    if not force_reapply and fixed_time_status_matches(
        time_status_before,
        control_hz=control_hz,
        physics_delta_time=args.physics_delta_time,
        rtf=args.rtf,
        user_control=requested_user_control,
    ):
        print("set_fixed_mode=skipped (requested fixed settings already active)")
        print(f"time_status_after={time_status_before}")
        return

    set_result = tcp.set_simulation_time_mode(
        TIME_MODE_FIXED,
        control_hz,
        args.physics_delta_time,
        args.rtf,
        requested_user_control,
    )
    print(f"set_fixed_mode={set_result}")
    if set_result.get("result_code", 0) != 0:
        raise MoraiTcpError(f"Fixed-step setup failed: {set_result}")

    time_status_after = tcp.get_simulation_time_status()
    print(f"time_status_after={time_status_after}")
    if not fixed_time_status_matches(
        time_status_after,
        control_hz=control_hz,
        physics_delta_time=args.physics_delta_time,
        rtf=args.rtf,
        user_control=requested_user_control,
    ):
        raise MoraiTcpError(
            "Fixed-step verification failed: "
            f"requested=(fps={control_hz}, physics_delta_time={args.physics_delta_time}, "
            f"rtf={args.rtf}, user_control={requested_user_control}), "
            f"actual={time_status_after}"
        )


def configure_variable_mode(
    tcp: MoraiTcpClient,
    args: argparse.Namespace,
    control_hz: int,
) -> None:
    time_status_before = tcp.get_simulation_time_status()
    print(f"time_status_before={time_status_before}")
    if variable_time_status_matches(
        time_status_before,
        control_hz=control_hz,
        physics_delta_time=args.physics_delta_time,
    ):
        print("set_variable_mode=skipped (requested variable settings already active)")
        return
    if args.rtf != 0 or args.user_control != 0:
        print(
            "variable_mode_note: TCP request uses rtf=0 and user_control=0 in variable mode; "
            "the simulator may normalize the reported rtf value; "
            f"requested rtf={args.rtf} user_control={args.user_control}"
        )
    set_result = tcp.set_simulation_time_mode(
        TIME_MODE_VARIABLE,
        control_hz,
        args.physics_delta_time,
        0,
        0,
    )
    print(f"set_variable_mode={set_result}")
    if set_result.get("result_code", 0) != 0:
        raise MoraiTcpError(f"Variable-mode setup failed: {set_result}")
    time_status_after = tcp.get_simulation_time_status()
    print(f"time_status_after={time_status_after}")
    if not variable_time_status_matches(
        time_status_after,
        control_hz=control_hz,
        physics_delta_time=args.physics_delta_time,
    ):
        raise MoraiTcpError(
            "Variable-mode verification failed: "
            f"requested=(fps={control_hz}, physics_delta_time={args.physics_delta_time}, "
            f"user_control=0; rtf ignored), actual={time_status_after}"
        )


def main(argv: list[str] | None = None) -> int:
    args = parse_runtime_args(argv)
    mode = select_runtime_mode(args)
    print(f"mode={mode}")
    print(
        "selected_modes: "
        f"execution_mode={args.execution_mode} control_mode={args.control_mode}"
    )
    print("No --xosc provided; skipping scenario preparation.")
    if args.source is None:
        print("No --suite provided; attaching to already active simulator state.")
    if not args.load_suite:
        print("--load-suite not provided; existing suite state will not be modified.")
    if not args.scenario_play:
        print("--scenario-play not provided; existing scenario play state will not be modified.")
    if not args.scenario_stop:
        print("--scenario-stop not provided; existing scenario stop state will not be modified.")

    if args.execution_mode == "gui" and args.load_suite:
        print("ERROR: GUI execution mode must not call LoadSuite. Use --execution-mode headless --load-suite.")
        return 2
    if args.load_suite and args.source is None:
        print("ERROR: --load-suite requires --suite.")
        return 2
    if args.dry_run and args.source is None:
        print("ERROR: attach-only --dry-run requires --suite because runtime Ego pose is unavailable.")
        return 2

    try:
        scenario = load_scenario(args.source, args.scenario_name) if args.source is not None else None
        settings = build_runtime_settings(args, scenario)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 2

    binding = scenario.binding if scenario is not None else None
    ego_id = settings.ego_id
    ego_candidates = settings.ego_candidates
    entity_ids = settings.entity_ids
    rx_ip = settings.rx_ip
    rx_port = settings.rx_port
    tx_ip = settings.tx_ip
    tx_port = settings.tx_port
    base_target_speed = settings.target_speed
    target_speed = base_target_speed
    if args.manual_test:
        return run_manual_test(args, tx_ip, tx_port, settings.control_hz)
    will_run_control = not (
        args.dry_run
        or args.plan_only
        or args.scenario_stop_only
        or args.scenario_control_only is not None
    )
    needs_tcp_management = (
        args.plan_only
        or args.check_suite
        or args.fixed_step
        or args.load_suite
        or args.scenario_play
        or args.scenario_stop
        or args.scenario_stop_only
        or args.scenario_control_only is not None
        or args.tester_control_csv is not None
    )
    if needs_tcp_management and args.tcp_port <= 0:
        print("ERROR: selected plan/check/control mode requires --tcp-port.")
        return 2
    if args.scenario_stop_only or args.scenario_control_only is not None:
        menu = "stop" if args.scenario_stop_only else args.scenario_control_only
        tcp = MoraiTcpClient(args.tcp_host, args.tcp_port, timeout=args.tcp_timeout)
        try:
            tcp.open()
            return 0 if run_scenario_control_once(tcp, binding, menu, args.scenario_name) else 2
        except (OSError, MoraiTcpError) as exc:
            print(f"ERROR: {exc}")
            return 2
        finally:
            tcp.close()
    try:
        control_hz, time_step = resolve_sut_timing(args, settings.control_hz)
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return 2
    config = PdmLiteMoraiConfig(fps=float(control_hz), time_step=time_step)
    try:
        apply_pdm_profile(config, args.pdm_profile)
        target_speed = base_target_speed * pdm_profile_target_speed_scale(args.pdm_profile)
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return 2
    print(
        "timing_config: "
        f"pdm_step_delta_ms={config.time_step * 1000.0:.6g} "
        f"pdm_target_fps={control_hz} "
        f"sim_physics_delta_ms={args.physics_delta_time}"
    )
    print(
        "pdm_profile: "
        f"name={args.pdm_profile} "
        f"base_target={base_target_speed:.3f}mps "
        f"effective_target={target_speed:.3f}mps "
        f"gap={config.leading_vehicle_min_gap:.2f}m/"
        f"{config.leading_vehicle_time_gap:.2f}s "
        f"lead_window={config.leading_vehicle_lateral_window:.2f}m "
        f"emergency_ttc={config.leading_vehicle_emergency_ttc:.2f}s "
        f"steer_rate={config.steer_rate_limit:.2f} "
        f"throttle_rate={config.throttle_increase_rate:.2f} "
        f"brake_rate={config.brake_increase_rate:.2f}"
    )

    if args.plan_only:
        udp = MoraiUdpClient(
            ego_info_ip=rx_ip,
            ego_info_port=rx_port,
            ctrl_ip=tx_ip,
            ctrl_port=tx_port,
            entity_ids=entity_ids,
            ego_id=ego_id,
        )
        tcp = MoraiTcpClient(args.tcp_host, args.tcp_port, timeout=args.tcp_timeout)
        try:
            udp.open()
            tcp.open()
            active_status, scenario_status, time_status = read_tcp_preflight(tcp)
            print_preflight_status(
                args=args,
                active_status=active_status,
                scenario_status=scenario_status,
                time_status=time_status,
                udp=udp,
                ego_candidates=ego_candidates,
            )
            suite_ok = verify_suite_preflight(
                args=args,
                binding=binding,
                active_status=active_status,
                scenario_status=scenario_status,
            )
            udp_ok = wait_for_udp_ready(
                udp,
                tcp=None,
                timeout=args.udp_ready_timeout,
                status_period=1.0 / max(0.1, args.status_hz),
                fixed_step=False,
                save_data_each_step=False,
                max_fixed_steps=0,
                require_stamp_advance=args.control_mode != "fixed-tcp-lockstep",
                ego_candidates=ego_candidates,
            )
            active_status, scenario_status, time_status = read_tcp_preflight(tcp)
            print_preflight_status(
                args=args,
                active_status=active_status,
                scenario_status=scenario_status,
                time_status=time_status,
                udp=udp,
                ego_candidates=ego_candidates,
            )
            return 0 if suite_ok and udp_ok else 2
        except (OSError, RuntimeError) as exc:
            print(f"ERROR: {exc}")
            return 2
        finally:
            tcp.close()
            udp.close()

    # TESTER_CONTROL_REPLAY_BEGIN
    if args.tester_control_csv is not None:
        if not args.fixed_step:
            print("ERROR: --tester-control-csv requires --fixed-step.")
            return 2
        if args.no_send:
            print("ERROR: --tester-control-csv cannot be combined with --no-send.")
            return 2
        if binding is None:
            print("ERROR: --tester-control-csv requires --suite.")
            return 2
        try:
            tester_controls = load_tester_controls(args.tester_control_csv, ego_id)
        except (OSError, ValueError, csv.Error) as exc:
            print(f"ERROR: {exc}")
            return 2
        tester_path = Path(args.tester_control_csv).expanduser().resolve(strict=False)
        print(
            f"tester_control_csv: {tester_path} "
            f"rows={len(tester_controls)} ego={ego_id}"
        )
        if args.dry_run:
            return 0
        return run_tester_control_replay(
            args,
            binding,
            tester_controls,
            tx_ip,
            tx_port,
            control_hz,
        )
    # TESTER_CONTROL_REPLAY_END

    try:
        route = MGeoRoutePlanner(
            settings.map_dir,
            spacing=1.0 / config.points_per_meter,
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(
            "ERROR: Route context unavailable. Provide --suite or configure "
            f"MGeo fallback route. detail={exc}"
        )
        return 2

    route_from_advanced = False
    route_initialized = False
    route_source = "pending runtime Ego pose"
    if scenario is not None and not args.route_from_runtime_pose:
        start_pose = scenario.start_pose
        if start_pose is None:
            print("ERROR: Ego start pose was not found in xosc.")
            return 2
        try:
            route.rebuild_from_pose(
                start_pose.x,
                start_pose.y,
                start_pose.heading,
                min_route_length=args.route_length,
            )
            route_from_advanced, route_source = apply_advanced_route_if_available(
                route,
                scenario,
                ego_id,
                args.route_entity,
            )
            route_initialized = True
        except RuntimeError as exc:
            print(f"ERROR: {exc}")
            return 2
    if args.dry_run:
        print_dry_run(
            scenario,
            route,
            ego_id,
            entity_ids,
            rx_ip,
            rx_port,
            tx_ip,
            tx_port,
            route_source,
            args.route_max_error,
        )
        return 0

    udp = MoraiUdpClient(
        ego_info_ip=rx_ip,
        ego_info_port=rx_port,
        ctrl_ip=tx_ip,
        ctrl_port=tx_port,
        entity_ids=entity_ids,
        ego_id=ego_id,
    )
    controller = PdmMoraiController(target_speed=target_speed, config=config)
    actor_lengths = actor_lengths_from_metadata(
        settings.actor_metadata,
        config.ego_length,
        ego_candidates,
    )
    period = config.time_step
    status_period = 1.0 / max(0.1, args.status_hz)
    end_time = None if args.duration <= 0 else time.monotonic() + args.duration
    next_status = 0.0
    inferred_mapping_reported = False
    play_gate_stamp: tuple[int, int] | None = None
    play_gate_released = not args.hold_until_play
    last_processed_stamp: tuple[int, int] | None = None
    lockstep_ego = None
    duplicate_state_polls = 0
    lockstep_wait_started: float | None = None
    dashboard_udp_modes = {"variable-udp", "fixed-auto-udp"}
    variable_live_status = (
        args.control_mode in dashboard_udp_modes and sys.stdout.isatty()
    )
    variable_live_lines = 0

    def finish_variable_live_status() -> None:
        nonlocal variable_live_lines
        if variable_live_lines:
            sys.stdout.write("\n")
            sys.stdout.flush()
            variable_live_lines = 0

    def render_variable_live_status(lines: list[str]) -> None:
        nonlocal variable_live_lines
        if variable_live_lines:
            if variable_live_lines > 1:
                sys.stdout.write(f"\x1b[{variable_live_lines - 1}F")
            else:
                sys.stdout.write("\r")
        sys.stdout.write("\n".join(f"\x1b[2K\r{line}" for line in lines))
        sys.stdout.flush()
        variable_live_lines = len(lines)

    transition_save_data_required = False
    control_steps = 0
    control_started = False
    trace_log = RuntimeTraceLog(None)
    current_scenario_name = (
        args.scenario_name
        or (binding.scenario_path.stem if binding is not None and binding.scenario_path is not None else None)
    )

    def reset_scenario_runtime(
        scenario_name: str,
        *,
        preserve_udp_state: bool = False,
    ) -> None:
        nonlocal current_scenario_name, route_initialized, route_from_advanced, route_source
        nonlocal last_processed_stamp, lockstep_ego, duplicate_state_polls
        nonlocal lockstep_wait_started, play_gate_stamp, play_gate_released, controller

        current_scenario_name = scenario_name
        route_initialized = False
        route_from_advanced = False
        route_source = "pending runtime Ego pose"
        route.current_links = []
        route.current_link_ids = []
        route.route_points = np.empty((0, 3), dtype=float)
        route.route_s = np.empty((0,), dtype=float)
        controller = PdmMoraiController(target_speed=target_speed, config=config)
        if not preserve_udp_state:
            udp.states.clear()
        last_processed_stamp = None
        lockstep_ego = None
        duplicate_state_polls = 0
        lockstep_wait_started = None
        play_gate_stamp = None
        play_gate_released = not args.hold_until_play

    def adopt_sim_scenario(
        tcp_client: MoraiTcpClient,
        change: ActiveScenarioChanged,
    ) -> bool:
        nonlocal lockstep_ego, transition_save_data_required

        print(
            "scenario_transition_detected: "
            f"owner=sim previous={change.previous_name!r} "
            f"active={change.active_name!r} "
            f"status={change.scenario_status}"
        )
        clock_reset_deadline = time.monotonic() + max(
            args.scenario_transition_wait,
            args.lockstep_state_timeout,
        )
        previous_stamp = last_processed_stamp
        clock_status = None
        while time.monotonic() < clock_reset_deadline:
            udp.poll()
            clock_status = tcp_client.get_simulation_time_status()
            clock_stamp = (
                int(clock_status.get('seconds', 0)),
                int(clock_status.get('nanos', 0)),
            )
            clock_step_index = int(clock_status.get('step_index', -1))
            if (
                clock_step_index == 0
                and (previous_stamp is None or clock_stamp < previous_stamp)
            ):
                break
            time.sleep(0.05)
        else:
            raise FixedAckUdpWaitTimeout(
                'fixed_ack_udp_wait_timeout waiting for scenario clock reset: '
                f'previous={format_stamp(previous_stamp)} current={clock_status}'
            )
        print(
            'scenario_transition_clock_reset: '
            f'previous={format_stamp(previous_stamp)} current={format_stamp(clock_stamp)} '
            f'step_index={clock_step_index}'
        )
        reset_scenario_runtime(change.active_name)
        settle_deadline = time.monotonic() + max(0.0, args.scenario_transition_wait)
        while time.monotonic() < settle_deadline:
            udp.poll()
            time.sleep(min(0.02, max(0.0, settle_deadline - time.monotonic())))
        print(
            "scenario_transition_sender_wakeup: "
            "rearming Variable->Fixed and enabling transition SaveData flush"
        )
        transition_save_data_required = True
        variable_result = tcp_client.set_simulation_time_mode(
            TIME_MODE_VARIABLE,
            control_hz,
            args.physics_delta_time,
            0,
            0,
        )
        if variable_result.get("result_code", 0) != 0:
            raise MoraiTcpError(
                f"Scenario transition Variable rearm failed: {variable_result}"
            )
        configure_fixed_step(
            tcp_client,
            args,
            control_hz,
            force_reapply=True,
        )
        lockstep_ego, _fixed_result, _save_result = bootstrap_fixed_tcp_ego(
            tcp=tcp_client,
            udp=udp,
            ego_id=ego_id,
            timeout=args.lockstep_state_timeout,
            status_period=status_period,
            save_data_each_step=True,
            ego_candidates=ego_candidates,
            send_control=not args.no_send,
            force_step=True,
            expected_scenario_name=change.active_name,
        )
        print(
            "scenario_transition_ready: "
            f"owner=sim active={change.active_name!r} "
            f"stamp={format_stamp(state_stamp(lockstep_ego)) if lockstep_ego is not None else '-'}"
        )
        return True

    def switch_to_next_scenario(
        tcp_client: MoraiTcpClient,
        reason: Exception,
        *,
        wait_before_play: bool = True,
    ) -> bool:
        nonlocal lockstep_ego

        if not args.auto_next_scenario:
            return False
        active_status = tcp_client.active_suite_status()
        next_name = next_scenario_from_status(active_status, current_scenario_name)
        if not next_name:
            print(
                "scenario_transition: no next scenario; "
                f"current={current_scenario_name!r} active={active_status}"
            )
            return False
        print(
            "scenario_transition: "
            f"current={current_scenario_name!r} next={next_name!r} "
            f"wait={args.scenario_transition_wait:.1f}s reason={reason}"
        )
        if wait_before_play and args.scenario_transition_wait > 0:
            time.sleep(args.scenario_transition_wait)
        result = tcp_client.scenario_control(SCENARIO_PLAY, next_name)
        print(f"scenario_transition_play name={next_name!r} result={result}")
        if result.get("result_code", 0) != 0:
            return False
        reset_scenario_runtime(next_name)
        if args.control_mode == "fixed-tcp-lockstep":
            lockstep_ego, _fixed_result, _save_result = bootstrap_fixed_tcp_ego(
                tcp=tcp_client,
                udp=udp,
                ego_id=ego_id,
                timeout=args.lockstep_state_timeout,
                status_period=status_period,
                save_data_each_step=(
                    args.save_data_each_step or transition_save_data_required
                ),
                ego_candidates=ego_candidates,
                send_control=not args.no_send,
                force_step=True,
            )
        print(f"scenario_transition_ready name={next_name!r}")
        return True

    def handle_scenario_event(tcp_client: MoraiTcpClient, event: Exception) -> bool:
        transition_hold_done = False
        if args.control_mode == "fixed-tcp-lockstep" and isinstance(
            event,
            (ActiveScenarioChanged, ScenarioBoundaryDetected, FixedAckUdpWaitTimeout),
        ):
            change = hold_for_sim_scenario_transition(
                tcp=tcp_client,
                udp=udp,
                previous_name=current_scenario_name,
                timeout=args.scenario_transition_wait,
                status_period=status_period,
                initial_change=event if isinstance(event, ActiveScenarioChanged) else None,
            )
            transition_hold_done = True
            if change is not None:
                return adopt_sim_scenario(tcp_client, change)
        return switch_to_next_scenario(
            tcp_client,
            event,
            wait_before_play=not transition_hold_done,
        )

    print(
        f"PDM-MORAI start: rx={rx_ip}:{rx_port}, "
        f"tx={tx_ip}:{tx_port}, ego_candidates={','.join(ego_candidates)}, "
        f"target={target_speed:.2f} m/s, send={not args.no_send}, "
        f"fixed_step={args.fixed_step}, hold_until_play={args.hold_until_play}, "
        f"fixed_control_transport={args.fixed_control_transport}, "
        f"pdm_step_delta_ms={config.time_step * 1000.0:.6g}, "
        f"pdm_target_fps={control_hz}, sim_physics_delta_ms={args.physics_delta_time}, "
        f"udp_ready_timeout={args.udp_ready_timeout:.1f}s"
    )
    print("PDM-MORAI SUT controller started.")
    if args.hold_until_play and args.fixed_step:
        print(
            "play_gate: fixed-step requests are held until UDP sim time advances externally."
        )
    if entity_ids:
        print(f"runtime_entities: {', '.join(entity_ids)}")
    if scenario is not None:
        ego_meta = scenario.actor_metadata.get(ego_id)
        if ego_meta is not None:
            print(
                "xosc_meta: actors={actors}, ego_type={ego_type}, ego_bbox={bbox}".format(
                    actors=len(scenario.actor_metadata),
                    ego_type=ego_meta.object_type,
                    bbox=ego_meta.dimensions or "-",
                )
            )
        else:
            print(f"xosc_meta: actors={len(scenario.actor_metadata)}, ego metadata missing")
    else:
        print("xosc_meta: unavailable (attach-only mode)")
    print(
        "missing_pdm_data: pedestrians, traffic_light_state, stop_line, stop_sign, "
        "static_obstacles, active_scenarios, TTC/hazard_annotation"
    )
    tcp: MoraiTcpClient | None = None
    try:
        udp.open()
        if needs_tcp_management:
            tcp = MoraiTcpClient(args.tcp_host, args.tcp_port, timeout=args.tcp_timeout)
            tcp.open()
            if should_stop_before_start(args):
                stop_before_start(tcp, binding, args.scenario_name)
            if args.load_suite:
                suite_path = binding.suite_path or binding.source_path
                load_result = tcp.load_suite(suite_path)
                print(f"load_suite={load_result}")
                if load_result.get("result_code") != 0:
                    raise MoraiTcpError(f"LoadSuite failed: {load_result}")
            if args.execution_mode == "headless" and binding is not None:
                active_status = wait_for_headless_suite(
                    tcp,
                    binding,
                    args.headless_timeout,
                    status_period,
                )
                print(f"headless_active_suite={active_status}")
            play_sent = False
            if args.control_mode == "fixed-tcp-lockstep" and args.scenario_play:
                configure_fixed_step(
                    tcp,
                    args,
                    control_hz,
                    user_control_override=0,
                )
                play_name = scenario_control_name(tcp, binding, args.scenario_name)
                play_result = tcp.scenario_control(SCENARIO_PLAY, play_name)
                print(f"scenario_play name={play_name!r} result={play_result}")
                if play_result.get("result_code") != 0:
                    raise MoraiTcpError(f"Scenario Play failed: {play_result}")
                play_sent = True
                if not wait_for_udp_ready(
                    udp,
                    tcp=None,
                    timeout=args.udp_ready_timeout,
                    status_period=status_period,
                    fixed_step=False,
                    save_data_each_step=False,
                    max_fixed_steps=0,
                    require_stamp_advance=False,
                    ego_candidates=ego_candidates,
                ):
                    raise RuntimeError(
                        "Scenario Play succeeded but initial Ego VehicleInfo was not received "
                        "before fixed-tcp-lockstep took ownership of the SIM clock"
                    )
                configure_fixed_step(tcp, args, control_hz)
            elif args.control_mode == "fixed-auto-udp":
                configure_fixed_step(
                    tcp,
                    args,
                    control_hz,
                    user_control_override=0,
                )
            elif args.fixed_step:
                configure_fixed_step(tcp, args, control_hz)
            else:
                configure_variable_mode(tcp, args, control_hz)
            if args.scenario_play and not play_sent:
                play_name = scenario_control_name(tcp, binding, args.scenario_name)
                play_result = tcp.scenario_control(SCENARIO_PLAY, play_name)
                print(f"scenario_play name={play_name!r} result={play_result}")
                if play_result.get("result_code") != 0:
                    raise MoraiTcpError(f"Scenario Play failed: {play_result}")
            if args.tcp_settle > 0:
                time.sleep(args.tcp_settle)
            active_status, scenario_status, time_status = read_tcp_preflight(tcp)
            active_scenario_name = str(
                active_status.get("active_scenario_name") or ""
            ).strip()
            if active_scenario_name:
                current_scenario_name = active_scenario_name
            print_preflight_status(
                args=args,
                active_status=active_status,
                scenario_status=scenario_status,
                time_status=time_status,
                udp=udp,
                ego_candidates=ego_candidates,
            )
            if not verify_suite_preflight(
                args=args,
                binding=binding,
                active_status=active_status,
                scenario_status=scenario_status,
            ):
                tcp.close()
                udp.close()
                return 2
    except (OSError, RuntimeError) as exc:
        print(f"ERROR: {exc}")
        if tcp is not None:
            tcp.close()
        udp.close()
        return 2
    try:
        if args.control_mode == "fixed-tcp-lockstep":
            if tcp is None:
                raise MoraiTcpError("fixed-tcp-lockstep requires an active TCP connection")
            lockstep_ego, _fixed_result, _save_result = bootstrap_fixed_tcp_ego(
                tcp=tcp,
                udp=udp,
                ego_id=ego_id,
                timeout=args.lockstep_state_timeout,
                status_period=status_period,
                save_data_each_step=(
                    args.save_data_each_step or transition_save_data_required
                ),
                ego_candidates=ego_candidates,
                send_control=not args.no_send,
            )
        elif args.control_mode == "fixed-udp" and not args.hold_until_play:
            if tcp is None:
                raise MoraiTcpError("fixed-udp requires an active TCP connection")
            # Fixed/UserControl mode does not publish the first Vehicle Info
            # until the simulation clock advances. Force exactly one wake-up
            # step, then wait for that state before sending the first UDP
            # control command.
            bootstrap_fixed_tcp_ego(
                tcp=tcp,
                udp=udp,
                ego_id=ego_id,
                timeout=args.lockstep_state_timeout,
                status_period=status_period,
                save_data_each_step=(
                    args.save_data_each_step or transition_save_data_required
                ),
                ego_candidates=ego_candidates,
                send_control=False,
                force_step=True,
                expected_scenario_name=current_scenario_name,
            )
        elif not wait_for_udp_ready(
            udp,
            None,
            timeout=args.udp_ready_timeout,
            status_period=status_period,
            fixed_step=False,
            save_data_each_step=(
                args.save_data_each_step or transition_save_data_required
            ),
            max_fixed_steps=args.udp_ready_steps,
            require_stamp_advance=not args.hold_until_play,
            ego_candidates=ego_candidates,
        ):
            if tcp is not None:
                tcp.close()
            udp.close()
            return 2
        if not args.fixed_step and tcp is not None:
            tcp.close()
            tcp = None
        trace_log = RuntimeTraceLog(args.trace_log)
        if (
            args.control_mode == "fixed-udp"
            and tcp is not None
            and sys.stdout.isatty()
        ):
            # Bootstrap, preflight, and route setup emit ordinary line logs.
            # Start the live dashboard only after those initialization messages
            # so they cannot invalidate its cursor-relative screen region.
            setattr(tcp, "_fixed_udp_dashboard", FixedUdpDashboard())
    except (OSError, MoraiTcpError, RuntimeError) as exc:
        if tcp is not None:
            dashboard = getattr(tcp, "_fixed_udp_dashboard", None)
            if isinstance(dashboard, FixedUdpDashboard):
                dashboard.finish()
        print(f"ERROR: {exc}")
        if tcp is not None:
            tcp.close()
        udp.close()
        return 2
    try:
        while True:
            loop_start = time.monotonic()
            if end_time is not None and loop_start >= end_time:
                break
            if args.max_control_steps > 0 and control_steps >= args.max_control_steps:
                print(f"control_step_limit_reached={control_steps}")
                break

            fixed_result = None
            save_result = None
            if args.control_mode == "fixed-tcp-lockstep":
                ego = lockstep_ego
            else:
                udp.poll()
                ego = resolve_runtime_ego(udp, ego_candidates)
            if udp.inferred_entity_ids is True and not inferred_mapping_reported:
                print(
                    "runtime_id_mapping: duplicate raw ids detected; "
                    "using record order Ego,NPC_1.. because --suite/--entity-ids "
                    "metadata was not provided."
                )
                inferred_mapping_reported = True
            if ego is None:
                if args.control_mode == "fixed-tcp-lockstep" and play_gate_released:
                    raise RuntimeError(
                        "fixed-tcp-lockstep requires an initial UDP Ego state "
                        "before the first ManualControlById/FixedStep iteration"
                    )
                if tcp is not None and play_gate_released:
                    try:
                        ego, fixed_result, save_result = fixed_step_and_wait_for_ego(
                            tcp=tcp,
                            udp=udp,
                            previous_stamp=None,
                            timeout=args.lockstep_state_timeout,
                            status_period=status_period,
                            save_data_each_step=(
                                args.save_data_each_step or transition_save_data_required
                            ),
                            ego_candidates=ego_candidates,
                            expected_scenario_name=(
                                current_scenario_name
                                if args.control_mode == "fixed-tcp-lockstep"
                                else None
                            ),
                        )
                    except (
                        ScenarioBoundaryDetected,
                        ActiveScenarioChanged,
                        FixedAckUdpWaitTimeout,
                    ) as exc:
                        if handle_scenario_event(tcp, exc):
                            continue
                        raise
                    if ego is not None:
                        current_stamp = state_stamp(ego)
                    else:
                        continue
                else:
                    if loop_start >= next_status:
                        error_text = f" error={udp.last_error}" if udp.last_error else ""
                        print(
                            "waiting for ego state..."
                            f" rx={udp.rx_records} chunks={udp.rx_chunks} "
                            f"invalid={udp.invalid_packets}{error_text}"
                        )
                        next_status = loop_start + status_period
                    time.sleep(0.005 if tcp is not None else period)
                    continue

            if ego is None:
                continue

            if not play_gate_released:
                if tcp is not None:
                    udp.poll()
                    ego = resolve_runtime_ego(udp, ego_candidates) or ego
                    lockstep_ego = ego
                current_stamp = state_stamp(ego)
                if play_gate_stamp is None:
                    play_gate_stamp = current_stamp
                elif current_stamp > play_gate_stamp:
                    play_gate_released = True
                    print(
                        "play_gate: released "
                        f"initial={format_stamp(play_gate_stamp)} "
                        f"current={format_stamp(current_stamp)}"
                    )
                if not play_gate_released:
                    if loop_start >= next_status:
                        print(
                            "waiting for user play..."
                            f" sim_time={format_stamp(play_gate_stamp)} "
                            f"rx={udp.rx_records} actors={len(udp.states)} "
                            f"invalid={udp.invalid_packets}"
                        )
                        next_status = loop_start + status_period
                    time.sleep(0.005 if tcp is not None else period)
                    continue

            current_stamp = state_stamp(ego)
            if not is_new_state_sample(ego, last_processed_stamp):
                if args.control_mode == "fixed-tcp-lockstep":
                    raise RuntimeError(
                        "fixed-tcp-lockstep received a non-advancing cached Ego state: "
                        f"last_processed={format_stamp(last_processed_stamp)} "
                        f"current={format_stamp(current_stamp)}"
                    )
                if tcp is not None:
                    try:
                        ego, fixed_result, save_result = fixed_step_and_wait_for_ego(
                            tcp=tcp,
                            udp=udp,
                            previous_stamp=last_processed_stamp,
                            timeout=args.lockstep_state_timeout,
                            status_period=status_period,
                            save_data_each_step=(
                                args.save_data_each_step or transition_save_data_required
                            ),
                            ego_candidates=ego_candidates,
                            expected_scenario_name=(
                                current_scenario_name
                                if args.control_mode == "fixed-tcp-lockstep"
                                else None
                            ),
                        )
                    except (
                        ScenarioBoundaryDetected,
                        ActiveScenarioChanged,
                        FixedAckUdpWaitTimeout,
                    ) as exc:
                        if handle_scenario_event(tcp, exc):
                            continue
                        raise
                    current_stamp = state_stamp(ego)
                else:
                    duplicate_state_polls += 1
                    if lockstep_wait_started is None:
                        lockstep_wait_started = loop_start
                    elif (
                        args.lockstep_state_timeout > 0
                        and loop_start - lockstep_wait_started >= args.lockstep_state_timeout
                    ):
                        raise RuntimeError(
                            "UDP VehicleInfo state timeout "
                            f"last_processed={format_stamp(last_processed_stamp)} "
                            f"waited={args.lockstep_state_timeout:g}s "
                            f"rx={udp.rx_records} invalid={udp.invalid_packets}"
                        )
                    stale_wait = loop_start - lockstep_wait_started
                    if stale_wait >= 0.5 and loop_start >= next_status:
                        finish_variable_live_status()
                        print(
                            "WARNING variable_udp_stale: no new VehicleInfo "
                            f"waited={stale_wait:.3f}s "
                            f"sim_time={format_stamp(current_stamp)} "
                            f"polls={duplicate_state_polls} "
                            f"udp_rx={udp.rx_records} control_udp_tx={udp.tx_packets}"
                        )
                        next_status = loop_start + status_period
                    elapsed = time.monotonic() - loop_start
                    if elapsed < period:
                        time.sleep(period - elapsed)
                    continue
            last_processed_stamp = current_stamp
            duplicate_state_polls = 0
            lockstep_wait_started = None

            if not route_initialized:
                try:
                    route_source = rebuild_route_from_runtime_pose(
                        route,
                        ego,
                        args.route_length,
                    )
                    route_initialized = True
                    print(
                        f"route_initialized: source={route_source}, ego={ego.id}, "
                        f"points={len(route.route_points)}"
                    )
                except RuntimeError as exc:
                    print(f"ERROR: {exc}")
                    return 2

            inside_drivable_area = is_inside_drivable_area(route, ego.x, ego.y)
            if (
                inside_drivable_area
                and (not route_from_advanced)
                and route.distance_to_route(ego.x, ego.y) > args.rebuild_distance
            ):
                try:
                    rebuild_route_from_runtime_pose(route, ego, args.route_length)
                except RuntimeError as exc:
                    print(f"ERROR: {exc}")
                    return 2
                inside_drivable_area = is_inside_drivable_area(route, ego.x, ego.y)

            route_error = route.distance_to_route(ego.x, ego.y)
            if (not inside_drivable_area) or route_error > args.route_max_error:
                guard_control = EgoManualControl(brake=1.0)
                if not args.no_send:
                    send_ego_control(
                        udp=udp,
                        tcp=tcp,
                        fixed_step=args.fixed_step,
                        fixed_control_transport=args.fixed_control_transport,
                        ego_id=ego_id,
                        control=guard_control,
                    )
                    if args.control_mode == "fixed-udp" and tcp is not None:
                        print_fixed_udp_control_event(
                            tcp=tcp,
                            udp=udp,
                            control=guard_control,
                        )
                    control_started = True
                if tcp is not None:
                    try:
                        lockstep_ego, fixed_result, save_result = fixed_step_and_wait_for_ego(
                            tcp=tcp,
                            udp=udp,
                            previous_stamp=current_stamp,
                            timeout=args.lockstep_state_timeout,
                            status_period=status_period,
                            save_data_each_step=(
                                args.save_data_each_step or transition_save_data_required
                            ),
                            ego_candidates=ego_candidates,
                            expected_scenario_name=current_scenario_name,
                        )
                    except (
                        ScenarioBoundaryDetected,
                        ActiveScenarioChanged,
                        FixedAckUdpWaitTimeout,
                    ) as exc:
                        if handle_scenario_event(tcp, exc):
                            continue
                        raise
                control_steps += 1
                if trace_log.enabled:
                    trace_log.write(build_runtime_trace_record(ego=ego))
                if loop_start >= next_status:
                    finish_variable_live_status()
                    guard_reason = "outside_drivable_area" if not inside_drivable_area else "route_error"
                    print(
                        "route_guard: brake control "
                        f"reason={guard_reason} "
                        f"ego={ego.id} raw={ego.raw_id} "
                        f"pos=({ego.x:.2f},{ego.y:.2f}) "
                        f"inside_drivable_area={inside_drivable_area} "
                        f"route_error={route_error:.2f}m "
                        f"max={args.route_max_error:.2f}m "
                        f"speed={ego.speed:.2f}mps "
                        f"cmd=(thr=0.00, brk=1.00, steer=0.00) "
                        f"rx={udp.rx_records} tx={udp.tx_packets}"
                    )
                    next_status = loop_start + status_period
                elapsed = time.monotonic() - loop_start
                if tcp is None and elapsed < period:
                    time.sleep(period - elapsed)
                continue

            route_segment = route.segment_ahead(ego.x, ego.y, count=args.route_points)
            map_speed_limit = route.speed_limit_at(ego.x, ego.y)
            leading_vehicle = estimate_leading_vehicle(
                ego,
                udp.states,
                route,
                config,
                actor_lengths,
            )
            step = controller.run_step(
                ego,
                route_segment,
                map_speed_limit=map_speed_limit,
                leading_vehicle=leading_vehicle,
            )
            if args.control_mode == "fixed-udp" and tcp is not None:
                dashboard = getattr(tcp, "_fixed_udp_dashboard", None)
                if isinstance(dashboard, FixedUdpDashboard):
                    speed_caps = [
                        value
                        for value in (step.map_speed_limit, step.curve_speed_cap)
                        if value is not None
                    ]
                    dashboard.prepare_control(
                        control=step.control,
                        current_speed=step.current_speed,
                        target_speed=step.target_speed,
                        speed_cap=min(speed_caps) if speed_caps else None,
                        leading=(
                            f"{step.leading_vehicle_id}@{step.leading_distance:.1f}m/"
                            f"{step.leading_speed * 3.6:.1f}kph"
                            if step.leading_vehicle_id is not None
                            and step.leading_distance is not None
                            and step.leading_speed is not None
                            else "-"
                        ),
                    )
            if not args.no_send:
                send_ego_control(
                    udp=udp,
                    tcp=tcp,
                    fixed_step=args.fixed_step,
                    fixed_control_transport=args.fixed_control_transport,
                    ego_id=ego_id,
                    control=step.control,
                )
                if args.control_mode == "fixed-udp" and tcp is not None:
                    print_fixed_udp_control_event(
                        tcp=tcp,
                        udp=udp,
                        control=step.control,
                    )
                control_started = True
            if tcp is not None:
                try:
                    lockstep_ego, fixed_result, save_result = fixed_step_and_wait_for_ego(
                        tcp=tcp,
                        udp=udp,
                        previous_stamp=current_stamp,
                        timeout=args.lockstep_state_timeout,
                        status_period=status_period,
                        save_data_each_step=(
                            args.save_data_each_step or transition_save_data_required
                        ),
                        ego_candidates=ego_candidates,
                        expected_scenario_name=current_scenario_name,
                    )
                except (
                    ScenarioBoundaryDetected,
                    ActiveScenarioChanged,
                    FixedAckUdpWaitTimeout,
                ) as exc:
                    if handle_scenario_event(tcp, exc):
                        continue
                    raise
            control_steps += 1
            if trace_log.enabled:
                trace_log.write(build_runtime_trace_record(ego=ego))

            if (
                args.control_mode != "fixed-udp"
                and (variable_live_status or loop_start >= next_status)
            ):
                lead_text = (
                    f"{step.leading_vehicle_id}@{step.leading_distance:.1f}m/"
                    f"{step.leading_speed * 3.6:.1f}kph"
                    if step.leading_vehicle_id is not None
                    and step.leading_distance is not None
                    and step.leading_speed is not None
                    else "-"
                )
                if args.control_mode in dashboard_udp_modes:
                    speed_caps = [
                        value
                        for value in (step.map_speed_limit, step.curve_speed_cap)
                        if value is not None
                    ]
                    cap_text = (
                        f"{min(speed_caps) * 3.6:.1f}kph" if speed_caps else "-"
                    )
                    diagnostics = ""
                    if udp.invalid_packets:
                        diagnostics += f" WARNING invalid={udp.invalid_packets}"
                    if udp.rx_chunks:
                        diagnostics += f" chunks={udp.rx_chunks}"
                    status_text = (
                        "variable_udp: "
                        f"ego={ego.id} pos=({ego.x:.2f},{ego.y:.2f}) "
                        f"yaw={ego.yaw_deg:.1f} "
                        f"speed={step.current_speed * 3.6:.1f}"
                        f"->{step.target_speed * 3.6:.1f}kph "
                        f"cap={cap_text} "
                        f"control=(T={step.control.throttle:.2f} "
                        f"B={step.control.brake:.2f} S={step.control.steer:.2f}) "
                        f"applied=(T={ego.throttle:.2f} "
                        f"B={ego.brake:.2f} S={ego.steer:.2f}) "
                        f"lead={lead_text} "
                        f"net=(rx={udp.rx_records} tx={udp.tx_packets})"
                        f"{diagnostics}"
                    )
                    if variable_live_status:
                        render_variable_live_status(
                            [
                                f"[ {args.control_mode} live ]",
                                f"Ego             : {ego.id}",
                                f"Position        : ({ego.x:.2f}, {ego.y:.2f})",
                                f"Yaw             : {ego.yaw_deg:.1f} deg",
                                (
                                    f"Speed           : {step.current_speed * 3.6:.1f} "
                                    f"-> {step.target_speed * 3.6:.1f} kph"
                                ),
                                f"Speed cap       : {cap_text}",
                                (
                                    "Control command : "
                                    f"T={step.control.throttle:.2f} "
                                    f"B={step.control.brake:.2f} "
                                    f"S={step.control.steer:.2f}"
                                ),
                                (
                                    "Applied control : "
                                    f"T={ego.throttle:.2f} "
                                    f"B={ego.brake:.2f} S={ego.steer:.2f}"
                                ),
                                f"Leading vehicle : {lead_text}",
                                (
                                    f"VehicleInfo RX  : {udp.rx_records}"
                                    f"{diagnostics}"
                                ),
                                f"ManualControl TX: {udp.tx_packets}",
                            ]
                        )
                    else:
                        print(status_text)
                        next_status = loop_start + status_period
                    elapsed = time.monotonic() - loop_start
                    if tcp is None and elapsed < period:
                        time.sleep(period - elapsed)
                    continue
                speed_limit_text = (
                    f"{step.map_speed_limit:.2f}" if step.map_speed_limit is not None else "-"
                )
                curve_cap_text = (
                    f"{step.curve_speed_cap:.2f}" if step.curve_speed_cap is not None else "-"
                )
                print(
                    "udp_vehicle_info: ego={ego_id} raw={raw_id} "
                    "pos=({x:.2f},{y:.2f}) "
                    "actors={actors} "
                    "yaw={yaw:.1f} speed={speed:.2f}mps({speed_kph:.1f}kph)/{target:.2f} "
                    "lead={lead} "
                    "raw_v=({vx:.2f},{vy:.2f},{vz:.2f}) "
                    "limit={limit} curve={curve} "
                    "cmd=(thr={thr:.2f}, brk={brk:.2f}, steer={steer:.2f}) "
                    "applied=(thr={athr:.2f}, brk={abrk:.2f}, steer={asteer:.2f}) "
                    "udp_rx={rx} chunks={chunks} invalid={invalid} "
                    "control_udp_tx={tx} route={route_points}".format(
                        ego_id=ego.id,
                        raw_id=ego.raw_id,
                        x=ego.x,
                        y=ego.y,
                        actors=len(udp.states),
                        yaw=ego.yaw_deg,
                        speed=step.current_speed,
                        speed_kph=step.current_speed * 3.6,
                        target=step.target_speed,
                        lead=lead_text,
                        vx=ego.velocity_x,
                        vy=ego.velocity_y,
                        vz=ego.velocity_z,
                        limit=speed_limit_text,
                        curve=curve_cap_text,
                        thr=step.control.throttle,
                        brk=step.control.brake,
                        steer=step.control.steer,
                        athr=ego.throttle,
                        abrk=ego.brake,
                        asteer=ego.steer,
                        rx=udp.rx_records,
                        chunks=udp.rx_chunks,
                        invalid=udp.invalid_packets,
                        tx=udp.tx_packets,
                        route_points=step.route_points,
                    )
                )
                next_status = loop_start + status_period

            elapsed = time.monotonic() - loop_start
            if tcp is None and elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        finish_variable_live_status()
        dashboard = getattr(tcp, "_fixed_udp_dashboard", None)
        if isinstance(dashboard, FixedUdpDashboard):
            dashboard.finish()
        print("stopped by user")
    except ScenarioBoundaryDetected as exc:
        finish_variable_live_status()
        dashboard = getattr(tcp, "_fixed_udp_dashboard", None)
        if isinstance(dashboard, FixedUdpDashboard):
            dashboard.finish()
        print(f"scenario_boundary_detected: {exc}")
        if args.exit_on_scenario_end:
            return 0
        return 2
    except (OSError, MoraiTcpError, RuntimeError) as exc:
        finish_variable_live_status()
        dashboard = getattr(tcp, "_fixed_udp_dashboard", None)
        if isinstance(dashboard, FixedUdpDashboard):
            dashboard.finish()
        print(f"ERROR: {exc}")
        return 2
    finally:
        finish_variable_live_status()
        dashboard = getattr(tcp, "_fixed_udp_dashboard", None)
        if isinstance(dashboard, FixedUdpDashboard):
            dashboard.finish()
        if not args.no_send and control_started:
            try:
                send_ego_control(
                    udp=udp,
                    tcp=tcp,
                    fixed_step=args.fixed_step,
                    fixed_control_transport=args.fixed_control_transport,
                    ego_id=ego_id,
                    control=EgoManualControl(),
                )
            except Exception:
                pass
        if tcp is not None:
            tcp.close()
        udp.close()
        trace_log.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
