# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
TOOLS_DIR = ROOT_DIR / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from pdm_morai_prepare_suite import (  # noqa: E402
    DEFAULT_OUTPUT_ROOT,
    PrepareError,
    generate_suite,
    parse_xosc,
    print_report,
)


@dataclass(frozen=True)
class PreparedSuite:
    xosc_path: Path
    suite_path: Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate PDM-MORAI runnable suites from variation XOSC files."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--gui", action="store_true", help="Generate a GUI SIM validation suite")
    mode.add_argument("--headless", action="store_true", help="Generate a headless-compatible suite and print TCP status-gated run command")
    parser.add_argument("--xosc", required=True, help="Input XOSC file or directory")
    parser.add_argument("--output-root", default=None, help="Generated suite output root")
    parser.add_argument("--ego-entity", default=None, help="Explicit Ego ScenarioObject name")
    parser.add_argument(
        "--manual-control-entity",
        default=None,
        help="Runtime Manual Control target override; defaults to detected Ego",
    )
    parser.add_argument("--rx-port", type=int, default=9097, help="UDP Vehicle Info receive port")
    parser.add_argument("--tx-port", type=int, default=9091, help="UDP Manual Control port")
    parser.add_argument("--feedback-hz", type=int, default=30)
    parser.add_argument("--control-hz", type=int, default=60)
    parser.add_argument(
        "--mgeo-mode",
        choices=["reference", "copy"],
        default="reference",
        help="reference keeps generated suites light; copy embeds MGeo in each generated suite",
    )
    parser.add_argument("--fixed-target-fps", type=int, default=50, help="Reserved for later headless fixed-step flow")
    parser.add_argument("--physics-delta-time", type=int, default=20, help="Reserved for later headless fixed-step flow")
    parser.add_argument("--steps", type=int, default=1000, help="Reserved for later headless fixed-step flow")
    parser.add_argument("--headless-runner", default=None, help="Reserved for later headless runner flow")
    parser.add_argument("--run-headless", action="store_true", help="Reserved for later headless runner flow")
    parser.add_argument("--dry-run", action="store_true", help="Parse and print without writing files")
    return parser


def iter_xosc_inputs(path: Path) -> list[Path]:
    if path.is_file():
        if path.suffix.lower() != ".xosc":
            raise PrepareError(f"Input file is not an XOSC: {path}")
        return [path]
    if path.is_dir():
        files = sorted(path.rglob("*.xosc"))
        if not files:
            raise PrepareError(f"No XOSC files found under directory: {path}")
        return files
    raise PrepareError(f"XOSC input does not exist: {path}")


def resolve_output_root(args: argparse.Namespace) -> Path:
    if args.output_root:
        return Path(args.output_root).expanduser().resolve(strict=False)
    return DEFAULT_OUTPUT_ROOT.resolve(strict=False)


def print_next_commands(prepared: PreparedSuite, headless: bool, tcp_port: int = 20000) -> None:
    print("next_controller_command:")
    if headless:
        print(
            "  .\\tools\\run_pdm_morai_control.ps1 "
            f'--suite "{prepared.suite_path}" --headless --tcp-port {tcp_port} --status-hz 2'
        )
        print("headless_action:")
        print("  Start MORAI headless runtime first, then run the controller command above.")
    else:
        print(
            "  .\\tools\\run_pdm_morai_control.ps1 "
            f'--suite "{prepared.suite_path}" --status-hz 2'
        )
        print("gui_action:")
        print("  Open this msuite in MORAI UI, press Play, then run the controller command above.")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.gui and not args.headless:
        args.gui = True
    if args.run_headless:
        print("ERROR: automatic headless runner launch is deferred. Generate the suite, start headless runtime manually, then run the controller.")
        return 2

    try:
        output_root = resolve_output_root(args)
        xosc_files = iter_xosc_inputs(Path(args.xosc).expanduser().resolve(strict=False))
        prepared_suites: list[PreparedSuite] = []

        print(f"mode: {'headless' if args.headless else 'gui'}")
        print(f"xosc_count: {len(xosc_files)}")
        print(f"output_root: {output_root}")

        blocked = False
        for xosc_path in xosc_files:
            print("")
            print(f"== {xosc_path} ==")
            info = parse_xosc(
                xosc_path,
                args.ego_entity,
                output_root,
                suite_path=None,
                in_place=False,
            )
            print_report(
                info,
                args.rx_port,
                args.tx_port,
                args.feedback_hz,
                args.control_hz,
                args.dry_run,
                args.manual_control_entity,
            )
            if info.blocking_errors:
                blocked = True
                continue
            if not args.dry_run:
                generate_suite(
                    info,
                    args.rx_port,
                    args.tx_port,
                    args.feedback_hz,
                    args.control_hz,
                    False,
                    args.manual_control_entity,
                    args.mgeo_mode,
                )
                prepared = PreparedSuite(xosc_path=xosc_path, suite_path=info.target_suite_path)
                prepared_suites.append(prepared)
                print(f"generated_msuite: {prepared.suite_path}")
                print_next_commands(prepared, args.headless)

        if blocked:
            return 2
    except (OSError, ET.ParseError, ValueError, PrepareError) as exc:
        print(f"ERROR: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
