# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
TOOLS_DIR = ROOT_DIR / "tools"
MIN_PYTHON = (3, 10)


def add_import_paths() -> None:
    for path in (ROOT_DIR, TOOLS_DIR):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))


def print_check(ok: bool, label: str, detail: str = "") -> None:
    marker = "OK" if ok else "FAIL"
    suffix = f" - {detail}" if detail else ""
    print(f"[{marker}] {label}{suffix}")


def check_file(path: Path, label: str) -> bool:
    ok = path.exists()
    print_check(ok, label, str(path))
    return ok


def check_import(module_name: str) -> bool:
    try:
        module = importlib.import_module(module_name)
    except Exception as exc:
        print_check(False, f"import {module_name}", f"{type(exc).__name__}: {exc}")
        return False
    version = getattr(module, "__version__", "")
    print_check(True, f"import {module_name}", str(version) if version else "")
    return True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check the PDM-MORAI runtime environment.")
    parser.add_argument(
        "--mgeo-dir",
        required=True,
        help="Authorized external MGeo directory containing link_set.json",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    add_import_paths()
    failures = 0

    py_ok = sys.version_info >= MIN_PYTHON
    print_check(
        py_ok,
        "Python version",
        f"{sys.version.split()[0]} required>={MIN_PYTHON[0]}.{MIN_PYTHON[1]}",
    )
    failures += 0 if py_ok else 1

    for module_name in (
        "numpy",
        "pdm_morai_prepare_suite",
        "pdm_morai_xosc",
        "pdm_morai_control",
    ):
        failures += 0 if check_import(module_name) else 1

    mgeo_dir = Path(args.mgeo_dir).expanduser().resolve(strict=False)
    if not (mgeo_dir / "link_set.json").is_file() and (
        mgeo_dir / "MGeo" / "link_set.json"
    ).is_file():
        mgeo_dir = mgeo_dir / "MGeo"

    required_files = (
        (ROOT_DIR / "tools" / "run_pdm_morai_xosc.ps1", "XOSC runner"),
        (ROOT_DIR / "tools" / "run_pdm_morai_control.ps1", "control runner"),
        (ROOT_DIR / "requirements-pdm-morai.txt", "PDM-MORAI requirements"),
        (mgeo_dir, "external MGeo root"),
        (mgeo_dir / "link_set.json", "MGeo link_set"),
        (mgeo_dir / "node_set.json", "MGeo node_set"),
        (mgeo_dir / "lane_boundary_set.json", "MGeo lane_boundary_set"),
    )
    for path, label in required_files:
        failures += 0 if check_file(path, label) else 1

    if failures:
        print("")
        print("Environment check failed. Install runtime dependency first:")
        print("  py -3 -m pip install -r requirements-pdm-morai.txt")
        return 2

    print("")
    print("PDM-MORAI basic environment check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
