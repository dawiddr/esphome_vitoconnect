"""Validate prepared VitoConnect source without installing Python packages."""
from __future__ import annotations

import argparse
import ast
import importlib.metadata as metadata
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import tomllib

from validate_documents import check_documents, excluded_path

ROOT = Path(__file__).resolve().parents[1]


def clean_environment() -> dict[str, str]:
    """Clear inherited filters, interpreter overrides, and Git source selection."""
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(("PYTEST_", "GIT_")) and key not in {
               "PYTHONPATH", "PYTHONHOME", "PYTHONOPTIMIZE", "PYTHONSTARTUP",
           }}
    env.update(PYTHONDONTWRITEBYTECODE="1", PYTHONNOUSERSITE="1")
    return env


def require_assertions() -> None:
    if sys.flags.optimize:
        raise RuntimeError("Validation must run without Python optimization")


def is_git_checkout(root: Path = ROOT) -> bool:
    marker = (root / ".git").exists() or (root / ".git").is_symlink()
    if not shutil.which("git"):
        if marker:
            raise RuntimeError("Git is required to inspect this checkout")
        return False
    result = subprocess.run(["git", "-C", str(root), "rev-parse", "--show-toplevel"],
                            env=clean_environment(), text=True, capture_output=True)
    if result.returncode:
        if not marker and "not a git repository" in result.stderr.lower():
            return False
        raise RuntimeError(f"Cannot inspect Git source: {result.stderr.strip()}")
    if Path(result.stdout.strip()).resolve() != root.resolve():
        if marker:
            raise RuntimeError("Git root differs from project root")
        return False
    return True


def describe_source(root: Path = ROOT) -> None:
    if not is_git_checkout(root):
        print("Revision: unavailable (source export; record its archive checksum)", flush=True)
        return
    for label, arguments in (("Revision", ["rev-parse", "HEAD"]),
                             ("Working-tree changes", ["status", "--short"])):
        result = subprocess.run(["git", "-C", str(root), *arguments],
                                env=clean_environment(), capture_output=True, text=True, check=True)
        print(f"{label}: {result.stdout.strip() or 'clean'}", flush=True)


def document_checks(root: Path = ROOT) -> None:
    documents, links = check_documents(root)
    print(f"PASS: {documents} Markdown files; {links} local file links (not anchors or URLs)", flush=True)
    if is_git_checkout(root):
        for arguments in (["diff", "--check"], ["diff", "--cached", "--check"]):
            subprocess.run(["git", "-C", str(root), *arguments],
                           env=clean_environment(), check=True)
        print("PASS: Git working-tree and index whitespace", flush=True)
    else:
        print("Git whitespace check: unavailable in source export", flush=True)


def check_environment(root: Path = ROOT) -> None:
    require_assertions()
    project = tomllib.loads((root / "pyproject.toml").read_text())
    lock = tomllib.loads((root / "uv.lock").read_text())
    if not (3, 12) <= sys.version_info[:2] < (3, 14):
        raise RuntimeError("Expected Python >=3.12,<3.14")
    if Path(sys.prefix).resolve() == Path(sys.base_prefix).resolve():
        raise RuntimeError("A prepared virtual environment is required; run uv sync --locked")
    host_pins = [re.fullmatch(r"(?i:pyyaml)==([A-Za-z0-9.+-]+)", entry)
                 for entry in project.get("dependency-groups", {}).get("dev", [])]
    host_pins = [match[1] for match in host_pins if match]
    packages = {package["name"]: package for package in lock["package"]}
    if len(host_pins) != 1 or host_pins[0] != packages["pyyaml"]["version"]:
        raise RuntimeError("PyYAML must have one exact development pin matching uv.lock")
    for name in ("esphome", "pyyaml"):
        expected = packages[name]["version"]
        installed = metadata.version(name)
        if installed != expected:
            raise RuntimeError(f"Expected {name}=={expected}; found {installed}; run uv sync --locked")
        print(f"Dependency: {name}=={installed}", flush=True)
    uv = shutil.which("uv")
    compiler = shutil.which("g++")
    if not uv or not compiler:
        raise RuntimeError("uv and g++ with C++17 support are required")
    print(f"Interpreter: {sys.executable} (Python {sys.version.split()[0]})", flush=True)
    version = subprocess.run([compiler, "--version"], capture_output=True, text=True,
                             check=True, env=clean_environment())
    print(f"Compiler: {version.stdout.splitlines()[0]}", flush=True)
    with tempfile.TemporaryDirectory(prefix="vito-dependency-check-") as cache:
        env = {key: value for key, value in clean_environment().items()
               if not key.startswith("UV_")}
        env.update(UV_CACHE_DIR=cache, UV_PROJECT_ENVIRONMENT=sys.prefix)
        subprocess.run([uv, "lock", "--check", "--offline"], cwd=root, env=env, check=True)
        subprocess.run([uv, "sync", "--locked", "--check", "--offline", "--all-groups",
                        "--python", sys.executable], cwd=root, env=env, check=True)
        subprocess.run([uv, "pip", "check", "--python", sys.executable], cwd=root, env=env, check=True)
    print("PASS: current lock, synchronized environment, direct pins, and dependency health", flush=True)


def check_host_result(result: subprocess.CompletedProcess[str]) -> int:
    """Require executed host scenarios and one parseable production status fixture."""
    if result.returncode:
        raise RuntimeError(f"Host suite failed:\n{result.stdout}\n{result.stderr}")
    cases = re.findall(r"^PASS: .+", result.stdout, re.MULTILINE)
    fixtures = re.findall(r"^STATUS_FIXTURE=(.+)$", result.stdout, re.MULTILINE)
    if not cases or len(fixtures) != 1:
        raise RuntimeError("Host result requires executed scenarios and exactly one status fixture")
    try:
        fixture = json.loads(fixtures[0])
    except ValueError as exc:
        raise RuntimeError("Host status fixture must be valid JSON") from exc
    if not isinstance(fixture, dict) or not fixture:
        raise RuntimeError("Host status fixture must be a nonempty JSON object")
    return len(cases)


def local_checks(root: Path = ROOT) -> None:
    check_environment(root)
    for directory in ("components", "scripts", "tests"):
        for path in (root / directory).rglob("*.py"):
            if not excluded_path(path.relative_to(root)):
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    print("PASS: Python source syntax", flush=True)
    result = subprocess.run([sys.executable, str(root / "tests/host_safety_checks.py"), str(root)],
                            cwd=root, env=clean_environment(), capture_output=True, text=True, timeout=120)
    count = check_host_result(result)
    print(f"PASS: {count} executed host scenario groups; C++ assertions enabled", flush=True)
    subprocess.run([sys.executable, str(root / "tests/check_id_reference_regression.py"), str(root)],
                   cwd=root, env=clean_environment(), check=True, timeout=600)
    subprocess.run([sys.executable, str(root / "tests/check_validation_tools.py")],
                   cwd=root, env=clean_environment(), check=True, timeout=120)
    document_checks(root)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--documents-only", action="store_true",
                        help="Check local document links and Git whitespace with only Python stdlib")
    parser.add_argument("--firmware", choices=("schema", "generate", "build"),
                        help="Explicit additional firmware stage; see tests/README.md")
    parser.add_argument("--configuration", action="append",
                        choices=("esp32-p300", "esp8266-p300", "esp32-gwg", "site-p300"))
    parser.add_argument("--allow-toolchain-setup", action="store_true",
                        help="Explicitly allow ESPHome generation/build to provision firmware dependencies")
    args = parser.parse_args(argv)
    if args.documents_only and (args.firmware or args.configuration or args.allow_toolchain_setup):
        parser.error("--documents-only cannot select firmware checks")
    if args.firmware and not args.configuration:
        parser.error("--firmware requires one or more --configuration selections")
    if args.configuration and not args.firmware:
        parser.error("--configuration requires --firmware")
    if args.firmware in {"generate", "build"} and not args.allow_toolchain_setup:
        parser.error("generation/build requires explicit --allow-toolchain-setup; ESPHome may provision dependencies")
    if args.allow_toolchain_setup and args.firmware not in {"generate", "build"}:
        parser.error("--allow-toolchain-setup requires --firmware generate or build")
    require_assertions()
    describe_source()
    if args.documents_only:
        document_checks()
        print("PASS: document validation; dependencies and tests were not run", flush=True)
        return
    local_checks()
    if args.firmware:
        from firmware_checks import run_firmware
        for configuration in dict.fromkeys(args.configuration):
            run_firmware(ROOT, configuration, args.firmware)
    print("PASS: local validation; firmware stages and physical acceptance are separate", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, ValueError, KeyError, subprocess.SubprocessError,
            metadata.PackageNotFoundError) as exc:
        print(f"FAIL: validation incomplete: {exc}", file=sys.stderr)
        sys.exit(1)
