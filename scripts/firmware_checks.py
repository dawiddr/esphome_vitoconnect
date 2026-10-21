"""Build temporary public-source configurations with synthetic credentials only."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

from validate import clean_environment

CONFIGURATIONS = {
    "esp32-p300": ("example-esp32.yaml", "esp32doit-devkit-v1", "P300", "esp-idf"),
    "esp8266-p300": ("example-esp8266.yaml", "nodemcuv2", "P300", "arduino"),
    "esp32-gwg": ("example-gwg-protocol.yaml", "esp32-s2-saola-1", "GWG", "esp-idf"),
    "site-p300": ("heating_optolink.yaml", "esp32-c3-devkitm-1", "P300", "esp-idf"),
}
SECRETS = {
    "wifi_ssid": "vito-validation-network", "wifi_password": "synthetic-validation-password",
    "wifi_ap_password": "synthetic-validation-ap", "esphome_ota_password": "synthetic-validation-ota",
    "esphome_api_encryption_key": "MDEyMzQ1Njc4OWFiY2RlZjAxMjM0NTY3ODlhYmNkZWY=",
}


def public_copy(source: Path, target: Path, root: Path) -> None:
    relative = source.relative_to(root)
    if not source.is_file() or any((root / part).is_symlink()
                                  for part in (relative, *relative.parents) if part != Path(".")):
        raise RuntimeError(f"Missing or linked firmware input: {relative}")
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)


def prepare_configuration(root: Path, target: Path, name: str) -> Path:
    """Keep YAML tags/lambdas, replace external sources, and never read secrets.yaml."""
    source = root / CONFIGURATIONS[name][0]
    public_copy(source, target / "configuration.yaml", root)
    text = source.read_text(encoding="utf-8")
    if "!include" in text:
        raise RuntimeError("Validation inputs must declare included files explicitly")
    text, count = re.subn(
        r"^external_components:.*?(?=^[^\s#]|\Z)",
        "external_components:\n  - source: components\n    components: [vitoconnect]\n\n",
        text, flags=re.MULTILINE | re.DOTALL,
    )
    if count != 1:
        raise RuntimeError("Expected one external component source block")
    keys = set(re.findall(r"!secret\s+([A-Za-z_][A-Za-z0-9_]*)", text))
    if keys - SECRETS.keys():
        raise RuntimeError("Unknown secret names need explicit synthetic validation values")
    (target / "secrets.yaml").write_text(json.dumps({key: SECRETS[key] for key in sorted(keys)}))
    # The current optional site helper is the only extra header required by YAML.
    includes = re.findall(r"^  includes:\n((?:[ \t]+[^\n]*\n)+)", text, re.MULTILINE)
    for block in includes:
        for header in re.findall(r"^    - ([A-Za-z_][A-Za-z0-9_]*\.h)\s*$", block, re.MULTILINE):
            public_copy(root / header, target / header, root)
    for path in (root / "components/vitoconnect").rglob("*"):
        if (path.suffix in {".h", ".cpp", ".py"} and path.is_file()
                and not any(part.startswith(".") or part == "__pycache__"
                            for part in path.relative_to(root).parts)):
            public_copy(path, target / path.relative_to(root), root)
    path = target / "configuration.yaml"
    path.write_text(text, encoding="utf-8")
    return path


def target_package_metadata(directory: Path) -> None:
    """Record installed PlatformIO packages omitted from normal build output."""
    for project in sorted(directory.rglob("platformio.ini")):
        result = subprocess.run(
            [sys.executable, "-m", "platformio", "pkg", "list", "--project-dir", str(project.parent), "--verbose"],
            env=clean_environment(), capture_output=True, text=True, check=True, timeout=120,
        )
        if not result.stdout.strip():
            raise RuntimeError("PlatformIO package metadata was empty")
        print("Installed target packages (read-only query):", flush=True)
        print(result.stdout, end="" if result.stdout.endswith("\n") else "\n", flush=True)


def run_firmware(root: Path, name: str, stage: str) -> None:
    filename, board, protocol, framework = CONFIGURATIONS[name]
    print(f"Firmware: {name}: {filename}; {board}; {protocol}; {framework}", flush=True)
    if stage in {"generate", "build"}:
        print("Explicit firmware setup: ESPHome may download or repair C++ libraries/framework/toolchains. "
              "No upload or device connection is performed.", flush=True)
    with tempfile.TemporaryDirectory(prefix=f"vito-{name}-") as temp:
        directory = Path(temp)
        configuration = prepare_configuration(root, directory, name)
        digest = hashlib.sha256()
        for path in sorted(path for path in directory.rglob("*") if path.is_file()):
            digest.update(path.relative_to(directory).as_posix().encode() + b"\0" + path.read_bytes())
        print(f"Sanitized firmware input SHA256: {digest.hexdigest()}", flush=True)
        stages = [("schema", ["config", str(configuration)])]
        if stage in {"generate", "build"}:
            stages.append(("generation", ["compile", "--only-generate", str(configuration)]))
        if stage == "build":
            stages.append(("target compilation", ["compile", str(configuration)]))
        for label, arguments in stages:
            result = subprocess.run([sys.executable, "-m", "esphome", *arguments], cwd=directory,
                                    env=clean_environment(), capture_output=True, text=True,
                                    timeout=1800 if label == "target compilation" else 300)
            if result.returncode:
                # All inputs are allowlisted public source and synthetic credentials.
                output = "\n".join((result.stdout + "\n" + result.stderr).splitlines()[-45:])
                raise RuntimeError(f"{name}: {label} failed (synthetic inputs only):\n{output}")
            if label == "target compilation":
                # Preserve actual framework/package/compiler versions before temporary
                # build files are removed. These inputs use synthetic credentials only.
                print(f"BEGIN SYNTHETIC TARGET BUILD LOG: {name}", flush=True)
                print(result.stdout, end="" if result.stdout.endswith("\n") else "\n", flush=True)
                print(result.stderr, end="" if result.stderr.endswith("\n") else "\n", flush=True)
                print(f"END SYNTHETIC TARGET BUILD LOG: {name}", flush=True)
            print(f"PASS: {name}: {label}", flush=True)
        if stage == "build":
            target_package_metadata(directory)
            binaries = [path for path in directory.rglob("*.bin") if path.name.startswith("firmware")]
            if not binaries:
                raise RuntimeError(f"{name}: compiler returned success without firmware output")
            for path in sorted(binaries):
                print(f"Artifact {path.name}: SHA256 {hashlib.sha256(path.read_bytes()).hexdigest()}", flush=True)
            for path in directory.rglob("build_info.json"):
                print(f"Build information: {path.read_text().strip()}", flush=True)
