# Development and validation

Run commands from the project root. This guide owns setup, local checks, and
firmware checks. [Actuator operation](../docs/ACTUATOR.md) and
[release records](../docs/RELEASING.md) have separate maintained guides.

## Environment

Install `uv` and Python `>=3.12,<3.14`, as declared in
[pyproject.toml](../pyproject.toml). Provide `g++` with C++17 support (GCC or Apple
Clang). Setup is explicit:

```sh
uv sync --locked
.venv/bin/python --version
.venv/bin/python -m esphome version
```

[uv.lock](../uv.lock) selects ESPHome 2026.9.1 and PyYAML 6.0.3. PyYAML is an
explicit host dependency in the development group. The optional
[requirements-dev.txt](../requirements-dev.txt) supports host-only use; it does
not replace the supported complete locked environment. The dev container also
uses `uv sync --locked`. Validation does not install or repair Python packages.

## Baseline review

Before framework API changes and release preparation:

1. Compare the lock with current stable [release notes](https://esphome.io/changelog/).
   Check relevant [documentation](https://esphome.io/) or
   [ESPHome source](https://github.com/esphome/esphome).
2. Check Python, API, board, framework, and toolchain requirements. Check the
   framework contract before changing a host substitute. Do not retain an
   obsolete API to support an unverified compatibility range.
3. Update the baseline when the work needs it or a routine update is practical.
   A new release does not require an immediate upgrade. Record the decision and
   limits if the existing lock remains suitable.
4. Use `uv lock --upgrade-package esphome` for a deliberate update, inspect the
   changes, then run `uv sync --locked`. Update Python constraints and setup
   instructions together when required. Run local and affected firmware checks.
5. Record date, stable release checked, exact tested Python/ESPHome versions,
   board, framework/toolchain versions, results, and unavailable checks. A
   dependency constraint or lock alone does not establish compatibility.

## Local validation

```sh
.venv/bin/python scripts/validate.py
```

The command checks the interpreter and locked direct dependency versions,
`uv lock --check --offline`, read-only `uv sync --locked --check --offline`
against the selected interpreter environment, dependency health, Python syntax, host scenarios,
ID mutation controls, validation-tool regressions, document links, and Git
working-tree/index whitespace. It rejects Python optimization and clears
inherited test filters and Python/Git source overrides for child processes.
C++ assertions remain enabled with `-UNDEBUG`. It reports the source identity and
changes. It accepts a source export using an external prepared interpreter;
record the export checksum because Git identity is unavailable there.

Affected checks can run directly before the complete local check:

```sh
.venv/bin/python tests/host_safety_checks.py
.venv/bin/python tests/check_id_reference_regression.py
.venv/bin/python tests/check_validation_tools.py
```

The host suite compiles production C++ and actuator lambdas from
[heating_optolink.yaml](../heating_optolink.yaml) with ESPHome substitutes, fake
boiler I/O, and a limited renderer. It checks restoration, command handling,
watchdog behavior, commissioning, and wire evidence. The ID check changes each
of four component pointer aliases in public temporary source copies. It requires
the intended pointer-access compile error under GCC or Clang, then runs the
corrected suite. Missing compilers and unrelated compile errors cannot pass.

These checks do not run ESPHome schema validation, its code generator, its real
scheduler, or a target toolchain. A passing host suite does not establish
physical boiler compatibility. Report failed, skipped required, empty, or
unavailable checks as incomplete validation. Do not report old results as new.

## Firmware checks

The runner uses temporary configurations, local component source, and synthetic
secrets. It never reads the checkout's `secrets.yaml`, uploads firmware, or
connects to a device. Schema output is captured because resolved configurations
can otherwise disclose credentials. Build inputs and firmware outputs have
reported SHA256 hashes. Successful synthetic target logs are printed to preserve
framework/compiler details before temporary files are removed. PlatformIO builds
also record installed framework/tool packages with its read-only package query.
Save the command output as release evidence when needed.

Select each configuration explicitly; repeat `--configuration` as needed:

| Selection | Source | Board | Protocol | Framework |
| --- | --- | --- | --- | --- |
| `esp32-p300` | [ESP32 example](../example-esp32.yaml) | `esp32doit-devkit-v1` | P300 | ESP-IDF |
| `esp8266-p300` | [ESP8266 example](../example-esp8266.yaml) | `nodemcuv2` | P300 | Arduino |
| `esp32-gwg` | [GWG example](../example-gwg-protocol.yaml) | `esp32-s2-saola-1` | GWG | ESP-IDF |
| `site-p300` | [Site actuator](../heating_optolink.yaml) | `esp32-c3-devkitm-1` | P300 | ESP-IDF |

```sh
.venv/bin/python scripts/validate.py --firmware schema --configuration esp32-p300
.venv/bin/python scripts/validate.py --firmware generate --allow-toolchain-setup --configuration esp32-p300
```

Each command first runs local validation. `schema` runs ESPHome `config`.
`generate` adds `compile --only-generate` and can download C++ library inputs,
so it also needs the explicit setup opt-in. Generation does not establish target
compilation. For affected firmware inputs, finish with the explicit setup/build:

```sh
.venv/bin/python scripts/validate.py --firmware build --allow-toolchain-setup --configuration esp32-p300
```

**Firmware preparation is explicit.** The setup opt-in permits ESPHome's normal
C++ library/framework/toolchain downloads and repair. It does not update `uv.lock` or the
project Python environment. The supported ESPHome command has no strict offline,
no-repair guarantee; use prepared tools in an externally network-restricted
build environment when that guarantee is required. Missing tools, blocked
network, unsupported host architecture, and build errors mean target validation
is incomplete. The runner does not turn those conditions into skipped passes.

Release preparation requires local validation and builds for all three reference
examples (`esp32-p300`, `esp8266-p300`, `esp32-gwg`). Also build `site-p300` for
actuator changes or a site firmware release. These examples do not cover KW or
all entities and protocol operations. GWG writes remain unverified; a build does
not establish write safety. Add suitable configurations when changing uncovered
behavior and report the gap. Record schema, generation, and target results
separately. Do not change the source examples to select a remote component while
validating local changes.

## Documents and joint checks

Documents only need the standard-library command:

```sh
.venv/bin/python scripts/validate.py --documents-only
```

It checks required guides and local Markdown file links, excluding environments,
private data, `.esphome`, `.pio`, and generated reports. It does not check anchors
or remote URLs, install dependencies, or execute tests. Check changed command
syntax manually. Git failures remain failures; exports report the missing Git
whitespace check.

Use the optimizer's maintained runner with an explicit checkout path for bounded
joint status checks, as described in the [actuator guide](../docs/ACTUATOR.md).
A sibling checkout is not a prerequisite for ordinary component work. Complete
command-path checks, real scheduling, target compilation, and attended physical
acceptance remain distinct evidence.
