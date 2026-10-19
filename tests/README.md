# Development and validation

Run commands from the project root. Use a recent stable ESPHome release as the
development reference, with exact Python dependency versions in
[uv.lock](../uv.lock). The lock currently selects ESPHome 2026.9.1 and PyYAML
6.0.3. These are the locked versions; test results must state what was actually
run. A dependency constraint or lock file alone does not establish compatibility.

## Environment

Install `uv` and a Python version allowed by [pyproject.toml](../pyproject.toml)
(`>=3.12,<3.14`). The host checks also need `g++` with C++17 support.

```sh
uv sync --locked
.venv/bin/python --version
.venv/bin/esphome version
```

This environment includes PyYAML for the host checks. The small
[requirements-dev.txt](../requirements-dev.txt) file lists the host test
dependency; it does not pin the complete ESPHome environment. Use `uv.lock` when
you need to repeat validation with the same Python dependencies.

## Baseline review

Before framework API changes and release preparation:

1. Compare the ESPHome version in `uv.lock` with the current stable release.
   Check the official [release notes](https://esphome.io/changelog/) and relevant
   [documentation](https://esphome.io/) or
   [ESPHome source](https://github.com/esphome/esphome).
2. Check Python requirements, component APIs, and the board and framework
   requirements for the proposed version. Check the actual framework contract
   before changing a host substitute. Do not retain an obsolete API only to
   satisfy a test for an unverified compatibility range.
3. Update the lock when needed for the work or when a routine update is
   practical. Each new stable release does not require an immediate upgrade.
   If the existing lock remains suitable, record that decision and any relevant
   compatibility limit. Keep exact versions in the lock; a broad dependency
   constraint is not a repeatable validation environment.
4. For an ESPHome update, use `uv lock --upgrade-package esphome`, inspect the
   dependency changes, then run `uv sync --locked`. If Python requirements
   change, update the project metadata, lock, and environment instructions
   together. Run the host checks and affected firmware checks below.
5. Record the review date, stable version checked, exact tested Python and
   ESPHome versions, board, firmware framework and toolchain versions, and
   validation results. State which checks failed, were skipped, or could not run.

## Host checks

```sh
.venv/bin/python tests/host_safety_checks.py
```

This suite compiles production C++ and the actuator lambdas from
[heating_optolink.yaml](../heating_optolink.yaml). It uses ESPHome substitutes,
fake boiler I/O, and a limited lambda renderer. It checks baseline restoration,
command handling, watchdog behavior, commissioning, and wire readback evidence.

For changes to component ID handling or the host lambda renderer, also run:

```sh
.venv/bin/python tests/check_id_reference_regression.py
```

This check changes each of four component pointer aliases in temporary copies.
It requires a compile failure for each broken alias, then runs the corrected
host suite. It does not modify production files.

Neither check runs ESPHome schema validation, its code generator, its scheduler,
or a target firmware toolchain. A passing host suite does not establish
compatibility with an ESPHome release or with a physical boiler.

## Firmware checks

For ESPHome API, schema, code generation, C++, or YAML changes, validate,
generate, and compile the affected configurations. For example, the supplied
ESP32 configuration uses local component sources:

```sh
.venv/bin/esphome config example-esp32.yaml
.venv/bin/esphome compile --only-generate example-esp32.yaml
.venv/bin/esphome compile example-esp32.yaml
```

Repeat these checks with [example-esp8266.yaml](../example-esp8266.yaml) when
ESP8266 is affected. Release preparation requires host checks and target builds
for the supported boards and affected protocols. The P300 examples do not
exercise all GWG, KW, or entity configurations. Add relevant configurations to
the validation work and record their coverage.

For the site actuator, run the same three commands with `heating_optolink.yaml`
when its configuration or behavior is affected. Use private local secrets or
synthetic values. Configuration output can contain resolved secrets; do not
include it in shared logs. Check that the configuration loads the local
`components` directory. The [GWG example](../example-gwg-protocol.yaml) uses a
remote source and private secrets; use a local validation copy that points to
this checkout before testing component changes.

Report schema validation, code generation, and target compilation separately.
Missing secrets, unavailable dependencies, or missing toolchains mean that the
relevant check is incomplete. A successful `--only-generate` command does not
establish that firmware compiles. A target build does not establish physical
boiler behavior or compatibility with other ESPHome versions.

## Documents and physical checks

For document changes, check changed local links and command syntax, then run
`git diff --check`. Code tests and firmware builds are not required for changes
that affect documents only.

[SAFETY_UPGRADE.md](../SAFETY_UPGRADE.md) contains the joint optimizer checks and
the attended acceptance procedure. Its results and the older
[0.2.0 notes](../SAFETY_UPGRADE_0.2.0.md) are historical records. Record current
results separately. Physical acceptance needs a requested, attended test on the
actual installation. Flashing and live boiler commands require a user request
for that work.
