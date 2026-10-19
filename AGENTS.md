# Agent instructions

## Scope and setup

This project is an ESPHome external component for Viessmann Optolink protocols.
Read [README.md](README.md), including its Development section, before work.
[SAFETY_UPGRADE.md](SAFETY_UPGRADE.md) describes the actuator safety contract.
Its validation results and the older 0.2.0 notes are historical records.

Confirm the project root and inspect `git status --short` before editing. Edit
files only when the user requests a change. Preserve unrelated changes. Do not
stage, unstage, or commit unless the user requests it. Use the existing `.venv`,
or follow the setup commands in the [test guide](tests/README.md).

## Development baseline

Use a recent stable ESPHome release as the development reference. Before
framework API changes and release preparation, compare the version in `uv.lock`
with the current stable release. Check relevant API changes in the official
documentation or ESPHome source.

Prefer current, documented APIs. Keep exact dependency versions in `uv.lock`
for repeatable validation. Update the baseline when needed for the work or when
a routine update is practical. Each new stable release does not require an
immediate upgrade. Do not add tests that require an obsolete API only to retain
an unverified compatibility range.

Report exact tested versions and relevant compatibility limits. Follow the
[baseline review procedure](tests/README.md#baseline-review).

## Checks

For code or test changes, run the affected checks first. Run the host safety
suite for component or actuator changes:

```sh
.venv/bin/python tests/host_safety_checks.py
```

For changes to component ID handling or the host lambda renderer, also run:

```sh
.venv/bin/python tests/check_id_reference_regression.py
```

For ESPHome API, schema, code generation, C++, or YAML changes, also validate,
generate, and compile the affected firmware configurations. Release preparation
requires host checks and target builds. Use local component sources. See the
[test guide](tests/README.md) for commands and configuration requirements.

For documents only, check changed local links and commands, then run
`git diff --check`. Report failed, skipped, or unavailable checks as incomplete
validation. Host tests use framework substitutes and fake boiler I/O; they do
not establish ESPHome or physical boiler compatibility. Report schema checks,
code generation, target builds, and physical checks separately. Do not report
historical results as a new test run.

## Component and actuator rules

- Keep protocol transport in the Optolink classes, datapoint handling in the
  component, and site control policy in the actuator YAML.
- Preserve command identity, bounded retries, fresh readback, persistent baseline
  restoration, and explicit operator capture of an unknown baseline.
- For an ESPHome API change, check the actual framework contract and update the
  relevant host substitute. Preserve the difference between component pointers
  and global values in lambda ID handling.
- Update current instructions when behavior changes. Keep project dependency
  constraints separate from the exact validation versions in `uv.lock`.
- Keep credentials and private site data out of test fixtures and reports. Use
  synthetic values for validation. Flashing, deployment, and live boiler
  commands require a user request for that work.
