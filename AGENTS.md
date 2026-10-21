# Agent instructions

## Scope and setup

This project is an ESPHome external component for Viessmann Optolink protocols.
Read [README.md](README.md) and the [test guide](tests/README.md) before work.
The [actuator guide](docs/ACTUATOR.md) owns current commissioning and contract
instructions. The safety upgrade reports contain historical evidence.

Confirm the project root and inspect `git status --short` before editing. Edit
files only when the user requests a change. Preserve unrelated changes. Do not
stage, unstage, or commit unless the user requests it. Use the prepared `.venv`,
or run the explicit setup command `uv sync --locked`. Validation does not install
or repair Python packages.

## Development baseline

Use a recent stable ESPHome release as the development reference. Before
framework API changes and release preparation, compare the version in `uv.lock`
with the current stable release. Check relevant API changes in the official
documentation or ESPHome source. Prefer current, documented APIs.

Keep exact dependency versions in `uv.lock`. Update the baseline when needed for
the work or when a routine update is practical. Each new stable release does not
require an immediate upgrade. Do not add tests that require an obsolete API only
to retain an unverified compatibility range. Report exact tested versions and
limits under the [baseline review procedure](tests/README.md#baseline-review).

## Checks

For code or test changes, run affected checks first, then:

```sh
.venv/bin/python scripts/validate.py
```

This runs prepared-environment checks, Python syntax, the host safety suite, all
four ID mutation controls, validation-tool regressions, documents, and whitespace.
The host compiler keeps C++ assertions enabled. Failed, empty, skipped required,
or unavailable checks mean validation is incomplete.

For documents only, run:

```sh
python3 scripts/validate.py --documents-only
```

For ESPHome API, schema, C++, or YAML changes, also select affected configurations
and run the schema, generation, and target stages in the [test guide](tests/README.md).
Release preparation requires local checks and the documented reference target
matrix. Firmware generation/build requires explicit `--allow-toolchain-setup` because
ESPHome can download C++ library inputs and provision toolchains. General component work does not
require a sibling optimizer checkout. Joint status and physical acceptance are
separate stages. Report each stage and its limits; a host pass is simulated I/O.

## Code boundaries

Keep protocol transport in the Optolink classes, datapoint handling in the
component, and site control policy in the actuator YAML. For an ESPHome API
change, check the actual framework contract and update the host substitute.
Preserve the difference between component pointers and global values in lambda
ID handling. Keep environment setup separate from validation. Do not add a
second cross-repository contract runner; use the optimizer's maintained runner
with an explicit checkout path.

## Control rules to preserve

Preserve command identity, bounded retries, fresh wire readback, persistent
baseline restoration, and explicit operator capture of an unknown baseline.
A cached value or an accepted command is not proof of a completed write.
A known-baseline restoration obligation survives restart, HA loss, and retry
exhaustion. Unknown-baseline commissioning never authorizes a guessed write.
Flashing, deployment, pairing, and live boiler commands require a user request
for that work. Offline validation does not authorize those operations.

## Documentation and release

Keep one current procedure per topic: [tests](tests/README.md),
[actuator operation](docs/ACTUATOR.md), and [release records](docs/RELEASING.md).
Update instructions when commands or behavior change. Link historical reports
as dated evidence, not as current commands or newly verified results. Record the
source commit and changes, or source export checksum; Python/ESPHome, board,
framework/toolchain versions; each result and limit; and firmware output hashes.
Pin external component revisions for a deployed build. Do not silently change
project versions or framework baselines as part of routine standards work.

## Private files

Keep credentials, private site data, environment files, generated firmware,
resolved configuration output, and device reports out of tracked source and
shared logs. Use `.private/` for private local data and `reports/` for generated
reports. `secrets.yaml`, `.env*`, environments, and build caches are ignored.
Temporary host/firmware input copies use explicit public source types and
synthetic secrets; do not copy the whole checkout. Do not open or print local
secrets to prepare validation.
