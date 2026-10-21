# Release preparation and evidence

This guide owns the release procedure and validation record. Use
[development instructions](../tests/README.md) for exact setup and commands, and
[actuator operation](ACTUATOR.md) for commissioning and physical acceptance.

## Prepare the source

Review current stable ESPHome changes under the baseline procedure. A routine
standards change does not itself require an upgrade. Keep `uv.lock` exact and
consistent with project metadata. Use local component source while testing.
Pin a deployed external component to an immutable revision and record it.

Inspect Git status and the intended changes. A release must identify the commit
and any source changes. Prefer a clean committed tree for a published firmware.
For a deliberate development export, record its exact archive SHA256 and label
it as an export. Git identity alone does not describe uncommitted input.

## Required checks

Run local validation, then the schema, generation, and target stages for all
three reference examples. Build the site configuration for site actuator changes
or a site release. Use the runner's synthetic configurations for shared logs.
Firmware setup/build needs explicit toolchain provisioning permission in the
command; it never uploads firmware. Keep Python environment setup separate.

Record KW, unrepresented entities/operations, and GWG write coverage as gaps
unless suitable additional checks are actually run. For contract changes, use a
matching optimizer checkout and the bounded joint status check. It does not
establish complete command-path acceptance. Required unavailable checks leave a
release's validation incomplete; do not convert them into success or historical
claims.

## Record and package

Record these values with each candidate:

- Date, source commit, changes, and source archive checksum when applicable.
- Python, `uv`, locked ESPHome/PyYAML, and host compiler versions.
- Configuration and sanitized input hash; board, protocol, framework, target
  compiler/toolchain, and all external source revisions.
- Local, schema, generation, target, joint, and attended acceptance results,
  including failed or unavailable checks and their limits.
- Firmware output filenames and SHA256 hashes; identify the installable image
  and preserve the matching source and configuration.

`uv.lock` identifies Python dependencies. It does not identify all downloaded
framework/toolchain inputs; retain build metadata or logs that identify those
versions. The runner prints successful synthetic target logs and firmware hashes
before removing temporary builds. Preserve that output with the candidate record.
Use the installation's actual configuration for an authorized site release;
synthetic validation firmware is not a commissioned deployment image.

Exclude secrets, `.env*`, `.private/`, Git internals, virtual environments,
`.esphome`/`.pio` caches, and private device reports from shared source archives.
Store generated local records under ignored `reports/`; move a sanitized dated
record into a deliberate evidence location only when it is meant to be tracked.
Keep current procedures separate from [historical safety evidence](../SAFETY_UPGRADE.md).
Publishing, flashing, deployment, and live operations require a user request for
that work.
