> Historical 0.2.0 implementation notes. Superseded by `SAFETY_UPGRADE.md` for 0.2.1 behavior and validation.

# Viessmann joint safety fixes

## Scope and delivery

This patch set addresses the eight findings in the joint review of
`viessmann_setpoint_optimizer` (`45c1d1c`) and `esphome_vitoconnect` (`281373f`).
The optimizer manifest is now **0.2.0**, a local patch version rather than a
published upstream release. Both the actual `heating_optolink.yaml` site
configuration and the optimizer's **separate** reference actuator package were
updated. The site configuration does not include the reference package; do not
add it alongside the existing site actuator.

**Unattended operation now requires a new `actuator_status_entity` mapping.**
Old unattended configurations without that mapping fail closed. Existing entity
names and the narrow target/arm/heartbeat/restore control surface are otherwise
preserved. No device was flashed and no Home Assistant instance was changed.

## Findings addressed

| Review finding | Implemented change | Regression coverage |
| --- | --- | --- |
| 1. Watchdog fallback abandoned after transport retries | A persistent baseline-restoration obligation is independent of arming, HA connectivity and a single transport attempt. Failed idle attempts retry with a 120-second interval; successful, fresh readback of both outputs is required to clear the obligation. The reference package also bounds stuck transactions. | Multiple complete failed retry budgets, link recovery while HA remains absent, both outputs restored, and retries stopping after verified recovery. |
| 2. HA restore deduplication suppresses a new emergency | Removed wall-clock restore deduplication. Serialized requests stop control first and still press restore if disarming fails. A previous completed or failed request cannot suppress the next one. | Restore → re-arm/elevated target → second emergency inside the old window; failed service → retry. |
| 3. ESP deduplication preserves conflicting writes | Restoration disarms and supersedes non-baseline intent before any no-op decision. In-flight writes receive compensating baseline intent; command identities cannot collide merely because two requests occur in the same millisecond. | Pending and already-transmitted elevated commands, late old acknowledgements, and restoration despite recent requests. |
| 4. Cached setpoints count as strong verification | Both actuators expose atomic wire-read evidence. HA checks receipt age, actual boiler-read ages/counters, pending writes and restore state as well as both numerical outputs. Command acknowledgement requires advancement of both read counters. ESP independently stops control when wire reads expire. | Live HA with failed Optolink, frozen status, malformed evidence, unchanged values with real fresh reads, partial-pair acknowledgement, and missing pre-command evidence. |
| 5. Evaluations re-arm after shutdown | Coordinator-owned tasks are tracked before execution or lock acquisition, cancelled and awaited during shutdown. Lifecycle guards prevent late arm/target/heartbeat calls. Unload remains distinct from recoverable startup. | Queued, lock-waiting and in-flight write tasks; late evaluate/recover/arm calls. |
| 6. Configured fault inputs fail open | A configured missing, unavailable, unknown or unparsable fault input blocks evaluation and heartbeat. Deliberately unconfigured optional inputs remain optional. | Missing, unknown, unavailable, malformed, negative and non-finite states. |
| 7. Capture overwrites the trusted baseline while optimizing | Capture requires disarmed, idle control, no write/restore/watchdog activity, and fresh coherent wire readings. Rejected capture does not alter the stored baseline. | Armed/pending/stale/incoherent rejection; valid idle capture. |
| 8. Optimistic publication acknowledges a rejected target | Site target uses `optimistic: false` and explicitly publishes only accepted/staged state. Actual-output verification remains separate. | Full target action, including the framework's optional post-action optimistic publication step. |

Additional boundary fixes preserve the original heartbeat lease on repeated arm
commands, prevent rejected arm commands from orphaning pending writes, reject
non-finite commands, fence duplicate/stale reference read callbacks, latch wire
expiry across millisecond-clock rollover, and restore a known baseline on boot
even when the last armed flag was not persisted before power loss.

## Shared actuator status contract: version 1

The site exposes the text sensor **Optimizer actuator status**. Its ESPHome ID
is `optimizer_actuator_status`. The reference package exposes
`${vso_name} optimizer actuator status`, with ESPHome ID `vso_actuator_status`.
Home Assistant exposes these text sensors in the `sensor` domain. Select the
actual discovered entity; do not assume its generated entity ID.

The sensor publishes every five seconds. A representative payload is:

```json
{"v":1,"n":22.00,"r":22.00,"na":0,"ra":0,"ns":12,"rs":12,"p":false,"restore":false,"healthy":true}
```

| Field | Meaning |
| --- | --- |
| `v` | Contract version, integer `1`. |
| `n`, `r` | Normal and reduced outputs from successful boiler reads, not requested values. |
| `na`, `ra` | Milliseconds since those successful wire reads. |
| `ns`, `rs` | Nonzero successful-read counters. Both must advance relative to the pre-command witness to acknowledge a command. The reference hook reports a pair and uses one shared counter for both. |
| `p` | A write or unapplied target remains pending. |
| `restore` | Baseline restoration remains outstanding, including between retry attempts. |
| `healthy` | The actuator's local wire-read evidence is still fresh. |

HA accepts status receipts no older than **20 seconds** and requires wire ages,
including elapsed time since receipt, no older than **180 seconds**. A recent
HA report does not by itself establish a recent boiler read. Numeric outputs
may stay unchanged indefinitely when independent wire evidence remains fresh.
Missing or malformed evidence cannot authorize unattended heartbeat.

When restoration starts without any valid status witness, HA still sends the
restore command. It then requires another read of each output after the first
valid status returns, rather than accepting that first potentially cached
report as a command acknowledgement.

The two new binary diagnostics are **Optimizer boiler link healthy** and
**Optimizer baseline restore required**. They aid observation; the atomic text
status is the required HA verification mapping. A staged target alone does not
mean the physical boiler has applied it.

## Upgrade procedure

1. **Disable optimization and restore the existing trusted baseline first.**
   Observe both boiler outputs and the disarmed switch. Preserve the original
   source, HA configuration and device credentials. Do not capture an elevated
   optimized value as a new baseline.
2. Replace the site YAML **and** its bundled `components/vitoconnect` source
   together. Retain your existing `secrets.yaml`, board/UART settings and any
   deliberate site-specific settings. The delivered archives intentionally omit
   credentials, Git history, generated build trees and caches. Validate and build
   in your configured ESPHome environment before flashing.
3. After the node reconnects, check baseline validity, both stored baselines,
   both actual outputs, wire health, and the new status sensor. A restore
   obligation or watchdog indication must clear only after successful reads;
   control remains off after restoration.
4. Install the patched `custom_components/viessmann_setpoint_optimizer`
   directory and restart Home Assistant. Open the integration options and map
   **Actuator wire-read status** to the new sensor. Preserve the existing zone
   configuration; rebuilding zones is not required.
5. Confirm that **both stored normal and reduced baselines match HA's single
   configured baseline**. The firmware can restore a distinct pair, but this HA
   optimizer's unattended contract still requires one common baseline. Do not
   change valid boiler settings just to dismiss a mismatch without checking the
   installation's intended baseline.
6. Perform attended normal-operation and fault-injection checks before
   unattended use. Inspect actual readbacks, not merely service success or the
   displayed target. Do not switch to supervised-test mode as a workaround for
   missing wire status: that mode intentionally retains weak verification.

A known valid baseline is restored on every ESP boot. A baseline which is
missing, invalid or outside the allowed range is not replaced with an arbitrary
safe-looking value to satisfy a restore. Such a node stays disarmed and needs
explicit commissioning. Capture is deliberately blocked while restoration is
outstanding; do not clear that obligation just to bypass its guard.

### Reference package hook requirements

The reference YAML is not a second actuator to install on top of the supplied
site configuration. Its three site hooks remain placeholders. A custom binding
must serialize normal/reduced wire writes and report a **new successful read of
the pair** with the exact positive `read_id` supplied to the read hook. Duplicate,
unsolicited and superseded callbacks are ignored. Do not republish a cached pair
as a successful new read. The hook must account for writes already queued in
its transport: stopping an ESPHome script cannot retract bytes already sent.
The supplied VitoConnect site implements that compensation in its pending
command machinery.

The reference transaction guard is 180 seconds; failed attempts are retried
with a 120-second interval after an attempt/timeout. These are retry controls,
not a guarantee of physical restoration within a fixed time. Actual recovery
depends on working communications and successful readbacks.

## Validation performed

- **138 optimizer pytest cases passed**: the original decision/safety suite,
  updated contract assertions, and new tests executing the real coordinator
  methods with deterministic Home Assistant I/O fakes.
- **12 site host scenario groups passed**: actual production VitoConnect C++
  translation units and the site YAML's safety lambdas, using mocked ESPHome
  interfaces and an injected Optolink transport.
- **9 reference host scenario groups passed**: actual reference-package lambdas
  and action branches, with synchronous fake wire hooks and a virtual clock.
- **2 cross-project status checks passed**: the JSON generated by each compiled
  actuator harness is accepted by the real HA verification helpers.
- Python byte-compilation, parsing of both YAML files, JSON parsing, and patch
  whitespace/application checks are included in the final validation record.

The coordinator harness substitutes the HA framework boundary, not coordinator
method bodies. The site harness models the template number's documented action
order; it does not run an installed ESPHome application. The reference harness
is a synchronous action runner and does **not** validate the ESPHome scheduler.
These tests therefore do not establish target-build or physical-system safety.

**Not performed:** full Home Assistant installation tests, full ESPHome schema
validation/code generation, an ESP-IDF firmware build, flashing, real UART
protocol tests, or physical boiler fault injection. ESPHome and its target
build environment were unavailable in the execution environment. The original
boiler's independent hardware safety systems are outside this patch set.

### Reproduce the software checks

With the two source directories beside each other, install the optimizer's
`requirements-dev.txt` and provide `g++`, then run:

```bash
cd viessmann_setpoint_optimizer
python -m pytest -o addopts='' -q
python -m compileall -q custom_components tests
python tests/check_joint_contract.py ../esphome_vitoconnect
```

The joint command runs both host suites and feeds their actual generated status
payloads to HA's verifier. The site suite can also be run independently with
`python tests/host_safety_checks.py` from the site repository.

In the existing ESPHome build environment, additionally run:

```bash
cd esphome_vitoconnect
esphome config heating_optolink.yaml
esphome compile heating_optolink.yaml
```

Those last two commands are deployment checks, **not commands executed as part
of the reported test results**.

### Physical acceptance checks still required

Check HA-only loss, Optolink-only loss while HA stays online, and overlapping
losses with Optolink recovering while HA remains offline. Keep the link down
long enough to exhaust several transport attempts. Verify eventual restoration
of **both** baseline outputs, stopped retries after fresh confirmation, and no
automatic re-arm from the ESP. Also test restore during an in-flight elevated
write, HA reload during a pending evaluation, guarded baseline capture, and a
power cycle during a paired write. Recovery after communications return is the
acceptance criterion; no software patch can complete a physical write while
the boiler link remains unavailable.
