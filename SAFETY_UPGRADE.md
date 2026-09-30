# Viessmann joint safety fixes — 0.2.1

**Date:** 28 September 2026
**Input:** the reviewed `viessmann_joint_fixes.zip` (optimizer 0.2.0)
**Input SHA-256:** `3787feffe0de2ad0f5241a86d4f0dee4feb927690c8d8fd427a10775b4d88d8b`
**Output:** local patch version 0.2.1; not a claim of an upstream release.

This update addresses R1–R3 from the fixed-bundle review. Both complete source trees include the earlier eight fixes. The actual `heating_optolink.yaml` site actuator and the optimizer's separate reference package are both updated. **Do not install the reference package alongside the actuator already implemented by the site YAML.**

No Home Assistant instance or boiler was changed, and no firmware was flashed.

## Findings and implementation

| Finding | Change | Positive regression coverage |
| --- | --- | --- |
| **R1 / P1 — critical fault waits behind ordinary target verification** | Ordinary target/arm verification checks current protective inputs and the safety generation on every two-second poll and immediately after service completion. A changed safety assumption ends optimization verification and the existing control owner disarms/restores. Baseline restoration is not cancelled merely because the protective input remains active. | Active, unavailable, unknown and removed fault inputs arriving during a pending target acknowledgement; input changes at service return and during polling; faults after arm service completion. First disarm is requested within the test's two-second interruption bound, rather than the 180-second verification timeout. |
| **R2 / P1 — internal state events create arm/restore feedback** | Re-arm prerequisites are checked before restoration, again after asynchronous restoration, before the arm service, and during arm verification. One-shot expected arm-state edges are distinguished from external changes. Event bursts share one draining evaluation task with a retained follow-up request. An unchanged blocking condition stays stopped once fresh baseline restoration is verified. | Persistent invalid room data, long DHW and unknown DHW remain off with one restore in the healthy-link scenarios; input recovery gives one re-arm. Coverage also includes actual service-generated callbacks, manual OFF after internal recovery, user-attributed OFF, failed arm services, 100-event bursts and recovery arriving during a running evaluation. |
| **R3 / P2 — no baseline means commissioning can never finish** | Both firmwares distinguish “no trusted baseline / commissioning required” from “known baseline must be restored.” Unknown destinations cause a stopped, explicitly uncommissioned state—not a guessed baseline write. Fresh polling remains possible. Only deliberate operator capture establishes a new trusted fallback. | Restore before first capture; invalid persisted baselines; migration from a persisted unknown restore flag; existing HA configuration reconnecting before commissioning; failed and recovered reads; no automatic adoption; known-baseline obligations cannot be bypassed. |

### R1: interruption semantics

There is still one write-capable coordinator owner and one operation lock. A fault callback revokes heartbeat eligibility and requests evaluation; the current optimization wait also observes the changed safety assumptions itself, so the protective action no longer waits behind the full verification timeout.

The configured polling interval is **two seconds**. This is a cooperative software check interval, **not a physical boiler response-time guarantee**. The acceptance tests model a command whose HA service call has returned but whose boiler acknowledgement is incomplete. An already awaited, stalled HA service call is not forcibly cancelled by this change; blocking services, a stalled event loop and transport latency can extend real response time. After a service returns, current inputs are checked before accepting its result or proceeding with control.

A protective input remaining active does not invalidate the baseline-restoration wait. Otherwise the stopping action could repeatedly cancel its own recovery. An interrupted optimization is reported as a safety block rather than charged as an ordinary actuator write failure.

### R2: stable blocking and recovery

An armed actuator can retain the existing short-DHW hold behavior. An already-disarmed actuator does **not** acquire a new lease while DHW is active or unknown. This makes repeated re-arm prerequisite checks consistent; reaching the first DHW check cannot accidentally make the next check permit arming.

Expected internal arm edges are one-shot and expire. A failed command removes its expectation; a command already at the requested state creates none. An explicitly user-attributed state change is never consumed as an internal acknowledgement. Other OFF transitions retain the manual/unknown disarm policy and require explicit recovery.

The stable-block optimization applies only when the same block is already recorded and the actuator is ready, freshly verified at baseline, disarmed, and has no pending command/restore. It is **not time-based restore-request deduplication**. Explicit restores still execute, and a new armed episode, pending write, stale evidence or incomplete restoration cannot be suppressed by an earlier completed restore.

### R3: commissioning state and wire fencing

`commissioning_required` (site) and `vso_commissioning_required` (reference) describe the absence of a trusted fallback. They are exposed as diagnostic binary sensors. They are not new mandatory HA configuration mappings.

A known, valid baseline retains the persistent `restore_required` behavior introduced in 0.2.0. Retry exhaustion, HA loss or a transport outage does not convert that obligation to commissioning. Only an absent/invalid baseline takes the uncommissioned path. Invalid stored values are retained for diagnosis until explicitly replaced, but are not used as write authority.

In the site implementation, an uncommissioned stop cancels not-yet-enqueued command intent and retries. **Already queued/transmitted requests cannot be recalled by this method**; they remain visible as in flight until completion/error. A read epoch rejects callbacks from reads queued before the stop. Capture therefore cannot use those old callbacks as fresh evidence and cannot proceed while a transmitted command remains active.

In the reference package, unknown-baseline restoration stops package/site scripts, advances the control/read identity, and allows read polling to resume. The new operator commissioning action requests a fresh pair and then invokes the guarded capture path. **Site bindings must serialize that read after any already submitted transport writes have completed or been cancelled.** A script stopping is not evidence that an independent transport queue was cancelled. The read hook must publish actual coherent wire values with the supplied `read_id`, never a cached pair or an unrelated late callback.

The site no longer automatically captures a baseline from its first healthy polls. Neither firmware silently adopts a potentially optimized value after a fault or reboot. A legacy watchdog flag can clear during explicitly authorized commissioning only when no valid baseline exists, no restore obligation remains, and fresh capture succeeds. This does not arm control.

## Upgrade procedure

1. **Disable optimization and restore the existing trusted baseline before upgrading**, where a trusted baseline exists. Observe both outputs and the disarmed state. Back up both projects, HA configuration and device credentials. An uncommissioned node must remain under deliberate operator control; do not substitute guessed baseline values.
2. Update the optimizer source and the site YAML **together with its bundled `components/vitoconnect` source**. Do not copy only the YAML: commissioning cancellation/read fencing also changes C++ headers and the hub callback implementation. Preserve the existing `secrets.yaml`, board/UART settings, hook bindings and intentional site customizations. Validate/build in the actual ESPHome environment before flashing.
3. Retain the 0.2.0 **`actuator_status_entity`** mapping to “Optimizer actuator status.” The JSON contract remains version 1. No additional mandatory HA selector is introduced. Observe baseline validity, both stored values, actual values, watchdog state, restore state and the new commissioning diagnostic.
4. A node with a valid stored baseline does **not** need baseline recapture merely because of this upgrade. It still restores the known pair on boot. A node reporting commissioning required uses the explicit procedure below.
5. After validation and any required commissioning, perform attended acceptance tests before enabling unattended operation.

### Explicit commissioning or recommissioning

**Site (`heating_optolink.yaml`):** keep HA optimization disabled and the ESP arm switch off. Establish the intended normal/reduced setpoints deliberately at the boiler, confirm them in fresh actual readbacks, and press **“Set optimizer baselines from current boiler.”** A prior restore request while uncommissioned cancels pending intent and requires post-stop reads; wait for those reads rather than using the old displayed cache. Capture requires an idle, disarmed actuator, no known-baseline restore obligation, in-range finite values, wire ages no greater than 180 seconds and pair skew no greater than 15 seconds. A refusal leaves the old values/invalid state intact. Repeat only after the reported reason is resolved.

**Reference package:** bind the existing three site write/read hooks correctly. With control off and no known-baseline restore outstanding, press **“[vso_name] commission baseline from fresh boiler reads.”** The action issues a new read request and allows up to 180 seconds for a matching pair. A timeout or invalid reply does not authorize capture. Restore cancels an in-progress commissioning action. Do not invoke the lower-level capture script automatically from ordinary polling callbacks.

After successful capture, verify that baseline-valid is on, commissioning-required is off and the arm switch remains off. For the supplied HA integration, **both stored baselines must match its single configured baseline**; a valid but distinct pair is not sufficient for HA readiness. Use explicit HA recovery after confirming all inputs are healthy. Capture itself never arms optimization.

`restore:false` by itself is not permission to control an uncommissioned node. The baseline-valid and consistency checks still block firmware arming and HA readiness.

## Validation performed

| Check | Result |
| --- | --- |
| Optimizer pytest suite | **160 passed**: the earlier 138 plus 22 event-driven/orchestration regression cases |
| Site firmware host suite | **17 scenario groups passed** |
| Reference firmware host suite | **13 scenario groups passed** |
| Firmware-produced status parsed/verified by HA actuator code | **2 checks passed** |
| Python syntax, production JSON and tagged YAML parsing | Passed; detailed counts in the validation log |
| Incremental patches against the reviewed fixed bundle | Applied and reproduced delivered source files byte-for-byte |
| Clean-extraction rerun | Results captured in `validation/delivery_archive_retest.txt` |

The site host suite compiles the production C++ component and production YAML lambdas with fake boiler I/O. The reference host suite executes production lambdas/action branches with synchronous fake site hooks. The coordinator tests execute production methods using deterministic HA services/states and actual asyncio callback scheduling; they are not a running HA deployment.

One older source-string assertion was updated intentionally: restore-required is now assigned from validated baseline existence rather than unconditionally set true. The new executable host scenarios check the behavior for both known and unknown baselines.

**Not performed:** full Home Assistant integration/deployment tests, ESPHome schema/code-generation or target compilation, device flashing, and physical-boiler fault injection. ESPHome was absent; installation/download attempts through the available package interface returned no matching distribution, so no target-build result is claimed. Those logs do not establish that the requested version is unavailable upstream.

The reference harness does not reproduce ESPHome scheduling: `script.wait` is approximated using synchronous hook completion; delays/timeouts advance a virtual clock. Script stop/cancellation behavior is modelled at action boundaries, not validated in the real framework. The hook-ordering requirement above needs validation against the actual site's asynchronous transport.

### Attended acceptance gate

Verify a critical fault during an unconfirmed elevated target; persistent invalid room data and DHW blocking with state events enabled; restore-before-commissioning and invalid persisted-baseline recovery without storage erasure; and Optolink returning after multiple exhausted restore attempts while HA remains offline. Inspect actual normal/reduced wire readbacks, not just the optimizer target entity. Also exercise manual OFF and HA reload during control activity.

## Bundle and patch use

The ZIP contains both complete updated source trees, this note, historical 0.2.0 notes, incremental patches, a recheck runner and validation logs. Credentials, Git metadata, bytecode and generated firmware/build caches are excluded.

The patches in `patches/from_fixed_0.2.0/` are **incremental against the reviewed fixed bundle**, not against the original uploaded repository revisions. From the matching clean project checkout:

```bash
git apply /path/to/bundle/patches/from_fixed_0.2.0/viessmann_setpoint_optimizer.patch
# Or, in the separate VitoConnect checkout:
git apply /path/to/bundle/patches/from_fixed_0.2.0/esphome_vitoconnect.patch
```

Alternatively, use the complete source archives. To rerun the bundled checks with Python, pytest, PyYAML and g++ available:

```bash
python tools/validate_bundle.py
# Or run directly:
cd viessmann_setpoint_optimizer
python -m pytest -o addopts='' -q
python tests/check_joint_contract.py ../esphome_vitoconnect
```

## Framework references

The event handling and script semantics were checked against the official Home Assistant “Listening for events” documentation (`developers.home-assistant.io/docs/integration_listen_events/`) and ESPHome “Script Component” documentation (`esphome.io/components/script/`), consulted 28 September 2026. These references explain the framework contract; the validation results above come from the delivered source/tests, not from those documentation pages.
