# Actuator operation and joint checks

This guide owns current site commissioning and actuator contract instructions.
The [test guide](../tests/README.md) owns build and validation commands. The
[0.2.1 report](../SAFETY_UPGRADE.md) and [0.2.0 report](../SAFETY_UPGRADE_0.2.0.md)
are dated implementation evidence; their delivery-bundle commands and results
are historical.

## Control ownership and contract

The [site YAML](../heating_optolink.yaml) owns the actual actuator. Use its
matching `components/vitoconnect` source. Do not install the optimizer's reference
actuator package alongside the same site actuator. Preserve local board/UART
settings and private credentials when preparing an installation.

Actuator status uses contract version 1, including command identity, pending and
restoration state, baseline validity, and independent wire-read evidence. An
accepted target or cached value does not prove a completed boiler write. The HA
optimizer requires the actuator status entity. Both stored normal and reduced
baselines must match its configured single baseline before control can start.
`restore:false` alone does not authorize control.

Known-baseline restoration survives restart, HA loss, and exhausted write retry
budgets. Unknown-baseline recovery remains disarmed and requires deliberate
operator commissioning. Do not infer a replacement baseline from a potentially
optimized cached pair. Preserve bounded retries, heartbeat eligibility, read
freshness, pair coherence, and command/read identities when changing either side.

## Manual heating curve and DHW controls

The site configuration provides three number entities for the configured
Vitotronic 200 HO1B controller. The heating curve controls apply to circuit
A1/M1 (HK1).

| Entity | Address | Wire format | Control range | Step |
| --- | --- | --- | --- | --- |
| Heating curve slope | `0x27D3` | 1 unsigned byte; raw value divided by 10 | 0.2 to 3.5 | 0.1 |
| Heating curve level | `0x27D4` | 1 signed byte; temperature difference in K | −13 to +40 K | 1 K |
| DHW setpoint | `0x6300` | 1 unsigned byte; whole °C | 10 to 60 °C | 1 °C |

The address and byte definitions come from the OpenV
[P300 command map](https://github.com/openv/vcontrold/blob/8ca47972c9ac5b0a14a7a36393b0dbfdb165f918/xml/300/vito.xml)
and its
[unit conversions](https://github.com/openv/vcontrold/blob/8ca47972c9ac5b0a14a7a36393b0dbfdb165f918/xml/300/vcontrold.xml),
checked on 2026-10-28. That map identifies device `20CB` as `VScotHO1` and uses
`0x27D3`/`0x27D4` for its curve reads and writes. The older
`0x2305`/`0x2304` defaults are not used in this site configuration.

The curve ranges match the HO1B specification in the Viessmann
[Vitodens technical guide, page 81](https://viessmanndirect.co.uk/AttachedFileProxy.aspx?file_Id=e534bbfe-ebbd-4fd5-9356-a39001291823).
The DHW control is limited to 60 °C. Viessmann documents a default 10–60 °C
range with coding `56:0` in the
[HO1B/300-K service guide, page 89](https://www.viessmann.ca/content/dam/public-brands/ca/pdfs/doc/b2ha_ctrl/vitotronic_300k-mw2c_is.pdf/_jcr_content/renditions/original./vitotronic_300k-mw2c_is.pdf).
Some boiler variants allow a higher maximum. These controls do not change that
coding or establish the permitted range for another boiler variant.

Each number reads the actual boiler setting. It does not write an initial or
restored value at startup. A command queues a write for the next Optolink poll;
the site polls every 60 seconds. The displayed value changes only after a wire
read. With the site's `verify_writes: true`, a write acknowledgement alone does
not publish the requested value. Readback requires a later poll, and each
command has the existing three-failure retry limit. Check the returned value
after a command; a failed command can leave the previous value displayed.

These are manual boiler settings. The optimizer owns the normal and reduced
room setpoints. Its baseline restore does not restore the curve or DHW setting.
Disable optimization, disarm control, and complete any required baseline restore
before tuning the curve. Record the current settings first. During authorized
physical acceptance, compare all three reads with the boiler display, then
check one deliberate change and its returned value. Offline tests do not
confirm the address mapping or write behavior on the installed boiler.

The **DHW setpoint** number replaces the sensor of the same name. Each address
has one Optolink datapoint. Home Assistant uses separate `sensor` and `number`
domains, so update dashboards and automations that refer to the old DHW sensor
to use the new number entity. The old entity can remain unavailable in the
Home Assistant entity registry until it is removed there. Heating setpoint
readbacks and optimizer status entities remain available for the control
contract. Actual tank, outlet, and flow temperatures remain sensors.

## Commissioning and upgrades

These are attended installation procedures. Flashing, deployment, pairing, and
live boiler commands require a user request for that work; running validation
does not authorize them.

1. Disable HA optimization and keep the actuator arm switch off. If a trusted
   baseline exists, restore it and confirm both actual values and disarmed state
   before an upgrade. Back up sources, HA configuration, and private credentials.
2. Validate and build the matching component and site configuration. Retain the
   status mapping and intentional site settings. A valid stored baseline does
   not need recapture only because firmware changes; it restores at boot.
3. If commissioning is required, deliberately establish the intended boiler
   setpoints. Wait for fresh actual normal/reduced readbacks and idle command
   state. Use **Set optimizer baselines from current boiler**. Capture requires
   disarmed control, no known-baseline restoration obligation, finite in-range
   values, wire ages at most 180 seconds, and pair skew at most 15 seconds.
4. A prior uncommissioned restore cancels pending intent and requires post-stop
   reads. In-flight transport operations remain visible until completion or
   error. Resolve a refused capture's cause before retrying. Do not erase stored
   state or guess fallback values to bypass the guard.
5. Confirm baseline-valid, commissioning-required off, and the arm switch off.
   Confirm both stored baselines match the optimizer configuration. Use explicit
   HA recovery only after all prerequisites are healthy. Capture never arms
   control. Complete attended acceptance before unattended operation.

## Bounded joint status validation

Obtain a matching optimizer checkout. Use its prepared environment and pass this
firmware checkout explicitly; no relative sibling location is assumed:

```sh
/path/to/optimizer/.venv/bin/python /path/to/optimizer/tests/check_joint_contract.py /path/to/esphome_vitoconnect
```

Follow that checkout's `docs/SYSTEM_CHECKS.md`. The maintained runner compiles and
runs this firmware host suite, consumes its emitted status JSON, and executes
production optimizer verification against synthetic companion HA entities. It
records both source revisions and changes. It currently covers six bounded
status expectations, including deliberately synthetic negative mutations. This
is separate from local component validation.

A passing joint status check does not execute a complete optimizer-command to
firmware exchange or the real ESPHome scheduler. A maintained complete simulated
command-path harness is still missing. Keep that limit explicit. Do not recreate
the historical bundle runner or claim old delivery evidence as a fresh run.

## Physical acceptance

With an authorized operator present, observe actual wire readbacks, baseline
restoration, and disarmed state. Exercise a critical fault during an unconfirmed
elevated command, persistent invalid room/DHW inputs, manual OFF, and HA reload.
Check restore-before-commissioning, invalid stored-baseline recovery without
erasure, and transport recovery after repeated exhausted retries while HA is
offline. Verify fresh pair capture and the intended ownership of both setpoints.

Host tests model software behavior with fake I/O. Target compilation does not
establish timing or physical boiler behavior. Record each attended result,
installation identity, source revisions, operator action, and remaining limit
separately, without publishing private device data.
