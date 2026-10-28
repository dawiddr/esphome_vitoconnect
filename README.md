# Vitoconnect for ESPHome

> Read [current actuator procedures](docs/ACTUATOR.md) before a site upgrade.
> Unattended HA control requires the actuator wire-status sensor. The
> [0.2.1 safety report](SAFETY_UPGRADE.md) contains historical implementation evidence.

Integration of Viessmann Optolink protocols [GWG], KW and P300 into [ESPHome]. This component is named after the cloud reading unit of Viessmann: Vitoconnect.

## Usage

For usage, simply add the following to your config file. Example: V200WO1.
Address, length and post processing can be retrieved from <https://github.com/openv/openv/wiki/Adressen>. Length of 2 bytes is by default interpreted as int16, 4 bytes as uint32. Both values are then converted to float.

```yaml
external_components:
  - source: github://dannerph/esphome_vitoconnect

esphome:
  name: viessmann-reader
  friendly_name: vitoconnect

esp32:
  board: esp32doit-devkit-v1

# esp8266:
  # board: nodemcuv2

uart:
  - id: uart_vitoconnect
    rx_pin: GPIO16              # ESP32 RX2
    tx_pin: GPIO17              # ESP32 TX2
    #rx_pin: GPIO03             # ESP8266 RX
    #tx_pin: GPIO01             # ESP8266 TX
    baud_rate: 4800
    data_bits: 8
    parity: EVEN
    stop_bits: 2
    # debug:
    #   direction: BOTH
    #   dummy_receiver: false
    #   after:
    #     delimiter: [0x06]
    #   sequence:
    #     - lambda: UARTDebug::log_hex(direction, bytes, ':');

vitoconnect:
  uart_id: uart_vitoconnect
  protocol: P300                # set protocol to GWG, KW or P300
  update_interval: 30s

sensor:
  - platform: vitoconnect
    name: "Außentemperatur"
    address: 0x01C1             # vitoconnect: address of the value
    length: 2                   # vitoconnect: length of the value
    unit_of_measurement: "°C"
    accuracy_decimals: 1
    filters:
      - multiply: 0.1
  - platform: vitoconnect
    name: "Betriebsstunden Verdichter"
    address: 0x0580
    length: 4
    unit_of_measurement: "h"
    accuracy_decimals: 1
    filters:
      # - multiply: 0.000277777777777778 # use multiply filter for ESP8266
      - lambda: return x / 3600.0;
  - platform: vitoconnect
    name: "Brennerleistung"
    address: 0xA38F
    length: 1
    unit_of_measurement: "%"
    accuracy_decimals: 1
    filters:
      - multiply: 0.5
binary_sensor:
  - platform: vitoconnect
    name: "Status Verdichter"
    address: 0x0400
```

Tested with OptoLink ESP32 adapter from here:
<https://github.com/openv/openv/wiki/Bauanleitung-ESP32-Adafruit-Feather-Huzzah32-and-Proto-Wing>

## Compatibility and requirements

Development follows recent stable ESPHome releases. The project environment uses
Python `>=3.12,<3.14`. [uv.lock](uv.lock) currently selects **ESPHome 2026.9.1**
and fixes the Python dependency versions for validation. The minimum ESPHome
constraint in [pyproject.toml](pyproject.toml) does not establish compatibility
with every permitted release.

Review the development reference before framework API changes and release
preparation. Each new stable release does not require an immediate upgrade.
Report the exact tested versions, board, and firmware framework with each
validation result. See the [test guide](tests/README.md) for the review procedure
and the limits of host tests, firmware builds, and physical checks.

## Heating curve and DHW controls in the site configuration

`heating_optolink.yaml` provides number controls for **Heating curve slope**,
**Heating curve level**, and **DHW setpoint**. The DHW number replaces its
read-only sensor. See the [actuator guide](docs/ACTUATOR.md#manual-heating-curve-and-dhw-controls)
for the HO1B addresses, ranges, readback behavior, and Home Assistant entity
migration.

## Estimated gas usage in the site configuration

`heating_optolink.yaml` provides a burner-based estimate. Keep `gas_estimator.h`
beside the YAML when building or copying this configuration. The estimate reads
the existing burner power and active status; it does not write boiler settings.

Home Assistant receives these entities:

| Entity | Purpose |
| --- | --- |
| Burner full-load equivalent runtime | Calibration counter in hours. One hour at 50% power adds 0.5 hours while the burner is active. |
| Gas flow at full burner power | Calibration setting in m³/h. Its initial value of 0 means calibration is not set. |
| Estimated gas flow | Current estimated flow in m³/h. |
| Estimated gas consumption | Accumulated estimated volume in m³, suitable for the Energy dashboard. |
| Gas estimate inputs valid | Indicates that both burner readings are recent and within the accepted range. |
| Gas estimate calibration set | Indicates that a positive calibration value has been entered. |

### Calibrate with two gas meter readings

1. Record the gas meter total **M1** in m³ and the full-load equivalent runtime
   **H1** in hours at the same time.
2. Let the boiler operate normally over a useful period, preferably a day or more.
   Other gas appliances must not consume gas during the calibration period,
   unless their consumption can be subtracted from the meter change.
3. Record the new meter total **M2** and equivalent runtime **H2** together.
4. Calculate **(M2 − M1) / (H2 − H1)**. Both differences must be positive.
5. Enter the result in **Gas flow at full burner power** in Home Assistant.

For example, a meter change of 3 m³ and an equivalent runtime change of 1.5 hours
give a calibration value of **2.000 m³/h**. This is an example, not a value for
your boiler. Gas flow and consumption remain unavailable until calibration is
set; the equivalent runtime counter runs before calibration. Gas consumption
starts accumulating after calibration and does not include the calibration
period. Changing calibration affects future increments only.

The estimate assumes that gas flow is proportional to reported burner power.
It covers both heating and hot water. Check it against later meter readings,
especially when the burner operates at different power levels. The 60-second
Optolink polling interval limits accuracy during short firing cycles.

The calculation runs every five seconds. It pauses when either reading is more
than 90 seconds old, when a value is invalid, or across a restart or a scheduler
gap longer than ten seconds. It does not fill missing periods after recovery.
Adjust the freshness limit in `gas_estimator.h` if the boiler polling interval
changes. Both counters and the calibration setting are restored after restart;
a sudden power loss can lose recent unsaved increments.

Check the calculation with:

```sh
g++ -std=c++17 -Wall -Wextra -Werror tests/gas_estimator_checks.cpp -o /tmp/vito-gas-checks
/tmp/vito-gas-checks
```

## Credits

Built based on [VitoWifi] by [Bert Melis] and inspired by [vitowifi_esphome] by [Philipp Hack].

## License

MIT License
Copyright (c) 2025 Philipp Danner

## Support Development

### Paypal

[![](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=848P2G8EA68PJ)

[ESPHome]: https://esphome.io/
[VitoWifi]: https://github.com/bertmelis/VitoWifi
[vitowifi_esphome]: https://github.com/phha/vitowifi_esphome
[Bert Melis]: https://github.com/bertmelis
[Philipp Hack]: https://github.com/phha
[GWG]: README_GWG.md
## Development

Prepare the locked environment with `uv sync --locked` and provide `g++` with
C++17 support, then run `.venv/bin/python scripts/validate.py`. For documents
only, use `.venv/bin/python scripts/validate.py --documents-only` without test dependencies.

See the [test guide](tests/README.md) for baseline review, required checks, and
explicit schema/generation/target build selection. Build setup can download
firmware tools and needs an explicit opt-in. Local validation does not install
or repair Python packages, flash devices, or establish physical compatibility.

Use current documented ESPHome APIs and check the framework source before an API
change. The [actuator guide](docs/ACTUATOR.md) owns commissioning and bounded
joint status checks. The [release guide](docs/RELEASING.md) owns source identity,
validation records, package exclusions, and output hashes. Historical safety
reports retain their original dates and limits.
