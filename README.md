# Vitoconnect for ESPHome

> Joint safety update **0.2.1**: read [SAFETY_UPGRADE.md](SAFETY_UPGRADE.md) before
> deployment. This update interrupts unsafe verification, prevents re-arm feedback,
> and makes unknown-baseline commissioning explicitly operator-controlled.
> Unattended HA control still requires the actuator wire-status sensor.

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

Install the locked environment with `uv sync --locked` and provide `g++`, then
run `.venv/bin/python tests/host_safety_checks.py`. For changes to component ID
handling or the host lambda renderer, also run
`.venv/bin/python tests/check_id_reference_regression.py`.

Use current, documented ESPHome APIs. Check the official documentation or the
locked ESPHome source before an API change, and update the relevant host
substitute to match that contract. Host checks use simulated I/O and do not
replace ESPHome validation or a target build.

See the [test guide](tests/README.md) for setup, baseline review, and firmware
checks. [SAFETY_UPGRADE.md](SAFETY_UPGRADE.md) contains the joint checks and
physical acceptance procedure. Its recorded results are historical evidence.
