# ESP32-C6 Zigbee CCT Lamp

Firmware for a homemade Zigbee CCT bedside/wakeup lamp based on an ESP32-C6. The lamp drives warm and cool white LED channels through a TLC59108 PWM driver, reports temperature and humidity from an MS8607 sensor, and presents itself to Zigbee as a dimmable color-temperature light.

## Basic Functionality

- Zigbee light control on endpoint `10`: on/off, brightness, and color temperature.
- Color temperature range: `200` to `455` mired.
- Temperature and humidity reporting to Zigbee2MQTT/Home Assistant.
- Local touch control with an AT42QT1011-style touch output on GPIO `4`.
- Local physical button on GPIO `18`.
- Wakeup light sequence with configurable start brightness, end brightness, start color temperature, end color temperature, and fade time.
- Zigbee OTA updates through Zigbee2MQTT.
- Status LED output on GPIO `19`, `20`, and `21`.

The current Zigbee identity used by the external converter is:

- Zigbee model: `CCT-SmartLamp`
- Manufacturer/fingerprint: `CK-Home`
- Exposed model: `CCT-SmartLamp-wakeup`
- Manufacturer code: `0x1234`
- OTA image type: `0x0001`

## Local Controls

Touch sensor:

- Single tap: toggles the light on/off.
- Double tap: switches the hold action between brightness and color temperature.
- Hold: ramps the selected value up or down.
- Touch during an active wakeup sequence: cancels the wakeup, freezes the lamp at the currently rendered brightness/color temperature, and reports `wakeup_start=false`.

Physical button:

- Single press: toggles the light on/off.
- Double press: factory-resets Zigbee pairing state.
- Long press: currently detected but not assigned to a user-facing action.

## Hardware Notes

Main GPIOs used by the firmware:

- I2C SDA: GPIO `6`
- I2C SCL: GPIO `7`
- Touch input: GPIO `4`
- Physical button: GPIO `18`, active low with internal pull-up
- Status LEDs: GPIO `19`, `20`, `21`
- TLC59108 power enable: GPIO `10`
- TLC59108 reset: GPIO `15`

The partition table includes a factory app plus two OTA app slots. Because of that, the first OTA-capable firmware must be flashed over serial before Zigbee OTA can be used.

## Build And Serial Flash

Set up ESP-IDF, then build for ESP32-C6:

```sh
. "$IDF_PATH/export.sh"
idf.py set-target esp32c6
idf.py build
```

Flash over serial:

```sh
idf.py -p /dev/ttyUSB0 flash monitor
```

Use the correct serial port for your machine.

## Home Assistant With Zigbee2MQTT

Home Assistant sees this lamp through Zigbee2MQTT MQTT discovery. Zigbee2MQTT needs the external converter in this repository because the lamp has custom wakeup attributes and local OTA metadata.

1. Copy `wakeup_converter.js` into your Zigbee2MQTT directory: `/config/zigbee2mqtt/external_converters/wakeup_converter.js`. 
2. Add it to your Zigbee2MQTT `configuration.yaml`:

```yaml
external_converters:
  - wakeup_converter.js
ota:
  zigbee_ota_override_index_location: my_index.json
  image_block_response_delay: 250
  default_maximum_data_size: 50
```

3. Restart Zigbee2MQTT.
4. Pair the lamp, or re-interview it if it was already paired before the converter was added.
5. In Home Assistant, make sure MQTT discovery is enabled. The light, temperature, humidity, and wakeup controls should appear through Zigbee2MQTT.

The converter exposes:

- `light`: on/off, brightness, color temperature
- `temperature`
- `humidity`
- `wakeup_start_bri`
- `wakeup_end_bri`
- `wakeup_start_ct`
- `wakeup_end_ct`
- `wakeup_fade_time`
- `wakeup_start`

If the device already exists in Zigbee2MQTT but entities are missing in Home Assistant, restart Zigbee2MQTT and run a device re-interview from the Zigbee2MQTT device page.

Zigbee2MQTT showing `Supported: external` is expected when this converter is loaded locally. It does not need to be submitted or approved upstream for your local device to work.

## Zigbee2MQTT OTA

The firmware exposes a Zigbee OTA Upgrade client on endpoint `10`. The external converter must have OTA enabled:

```js
ota: true,
```

The repository includes a local Zigbee2MQTT OTA index in `my_index.json` and packaged OTA images under `ota/`.

To use the local OTA index, copy these into the Zigbee2MQTT data directory:

- `/config/zigbee2mqtt/my_index.json`
- the matching `.ota` file from `ota/`, keeping the `ota/` subdirectory path used by the index

Example layout in the Zigbee2MQTT data directory:

```text
configuration.yaml
wakeup_converter.js
my_index.json
ota/
  zigbee_cct_led_controller-0x0001000E.ota
```

Restart Zigbee2MQTT after changing the converter, OTA index, or OTA settings.

To check from MQTT, publish to `zigbee2mqtt/bridge/request/device/ota_update/check`:

```json
{"id":"0x588c81fffe5d2630"}
```

Replace the IEEE address with your lamp address. A successful check returns `update_available: true` when `my_index.json` contains a file version greater than the version currently running on the lamp.

## Creating A New OTA Image

1. Increase `OTA_UPGRADE_RUNNING_FILE_VERSION` in `components/zigbee_app/zigbee_app.h`.
2. Keep the Basic cluster `SWBuildID` in `components/zigbee_app/zigbee_app.c` aligned with the version string.
3. Build the app:

```sh
idf.py build
```

4. Wrap the app binary as a Zigbee OTA image, using the same version value:

```sh
python3 scripts/make_zigbee_ota.py \
  build/zigbee_cct_led_controller.bin \
  ota/zigbee_cct_led_controller-0x0001000E.ota \
  0x0001000E
```

5. Update `my_index.json` so `fileName`, `fileVersion`, `fileSize`, `url`, and `sha512` match the new `.ota` file.
6. Copy the updated `.ota` file and `my_index.json` to the Zigbee2MQTT data directory.
7. Restart Zigbee2MQTT and run an OTA check/update from the device page.

If the serial-flashed firmware and OTA image use the same file version, Zigbee2MQTT will report that no update is available.

## OTA Troubleshooting

- Device says `does not support OTA updates`: make sure Zigbee2MQTT is using `wakeup_converter.js`, the converter has `ota: true`, and the device has been re-interviewed.
- `update_available: false`: the OTA image version is not greater than the running firmware version, or Zigbee2MQTT is reading a different index than expected.
- `No image currently available`: check that `my_index.json` points to the real `.ota` path in the Zigbee2MQTT data directory and that `fileSize`/`sha512` match the file.
- `Firmware ID unknown` in Zigbee2MQTT is not by itself fatal, but the device still needs the OTA client cluster on endpoint `10` and a matching external converter/index.
- `Supported: external` is normal for this device when using the local converter.
- OTA over Zigbee is slow. Keep the lamp close to the coordinator/router during updates and avoid restarting Zigbee2MQTT or the lamp while an update is in progress.

More OTA detail is in `docs/zigbee2mqtt_ota.md`.
