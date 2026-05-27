# Zigbee2MQTT OTA

This firmware exposes a Zigbee OTA Upgrade client on endpoint `10`.

OTA identity:

- Manufacturer code: `0x1234`
- Image type: `0x0001`
- Current firmware version: see `OTA_UPGRADE_RUNNING_FILE_VERSION` in `components/zigbee_app/zigbee_app.h`
- Hardware version: `0x0002`

The `0x0001000A` test build flashes the status LED green+yellow rapidly after the first successful boot into an OTA image. It also exposes Basic cluster `SW_BUILD_ID` as `0x0001000A`, which can be refreshed by re-interviewing the device after the update. OTA block size is set to `100` bytes.

Build and wrap an update image. Use a file version higher than the version currently running on the lamp:

```sh
idf.py build
python3 scripts/make_zigbee_ota.py \
  build/zigbee_cct_led_controller.bin \
  ota/zigbee_cct_led_controller-0x0001000A.ota \
  0x0001000A
```

Place the `.ota` file somewhere Zigbee2MQTT can read it, then add a local OTA override index in the Zigbee2MQTT data directory:

```json
[
  {
    "fileName": "zigbee_cct_led_controller-0x0001000A.ota",
    "fileVersion": 65546,
    "fileSize": 696510,
    "url": "./ota/zigbee_cct_led_controller-0x0001000A.ota",
    "imageType": 1,
    "manufacturerCode": 4660,
    "sha512": "ff373c20ad7452411668ca8bcb8a67f3148e3f83350e6b63bf4cbfef09ac7f9a07f29eb68daf4ad3dc80f2b21dddfebb19fec6f55355a1e81aa64b6fc2afe0d6",
    "otaHeaderString": "CK-Home CCT SmartLamp OTA"
  }
]
```

Example Zigbee2MQTT config:

```yaml
ota:
  zigbee_ota_override_index_location: my_index.json
  image_block_response_delay: 50
  default_maximum_data_size: 100
```

The external converter must also enable Zigbee OTA for the device definition:

```js
const definition = {
  // existing definition fields
  ota: true,
};
```

Zigbee2MQTT documents these OTA settings in its OTA configuration and usage guides:

- https://www.zigbee2mqtt.io/guide/configuration/ota-device-updates.html
- https://www.zigbee2mqtt.io/guide/usage/ota_updates.html

The first OTA-capable firmware must be flashed over serial because the partition table changes from a single app slot to OTA app slots. After that, bump `OTA_UPGRADE_RUNNING_FILE_VERSION` in `components/zigbee_app/zigbee_app.h`, rebuild, wrap the binary with the same version value, copy the updated external converter into Zigbee2MQTT, restart Zigbee2MQTT, re-interview the device, and trigger the update from Zigbee2MQTT. If the serial-flashed firmware and OTA file have the same file version, Zigbee2MQTT will correctly report no update.
