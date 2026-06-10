# Zigbee2MQTT OTA

This firmware exposes a Zigbee OTA Upgrade client on endpoint `10`.

OTA identity:

- Manufacturer code: `0x1234`
- Image type: `0x0001`
- Current firmware version: see `OTA_UPGRADE_RUNNING_FILE_VERSION` in `components/zigbee_app/zigbee_app.h`
- Hardware version: `0x0002`

The `0x0001000E` test build flashes the status LED green+yellow rapidly after the first successful boot into an OTA image. It also exposes Basic cluster `SW_BUILD_ID` as `0x0001000E`, which can be refreshed by re-interviewing the device after the update. OTA block size is set to `100` bytes. This build uses size optimization, trims unused build components, logs OTA receive progress every 16 KB instead of every block, and keeps wakeup/touch state synced to the currently rendered brightness and color temperature.

Build and wrap an update image. Use a file version higher than the version currently running on the lamp:

```sh
idf.py build
python3 scripts/make_zigbee_ota.py \
  build/zigbee_cct_led_controller.bin \
  ota/zigbee_cct_led_controller-0x0001000E.ota \
  0x0001000E
```

Place the `.ota` file somewhere Zigbee2MQTT can read it, then add a local OTA override index in the Zigbee2MQTT data directory:

```json
[
  {
    "fileName": "zigbee_cct_led_controller-0x0001000E.ota",
    "fileVersion": 65550,
    "fileSize": 648942,
    "url": "./ota/zigbee_cct_led_controller-0x0001000E.ota",
    "imageType": 1,
    "manufacturerCode": 4660,
    "sha512": "baae27807098ab2c389a55f9b8ae63bf0720a18eac33b85e5ffbfd2357e17c92d619940d09132dbb87e019d84226edac9aa842060e5657402e20bb44472b46a3",
    "otaHeaderString": "CK-Home CCT SmartLamp OTA"
  }
]
```

Example Zigbee2MQTT config:

```yaml
ota:
  zigbee_ota_override_index_location: my_index.json
  image_block_response_delay: 10
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
