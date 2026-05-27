#!/usr/bin/env python3
"""Wrap an ESP-IDF app binary in a Zigbee OTA image file."""

from __future__ import annotations

import argparse
import struct
from pathlib import Path


OTA_FILE_MAGIC = 0x0BEEF11E
OTA_HEADER_VERSION = 0x0100
OTA_HEADER_LENGTH = 56
OTA_FIELD_CONTROL = 0x0000
OTA_STACK_VERSION = 0x0002
OTA_UPGRADE_IMAGE_TAG = 0x0000

DEFAULT_MANUFACTURER = 0x1234
DEFAULT_IMAGE_TYPE = 0x0001


def parse_u32(value: str) -> int:
    parsed = int(value, 0)
    if not 0 <= parsed <= 0xFFFFFFFF:
        raise argparse.ArgumentTypeError(f"{value!r} is outside uint32 range")
    return parsed


def parse_u16(value: str) -> int:
    parsed = int(value, 0)
    if not 0 <= parsed <= 0xFFFF:
        raise argparse.ArgumentTypeError(f"{value!r} is outside uint16 range")
    return parsed


def build_ota_image(app_bin: bytes, manufacturer: int, image_type: int, file_version: int) -> bytes:
    subelement = struct.pack("<HI", OTA_UPGRADE_IMAGE_TAG, len(app_bin)) + app_bin
    image_size = OTA_HEADER_LENGTH + len(subelement)
    header_string = b"CK-Home CCT SmartLamp OTA"
    header_string = header_string[:32].ljust(32, b"\x00")

    header = struct.pack(
        "<IHHHHHIH32sI",
        OTA_FILE_MAGIC,
        OTA_HEADER_VERSION,
        OTA_HEADER_LENGTH,
        OTA_FIELD_CONTROL,
        manufacturer,
        image_type,
        file_version,
        OTA_STACK_VERSION,
        header_string,
        image_size,
    )
    return header + subelement


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_bin", type=Path, help="ESP-IDF app .bin file")
    parser.add_argument("output_ota", type=Path, help="output Zigbee OTA file")
    parser.add_argument("file_version", type=parse_u32, help="OTA file version, for example 0x00010001")
    parser.add_argument("--manufacturer", type=parse_u16, default=DEFAULT_MANUFACTURER)
    parser.add_argument("--image-type", type=parse_u16, default=DEFAULT_IMAGE_TYPE)
    args = parser.parse_args()

    app_bin = args.input_bin.read_bytes()
    ota_image = build_ota_image(app_bin, args.manufacturer, args.image_type, args.file_version)

    args.output_ota.parent.mkdir(parents=True, exist_ok=True)
    args.output_ota.write_bytes(ota_image)

    print(f"Wrote {args.output_ota}")
    print(f"  manufacturer: 0x{args.manufacturer:04x}")
    print(f"  image_type:   0x{args.image_type:04x}")
    print(f"  file_version: 0x{args.file_version:08x}")
    print(f"  app_size:     {len(app_bin)} bytes")
    print(f"  ota_size:     {len(ota_image)} bytes")


if __name__ == "__main__":
    main()
