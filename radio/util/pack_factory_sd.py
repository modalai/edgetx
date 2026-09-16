#!/usr/bin/env python3
"""Build a deterministic, read-only FAT12 image for factory resources."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


IMAGE_KIB = 256
VOLUME_LABEL = "HELMFACTORY"
VOLUME_ID = "48454c4d"
FIXED_TIME = 1_767_225_600  # 2026-01-01 00:00:00 UTC
FAT_DATE = ((2026 - 1980) << 9) | (1 << 5) | 1


def run(command: list[str], environment: dict[str, str]) -> None:
    subprocess.run(command, check=True, env=environment)


def normalized_tree(source: Path, destination: Path) -> None:
    for path in sorted(source.rglob("*"), key=lambda item: item.as_posix()):
        if path.is_symlink():
            raise ValueError(f"symbolic links are not supported: {path}")

        output = destination / path.relative_to(source)
        if path.is_dir():
            output.mkdir(parents=True, exist_ok=True)
        elif path.is_file():
            output.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, output)
        else:
            raise ValueError(f"unsupported directory entry: {path}")

        os.utime(output, (FIXED_TIME, FIXED_TIME))


def normalize_fat_timestamps(path: Path) -> None:
    image = bytearray(path.read_bytes())
    sector_size = int.from_bytes(image[11:13], "little")
    sectors_per_cluster = image[13]
    reserved_sectors = int.from_bytes(image[14:16], "little")
    fat_count = image[16]
    root_entries = int.from_bytes(image[17:19], "little")
    sectors_per_fat = int.from_bytes(image[22:24], "little")
    root_sectors = (root_entries * 32 + sector_size - 1) // sector_size
    fat_offset = reserved_sectors * sector_size
    root_offset = (reserved_sectors + fat_count * sectors_per_fat) * sector_size
    data_offset = root_offset + root_sectors * sector_size
    cluster_size = sector_size * sectors_per_cluster

    def next_cluster(cluster: int) -> int:
        offset = fat_offset + cluster + cluster // 2
        value = int.from_bytes(image[offset : offset + 2], "little")
        return (value >> 4) & 0x0FFF if cluster & 1 else value & 0x0FFF

    def cluster_offsets(first_cluster: int):
        cluster = first_cluster
        while 2 <= cluster < 0xFF8:
            start = data_offset + (cluster - 2) * cluster_size
            yield from range(start, start + cluster_size, 32)
            cluster = next_cluster(cluster)

    visited: set[int] = set()

    def normalize_directory(offsets) -> None:
        for offset in offsets:
            first = image[offset]
            if first == 0x00:
                break
            if first == 0xE5 or image[offset + 11] == 0x0F:
                continue

            image[offset + 13] = 0
            image[offset + 14 : offset + 16] = (0).to_bytes(2, "little")
            image[offset + 16 : offset + 18] = FAT_DATE.to_bytes(2, "little")
            image[offset + 18 : offset + 20] = FAT_DATE.to_bytes(2, "little")
            image[offset + 22 : offset + 24] = (0).to_bytes(2, "little")
            image[offset + 24 : offset + 26] = FAT_DATE.to_bytes(2, "little")

            name = bytes(image[offset : offset + 11])
            attributes = image[offset + 11]
            cluster = int.from_bytes(image[offset + 26 : offset + 28], "little")
            if attributes & 0x10 and name not in (b".          ", b"..         "):
                if cluster >= 2 and cluster not in visited:
                    visited.add(cluster)
                    normalize_directory(cluster_offsets(cluster))

    normalize_directory(range(root_offset, root_offset + root_entries * 32, 32))
    path.write_bytes(image)


def build_image(source: Path, image: Path) -> int:
    image.parent.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment["MTOOLS_DATE_STRING"] = "20260101000000"

    with tempfile.TemporaryDirectory(prefix="factory-sd-") as temporary:
        tree = Path(temporary) / "tree"
        tree.mkdir()
        normalized_tree(source, tree)

        temporary_image = Path(temporary) / "factory_sd.img"
        run(
            [
                "mkfs.fat",
                "--invariant",
                "-C",
                "-F",
                "12",
                "-n",
                VOLUME_LABEL,
                "-i",
                VOLUME_ID,
                str(temporary_image),
                str(IMAGE_KIB),
            ],
            environment,
        )

        entries = sorted(tree.iterdir(), key=lambda item: item.as_posix())
        files = [path for path in tree.rglob("*") if path.is_file()]
        for path in entries:
            run(
                ["mcopy", "-s", "-m", "-i", str(temporary_image), str(path), "::/"],
                environment,
            )

        normalize_fat_timestamps(temporary_image)

        shutil.copyfile(temporary_image, image)

    if image.stat().st_size != IMAGE_KIB * 1024:
        raise ValueError(f"unexpected factory image size: {image.stat().st_size}")
    return len(files)


def render(image: Path, file_count: int) -> str:
    data = image.read_bytes()
    lines = [
        "// Generated by radio/util/pack_factory_sd.py. Do not edit.",
        '#include "storage/factory_volume.h"',
        "",
        '#define FACTORY_SD_SECTION __attribute__((section(".factory_sd"), used))',
        "",
        "const uint8_t factorySdImage[] FACTORY_SD_SECTION = {",
    ]

    for offset in range(0, len(data), 16):
        chunk = data[offset : offset + 16]
        lines.append("  " + ", ".join(f"0x{value:02x}" for value in chunk) + ",")

    lines.extend(
        [
            "};",
            "",
            f"const uint32_t factorySdImageSize FACTORY_SD_SECTION = {len(data)}u;",
            f"const uint16_t factorySdFileCount FACTORY_SD_SECTION = {file_count}u;",
            "",
            "#undef FACTORY_SD_SECTION",
            "",
        ]
    )
    return "\n".join(lines)


def atomic_write(path: Path, contents: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as output:
            output.write(contents)
        os.replace(temporary_name, path)
    except Exception:
        os.unlink(temporary_name)
        raise


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--image", required=True, type=Path)
    args = parser.parse_args()

    source = args.input.resolve()
    if not source.is_dir():
        parser.error(f"factory SD directory does not exist: {source}")

    file_count = build_image(source, args.image)
    if file_count == 0:
        parser.error(f"factory SD directory contains no files: {source}")

    atomic_write(args.output, render(args.image, file_count))


if __name__ == "__main__":
    main()
