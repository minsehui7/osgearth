#!/usr/bin/env python3
"""Regenerate LocalTerrainBlankRootData.inc from data/quantized-mesh-blank-root/*.terrain."""

from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
REF = ROOT / "data" / "quantized-mesh-blank-root"
OUT = ROOT / "src" / "osgEarth" / "LocalTerrainBlankRootData.inc"

PAIRS = (
    ("kBlankRootTileGzipX0", REF / "0-0-0.terrain"),
    ("kBlankRootTileGzipX1", REF / "0-1-0.terrain"),
)


def emit_array(name: str, data: bytes) -> str:
    lines = []
    for i in range(0, len(data), 12):
        chunk = data[i : i + 12]
        lines.append("    " + ", ".join(f"0x{b:02x}" for b in chunk) + ",")
    body = "\n".join(lines)
    return (
        f"static const unsigned char {name}[] = {{\n{body}\n}};\n"
        f"static const std::size_t {name}_size = sizeof({name});\n"
    )


def main() -> int:
    chunks = []
    for name, path in PAIRS:
        if not path.is_file():
            print(f"error: missing {path}", file=sys.stderr)
            return 1
        chunks.append(emit_array(name, path.read_bytes()))

    OUT.write_text("".join(chunks), encoding="utf-8")
    print(f"wrote {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
