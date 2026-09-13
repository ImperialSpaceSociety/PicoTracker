from __future__ import annotations

import re
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware"
COMPAT = Path(__file__).resolve().parent / "stm8_compat"
FLASH_START = 0x8000
FLASH_END = 0xA000
RAM_SIZE = 1024


def run(*args: str, cwd: Path | None = None) -> None:
    subprocess.run(args, cwd=cwd, check=True)


def adapted_source(source: Path) -> str:
    lines: list[str] = []
    for line in source.read_text().splitlines(keepends=True):
        if line.lstrip().startswith("#pragma vector"):
            continue
        lines.append(line.replace("__interrupt void ", "void "))
    return "".join(lines)


def parse_area(map_text: str, name: str) -> tuple[int, int]:
    match = re.search(
        rf"^{re.escape(name)}\s+([0-9A-F]{{8}})\s+([0-9A-F]{{8}})\s+=",
        map_text,
        re.MULTILINE,
    )
    if not match:
        raise RuntimeError(f"missing {name} area in SDCC map")
    return int(match.group(1), 16), int(match.group(2), 16)


def main() -> None:
    sdcc = shutil.which("sdcc")
    if not sdcc:
        raise SystemExit("sdcc is required for the STM8 structural target check")

    with tempfile.TemporaryDirectory(prefix="picotracker-sdcc-") as tmp_name:
        tmp = Path(tmp_name)
        src_dir = tmp / "src"
        obj_dir = tmp / "obj"
        src_dir.mkdir()
        obj_dir.mkdir()

        objects: list[Path] = []
        for source in sorted(FIRMWARE.glob("*.c")):
            adapted = src_dir / source.name
            adapted.write_text(adapted_source(source))
            obj = obj_dir / f"{source.stem}.rel"
            run(
                sdcc,
                "-mstm8",
                "--std-c99",
                "--opt-code-size",
                f"-I{COMPAT}",
                f"-I{FIRMWARE}",
                "-c",
                str(adapted),
                "-o",
                str(obj),
            )
            objects.append(obj)

        stub = tmp / "stub.c"
        stub.write_text("#include <stdint.h>\nvolatile uint8_t stm8_reg8;\n")
        stub_obj = obj_dir / "stub.rel"
        run(sdcc, "-mstm8", "--std-c99", "--opt-code-size", "-c", str(stub), "-o", str(stub_obj))
        objects.append(stub_obj)

        image = tmp / "picotracker-sdcc.ihx"
        run(
            sdcc,
            "-mstm8",
            "--out-fmt-ihx",
            "--code-loc",
            hex(FLASH_START),
            "--code-size",
            str(FLASH_END - FLASH_START),
            "--data-loc",
            "0x0000",
            "--iram-size",
            str(RAM_SIZE),
            "--stack-loc",
            "0x03ff",
            *(str(obj) for obj in objects),
            "-o",
            str(image),
        )

        map_text = image.with_suffix(".map").read_text()
        flash_areas = [
            parse_area(map_text, name) for name in ("HOME", "GSINIT", "INITIALIZER", "CODE")
        ]
        flash_end = max(address + size for address, size in flash_areas)
        data_address, data_size = parse_area(map_text, "DATA")

        if flash_end > FLASH_END:
            raise SystemExit(
                f"STM8 structural image exceeds flash: end=0x{flash_end:04X}, limit=0x{FLASH_END:04X}"
            )
        if data_address + data_size > RAM_SIZE:
            raise SystemExit(
                f"STM8 structural image exceeds RAM: {data_address + data_size} > {RAM_SIZE} bytes"
            )

        print(
            f"STM8 structural target compile passed with SDCC {subprocess.check_output([sdcc, '--version'], text=True).splitlines()[0]}"
        )
        print(f"flash span: {flash_end - FLASH_START} / {FLASH_END - FLASH_START} bytes")
        print(f"static DATA: {data_size} / {RAM_SIZE} bytes")
        print(
            "note: IAR register names and interrupt declarations are shimmed; this image is not flashable"
        )


if __name__ == "__main__":
    main()
