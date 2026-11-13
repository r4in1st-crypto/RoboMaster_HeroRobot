"""Windows VS Code task entry points for the direct-mode C-board chassis."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess

ROOT = Path(__file__).resolve().parent
BUNDLES = Path(os.environ.get("LOCALAPPDATA", "")) / "stm32cube/bundles"
FLASH_SIZE = 1024 * 1024
SECTOR_SIZE = 16 * 1024


def tool(variable, exe, package, preferred=None):
    configured = os.environ.get(variable)
    if configured:
        candidate = Path(configured) / exe
        if candidate.is_file():
            return candidate
        raise ValueError(f"{variable}: missing {candidate}")
    base = BUNDLES / package
    candidates = [base / preferred / "bin" / exe] if preferred else []
    if found := shutil.which(exe):
        candidates.append(Path(found))
    candidates.extend(sorted(base.glob(f"*/bin/{exe}"), reverse=True))
    for path in candidates:
        if path.is_file():
            return path.resolve()
    raise ValueError(f"Set {variable} to the directory containing {exe}")


def checked_image():
    path = ROOT / "build/type_c_dr16_direct.bin"
    manifest = json.loads((path.parent / "build.json").read_text(encoding="utf-8-sig"))
    for entry in manifest["source_sha256"]:
        source = ROOT / entry["name"]
        if hashlib.sha256(source.read_bytes()).hexdigest() != entry["sha256"].lower():
            raise ValueError(f"Source differs from build manifest: {source.name}")
    image = path.read_bytes()
    if not 8 <= len(image) <= SECTOR_SIZE:
        raise ValueError("Unexpected F407 image size")
    sp, entry = struct.unpack_from("<II", image)
    if sp != 0x20020000 or not entry & 1 or not 0x08000000 <= entry < 0x08000000 + len(image):
        raise ValueError("Unexpected F407 startup vectors")
    digest = hashlib.sha256(image).hexdigest()
    if not any(a["name"] == path.name and a["sha256"].lower() == digest for a in manifest["outputs"]):
        raise ValueError("Image differs from build manifest")
    return image, digest


def flash(cli, image):
    out = ROOT / "flash_backups" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out.mkdir(parents=True)
    print(f"Backup and log: {out}", flush=True)
    port = ["port=SWD", "freq=1000"]

    def run(*args):
        command = [str(cli), "-c", *port, *map(str, args)]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        with (out / "flash.log").open("ab") as log:
            log.write(result.stdout)
        output = result.stdout.decode(errors="replace")
        print(output, flush=True)
        result.check_returncode()
        return output

    probe = run("-r32", "0xE0042000", "4")
    serial = re.search(r"ST-LINK SN\s*:\s*([0-9a-f]+)\b", probe, re.I)
    chip = re.search(r"Device ID\s*:\s*0x0*413\b", probe, re.I)
    if not serial or not chip:
        raise ValueError("Expected an identified ST-LINK and STM32F405/407 device ID 0x413")
    port.append("sn=" + serial[1])

    def read(address, length, filename):
        dest = out / filename
        run("-u", hex(address), hex(length), dest)
        data = dest.read_bytes()
        if len(data) != length:
            raise ValueError(f"Short read: {dest}")
        return data

    if int.from_bytes(read(0x1FFF7A22, 2, "flash_size.bin"), "little") != 1024:
        raise ValueError("Expected 1024 KiB Flash")
    before = read(0x08000000, FLASH_SIZE, "before.bin")
    if before != read(0x08000000, FLASH_SIZE, "before_second.bin"):
        raise ValueError("Pre-flash backup reads disagree")
    staged = out / "type_c_dr16_direct.bin"
    staged.write_bytes(image)
    run("-d", staged, "0x08000000", "-v")
    after = read(0x08000000, FLASH_SIZE, "after.bin")
    if after[:len(image)] != image or after[SECTOR_SIZE:] != before[SECTOR_SIZE:]:
        raise ValueError("Independent flash readback mismatch")
    run("-rst")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("rebuild", "check", "flash"))
    action = parser.parse_args().action
    if action == "rebuild":
        gcc = tool("HERO_ARM_BIN", "arm-none-eabi-gcc.exe", "gnu-tools-for-stm32", "14.3.1+st.2")
        subprocess.run(["pwsh", "-NoProfile", "-File", str(ROOT / "build.ps1"),
                        "-ToolchainBin", str(gcc.parent)], check=True)
        return
    cli = tool("HERO_PROGRAMMER_BIN", "STM32_Programmer_CLI.exe", "programmer")
    image, digest = checked_image()
    print(f"Programmer: {cli}\nImage: {len(image)} bytes, SHA-256 {digest}", flush=True)
    if action == "flash":
        flash(cli, image)


if __name__ == "__main__":
    main()
