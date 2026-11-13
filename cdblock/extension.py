"""Card code for a run: assembled from .s with the reader's binutils, read from .elf or .bin, or
decoded from a card ROM already encoded. payload.py builds the card ROM for the others.
"""
import os
import pathlib
import shutil
import struct
import subprocess
import sys
import tempfile
from collections import namedtuple

from . import _scripts

payload = _scripts.load("payload", "payload")

# The expansion loader decodes card code here, where the known cards' code is linked to run.
LOAD = 0x0907B000
CARD_ROM = payload.CARD_ROM

# name: what it came from. code: the plain code the loader will decode. card: the card ROM.
# code_ranges: where its instructions lie once loaded, as (start, end).
Extension = namedtuple("Extension", "name code card code_ranges")

# The assembler the Dockerfile builds for this platform, beside its linker.
BINUTILS = pathlib.Path(__file__).resolve().parents[1] / "build" / "binutils"
BUILT_ASSEMBLER = {"win32": BINUTILS / "windows" / "sh-elf-as.exe", "linux": BINUTILS / "linux" / "sh-elf-as"}.get(sys.platform)


class ExtensionError(Exception):
    pass


def elf_image(data):
    """A linked ELF file as objcopy -O binary gives it, from its lowest load address, with the
    address and the ranges of its executable sections."""
    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 2:
        raise ExtensionError("not a 32-bit big-endian ELF file")
    (_, machine, _, _, phoff, shoff, _, _, phentsize, phnum, shentsize, shnum, _) = struct.unpack_from(">HHIIIIIHHHHHH", data, 16)
    if machine != 42:
        raise ExtensionError(f"ELF machine {machine}, not SuperH (42)")
    segments = []
    for i in range(phnum):
        kind, offset, _, paddr, filesz, _, _, _ = struct.unpack_from(">IIIIIIII", data, phoff + i * phentsize)
        if kind == 1 and filesz:
            if offset + filesz > len(data):
                raise ExtensionError("a loadable segment runs past the end of the file")
            segments.append((paddr, data[offset:offset + filesz]))
    if not segments:
        raise ExtensionError("no loadable segment")
    base = min(address for address, _ in segments)
    image = bytearray(max(address + len(content) for address, content in segments) - base)
    for address, content in segments:
        image[address - base:address - base + len(content)] = content
    ranges = []
    for i in range(shnum):
        _, _, flags, address, _, size, _, _, _, _ = struct.unpack_from(">IIIIIIIIII", data, shoff + i * shentsize)
        if flags & 0x4 and size:  # SHF_EXECINSTR
            ranges.append((address, address + size))
    return base, bytes(image), ranges


def binutils_prefix():
    """CDBLOCK_BINUTILS, the first of sh-elf- and sh2eb-elf- on PATH, or the binutils the Dockerfile
    built in build/binutils/."""
    prefix = os.environ.get("CDBLOCK_BINUTILS")
    if prefix:
        return prefix
    for candidate in ("sh-elf-", "sh2eb-elf-"):
        if shutil.which(candidate + "as"):
            return candidate
    if BUILT_ASSEMBLER and BUILT_ASSEMBLER.is_file():
        return str(BUILT_ASSEMBLER.parent / "sh-elf-")
    raise ExtensionError("no sh-elf- or sh2eb-elf- binutils on PATH or in build/binutils/, set CDBLOCK_BINUTILS to their prefix")


def assemble(source, prefix=None):
    """Assembles source for big-endian SH-1 and links it with the .ld beside it. Returns the ELF."""
    script = source.with_suffix(".ld")
    if not script.exists():
        raise ExtensionError(f"{script.name} not found beside {source.name}: the linker script that places the code")
    prefix = prefix or binutils_prefix()
    with tempfile.TemporaryDirectory() as work:
        obj, elf = pathlib.Path(work) / "code.o", pathlib.Path(work) / "code.elf"
        for command in ([f"{prefix}as", "--big", "--isa=sh", "-o", str(obj), str(source)],
                        [f"{prefix}ld", "--no-relax", "-T", str(script), "-o", str(elf), str(obj)]):
            try:
                result = subprocess.run(command, capture_output=True, text=True)
            except FileNotFoundError:
                raise ExtensionError(f"{command[0]} not found") from None
            if result.returncode:
                raise ExtensionError(f"{command[0]} failed:\n{result.stderr.strip()}")
        return elf.read_bytes()


def from_code(path, prefix=None):
    """An extension from .s, .elf or .bin, its card ROM built by payload.py."""
    path = pathlib.Path(path)
    suffix = path.suffix.lower()
    if suffix in (".s", ".elf"):
        elf = assemble(path, prefix) if suffix == ".s" else path.read_bytes()
        try:
            base, code, ranges = elf_image(elf)
        except (IndexError, struct.error):
            raise ExtensionError(f"{path.name}: the ELF file ends inside its headers") from None
        if base != LOAD:
            raise ExtensionError(f"{path.name} is linked at {base:#010x}, the expansion loader runs card code at {LOAD:#010x}")
    elif suffix == ".bin":
        code = path.read_bytes()
        ranges = [(LOAD, LOAD + len(code))]
    else:
        raise ExtensionError(f"{path.name}: an extension is .s, .elf or .bin")
    try:
        card = payload.card_rom(code)
    except payload.PayloadError as error:
        raise ExtensionError(f"{path.name}: {error}") from None
    return Extension(path.name, code, card, ranges)


def from_card(path):
    """An extension from a card ROM already encoded, loaded unchanged. Its code is decoded first, so
    a ROM the loader would refuse fails here."""
    path = pathlib.Path(path)
    rom = path.read_bytes()
    if not rom or len(rom) > CARD_ROM:
        raise ExtensionError(f"{path.name}: a card ROM is 1 byte to 512 KB, this is {len(rom)}")
    try:
        code = payload.card_code(rom)
    except payload.PayloadError as error:
        raise ExtensionError(f"{path.name}: {error}") from None
    return Extension(path.name, code, rom, [(LOAD, LOAD + len(code))])
