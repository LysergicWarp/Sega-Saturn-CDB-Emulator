"""Module packages for module-extension.s: a module's image from the module slot, header first, with
the CRC-32 of the bytes before its last longword in that longword. Assembled from .s with the
reader's binutils and the .ld beside it, read from .elf, or taken from a plain .bin.
"""
import pathlib
import struct
import zlib
from collections import namedtuple

from . import extension

# The module slot of module-extension.s, where a module is linked to run.
SLOT, SLOT_END = 0x0907C000, 0x09080000
MAGIC = 0x444F534D  # "DOSM"

# name: what it came from. package: the image with its CRC. code_ranges: where its instructions lie
# once installed, as (start, end).
Module = namedtuple("Module", "name package code_ranges")


def with_crc(image):
    """The image with its last longword the CRC-32 of the bytes before it."""
    image = bytes(image)
    if len(image) < 36 or len(image) % 4:
        raise extension.ExtensionError(f"a module image is a multiple of 4 bytes from 36, this is {len(image)}")
    magic, _, _, size = struct.unpack_from(">IHHI", image)
    if magic != MAGIC:
        raise extension.ExtensionError(f"no module header: {magic:#010x} at +0, not {MAGIC:#010x}")
    if size != len(image):
        raise extension.ExtensionError(f"the header gives {size} image bytes, the image is {len(image)}")
    return image[:-4] + struct.pack(">I", zlib.crc32(image[:-4]))


def from_code(path, prefix=None):
    """A module's package from .s, .elf or .bin."""
    path = pathlib.Path(path)
    suffix = path.suffix.lower()
    if suffix in (".s", ".elf"):
        elf = extension.assemble(path, prefix) if suffix == ".s" else path.read_bytes()
        try:
            base, image, ranges = extension.elf_image(elf)
        except (IndexError, struct.error):
            raise extension.ExtensionError(f"{path.name}: the ELF file ends inside its headers") from None
        if base != SLOT:
            raise extension.ExtensionError(f"{path.name} is linked at {base:#010x}, a module runs in the slot at {SLOT:#010x}")
    elif suffix == ".bin":
        image = path.read_bytes()
        ranges = [(SLOT, SLOT + len(image))]
    else:
        raise extension.ExtensionError(f"{path.name}: a module is .s, .elf or .bin")
    if SLOT + len(image) > SLOT_END:
        raise extension.ExtensionError(f"{path.name}: {len(image)} bytes, the slot holds {SLOT_END - SLOT}")
    try:
        return Module(path.name, with_crc(image), ranges)
    except extension.ExtensionError as error:
        raise extension.ExtensionError(f"{path.name}: {error}") from None
