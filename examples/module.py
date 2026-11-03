"""Uploads SH-1 modules to the module extension and runs a job of each.

  python examples/prs.py ORIGINAL
  python -m cdblock --firmware DUMP --extension examples/module-extension.s examples/module.py

prs.py writes decompress-input.prs, which this scenario reads from the current directory. The host
loads the card and turns it on, then builds the packages of decompress-module.s and
transform-module.s, from the example code beside module-extension.s, with the same binutils. Each
package goes into partition 0 through filter 0 with 64, AB operation 1 loads it, and AC gives its
handle. A job of the decompress module decodes the stream from partition 0 into filter 1, written
to decompress-output.bin, which equals ORIGINAL. The transform module then replaces it: the old
handle is rejected, and a job of the new one transforms a sector of vertices, written to
transform-output.bin and checked against the transform computed here. 62 deletes each partition
once its job has ended.
"""
import pathlib
import struct

from cdblock import module as modules
from cdblock.host import EHST, MPED, REJECT, RunError

CODE = pathlib.Path(__file__).resolve().parent
STREAM = pathlib.Path("decompress-input.prs")
DECOMPRESSED, TRANSFORMED = pathlib.Path("decompress-output.bin"), pathlib.Path("transform-output.bin")
SECTOR = 2048
INPUT_MOST = 32
# The buffer's slots. Those the input leaves free bound the output.
SLOTS = 200
INPUT, OUTPUT_FILTER = 0, 1
PROTOCOL = 4
LOAD = 1
JOB_ID = 1
DONE, FAILED = 3, 4
# A rotation by 30 degrees about z in Q12 with a translation, then 252 vectors (x, y, z, w).
MATRIX = ((3547, -2048, 0, 100), (2048, 3547, 0, -200), (0, 0, 4096, 300))


def words(data):
    data = data.ljust(-(-len(data) // SECTOR) * SECTOR, b"\0")
    return [data[i] << 8 | data[i + 1] for i in range(0, len(data), 2)]


def joined(values):
    return b"".join(w.to_bytes(2, "big") for w in values)


def result(what):
    """AC after a job's MPED: its result, or RunError for a failed job."""
    answer = host.command([0xAC00, 0, 0, 0], name=f"AC {what}")
    value = answer[2] << 16 | answer[3]
    if answer[0] & 0xFF == FAILED:
        raise RunError(f"{what} failed: reason {value >> 24}, {value & 0xFFFFFF} below it")
    if answer[0] & 0xFF != DONE or answer[1] != JOB_ID:
        raise RunError(f"AC answered {answer}")
    return value


def delete(partition):
    host.command([0x6200, 0, partition << 8, 0xFFFF], EHST, "62 delete")


def load(source):
    """Puts the module's package into partition 0, loads it and deletes the package. Returns the
    module's handle. Its code joins the code a strict run allows."""
    module = modules.from_code(CODE / source)
    for start, end in module.code_ranges:
        board.add_code_range(start, end)
    sectors = -(-len(module.package) // SECTOR)
    host.put(INPUT, sectors, words(module.package))
    host.command([0xAB00 | LOAD, JOB_ID, INPUT << 8, 0], MPED, f"AB load {source}")
    handle = result(f"load {source}")
    delete(INPUT)
    return handle


def vertices():
    rows = [v for row in MATRIX for v in row] + [0] * 4
    for i in range(252):
        rows += [(i * 37) % 4096 - 2048, (i * 91) % 4096 - 2048, (i * 13) % 2048 - 1024, i]
    return struct.pack(">1024h", *rows)


def transformed(sector):
    """The transform as transform-module.s gives it."""
    w = struct.unpack(">1024h", sector)
    out = list(w[:16])
    for v in range(252):
        x = w[16 + 4 * v:20 + 4 * v]
        for r in range(3):
            m = w[4 * r:4 * r + 4]
            total = (m[0] * x[0] + m[1] * x[1] + m[2] * x[2]) & 0xFFFFFFFF
            out.append(((total >> 12) + m[3]) & 0xFFFF)
        out.append(x[3])
    return struct.pack(">1024H", *(v & 0xFFFF for v in out))


if not STREAM.exists():
    raise RunError(f"{STREAM} not found: write it with examples/prs.py")
stream = STREAM.read_bytes()
sectors = -(-len(stream) // SECTOR)
if not stream:
    raise RunError(f"{STREAM} is empty: a stream holds at least its end code")
if sectors > INPUT_MOST:
    raise RunError(f"{STREAM} takes {sectors} sectors, a job takes up to {INPUT_MOST}")

host.load_card()
host.activate_card()
identity = host.command([0xAD00, 0, 0, 0], name="AD identity")
if identity[2:] != [0x444F, 0x5345] or identity[0] & 0xFF != PROTOCOL:
    raise RunError(f"AD answered {identity}: not the module extension")

decompress = load("decompress-module.s")
host.put(INPUT, sectors, words(stream))
host.command([0xAB00 | decompress, JOB_ID, INPUT << 8 | OUTPUT_FILTER, SLOTS - sectors], MPED, "AB decompress")
length = result("decompress")
filed = -(-length // SECTOR)
output = joined(host.get(OUTPUT_FILTER, 0, filed, filed * SECTOR // 2)) if filed else b""
delete(INPUT)
if filed:
    delete(OUTPUT_FILTER)
DECOMPRESSED.write_bytes(output[:length])

transform = load("transform-module.s")
stale = host.command([0xAB00 | decompress, JOB_ID, INPUT << 8 | OUTPUT_FILTER, 1], name="AB old handle")
if stale[0] >> 8 != REJECT:
    raise RunError(f"the decompress module's handle after the transform's load answered {stale}")
sector = vertices()
host.put(INPUT, 1, words(sector))
host.command([0xAB00 | transform, JOB_ID, INPUT << 8 | OUTPUT_FILTER, 1], MPED, "AB transform")
if result("transform") != 1:
    raise RunError("the transform job filed other than one sector")
output = joined(host.get(OUTPUT_FILTER, 0, 1, SECTOR // 2))
delete(INPUT)
delete(OUTPUT_FILTER)
TRANSFORMED.write_bytes(output)
if output != transformed(sector):
    raise RunError("the transform's output differs from the transform computed here")
print(f"handles {decompress} and {transform}: {len(stream):,} bytes of stream to {length:,} in {DECOMPRESSED}, "
      f"252 vertices to {TRANSFORMED}")
