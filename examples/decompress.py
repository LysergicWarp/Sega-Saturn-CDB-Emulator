"""Decompresses a PRS stream on the CD block with the decompress extension.

  python examples/prs.py ORIGINAL
  python -m cdblock --firmware DUMP --extension examples/decompress-extension.s examples/decompress.py

prs.py writes decompress-input.prs, which this scenario reads from the current directory. The host
loads the card and turns it on, puts the stream into partition 0 through filter 0 with 64, and
submits a decompress job with AB, its output filed through filter 1. Task 9 decodes the stream a
sector per pass while the CD tasks go on, and writes MPED at the end. AC gives the output's length,
61 gets the output from partition 1, and 62 deletes both partitions, each delete ending with EHST.
62 on an empty partition answers WAIT, so an empty output is not deleted.
The output goes to decompress-output.bin, which equals ORIGINAL.
"""
import pathlib

from cdblock.host import EHST, MPED, RunError

STREAM, OUTPUT = pathlib.Path("decompress-input.prs"), pathlib.Path("decompress-output.bin")
SECTOR = 2048
INPUT_MOST = 32
# The buffer's slots. Those the input leaves free bound the output.
SLOTS = 200
INPUT, OUTPUT_FILTER = 0, 1
PROTOCOL = 2
JOB_ID = 1
DONE, FAILED = 3, 4
REASONS = {1: "no input sector or more than 32", 2: "the input ends before the end code",
           3: "a copy from before the output's start", 4: "more output than the limit", 5: "no free slot"}


def words(data):
    return [data[i] << 8 | data[i + 1] for i in range(0, len(data), 2)]


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
    raise RunError(f"AD answered {identity}: not the decompress extension")

host.put(INPUT, sectors, words(stream.ljust(sectors * SECTOR, b"\0")))
host.command([0xAB01, JOB_ID, INPUT << 8 | OUTPUT_FILTER, SLOTS - sectors], MPED, "AB decompress")
report = host.command([0xAC00, 0, 0, 0], name="AC job report")
result = report[2] << 16 | report[3]
if report[0] & 0xFF == FAILED:
    raise RunError(f"the job failed: {REASONS.get(result >> 24, result >> 24)}, {result & 0xFFFFFF} bytes filed")
if report[0] & 0xFF != DONE or report[1] != JOB_ID:
    raise RunError(f"AC answered {report}")

filed = -(-result // SECTOR)
output = b"".join(w.to_bytes(2, "big") for w in host.get(OUTPUT_FILTER, 0, filed, filed * SECTOR // 2)) if filed else b""
host.command([0x6200, 0, INPUT << 8, 0xFFFF], EHST, "62 delete input")
if filed:
    host.command([0x6200, 0, OUTPUT_FILTER << 8, 0xFFFF], EHST, "62 delete output")
OUTPUT.write_bytes(output[:result])
print(f"{len(stream):,} bytes of stream in {sectors} sectors, {result:,} bytes out to {OUTPUT}")
