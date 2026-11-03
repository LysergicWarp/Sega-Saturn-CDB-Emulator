"""Decompresses a PRS stream read from a disc, with the disc decompress extension.

  python examples/prs.py ORIGINAL
  python -m cdblock --firmware DUMP --extension examples/disc-decompress-extension.s examples/disc-decompress.py

prs.py writes decompress-input.prs, which this scenario reads from the current directory and puts
on a disc in mode 1 sectors from frame address 1000 (00:13:25). The disc goes into the drive, which
reads its TOC and pauses, and E0 checks it: the drive answers with values the firmware accepts,
Yabause's seek answers and the ring sectors built from the dump, and E1 gives result 4. An accepted
disc clears the card's expansion area, so the card loads after the check, and 93 turns it on. 30
connects the drive to filter 0 and 10 reads the stream's sectors into partition 0. AB decodes them
there into sectors filed through filter 1, so only the output reaches the host: 61 gets it from
partition 1, and 62 deletes both partitions. The output goes to decompress-output.bin, which equals
ORIGINAL. The disc needs the firmware's profile, so this runs on the four known revisions.
"""
import pathlib

from cdblock.disc import Disc, Drive
from cdblock.host import EHST, MPED, RunError

STREAM, OUTPUT = pathlib.Path("decompress-input.prs"), pathlib.Path("decompress-output.bin")
SECTOR = 2048
INPUT_MOST = 32
# The buffer's slots. Those the input leaves free bound the output.
SLOTS = 200
STREAM_FRAME_ADDRESS = 1000
# The partition the check reads into, and the job's input partition and output filter.
CHECK, INPUT, OUTPUT_FILTER = 2, 0, 1
ACCEPTED = 4
PROTOCOL = 3
JOB_ID = 1
DONE, FAILED = 3, 4
REASONS = {1: "no input sector or more than 32", 2: "the input ends before the end code",
           3: "a copy from before the output's start", 4: "more output than the limit", 5: "no free slot",
           6: "a mode 2 form 2 input sector"}

if not STREAM.exists():
    raise RunError(f"{STREAM} not found: write it with examples/prs.py")
stream = STREAM.read_bytes()
sectors = -(-len(stream) // SECTOR)
if not stream:
    raise RunError(f"{STREAM} is empty: a stream holds at least its end code")
if sectors > INPUT_MOST:
    raise RunError(f"{STREAM} takes {sectors} sectors, a job takes up to {INPUT_MOST}")

drive = Drive(host)
disc = Disc(board.peek(0, 0x10000))
disc.write(STREAM_FRAME_ADDRESS, stream)
drive.insert(disc)
result = drive.authenticate(CHECK)
if result & 0xFF != ACCEPTED:
    raise RunError(f"E1 answered result {result & 0xFF}: the disc was not accepted")

host.load_card()
host.activate_card()
identity = host.command([0xAD00, 0, 0, 0], name="AD identity")
if identity[2:] != [0x444F, 0x5345] or identity[0] & 0xFF != PROTOCOL:
    raise RunError(f"AD answered {identity}: not the disc decompress extension")

drive.read_sectors(INPUT, STREAM_FRAME_ADDRESS, sectors)
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
print(f"{len(stream):,} bytes of stream in {sectors} sectors read from the disc, {result:,} bytes out to {OUTPUT}")
