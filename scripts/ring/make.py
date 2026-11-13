#!/usr/bin/env python3
"""Build raw sectors for the Saturn CD block's sector checks from a firmware dump.

usage:
  make.py [--shift N] DUMP
  make.py [--shift N] DUMP TIMER OUTPUT [G0 G1 G2]

With DUMP alone, make.py writes the sectors full-check.txt runs on to the current directory:
accepted.bin, short.bin and failing.bin.

TIMER is an ITU4 counter value (hex), as for ring.py's timer command. It picks the three check
positions. OUTPUT receives raw 2352-byte sectors from the first check position through the third,
about 7 MB. The three sectors at check position i read as logo row zone + i in Gi of the 106
columns (default 106). Bad columns hold the opposite byte and are spread across the row. All
other bytes re-scramble to 59, the background byte.

Each sector has the sync pattern and a Mode 1 header, but it is not a valid Mode 1 sector: its
EDC, ECC and zero fields hold bytes for the check like the rest. The comparison reads them as
stream bytes and never as EDC or ECC. What a real decoder reports for such a sector is unknown.
Another tool may reject the sectors.

make.py prints the ring.py command that checks the file. CDB104, CDB105, CDB106 and YGR022 are
recognized by SHA-1. For another dump, --shift gives the distance of its tables from their CDB105
addresses.

examples:
  make.py cdb105.bin
  make.py cdb105.bin 0145 synthetic.bin
  make.py cdb105.bin 0145 synthetic.bin 106 84 85
"""

import argparse
import pathlib
import sys

from ring import (CLEAR, COMPARE_BYTES, COMPARE_START, LOGO_COLUMNS, SCRAMBLER, SECTOR,
                  SECTOR_SMALL_FRAMES, SET, SMALL_FRAME, Check, Failure, hexadecimal, load, msf,
                  where)


def bcd(value):
    return value // 10 << 4 | value % 10


def bcd_address(frame_address):
    """A sector header's address field: minute, second and frame, each in BCD."""
    return bytes([bcd(frame_address // 4500), bcd(frame_address // 75 % 60), bcd(frame_address % 75)])


def header(frame_address):
    """Sync pattern and Mode 1 header for `frame_address`."""
    return b"\x00" + b"\xff" * 10 + b"\x00" + bcd_address(frame_address) + bytes([1])


def encode(buffer):
    """Raw sector bytes 16..2351 that re-scramble to bytes 16..2351 of `buffer`."""
    return bytes(buffer[12 + (j ^ 1)] ^ SCRAMBLER[j] for j in range(4, SECTOR - 12))


# A sector whose bytes re-scramble to 59 throughout, the background byte of the stream.
BACKGROUND = encode(bytes([CLEAR]) * SECTOR)

# The sectors full-check.txt runs on: timer, file, and good columns in each pass.
EXAMPLES = [(0x0145, "accepted.bin", [LOGO_COLUMNS] * 3),
            (0x0145, "short.bin", [106, 84, 85]),
            (0x0279, "failing.bin", [84, 85, 5])]


def spread(good):
    """`good` good columns of 106, with the bad ones spread evenly across the row."""
    bad = LOGO_COLUMNS - good
    return [(k + 1) * bad // LOGO_COLUMNS == k * bad // LOGO_COLUMNS for k in range(LOGO_COLUMNS)]


def build(tables, position, row, good):
    """Three raw sectors from frame address `position` whose stream reads as logo row `row` in
    each column where `good` is true, and as the opposite bit elsewhere. Bytes 0..15 of each
    sector, and the stream bytes they supply, stay as a drive delivers them. Every other byte
    re-scrambles to 59."""
    source = {}
    for s in range(3):
        for f in range(SECTOR_SMALL_FRAMES):
            for k, offset in enumerate(tables.spread):
                p = (SECTOR_SMALL_FRAMES * s + f) * SMALL_FRAME + offset
                if COMPARE_START <= p < COMPARE_START + COMPARE_BYTES:
                    source[p] = (s, SMALL_FRAME * f + k)
    buffers = [bytearray([CLEAR]) * SECTOR for _ in range(3)]
    p = COMPARE_START
    for logo_column in range(LOGO_COLUMNS):
        value = SET if tables.bit(row, logo_column) ^ (not good[logo_column]) else CLEAR
        for _ in range(tables.sizes[logo_column]):
            s, index = source[p]
            p += 1
            if index >= 16:
                buffers[s][index] = value
    return [header(position + s) + encode(buffers[s]) for s in range(3)]


def image(tables, check, counts):
    """Sectors from the first check position through the third, background between them."""
    first, end = check.positions[0], check.positions[2] + 3
    data = bytearray()
    for frame_address in range(first, end):
        data += header(frame_address) + BACKGROUND
    for index, position in enumerate(check.positions):
        sectors = build(tables, position, check.zone + index, spread(counts[index]))
        start = (position - first) * SECTOR
        data[start:start + 3 * SECTOR] = b"".join(sectors)
    return bytes(data)


def quoted(path):
    """A path as a command line takes it, in quotes when it has a space."""
    text = str(path)
    return f'"{text}"' if " " in text else text


def main():
    parser = argparse.ArgumentParser(
        usage=argparse.SUPPRESS, description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dump")
    parser.add_argument("timer", nargs="?")
    parser.add_argument("output", nargs="?")
    parser.add_argument("counts", nargs="*")
    parser.add_argument("--shift", type=lambda text: int(text, 0))
    options = parser.parse_intermixed_args()
    try:
        if options.timer is None:
            jobs = EXAMPLES
        else:
            if options.output is None:
                raise Failure("give TIMER and OUTPUT, or neither")
            if len(options.counts) not in (0, 3):
                raise Failure("give three good-column counts or none")
            if not all(count.isdigit() and int(count) <= LOGO_COLUMNS for count in options.counts):
                raise Failure(f"good-column counts are 0 to {LOGO_COLUMNS}")
            counts = [int(count) for count in options.counts] or [LOGO_COLUMNS] * 3
            jobs = [(hexadecimal(options.timer, 0xFFFF), options.output, counts)]
        tables = load(options.dump, options.shift)
        for timer, output, counts in jobs:
            check = Check(timer)
            data = image(tables, check, counts)
            with open(output, "wb") as file:
                file.write(data)
            first = check.positions[0]
            print(f"{output}: {len(data) // SECTOR:,} sectors from {where(first)}")
            for index, position in enumerate(check.positions):
                print(f"  pass {index}  {where(position)}, logo row {check.zone + index}, "
                      f"{counts[index]} good columns")
    except (Failure, OSError) as error:
        sys.exit(f"make.py: {error}")
    # ring.py and full-check.txt by the path this script was run with, so the command works from
    # the same folder.
    here = pathlib.Path(sys.argv[0]).parent
    print("check it with:")
    if options.timer is None:
        print(f"  python {quoted(here / 'ring.py')} {quoted(options.dump)} {quoted(here / 'full-check.txt')}")
    else:
        print(f'  python {quoted(here / "ring.py")} {quoted(options.dump)} -c "timer {timer:04x}; '
              f'image {output} {msf(first)}; pass; pass; pass"')


if __name__ == "__main__":
    main()
