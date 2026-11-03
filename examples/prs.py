#!/usr/bin/env python3
"""Compress a file into the PRS stream the decompress extensions decode.

usage:
  prs.py INPUT [OUTPUT]

OUTPUT defaults to decompress-input.prs, the stream decompress.py puts into the CD block's buffer.
A job takes up to 32 sectors of it, 65,536 bytes.

The stream is the one in the header of decompress-extension.s, PRS as fuzziqer's compressor writes
it. Control bits are taken lowest first from control bytes, each read from the stream when the one
before is used up:
- 1, then a literal byte.
- 0, 0, then bits b1 and b2, then a byte o: a copy of b1 * 2 + b2 + 2 bytes from 256 - o back.
- 0, 1, then a little-endian word w: 0 ends the stream. Otherwise a copy from 8192 - (w >> 3)
  back, of (w & 7) + 2 bytes, or of the next byte + 1 when w & 7 is 0.

Each copy is the longest found among the 64 nearest earlier positions that start with the same two
bytes, the nearest of equal ones.

examples:
  prs.py original.bin
  prs.py original.bin stream.prs
"""

import argparse
import sys

SHORT_REACH, SHORT_MOST = 256, 5
# A distance of 8192 with the length in the next byte would encode as the end, so the reach stops
# one short of it.
LONG_REACH, LONG_WORD_MOST, LONG_MOST = 8191, 9, 256
# Earlier positions with the same next two bytes that are tried for a copy, the nearest first.
CANDIDATES = 64
OUTPUT = "decompress-input.prs"


class Stream:
    """The compressed bytes, each control byte where the decoder reads it, among the data bytes."""

    def __init__(self):
        self.data = bytearray()
        self.control = 0
        self.used = 8

    def bit(self, value):
        if self.used == 8:
            self.control = len(self.data)
            self.data.append(0)
            self.used = 0
        self.data[self.control] |= value << self.used
        self.used += 1

    def byte(self, value):
        self.data.append(value)


def longest(data, at, earlier):
    """The longest copy for the bytes at `at`: its length and distance, (0, 0) for none. A copy may
    run into the bytes it produces."""
    best = (0, 0)
    end = min(len(data), at + LONG_MOST)
    for start in reversed(earlier.get(data[at:at + 2], [])[-CANDIDATES:]):
        distance = at - start
        if distance > LONG_REACH:
            break
        n = 0
        while at + n < end and data[start + n] == data[at + n]:
            n += 1
        if n > best[0] and (n >= 3 or distance <= SHORT_REACH):
            best = (n, distance)
            if at + n == end:
                break
    return best


def compress(data):
    data = bytes(data)
    out = Stream()
    earlier = {}
    at = 0
    while at < len(data):
        n, distance = longest(data, at, earlier)
        if 2 <= n <= SHORT_MOST and distance <= SHORT_REACH:
            for bit in (0, 0, (n - 2) >> 1, (n - 2) & 1):
                out.bit(bit)
            out.byte(256 - distance & 0xFF)
        elif n >= 3:
            out.bit(0)
            out.bit(1)
            word = (8192 - distance) << 3 | (n - 2 if n <= LONG_WORD_MOST else 0)
            out.byte(word & 0xFF)
            out.byte(word >> 8)
            if n > LONG_WORD_MOST:
                out.byte(n - 1)
        else:
            n = 1
            out.bit(1)
            out.byte(data[at])
        for k in range(at, at + n):
            earlier.setdefault(data[k:k + 2], []).append(k)
        at += n
    out.bit(0)
    out.bit(1)
    out.byte(0)
    out.byte(0)
    return bytes(out.data)


def main():
    parser = argparse.ArgumentParser(
        usage=argparse.SUPPRESS, description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input")
    parser.add_argument("output", nargs="?", default=OUTPUT)
    args = parser.parse_args()
    try:
        with open(args.input, "rb") as file:
            data = file.read()
        stream = compress(data)
        with open(args.output, "wb") as file:
            file.write(stream)
    except OSError as error:
        sys.exit(f"prs.py: {error}")
    sectors = -(-len(stream) // 2048)
    print(f"{args.output}: {len(stream):,} bytes from {len(data):,}, {sectors} sectors of 2048")


if __name__ == "__main__":
    main()
