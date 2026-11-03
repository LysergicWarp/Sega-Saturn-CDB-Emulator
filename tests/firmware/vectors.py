"""The payload decoder's numerical cases, from the decoder's documented equations.

Each case is an input the firmware's decoder takes and what it must give: its result, the check
value it accumulates (A and B) and its output. No firmware code runs here. The cases cover every
length up to 23 bytes, four fixed patterns with a good and a bad check value, random lengths, every
alignment of the input and the output, an output overlapping its input below it, and the expansion
loader's largest payload. The same seed gives the same cases.

The equations are written out here apart from payload.py on purpose: these cases are what the
encoding test holds payload.py to, so they do not share its code.
"""
import random

MASK = 0xFFFFFFFF
SOURCE, DESTINATION = 0x09010000, 0x09030000
PATTERNS = {"zero": lambda i: 0, "ones": lambda i: 0xFF, "alternating": lambda i: 0xAA if i % 2 == 0 else 0x55,
            "ascending": lambda i: i & 0xFF}


def rotate(v, n):
    return ((v << n) | (v >> (32 - n))) & MASK


def swap_w(v):
    return rotate(v, 16)


def swap_b(v):
    return (v & 0xFFFF0000) | (v & 0xFF) << 8 | (v >> 8) & 0xFF


def upper(v):
    return swap_w(swap_b(swap_w(v)))


def product(v):
    return (v & 0xFFFF) * (v >> 16)


def transform(x, y, k):
    """One block, x and y, under key k: the output block and the two words halfway through, which
    the check value accumulates."""
    s0 = ((k + 0x6368692E) & MASK) ^ 0x4DBEA1EC
    q = (rotate(s0, 3) + 0x17) & MASK
    s1 = 0x48697461 ^ (q ^ rotate((product(q) + 0x69634B65) & MASK, 8))
    q = rotate((s1 + 1) & MASK, 1) ^ 0xE77F6203
    s2 = s0 ^ ((swap_w(q) + q) & MASK)
    q = (s2 + 0x39) & MASK
    s3 = s1 ^ (rotate(q, 2) ^ q)
    q = (s3 + 0x79436970) & MASK
    v = (rotate(q, 8) - q) & MASK
    q = ((swap_b(v) + v) & MASK) ^ q
    s4 = s2 ^ ((upper((product((q + 0x11) & MASK) + 1) & MASK) - 0x40044804) & MASK)
    q = ((s4 << 1) + 1) & MASK
    s5 = s3 ^ (((q ^ swap_b(q)) + 0x6865722E) & MASK)
    q = (s5 + 0xFFFFFD43) & MASK
    q = rotate(q, 3) ^ q
    s6 = s4 ^ ((swap_w(q) + q) & MASK)
    q = (s6 + 3) & MASK
    s7 = s5 ^ (rotate(q, 1) ^ q)
    y ^= (rotate(x, 1) + x + 1) & MASK
    q = (y + s0) & MASK
    q = rotate(q, 2) ^ q
    x ^= (rotate(q, 4) - q) & MASK
    x, y = ((x << 1) | (y >> 31)) & MASK, ((y << 1) | (x >> 31)) & MASK
    q = (x + s1) & MASK
    q = (swap_w(q) + q) & MASK
    q = rotate(q, 2) ^ q
    q = (q + s2) & MASK
    q = (rotate(q, 1) - q) & MASK
    y ^= rotate(q, 8) ^ (q | x)
    q = (swap_b(y) + s3) & MASK
    x ^= rotate(q, 2) ^ (q & y)
    middle = x, y
    x = rotate(x, 1)
    y = rotate(y, 1) ^ x
    q = rotate(y, 3) ^ s4
    x ^= (upper(q) + q) & MASK
    x, y = ((x << 16) | (y >> 16)) & MASK, ((y << 16) | (x >> 16)) & MASK
    q = (x + s5) & MASK
    q = rotate(q, 9) ^ q
    q = (rotate(q, 2) - q) & MASK
    q = (q - 1 + s6) & MASK
    y ^= (swap_w(q) + (q | x)) & MASK
    q = (rotate(y, 1) + s7) & MASK
    x ^= y ^ rotate(product(q), 3)
    return x, y, *middle


def word(data, offset):
    return int.from_bytes(data[offset:offset + 4], "big")


def case(rng, count, name, length, pattern="random", bad=False, source=SOURCE, destination=DESTINATION):
    """A case as (name, source, destination, length, seed, result, A, B, input, output). The last
    block of a whole input is its check value, made good unless bad. Under two blocks the decoder
    refuses the input, writes nothing and leaves A and B as the board's fill."""
    data = bytearray(rng.randbytes(length))
    if pattern in PATTERNS:
        data = bytearray(PATTERNS[pattern](i) for i in range(length))
    blocks = length >> 3
    a = b = 0xA5A5A5A5
    result = MASK
    output = bytearray(max(0, 8 * (blocks - 1)))
    if blocks >= 2:
        a = b = 0
        for i in range(blocks - 1):
            x, y, mid_x, mid_y = transform(word(data, i * 8), word(data, i * 8 + 4), blocks - 1 - i)
            output[i * 8:i * 8 + 8] = x.to_bytes(4, "big") + y.to_bytes(4, "big")
            a ^= mid_x
            b ^= mid_y
        data[(blocks - 1) * 8:blocks * 8] = a.to_bytes(4, "big") + b.to_bytes(4, "big")
        if bad:
            data[blocks * 8 - 1] ^= 1
        else:
            result = 0
    seed = rng.randrange(0x7FFFFFFF) * 2 + count % 2
    return name, source, destination, length, seed, result, a, b, bytes(data), bytes(output)


def rows(seed=7034):
    rng = random.Random(seed)
    out = []

    def add(*args, **options):
        out.append(case(rng, len(out), *args, **options))

    for length in range(24):
        add(f"length-{length}", length)
    for pattern in PATTERNS:
        for length in (16, 24, 32, 80, 256):
            add(f"{pattern}-{length}", length, pattern)
            add(f"{pattern}-{length}-bad", length, pattern, True)
    for i in range(64):
        add(f"random-{i}", rng.randrange(2, 66) * 8 + i % 8, "random", i % 3 == 0)
    for source_low in range(4):
        for destination_low in range(4):
            add(f"alignment-{source_low}-{destination_low}", 39, "ascending", False, SOURCE + source_low,
                DESTINATION + destination_low)
    add("disc-overlap", 512, "ascending", False, 0x09075848, 0x09075800)
    add("disc-overlap-bad", 512, "ascending", True, 0x09075848, 0x09075800)
    add("expansion-limit", 0x5000)
    return out
