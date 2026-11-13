"""R-W subcode as a disc holds it, for the disc drive's subcode stream.

A pack is 24 six-bit symbols: symbols 0 and 1 and 16 data symbols, the Q parity in symbols 2 and 3
over symbols 0..3, and the P parity in symbols 20..23 over the whole pack. Both are Reed-Solomon
codes over GF(64) with the field polynomial x^6 + x + 1. On the disc symbols 1 and 18, 2 and 5, 3
and 23 are swapped and symbol n comes n mod 8 packs late. A sector's subcode holds four packs of the
stream, 96 symbols.
"""


def _field():
    """The powers of alpha = x, twice over, and the logarithms."""
    exp, log, x = [0] * 126, [0] * 64, 1
    for i in range(63):
        exp[i], log[x] = x, i
        x <<= 1
        if x & 0x40:
            x ^= 0x43
    exp[63:] = exp[:63]
    return exp, log


EXP, LOG = _field()
PACK, SECTOR_PACKS = 24, 4
# The pack number a pack carries in symbols 4 and 5 wraps here.
NUMBERS = 1 << 12


def mul(a, b):
    return EXP[LOG[a] + LOG[b]] if a and b else 0


def div(a, b):
    return EXP[LOG[a] + 63 - LOG[b]] if a else 0


def solve(a, b):
    """A e = b over GF(64) for a regular square A."""
    a, b, n = [list(row) for row in a], list(b), len(b)
    for c in range(n):
        r = next(r for r in range(c, n) if a[r][c])
        a[r], a[c] = a[c], a[r]
        b[r], b[c] = b[c], b[r]
        inverse = div(1, a[c][c])
        a[c] = [mul(v, inverse) for v in a[c]]
        b[c] = mul(b[c], inverse)
        for k in range(n):
            if k != c and a[k][c]:
                f = a[k][c]
                a[k] = [v ^ mul(f, w) for v, w in zip(a[k], a[c])]
                b[k] ^= mul(f, b[c])
    return b


def p_weight(k, i):
    """Symbol i's weight in the P syndrome S_k: symbol 23 - j has the locator alpha^j."""
    return EXP[k * (23 - i) % 63]


def q_parity(s):
    """Symbols 2 and 3, so that symbols 0..3 sum to 0 plainly and weighted by alpha^(3 - i)."""
    s[2], s[3] = solve([[1, 1], [EXP[1], 1]], [s[0] ^ s[1], mul(s[0], EXP[3]) ^ mul(s[1], EXP[2])])


def p_parity(s):
    """Symbols 20..23, so that the four P syndromes of the pack are 0."""
    a = [[p_weight(k, 20 + j) for j in range(4)] for k in range(4)]
    b = [0] * 4
    for k in range(4):
        for i in range(20):
            b[k] ^= mul(s[i], p_weight(k, i))
    s[20:24] = solve(a, b)


def pack(x):
    """Pack x: mode 1, item 1 in symbol 0, an instruction, x in symbols 4 and 5, 14 more data
    symbols that follow from x, and both parities."""
    x %= NUMBERS
    s = [0] * PACK
    s[0], s[1], s[4], s[5] = 0x09, 0x06, x & 63, x >> 6
    for i in range(6, 20):
        s[i] = (x * 7 + i * 13 + 1) & 63
    q_parity(s)
    p_parity(s)
    return s


def number(symbols):
    """The pack number a pack carries."""
    return symbols[4] | symbols[5] << 6


def swapped(s):
    s = list(s)
    s[1], s[18], s[2], s[5], s[3], s[23] = s[18], s[1], s[5], s[2], s[23], s[3]
    return s


def sector(n):
    """The subcode of sector n of the stream, 96 symbols: stream packs 4n..4n + 3, where symbol m of
    stream pack y is symbol m of pack y - m mod 8 after the swaps."""
    packs = {}
    out = bytearray()
    for y in range(SECTOR_PACKS * n, SECTOR_PACKS * (n + 1)):
        for m in range(PACK):
            x = y - m % 8
            if x not in packs:
                packs[x] = swapped(pack(x))
            out.append(packs[x][m])
    return bytes(out)
