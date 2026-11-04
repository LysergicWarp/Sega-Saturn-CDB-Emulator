#!/usr/bin/env python3
"""Encode and decode Saturn CD block extension payloads, and build MPEG card ROMs.

usage:
  payload.py encode INPUT OUTPUT
  payload.py decode INPUT OUTPUT
  payload.py card INPUT OUTPUT
  payload.py uncard INPUT OUTPUT

encode turns plain data into a payload the firmware's decoder accepts: the encoded blocks and the
eight-byte check value. decode reverses it and fails when the check value does not match. Plain
data is padded with zeros to whole eight-byte blocks.

card writes a 512 KB card ROM for INPUT, plain extension code: the encoded length as a big-endian
longword, the payload, and zeros. The expansion loader accepts an encoded length of 1 to 0x5000
bytes, so the code can be up to 0x4ff8 bytes. uncard decodes the code from a card ROM, the known
cards' included.

The block transform and its inverse are written from the decoder's equations, with no firmware
code. W, SB, U and P in the script's helpers are the equations' names, from the Conventions of the
firmware documentation's decoder guide:
https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/decoder.md#conventions

examples:
  payload.py card lifecycle-extension.bin lifecycle-card.bin
  payload.py uncard card.bin card-code.bin
"""

import argparse
import sys

MASK = 0xFFFFFFFF
CARD_ROM = 0x80000
CARD_PAYLOAD = 0x5000


class PayloadError(Exception):
    pass


def rotl(v, n):
    return ((v << n) | (v >> (32 - n))) & MASK


def halves(v):
    """W: the two halfwords swapped."""
    return rotl(v, 16)


def low_bytes(v):
    """SB: the low two bytes swapped."""
    return (v & 0xFFFF0000) | ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)


def high_bytes(v):
    """U: the high two bytes swapped."""
    return halves(low_bytes(halves(v)))


def product(v):
    """P: the low halfword times the high one, as MULU.W gives it."""
    return (v & 0xFFFF) * (v >> 16)


def pair_rotl(x, y, n):
    """The 64-bit pair x:y rotated left by n, 1 <= n < 32."""
    return (x << n | y >> (32 - n)) & MASK, (y << n | x >> (32 - n)) & MASK


def pair_rotr(x, y, n):
    """The 64-bit pair x:y rotated right by n, which undoes pair_rotl."""
    return (x >> n | y << (32 - n)) & MASK, (y >> n | x << (32 - n)) & MASK


def schedule(k):
    """The eight schedule words for remaining block count k.

    There is no key: the firmware reads its large constants from its own ROM. 0x48697461,
    0x6368692E, 0x69634B65, 0x79436970 and 0x6865722E are "Hita", "chi.", "icKe", "yCip" and "her."
    of the text Hitachi.PublicKeyCipher., and 0xE77F6203 and 0x40044804 are ROM words from its code.
    0x4DBEA1EC and 0xFFFFFD43 are computed from ROM words and code addresses, and the small ones are
    immediates.
    """
    s = [0] * 8
    s[0] = ((k + 0x6368692E) & MASK) ^ 0x4DBEA1EC
    q = (rotl(s[0], 3) + 0x17) & MASK
    s[1] = 0x48697461 ^ q ^ rotl((product(q) + 0x69634B65) & MASK, 8)
    q = rotl((s[1] + 1) & MASK, 1) ^ 0xE77F6203
    s[2] = s[0] ^ ((halves(q) + q) & MASK)
    q = (s[2] + 0x39) & MASK
    s[3] = s[1] ^ rotl(q, 2) ^ q
    q = (s[3] + 0x79436970) & MASK
    v = (rotl(q, 8) - q) & MASK
    q = ((low_bytes(v) + v) & MASK) ^ q
    s[4] = s[2] ^ ((high_bytes((product((q + 0x11) & MASK) + 1) & MASK) - 0x40044804) & MASK)
    q = ((s[4] << 1) + 1) & MASK
    s[5] = s[3] ^ (((q ^ low_bytes(q)) + 0x6865722E) & MASK)
    q = (s[5] + 0xFFFFFD43) & MASK
    q = rotl(q, 3) ^ q
    s[6] = s[4] ^ ((halves(q) + q) & MASK)
    q = (s[6] + 3) & MASK
    s[7] = s[5] ^ rotl(q, 1) ^ q
    return s


def decode_block(x, y, k):
    """The decoder's transform of one block for count k: the output pair and the intermediate pair
    its check value covers."""
    s = schedule(k)
    y ^= (rotl(x, 1) + x + 1) & MASK
    q = (y + s[0]) & MASK
    q = rotl(q, 2) ^ q
    x ^= (rotl(q, 4) - q) & MASK
    x, y = pair_rotl(x, y, 1)
    q = (x + s[1]) & MASK
    q = (halves(q) + q) & MASK
    q = rotl(q, 2) ^ q
    q = (q + s[2]) & MASK
    q = (rotl(q, 1) - q) & MASK
    y ^= rotl(q, 8) ^ (q | x)
    q = (low_bytes(y) + s[3]) & MASK
    x ^= rotl(q, 2) ^ (q & y)
    middle = x, y
    x = rotl(x, 1)
    y = rotl(y, 1) ^ x
    q = rotl(y, 3) ^ s[4]
    x ^= (high_bytes(q) + q) & MASK
    x, y = pair_rotl(x, y, 16)
    q = (x + s[5]) & MASK
    q = rotl(q, 9) ^ q
    q = (rotl(q, 2) - q) & MASK
    q = (q - 1 + s[6]) & MASK
    y ^= (halves(q) + (q | x)) & MASK
    q = (rotl(y, 1) + s[7]) & MASK
    x ^= y ^ rotl(product(q), 3)
    return (x, y), middle


def encode_block(x, y, k):
    """The inverse: from the wanted output pair for count k to the encoded pair, with the
    intermediate pair the decoder will reach."""
    s = schedule(k)
    q = (rotl(y, 1) + s[7]) & MASK
    x ^= y ^ rotl(product(q), 3)
    q = (x + s[5]) & MASK
    q = rotl(q, 9) ^ q
    q = (rotl(q, 2) - q) & MASK
    q = (q - 1 + s[6]) & MASK
    y ^= (halves(q) + (q | x)) & MASK
    x, y = pair_rotr(x, y, 16)
    q = rotl(y, 3) ^ s[4]
    x ^= (high_bytes(q) + q) & MASK
    y = rotl(y ^ x, 31)
    x = rotl(x, 31)
    middle = x, y
    q = (low_bytes(y) + s[3]) & MASK
    x ^= rotl(q, 2) ^ (q & y)
    q = (x + s[1]) & MASK
    q = (halves(q) + q) & MASK
    q = rotl(q, 2) ^ q
    q = (q + s[2]) & MASK
    q = (rotl(q, 1) - q) & MASK
    y ^= rotl(q, 8) ^ (q | x)
    x, y = pair_rotr(x, y, 1)
    q = (y + s[0]) & MASK
    q = rotl(q, 2) ^ q
    x ^= (rotl(q, 4) - q) & MASK
    y ^= (rotl(x, 1) + x + 1) & MASK
    return (x, y), middle


def words(data, at):
    return int.from_bytes(data[at:at + 4], "big"), int.from_bytes(data[at + 4:at + 8], "big")


def pair_bytes(pair):
    return pair[0].to_bytes(4, "big") + pair[1].to_bytes(4, "big")


def pad(data):
    """Zeros to whole eight-byte blocks."""
    return bytes(data) + bytes(-len(data) % 8)


def decode_with_check(encoded):
    """What the decoder computes for a payload of len(encoded) bytes: the output, the check value
    it reaches and the trailer it compares with. Bytes past the last whole block are ignored."""
    count = len(encoded) >> 3
    if count < 2:
        raise PayloadError("a payload needs at least two blocks, the data and the check value")
    out, a, b = bytearray(), 0, 0
    for i in range(count - 1):
        pair, middle = decode_block(*words(encoded, 8 * i), count - 1 - i)
        out += pair_bytes(pair)
        a ^= middle[0]
        b ^= middle[1]
    return bytes(out), (a, b), words(encoded, 8 * (count - 1))


def decode(encoded):
    out, check, trailer = decode_with_check(encoded)
    if check != trailer:
        raise PayloadError(f"check value {pair_bytes(check).hex()} does not match the trailer "
                           f"{pair_bytes(trailer).hex()}")
    return out


def encode(plain):
    """The payload for plain data, padded to whole blocks: the encoded blocks, then the check value."""
    plain = pad(plain)
    count = len(plain) // 8
    if not count:
        raise PayloadError("nothing to encode")
    out, a, b = bytearray(), 0, 0
    for i in range(count):
        pair, middle = encode_block(*words(plain, 8 * i), count - i)
        out += pair_bytes(pair)
        a ^= middle[0]
        b ^= middle[1]
    return bytes(out) + pair_bytes((a, b))


def card_rom(code):
    """A 512 KB card ROM for plain extension code."""
    payload = encode(code)
    if len(payload) > CARD_PAYLOAD:
        raise PayloadError(f"{len(code):#x} bytes of code encode to {len(payload):#x} bytes, past the "
                           f"loader's {CARD_PAYLOAD:#x}")
    rom = len(payload).to_bytes(4, "big") + payload
    return rom + bytes(CARD_ROM - len(rom))


def card_code(rom):
    """The plain code of a card ROM, as the expansion loader decodes it."""
    length = int.from_bytes(rom[:4], "big")
    if not 1 <= length <= CARD_PAYLOAD:
        raise PayloadError(f"encoded length {length:#x} outside the loader's 1..{CARD_PAYLOAD:#x}")
    return decode(rom[4:4 + length])


def main():
    parser = argparse.ArgumentParser(
        usage=argparse.SUPPRESS, description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=["encode", "decode", "card", "uncard"])
    parser.add_argument("input")
    parser.add_argument("output")
    args = parser.parse_args()
    with open(args.input, "rb") as f:
        data = f.read()
    action = {"encode": encode, "decode": decode, "card": card_rom, "uncard": card_code}[args.action]
    try:
        result = action(data)
    except PayloadError as error:
        sys.exit(f"payload.py: {error}")
    with open(args.output, "wb") as f:
        f.write(result)
    print(f"{args.output}: {len(result)} bytes")


if __name__ == "__main__":
    main()
