"""payload.py, the encoder and decoder of extension payloads, against vectors.py's cases and the known
cards' ROMs. No emulator.

- Each case's input decodes to its output and check value, as the firmware's decoder gives them.
- Plain data of assorted sizes encodes to a payload that decodes back, alone and in a card ROM.
- Each known card's code encodes back to the payload in its ROM, byte for byte.
"""
import hashlib
import random

from cdblock.extension import payload

from . import vectors
from .common import CARDS, Suite

# The card code's two builds, by SHA-1 of the decoded code, and their version bytes.
DECODED = {"74beb5ab13c72279f99fa82cbf5905c1b162b607": 1, "b9bd40dea50a5ed00abed5c9cab5bcab6026293b": 2}


def check_vectors(suite, rows):
    case = suite.case("decoder vectors")
    for name, _, _, length, _, ret, a, b, data, expected in rows:
        try:
            out, check, trailer = payload.decode_with_check(data[:length])
        except payload.PayloadError:
            # Under two blocks the firmware's decoder returns failure and writes nothing.
            case.check(length < 16 and ret == 0xFFFFFFFF and not expected, f"{name}: refused")
            continue
        case.check(length >= 16, f"{name}: decoded {length} bytes, under two blocks")
        case.equal(out, expected, f"{name} output")
        case.equal(check, (a, b), f"{name} check value")
        case.equal(ret, 0 if check == trailer else 0xFFFFFFFF, f"{name} result")
    return case


def check_round_trips(suite):
    case = suite.case("round trips")
    rng = random.Random(1)
    sizes = [1, 7, 8, 9, 16, 1000, 0x4FF0, 0x4FF8] + [rng.randrange(1, 0x4FF8) for _ in range(40)]
    for size in sizes:
        plain = bytes(rng.randrange(256) for _ in range(size))
        padded = payload.pad(plain)
        case.check(payload.decode(payload.encode(plain)) == padded, f"{size:#x} bytes: payload")
        case.check(payload.card_code(payload.card_rom(plain)) == padded, f"{size:#x} bytes: card ROM")
    try:
        payload.card_rom(bytes(0x4FF9))
        case.check(False, "0x4ff9 bytes of code accepted, past the loader's 0x5000")
    except payload.PayloadError:
        pass
    case.info["sizes"] = len(sizes)
    return case


def check_cards(suite, cards):
    for name, path in sorted(cards.items()):
        case = suite.case(f"card {name}")
        rom = path.read_bytes()
        length = int.from_bytes(rom[:4], "big")
        code = payload.card_code(rom)
        sha1 = hashlib.sha1(code).hexdigest()
        case.check(sha1 in DECODED, f"decoded SHA-1 {sha1}, not a known card build")
        case.equal(code[0], DECODED.get(sha1), "version byte")
        case.check(payload.card_rom(code)[:4 + length] == rom[:4 + length], "re-encoded payload differs from the ROM's")


def run(context):
    suite = Suite("encoding")
    check_vectors(suite, vectors.rows())
    check_round_trips(suite)
    cards = context.dumps.cards
    check_cards(suite, cards)
    for name in sorted(set(CARDS) - set(cards)):
        suite.skip(f"card {name}")
    suite.pin("passed", sum(c.passed for c in suite.cases), 2 + len(cards))
    return [suite]
