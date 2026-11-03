"""Takes a card extension through its lifecycle, as the card extension lifecycle guide describes:
https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/sh1-execution/lifecycle.md

  python -m cdblock --firmware DUMP --extension examples/lifecycle-extension.s examples/lifecycle.py

The decompress extension goes through it the same way. E0 subcode 1 loads the extension and 01
reports its MPEG version. Its commands are rejected until 93 turns the card on, which counts the
activation. AB submits a ping that task 9 runs, its end writes MPED, and AC reports it. 93 with
bit 0 clear stops the card and the extension keeps its record. 93 turns it on again, which clears
the job, and 93 while it is on resets it and turns it on once more. After a stop, E0 subcode 1
loads the image again, which resets the record.
"""
from cdblock.host import MPED, RunError

REJECT = 0xFF
DONE = 3


def check(condition, what, answer):
    if not condition:
        raise RunError(f"{what}: answered {' '.join(f'{w:04x}' for w in answer)}")


def identity(activations):
    answer = host.command([0xAD00, 0, 0, 0], name="AD identity")
    check(answer[2:] == [0x444F, 0x5345] and answer[1] == activations, f"AD, {activations} activations", answer)


def rejected(cr, name):
    answer = host.command(cr, name=name)
    check(answer[0] >> 8 == REJECT, f"{name} rejected with the card off", answer)


def ping(job, argument):
    host.command([0xAB00, job, argument >> 16, argument & 0xFFFF], MPED, "AB ping")
    answer = host.command([0xAC00, 0, 0, 0], name="AC job report")
    inverted = ~argument & 0xFFFFFFFF
    check(answer[0] & 0xFF == DONE and answer[1] == job and answer[2:] == [inverted >> 16, inverted & 0xFFFF],
          "AC, the ping done", answer)


def card(on, name):
    host.command([0x9300, 1 if on else 0, 0, 0], MPED, name)


host.load_card()
mpeg = host.hardware_info()["mpeg"]
check(mpeg == extension.code[0], "01, the MPEG version", [mpeg])
rejected([0xAD00, 0, 0, 0], "AD")

host.activate_card()
identity(1)
ping(0x1234, 0x89ABCDEF)

card(False, "93 card off")
rejected([0xAC00, 0, 0, 0], "AC")
card(True, "93 card on")
identity(2)
ping(0x0005, 0x00010002)

# 93 while the card is on resets it. Task 9 hands the stop to task 8, which turns the card on
# again and writes MPED.
card(True, "93 card reset")
identity(3)

card(False, "93 card off")
host.load_card()
host.activate_card()
identity(1)
ping(0x0006, 0xFFFFFFFE)
