"""The scripts as a reader runs them, through their command lines: ring.py on its example files and
make.py's sectors on CDB105, and payload.py's card ROM and back. No emulator.

- positions.txt: each timer value's zone and column, as the file's comments give them.
- seeks.txt: three checks whose seeks pass and four that end with 3, as its comments say.
- make.py writes accepted.bin, short.bin and failing.bin, and full-check.txt scores them 3, 2 and
  1: the disc accepted, the sector checks passed without the seeks, and a check that ends with 3
  after the walk stopped at column 11. The sectors are deleted after.
- payload.py card, then uncard, gives back the code padded to whole blocks in a 512 KB card ROM.
"""
import pathlib
import subprocess
import sys

from .common import Suite

SCRIPTS = pathlib.Path(__file__).resolve().parents[2] / "scripts"
RING = SCRIPTS / "ring"
PAYLOAD = SCRIPTS / "payload" / "payload.py"
SECTORS = ("accepted.bin", "short.bin", "failing.bin")
CARD_ROM = 0x80000


def script(case, work, path, *arguments):
    """Runs a script in the work folder. Returns what it printed, or None when it failed."""
    result = subprocess.run([sys.executable, str(path), *map(str, arguments)], cwd=work, capture_output=True,
                            text=True)
    if not case.check(result.returncode == 0, f"{path.name} exited {result.returncode}: {result.stderr.strip()[-300:]}"):
        return None
    return result.stdout


def has_lines(case, out, lines):
    for line in lines:
        case.check(line in out, f"no line {line!r}")


def check_positions(suite, work, dump):
    case = suite.case("ring.py positions.txt")
    out = script(case, work, RING / "ring.py", dump, RING / "positions.txt")
    if out is not None:
        has_lines(case, out, ("timer 0145: zone 5, column 20", "timer 0467: zone 7, column 0",
                              "timer ffff: zone 5, column 57", "timer 001a: zone 0, column 1",
                              "timer 0279: zone 12, column 39"))


def check_seeks(suite, work, dump):
    case = suite.case("ring.py seeks.txt")
    out = script(case, work, RING / "ring.py", dump, RING / "seeks.txt")
    if out is not None:
        case.equal(out.count("The seeks pass"), 3, "checks whose seeks pass")
        case.equal(out.count("The check ends with 3"), 4, "checks that end with 3")


def check_full(suite, work, dump):
    case = suite.case("make.py, then ring.py full-check.txt")
    try:
        if script(case, work, RING / "make.py", dump) is None:
            return
        out = script(case, work, RING / "ring.py", dump, RING / "full-check.txt")
        if out is not None:
            has_lines(case, out, ("score 3: the disc is accepted",
                                  "score 2: the sector checks pass. The seeks are not complete",
                                  "walk stopped at column 11",
                                  "score 1: fewer than two passes score. The check ends with 3"))
    finally:
        for name in SECTORS:
            (work / name).unlink(missing_ok=True)


def check_payload(suite, work):
    case = suite.case("payload.py card, then uncard")
    # Not whole eight-byte blocks, so the padding shows.
    code = bytes(range(256)) * 9 + b"\x01\x02\x03"
    (work / "code.bin").write_bytes(code)
    if script(case, work, PAYLOAD, "card", "code.bin", "card.bin") is None:
        return
    if script(case, work, PAYLOAD, "uncard", "card.bin", "back.bin") is None:
        return
    case.equal(len((work / "card.bin").read_bytes()), CARD_ROM, "card ROM size")
    case.check((work / "back.bin").read_bytes() == code + bytes(-len(code) % 8),
               "the code decoded from the card ROM differs from the code padded to whole blocks")


def run(context):
    suite = Suite("scripts")
    work = context.out / "scripts"
    work.mkdir(parents=True, exist_ok=True)
    check_payload(suite, work)
    if "cdb105" not in context.dumps.roms:
        suite.skip("ring.py and make.py: need cdb105.bin")
        return [suite]
    dump = context.dumps.roms["cdb105"]
    check_positions(suite, work, dump)
    check_seeks(suite, work, dump)
    check_full(suite, work, dump)
    return [suite]
