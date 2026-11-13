"""The scripts as a reader runs them, through their command lines: ring.py on its example files and
make.py's sectors on CDB105, and payload.py's card ROM and back. No emulator.

- positions.txt: each timer value's zone and column, as the file's comments give them, and the
  first seek target and check position of the first timer value.
- seeks.txt: three checks whose seeks pass and five that end with 3, as its comments say, with the
  two distances its comments give.
- make.py writes accepted.bin, short.bin and failing.bin, and full-check.txt scores them 3, 2 and
  1: the disc accepted, the sector checks passed without the seeks, and a check that ends with 3
  after the walk stopped at column 11. The sectors are deleted after.
- make.py with a timer value and counts names ring.py by its path. Its sectors score 2 as an
  image, and pass reads three of them from the file by sector index.
- ring.py fails a kept word of 8000 or above, refuses report bytes and addresses that do not
  parse, and reads a command file as UTF-8 with a byte order mark and as UTF-16.
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


def fails(case, work, path, *arguments, message):
    """Runs a script that has to fail in the work folder, and looks for message in its error."""
    result = subprocess.run([sys.executable, str(path), *map(str, arguments)], cwd=work, capture_output=True,
                            text=True)
    case.check(result.returncode != 0 and message in result.stderr,
               f"{path.name} {arguments[-1]!r} exited {result.returncode}: {result.stderr.strip()[-300:]!r}, "
               f"expected a failure with {message!r}")


def has_lines(case, out, lines):
    for line in lines:
        case.check(line in out, f"no line {line!r}")


def check_positions(suite, work, dump):
    case = suite.case("ring.py positions.txt")
    out = script(case, work, RING / "ring.py", dump, RING / "positions.txt")
    if out is not None:
        has_lines(case, out, ("timer 0145: zone 5, column 20", "timer 0467: zone 7, column 0",
                              "timer ffff: zone 5, column 57", "timer 001a: zone 0, column 1",
                              "timer 0279: zone 12, column 39",
                              "  seek 0  57:35:00 (LBA 258,975)",
                              "  pass 0  68:14:20 (LBA 306,920), logo row 5"))


def check_seeks(suite, work, dump):
    case = suite.case("ring.py seeks.txt")
    out = script(case, work, RING / "ring.py", dump, RING / "seeks.txt")
    if out is not None:
        case.equal(out.count("The seeks pass"), 3, "checks whose seeks pass")
        case.equal(out.count("The check ends with 3"), 5, "checks that end with 3")
        has_lines(case, out, ("32,748 from 0004", "4,294,967,276 from fff0"))


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


def check_built(suite, work, dump):
    case = suite.case("make.py with a timer and counts, then ring.py on the image and on the file")
    ring = RING / "ring.py"
    try:
        out = script(case, work, RING / "make.py", dump, "0145", "built.bin", 106, 84, 85)
        if out is None:
            return
        case.check(str(ring) in out, "make.py's command does not name ring.py by its path")
        out = script(case, work, ring, dump, "-c", "timer 0145; image built.bin 68:14:20; pass; pass; pass")
        if out is not None:
            has_lines(case, out, ("84 good columns. Needs 85 to score",
                                  "score 2: the sector checks pass. The seeks are not complete"))
        out = script(case, work, ring, dump, "-c", "timer 0145; pass built.bin 0")
        if out is not None:
            has_lines(case, out, ("from built.bin at sector 0", "106 good columns. Scores"))
        fails(case, work, ring, dump, "-c", "timer 0145; pass built.bin 99999", message="out of range")
    finally:
        (work / "built.bin").unlink(missing_ok=True)


def check_commands(suite, work, dump):
    case = suite.case("ring.py's seek words, report bytes, addresses and command file encodings")
    ring = RING / "ring.py"
    out = script(case, work, ring, dump, "-c", "timer 0145; seek 8000; seek 8000")
    if out is not None:
        has_lines(case, out, ("which is read back sign-extended, more than 18. The check ends with 3",))
    fails(case, work, ring, dump, "-c", "timer 0145; seek 1000 zz", message="give all four or none")
    fails(case, work, ring, dump, "-c", "timer 0145; seek 1000 zz 00 00 00", message="zz is not hexadecimal")
    fails(case, work, ring, dump, "-c", "image none.bin 1:99:99", message="1:99:99 is not an LBA or MM:SS:FF")
    fails(case, work, ring, dump, "-c", "image none.bin -151", message="-151 is not an LBA or MM:SS:FF")
    commands = "timer 0145\nseek 1000\nseek 1012\n"
    for encoding in ("utf-8-sig", "utf-16"):
        name = f"commands-{encoding}.txt"
        (work / name).write_bytes(commands.encode(encoding))
        out = script(case, work, ring, dump, name)
        if out is not None:
            has_lines(case, out, ("timer 0145: zone 5, column 20", "18 from 1000"))


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
    check_built(suite, work, dump)
    check_commands(suite, work, dump)
    return [suite]
