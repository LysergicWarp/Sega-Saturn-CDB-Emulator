#!/usr/bin/env python3
"""Replay the disc authentication checks of a Saturn CD-block firmware dump.

The CD block checks a disc with four seeks, whose drive answers must agree, and three sector
checks. Each sector check scrambles three raw sectors again, interleaves them into one stream and
compares part of the stream with one row of the SEGA logo stored in the ROM. This script runs the
same checks on values and sectors you supply, with the tables from your own dump.

usage:
  ring.py [--shift N] DUMP [SCRIPT]
  ring.py [--shift N] DUMP -c "COMMAND; COMMAND; ..."

Commands come from SCRIPT, from -c, or from standard input, one per line. Text after # is a
comment. File paths in commands are relative to the current directory. CDB104, CDB105, CDB106 and
YGR022 are recognized by SHA-1. For another dump, --shift gives the distance of its tables from
their CDB105 addresses.

commands:
  logo                  print the logo bitmap with its row numbers
  timer VALUE           start a check from an ITU4 counter value (hex): the zone, the column,
                        the seek targets and the check positions
  seek WORD [A B C D]   the drive's answer to the next seek (hex): the report word, and for
                        seeks 2 and 3 the report bytes a, b, c and d
  image FILE ADDRESS    raw 2352-byte sectors, the first one at ADDRESS (LBA or MM:SS:FF)
  pass [FILE [SECTOR]]  the next sector check, on three raw 2352-byte sectors from FILE starting
                        at sector index SECTOR (default 0), or from the image at the check
                        position

make.py, next to this script, builds sectors for the sector checks from your dump.

examples, next to this script:
  positions.txt         how the counter value picks the seek targets and check positions
  seeks.txt             drive answers that pass, retry and fail
  sectors.txt           one sector check on sectors from a raw disc image
  full-check.txt        whole checks on sectors built by make.py

  ring.py cdb105.bin positions.txt
  ring.py cdb105.bin -c "timer 0145; seek 1000; seek 1013"
"""

import argparse
import hashlib
import inspect
import sys

# SHA-1 of each known dump, and the distance of its tables from their CDB105 addresses.
REVISIONS = {
    "fb0993246ed0ad27ca086854295eccd412196254": ("CDB104", 0x00),
    "eb8393058f324e922c11b43709b64fc6ca94ab86": ("CDB105", 0x00),
    "b3c20fbe57cd2eb595e9edac86817e5948dccae4": ("CDB106", 0x10),
    "f4f6c2aac68c352814d396ae41f81f54ad228e68": ("YGR022", 0x10),
}

# CDB105 table addresses.
LOGO_BITMAP = 0x8FB4           # 17 rows of 16 bytes, 106 bits each, most significant bit first
COLUMN_MASK = 0x90C4           # one bit per column
SMALL_FRAME_SPREAD = 0x90D4    # 24 big-endian halfwords: where byte k of a small frame goes
COLUMN_BYTES = 0x9104          # stream bytes per column

ROWS = 17
LOGO_COLUMNS = 106
SECTOR = 2352
SMALL_FRAME = 24
SECTOR_SMALL_FRAMES = 98
STREAM = 402 * SMALL_FRAME
COMPARE_START = 196 * SMALL_FRAME
COMPARE_BYTES = 884
SET, CLEAR = 0xA8, 0x59

FIRST_POSITION = 0x49124       # frame address 299,300, MSF 66:30:50
ZONE_FRAMES = 1470
COLUMN_FRAMES = 21
SEEK_TARGETS = (0x3F435, 0x3FE51)
SEEK_2_DISTANCE = 0xDEA
SEEK_LIMITS = {1: 18, 3: 20}
RETRY_FRAMES = 206

# The re-scramble writes from byte 12 of its buffer and leaves bytes 0..11 as they were. They
# hold 00 after the startup RAM check.
BUFFER_HEAD = bytes(12)


class Failure(Exception):
    pass


def scrambler():
    """ECMA-130 Annex B: x^15 + x + 1 from register value 1, least significant bit first."""
    register, out = 1, bytearray(SECTOR - 12)
    for n in range(len(out)):
        for bit in range(8):
            out[n] |= (register & 1) << bit
            register = (register >> 1) | (((register ^ (register >> 1)) & 1) << 14)
    return bytes(out)


SCRAMBLER = scrambler()


class Tables:
    def __init__(self, rom, shift):
        def take(address, length):
            return rom[address + shift:address + shift + length]
        self.logo = take(LOGO_BITMAP, ROWS * 16)
        self.mask = take(COLUMN_MASK, 14)
        self.spread = [int.from_bytes(take(SMALL_FRAME_SPREAD + 2 * k, 2), "big") for k in range(SMALL_FRAME)]
        self.sizes = take(COLUMN_BYTES, LOGO_COLUMNS)
        if (sum(self.sizes) != COMPARE_BYTES
                or sorted(offset % SMALL_FRAME for offset in self.spread) != list(range(SMALL_FRAME))):
            raise Failure(f"no check tables at shift {shift:#x}")

    def bit(self, row, logo_column):
        return self.logo[16 * row + logo_column // 8] >> (7 - logo_column % 8) & 1

    def threshold(self, logo_column):
        """Matching bytes a column needs: its byte count less its mask bit (2 in column 8),
        halved and rounded down."""
        masked = self.mask[logo_column // 8] >> (7 - logo_column % 8) & 1
        if masked and logo_column == 8:
            masked = 2
        return (self.sizes[logo_column] - masked) // 2


def rescramble(raw):
    """Scramble raw sector bytes 12..2351 again, as they were on the disc. Each pair of bytes is
    stored swapped."""
    out = bytearray(BUFFER_HEAD) + bytearray(SECTOR - 12)
    for j in range(SECTOR - 12):
        out[12 + (j ^ 1)] = raw[12 + j] ^ SCRAMBLER[j]
    return out


def interleave(sectors, tables):
    """Spread three re-scrambled sectors as 98 small frames of 24 bytes each into one stream.
    Byte k of a small frame moves to offset k of the spread table, 3 to 108 small frames later."""
    stream = bytearray(STREAM)
    for s, raw in enumerate(sectors):
        buffer = rescramble(raw)
        for f in range(SECTOR_SMALL_FRAMES):
            start = (SECTOR_SMALL_FRAMES * s + f) * SMALL_FRAME
            for k, offset in enumerate(tables.spread):
                stream[start + offset] = buffer[SMALL_FRAME * f + k]
    return stream


class Walk:
    def __init__(self, stream, row, tables):
        """Walk the 106 columns from small frame 196. A set row bit expects byte a8, a clear one 59.
        The walk stops after 11 bad columns in a row."""
        self.good, self.stopped = 0, None
        self.read, self.marks = [], []
        position, bad = COMPARE_START, 0
        for logo_column in range(LOGO_COLUMNS):
            size = tables.sizes[logo_column]
            chunk = stream[position:position + size]
            position += size
            self.read.append(self.reading(chunk, tables.threshold(logo_column)))
            if self.stopped is not None:
                self.marks.append("-")
                continue
            expected = SET if tables.bit(row, logo_column) else CLEAR
            if chunk.count(expected) >= tables.threshold(logo_column):
                self.good += 1
                bad = 0
                self.marks.append(" ")
            else:
                bad += 1
                self.marks.append("x")
                if bad == 11:
                    self.stopped = logo_column

    @staticmethod
    def reading(chunk, threshold):
        set_bytes, clear_bytes = chunk.count(SET), chunk.count(CLEAR)
        if set_bytes >= threshold and set_bytes > clear_bytes:
            return "#"
        if clear_bytes >= threshold and clear_bytes > set_bytes:
            return " "
        return "?"


def load(path, shift=None):
    """Tables from the dump at `path`, at the shift its SHA-1 selects unless `shift` is given."""
    with open(path, "rb") as file:
        rom = file.read()
    if len(rom) != 65536:
        raise Failure(f"{path} is not a 65,536-byte dump")
    digest = hashlib.sha1(rom).hexdigest()
    name, known = REVISIONS.get(digest, (None, None))
    if shift is None:
        if name is None:
            raise Failure(f"unknown dump {digest}, pass --shift")
        shift = known
    return Tables(rom, shift)


def msf(frame_address):
    return f"{frame_address // 4500:02d}:{frame_address // 75 % 60:02d}:{frame_address % 75:02d}"


def where(frame_address):
    return f"{msf(frame_address)} (LBA {frame_address - 150:,})"


def address(text):
    try:
        if ":" in text:
            minutes, seconds, frames = (int(part) for part in text.split(":"))
            return (minutes * 60 + seconds) * 75 + frames
        return int(text) + 150
    except ValueError:
        raise Failure(f"{text} is not an LBA or MM:SS:FF") from None


def hexadecimal(text, limit):
    try:
        value = int(text, 16)
    except ValueError:
        raise Failure(f"{text} is not hexadecimal") from None
    if not 0 <= value <= limit:
        raise Failure(f"{text} is out of range")
    return value


def positions(zone, grid_column):
    """Frame addresses of the three check positions."""
    if zone == 12:
        offsets = (21 * (grid_column % 39), 21 * (grid_column % 39), 21 * (grid_column % 19))
        return [FIRST_POSITION + ZONE_FRAMES * (12 + i) + offsets[i] for i in range(3)]
    found = [FIRST_POSITION + ZONE_FRAMES * (zone + i) + COLUMN_FRAMES * grid_column for i in range(3)]
    if zone == 0 and grid_column < 2:
        found[0] = FIRST_POSITION + 42
    return found


class Check:
    def __init__(self, timer):
        self.zone = (timer & 31) % 13
        grid_column = timer >> 4 & 127
        self.grid_column = grid_column - 70 if grid_column >= 70 else grid_column
        self.positions = positions(self.zone, self.grid_column)
        self.targets = [*SEEK_TARGETS, self.positions[0] + SEEK_2_DISTANCE, self.positions[0]]
        self.seek, self.retries, self.kept = 0, 0, None
        self.passes, self.score = 0, 0
        self.ended = False


class Session:
    def __init__(self, tables):
        self.tables = tables
        self.check = None
        self.image = None

    def run(self, line):
        words = line.split("#", 1)[0].split()
        if not words:
            return
        name, arguments = words[0], words[1:]
        command = getattr(self, "command_" + name, None)
        if command is None:
            raise Failure(f"unknown command {name}")
        try:
            inspect.signature(command).bind(*arguments)
        except TypeError:
            raise Failure(f"wrong arguments for {name}") from None
        command(*arguments)

    def command_logo(self):
        for row in reversed(range(ROWS)):
            line = "".join("#" if self.tables.bit(row, c) else " " for c in range(LOGO_COLUMNS))
            print(f"{row:2d} |{line}|")

    def command_timer(self, value):
        timer = hexadecimal(value, 0xFFFF)
        if self.check is not None:
            print()
        check = self.check = Check(timer)
        print(f"timer {timer:04x}: zone {check.zone}, column {check.grid_column}")
        for phase, target in enumerate(check.targets):
            print(f"  seek {phase}  {where(target)}")
        for index, position in enumerate(check.positions):
            print(f"  pass {index}  {where(position)}, logo row {check.zone + index}")

    def started(self):
        if self.check is None:
            raise Failure("start a check with timer first")
        if self.check.ended:
            raise Failure("this check has ended, start another with timer")
        return self.check

    def command_seek(self, word, *report):
        check = self.started()
        if check.seek == 4:
            raise Failure("all four seeks are done")
        phase, word = check.seek, hexadecimal(word, 0xFFFF)
        target = check.targets[phase] + RETRY_FRAMES * check.retries
        head = f"seek {phase} to {msf(target)}: word {word:04x}"
        if word in (0, 0xFFFF):
            check.retries += 1
            if check.retries == 3:
                check.ended = True
                print(f"{head}, the third such answer. The check ends with 3")
            else:
                print(f"{head}. The drive seeks again {RETRY_FRAMES} frames further on")
            return
        check.retries = 0
        if phase >= 2:
            if len(report) != 4:
                raise Failure(f"seek {phase} needs the report bytes a, b, c and d")
            a, b, c, d = (hexadecimal(value, 0xFF) for value in report)
            head += f", bytes {a:02x} {b:02x} {c:02x} {d:02x}"
            if b < a // 2 or d < c // 2:
                check.ended = True
                print(f"{head}. b must be at least a / 2 and d at least c / 2. "
                      "The check ends with 3")
                return
        if phase in SEEK_LIMITS:
            distance, limit = abs(word - check.kept), SEEK_LIMITS[phase]
            head += f", {distance:,} from {check.kept:04x}"
            if distance > limit:
                check.ended = True
                print(f"{head}, more than {limit}. The check ends with 3")
                return
        else:
            check.kept = word
            head += ", kept"
        check.seek += 1
        print(head + (". The seeks pass" if check.seek == 4 else ""))

    def command_image(self, path, first):
        first = address(first)
        with open(path, "rb") as file:
            data = file.read()
        self.image = (data, first)
        print(f"image {path}: {len(data) // SECTOR:,} sectors from {where(first)}")

    def command_pass(self, path=None, first="0"):
        check = self.started()
        position = check.positions[check.passes]
        if path is None:
            if self.image is None:
                raise Failure("pass needs a file or an image")
            data, start = self.image
            index = position - start
            source = f"the image at sector {index:,}"
        else:
            with open(path, "rb") as file:
                data = file.read()
            if not first.isdigit():
                raise Failure(f"{first} is not a sector index")
            index = int(first)
            source = f"{path} at sector {index:,}"
        if index < 0 or len(data) < (index + 3) * SECTOR:
            raise Failure(f"three sectors from {source} are out of range")
        sectors = [data[(index + s) * SECTOR:(index + s + 1) * SECTOR] for s in range(3)]
        row = check.zone + check.passes
        walk = Walk(interleave(sectors, self.tables), row, self.tables)
        expected = "".join("#" if self.tables.bit(row, c) else " " for c in range(LOGO_COLUMNS))
        print(f"pass {check.passes} at {where(position)}, logo row {row}, from {source}")
        print(f"  expected |{expected}|")
        print(f"  read     |{''.join(walk.read)}|")
        print(f"  bad      |{''.join(walk.marks)}|")
        summary = f"  {walk.good} good column" + ("" if walk.good == 1 else "s")
        if walk.stopped is not None:
            summary += f", walk stopped at column {walk.stopped} after 11 bad columns in a row"
        scored = walk.good >= 85
        check.score += scored
        print(summary + (". Scores" if scored else ". Needs 85 to score"))
        check.passes += 1
        if check.passes < 3:
            return
        check.ended = True
        if check.score < 2:
            print(f"score {check.score}: fewer than two passes score. The check ends with 3")
        elif check.seek < 4:
            print(f"score {check.score}: the sector checks pass. The seeks are not complete")
        else:
            print(f"score {check.score}: the disc is accepted with 4, or 0x0104 for a "
                  "SEGASYSTEM product number")


def main():
    parser = argparse.ArgumentParser(
        usage=argparse.SUPPRESS, description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dump")
    parser.add_argument("script", nargs="?")
    parser.add_argument("-c", dest="commands")
    parser.add_argument("--shift", type=lambda text: int(text, 0))
    options = parser.parse_args()
    try:
        session = Session(load(options.dump, options.shift))
        if options.commands is not None:
            lines = options.commands.split(";")
        elif options.script is not None:
            with open(options.script) as file:
                lines = file.read().splitlines()
        else:
            lines = sys.stdin
    except (Failure, OSError) as error:
        sys.exit(f"ring.py: {error}")
    interactive = options.commands is None and options.script is None and sys.stdin.isatty()
    for number, line in enumerate(lines, 1):
        try:
            session.run(line)
        except (Failure, OSError) as error:
            message = f"line {number}: {error}" if options.commands is None else str(error)
            sys.stdout.flush()
            print(f"ring.py: {message}", file=sys.stderr)
            if not interactive:
                sys.exit(1)


if __name__ == "__main__":
    main()
