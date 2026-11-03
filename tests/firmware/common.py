"""What the firmware tests share: the dumps, boards, and each test's cases and pinned totals.

The dumps come from a folder laid out as dumps/README.md lists them: CD-block ROMs at its top and
MPEG card ROMs in mpeg-card/, each known by its SHA-1. A test runs the parts its dumps allow and
names the rest skipped. A case collects every mismatch instead of stopping at the first, so one run
shows the whole picture. A pinned total, such as the instructions a run takes, is compared only when
every dump it covers is there.
"""
import hashlib

from cdblock import binding, cdb104, cdb105, cdb106, ygr022
from cdblock.board import Board, EmulatorError  # noqa: F401

PROFILES = {profile.NAME.lower(): profile for profile in (cdb104, cdb105, cdb106, ygr022)}
ROMS = tuple(PROFILES)
# The known MPEG cards' ROMs, by their MAME set names.
CARDS = {
    "hivcd1d": "40f284e79f205728b72c7d7c0cf0bae0ade38f76",
    "hivcd2d": "88aa35e48e655540e87ebeae10220413a8feee81",
    "jvcvc1": "eaac576b8b5e79e7df5e3efd70275b220ff70d78",
    "jvcvc2": "286f2c75b1a5bb9d7df23db9f0a381e61b1ac76c",
    "jvcvc20": "bb6459da484864226616c5d45b779d4b1644c527",
    "jvcvc2a": "567f82a31d5f93676120d699869b3ca64995e167",
    "ssmovie": "f7171b2a8f1c464696771941225c32d636b59349",
    "vcd": "2f5ebd434f6d5b82e93d03e5626007bd5380f652",
}

_lib = None


def lib():
    global _lib
    if _lib is None:
        _lib = binding.load()
    return _lib


def board(rom_bytes, **options):
    return Board(rom_bytes, lib=lib(), **options)


class Dumps:
    """The dumps in a folder: the CD-block ROMs by revision and the card ROMs by name, as paths, and
    a line for each one missing or not the known dump."""

    def __init__(self, folder):
        self.roms, self.cards, self.missing = {}, {}, []
        for name, profile in PROFILES.items():
            self._find(folder, f"{name}.bin", profile.SHA1, profile.NAME, self.roms, name)
        for name, sha1 in CARDS.items():
            self._find(folder, f"mpeg-card/{name}.bin", sha1, name, self.cards, name)

    def _find(self, folder, where, sha1, what, found, name):
        path = folder / where
        if not path.exists():
            self.missing.append(f"{where} not found")
        elif hashlib.sha1(path.read_bytes()).hexdigest() != sha1:
            self.missing.append(f"{where} is not {what} by its SHA-1")
        else:
            found[name] = path

    def rom(self, name):
        return self.roms[name].read_bytes()

    @property
    def complete(self):
        return not self.missing


def h(value):
    return f"0x{value:08x}" if isinstance(value, int) else repr(value)


class Case:
    def __init__(self, name):
        self.name = name
        self.failures = []
        self.info = {}

    def check(self, condition, what):
        if not condition:
            self.failures.append(what)
        return condition

    def equal(self, actual, expected, what):
        if actual != expected:
            self.failures.append(f"{what}: {h(actual)}, expected {h(expected)}")
            return False
        return True

    @property
    def passed(self):
        return not self.failures

    def record(self):
        return {"name": self.name, "passed": self.passed, "failures": self.failures, **self.info}


class Suite:
    """A test's cases, the totals it pins and what it skipped."""

    def __init__(self, name):
        self.name = name
        self.cases = []
        self.totals = {}
        self.pins = {}
        self.skipped = []

    def case(self, name):
        c = Case(name)
        self.cases.append(c)
        return c

    def pin(self, what, actual, expected):
        self.totals[what] = actual
        self.pins[what] = expected

    def skip(self, what):
        self.skipped.append(what)

    def summary(self):
        passed = sum(c.passed for c in self.cases)
        mismatched = {k: (self.totals[k], v) for k, v in self.pins.items() if self.totals[k] != v}
        return passed, mismatched

    def record(self):
        passed, mismatched = self.summary()
        return {"suite": self.name, "passed": passed, "cases": [c.record() for c in self.cases],
                "totals": self.totals, "pins": self.pins, "skipped": self.skipped,
                "pinMismatches": {k: {"actual": a, "expected": e} for k, (a, e) in mismatched.items()}}
