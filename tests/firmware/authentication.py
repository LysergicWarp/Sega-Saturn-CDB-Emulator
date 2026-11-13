"""Disc authentication on the library's disc drive, CDB105 strict with no card, each run on a copy of
one booted board.

- The control check, the harness's default disc with Yabause's seek answers, at each edge of each drive setting's
  band, one setting at a time: result 4 at the last value inside, the band's failure at the first
  value outside. The bands are what the firmware accepts, not what a drive does. Reads land at the
  default landing, 4 frames early.
- Eight cases at the defaults and at every corner of the zone, where each setting sits at either
  edge of its range there: the control (4), no Saturn header in the scan (2, the disc still read),
  seek answers failing (3, the first seek tried three times 206 frames apart), equal seek words of
  8000 (3 at the second seek, the kept word read back sign-extended), passing answers with 84 good
  logo columns (3) and with 85 (4), missing sectors after the scan (3, the firmware's read error,
  well before the timeout) and frame addresses after the scan the drive never reaches (3 at the
  timeout).
- Each run is recorded in authentication/ under the output directory: its settings, its disc, the seek
  answers given, the drive's log and E1's answer, enough to replay it.
"""
import itertools
import json

from cdblock import binding, strict
from cdblock.disc import Disc, Drive, failing, passing
from cdblock.host import Host, RunError

from .common import PROFILES, EmulatorError, Suite, board
from .package import HARNESS_FLAGS

# 99:59:74, the last frame address.
LAST = 449_999
# E0 reads into partition 5, and the disc without a header is read at frame address 1000 into
# partition 0.
PARTITION, DATA_FRAME_ADDRESS = 5, 1000
DATA = b"read after result 2".ljust(2048, b"\0")
# After the 15-sector header scan from frame address 150.
AFTER_SCAN = 165
# E0's timer, 200,000 units of 100 µs, runs out after 20 s of the 20 MHz clock.
TIMEOUT_CYCLES = 400_000_000
DEFAULTS = {"sector_cycles": 266_667, "strobe_cycles": 4_000, "seek_cycles": 1_333_333}
# Each setting's band, the others at the defaults: (last inside, first outside, what the first
# outside gives) at each end, None outside where the band reaches the setting's bound.
# - Sector period: the firmware locks on to pulses within 14 ITU4 counts (112 cycles) of 16,666 at
#   double speed, 266,656 +- 224 at single speed. When the handler reads TCNT4 widens that a little,
#   to the edges pinned here, -238 and +239. Outside no read locks: result 0.
# - Strobe spacing: a byte takes 3200 cycles at BRR 99 and the SH-1 starts it 189 after the strobe.
#   The top is the drive's own bound, 13 strobes within a sector at double speed.
# - Seek latency: the check's 8 seeks (the header, four ring seeks, each pass) under E0's 20 s, each
#   read landing early enough to be taken at its first try.
MISSED = "error: drive check: the SH-1 missed a transfer of a link frame"
BANDS = {"sector_cycles": ((266_418, 266_417, 0), (266_895, 266_896, 0)),
         "strobe_cycles": ((3_389, 3_388, MISSED), (10_256, None, None)),
         "seek_cycles": ((0, None, None), (49_281_210, 49_281_211, 3))}
# The zone: every corner gives each case its result. The strobe top is what the low sector period
# allows. Near either sector edge the pulse lock can take more sectors than a landing gives, so reads
# are still sent again, and the seek top falls under the timeout first at a corner: one cycle above
# it, at the low sector period and the high strobe spacing, each read takes up to eight tries and
# the control ends with result 3.
ZONE = {"sector_cycles": (266_418, 266_895), "strobe_cycles": (3_390, 10_246), "seek_cycles": (0, 11_855_612)}


def failing_seeks(record):
    """The first seek tried three times, each 206 frames on, then given up."""
    addresses = [address for address, _, _ in record["seeks"]]
    return len(addresses) == 3 and addresses[1] == addresses[0] + 206 and addresses[2] == addresses[0] + 412


# Each case: name, the disc, E1's result, a check of its record, what the check asks.
CASES = [
    ("control", lambda image: Disc(image), 4, None, None),
    ("no header", lambda image: no_header(image), 2, lambda r: r.get("read") == "the disc's data", "the disc read after"),
    ("seek answers failing", lambda image: Disc(image, seeks=failing()), 3, failing_seeks, "three tries of the first seek"),
    ("seek words of 8000", lambda image: Disc(image, seeks=passing(word=0x8000)), 3,
     lambda r: len(r["seeks"]) == 2, "ended at the second seek"),
    ("too few logo matches", lambda image: Disc(image, seeks=passing(), logo=84), 3, None, None),
    ("enough logo matches", lambda image: Disc(image, seeks=passing(), logo=85), 4, None, None),
    ("sectors missing after the scan", lambda image: stalled(image, "missing"), 3,
     lambda r: r["cycles"] < TIMEOUT_CYCLES // 2, "ended by the read error, before the timeout"),
    ("frames unreachable after the scan", lambda image: stalled(image, "unreachable"), 3,
     lambda r: TIMEOUT_CYCLES <= r["cycles"] < TIMEOUT_CYCLES + 1_000_000, "ended by the timeout"),
]


def no_header(image):
    disc = Disc(image, header=None)
    disc.write(DATA_FRAME_ADDRESS, DATA)
    return disc


def stalled(image, how):
    disc = Disc(image)
    getattr(disc, how)(AFTER_SCAN, LAST + 1 - AFTER_SCAN)
    return disc


def booted(image):
    profile = PROFILES["cdb105"]
    bd = board(image, flags=HARNESS_FLAGS)
    strict.apply(bd, profile, None)
    host = Host(bd, idle=profile.IDLE)
    host.boot()
    return image, bd, profile


def check(base, profile, disc, settings, read=False):
    """The check on a copy of base with settings, as a record. With read, a disc refused with result
    2 is then read at DATA_FRAME."""
    bd = base.clone()
    record = {"settings": settings, "result": None, "error": None, "seeks": [], "log": []}
    if disc.seeks is not None:
        disc.seeks.asked.clear()
    try:
        host = Host(bd, idle=profile.IDLE)
        drive = Drive(host, **settings)
        drive.insert(disc)
        mark, start = bd.log_count(), bd.cycles
        answer = drive.authenticate(PARTITION)
        record["cycles"] = bd.cycles - start
        record["e1"] = answer
        record["result"] = answer & 0xFF
        record["log"] = [{"kind": e.kind, "address": e.address, "value": e.value, "text": e.text}
                         for e in bd.log(None, mark)
                         if e.kind in (binding.LOG_DRIVE_COMMAND, binding.LOG_DRIVE_REPORT, binding.LOG_DRIVE_SECTOR)]
        if read and record["result"] == 2:
            drive.read_sectors(0, DATA_FRAME_ADDRESS, 1)
            got = host.get(0, 0, 1, 1024)
            data = bytes(b for w in got for b in (w >> 8, w & 0xFF))
            record["read"] = "the disc's data" if data == DATA else "other data"
    except (RunError, EmulatorError) as error:
        record["error"] = str(error)
    finally:
        bd.close()
    if disc.seeks is not None:
        record["seeks"] = [list(s) for s in disc.seeks.asked]
    return record


def outcome(record):
    return record["result"] if record["error"] is None else f"error: {record['error']}"


def describe(disc):
    return {"header": disc.header, "tracks": disc.tracks, "leadOut": disc.lead_out, "logo": disc.logo,
            "states": disc.states, "seeks": type(disc.seeks).__name__ if disc.seeks is not None else "Yabause"}


def run(context):
    suite = Suite("authentication")
    if "cdb105" not in context.dumps.roms:
        suite.skip("every case: needs cdb105.bin")
        return [suite]
    records = context.out / "authentication"
    records.mkdir(parents=True, exist_ok=True)
    try:
        image, base, profile = booted(context.dumps.rom("cdb105"))
    except (RunError, EmulatorError) as error:
        suite.case("boot").check(False, f"failed: {error}")
        return [suite]

    def keep(name, disc, record):
        record["disc"] = describe(disc)
        (records / f"{name}.json").write_text(json.dumps(record, indent=1))

    # What each edge's runs gave, inside and outside.
    control = Disc(image)
    edges, expected_edges = {}, {}
    for setting, ends in BANDS.items():
        case = suite.case(f"{setting} band")
        for side, (inside, outside, failure) in zip(("low", "high"), ends):
            record = check(base, profile, control, dict(DEFAULTS, **{setting: inside}))
            keep(f"band-{setting}-{side}-inside", control, record)
            edges[f"{setting} {side} inside"] = outcome(record)
            expected_edges[f"{setting} {side} inside"] = 4
            case.equal(outcome(record), 4, f"{side} edge {inside:,}")
            if outside is not None:
                record = check(base, profile, control, dict(DEFAULTS, **{setting: outside}))
                keep(f"band-{setting}-{side}-outside", control, record)
                edges[f"{setting} {side} outside"] = outcome(record)
                expected_edges[f"{setting} {side} outside"] = failure
                case.equal(outcome(record), failure, f"{side}, {outside:,} outside")
    suite.pin("bands", edges, expected_edges)

    points = [("default", DEFAULTS)]
    for pick in itertools.product((0, 1), repeat=3):
        label = "".join("LH"[i] for i in pick)
        points.append((label, {name: ZONE[name][i] for name, i in zip(ZONE, pick)}))
    results = {}
    for name, make, expected, extra, what in CASES:
        case = suite.case(name)
        disc = make(image)
        for label, settings in points:
            record = check(base, profile, disc, settings, read=name == "no header")
            keep(f"{name.replace(' ', '-')}-{label}", disc, record)
            got = outcome(record)
            results[f"{name} {label}"] = got
            case.equal(got, expected, f"{label} {settings}")
            if extra is not None and record["error"] is None:
                case.check(extra(record), f"{label}: {what}")
    suite.pin("zone", {f"{name} {label}": results[f"{name} {label}"] for name, *_ in CASES for label, _ in points},
              {f"{name} {label}": expected for name, _, expected, _, _ in CASES for label, _ in points})
    base.close()
    suite.pin("cases", len(suite.cases), 3 + len(CASES))
    return [suite]
