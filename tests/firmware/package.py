"""The cdblock package on the emulator with device timing and the clock, as python -m cdblock runs a
known revision: strict, the clock skipping on at the profile's idle loop.

- Every revision with no card and with each known card's ROM: boot to the signature, then the
  card loaded through E0, turned on with 93 and 01 answering its MPEG version. Startup's
  diagnostic words show its card check passed with a card and failed all four bits without.
  CDB104's vcd run again with the clock running at the idle loop takes the same instructions.
- Each profile's code ranges, pinned by their size.
- Four sectors put through 64 and read back through 61 on every revision, the steps back to back
  and each 06 step with the firmware's word count and the transfer's sectors.
- The lifecycle and module example extensions, each as source, ELF, plain code and card ROM on
  CDB105, each form run the same, and its card ROM on the other revisions. These need binutils.
- The command line: its exit status for a run, strict on a known revision and lenient on a dump
  of none, refused inputs, a library that is not there and a scenario that mixes the host's
  helpers with a run to an address.
- A wait that never ends fails on the clock's cycles, as skipped time costs no instructions.
- The drive's sector provider in Python on CDB105: what it leaves or replaces reaches the host, and
  its exceptions come back from the run on the board that ran it, its copy's too.
- The drive's subcode on CDB105: an audio play with 04's R-W setting on, where 20 returns each pack
  the disc holds once, in order and without an error flag, until the pause ends them.
- The drive's subcode provider in Python on CDB105, as the sector provider: what it leaves or
  replaces reaches the host as packs, and its exceptions come back from the run on the board that
  ran it.

The only device events raised here are the host's commands and transfers. The card, the timers and
the drive act on their own.
"""
import contextlib
import hashlib
import io
import os
import pathlib

from cdblock import __main__ as cli
from cdblock import binding, strict
from cdblock import extension as extensions
from cdblock.disc import DATA, DATA_TRACK, FIRST, Disc, Drive, sector
from cdblock.host import MPCM, PEND, REJECT, WAIT, WAIT_CYCLES, Host, RunError

from . import rw
from .common import PROFILES, ROMS, EmulatorError, Suite, board

EXAMPLES = pathlib.Path(__file__).resolve().parents[2] / "examples"
LIFECYCLE = EXAMPLES / "lifecycle-extension.s"
MODULE = EXAMPLES / "module-extension.s"
# Instructions from reset to the signature. CDB106 and YGR022 take seven more.
BOOT = {"cdb104": 3_601_460, "cdb105": 3_601_460, "cdb106": 3_601_467, "ygr022": 3_601_467}
# Startup's diagnostic words: the address lines that failed, then bits 0..3 its interface and
# buffer tests and bits 4..7 the card check.
DIAGNOSTICS = 0x0F000000
SECTOR_WORDS = 1024
# Sectors in each round trip: more than one, so DEI1 rearms the channel while the host moves words.
SECTORS = 4
# The board as python -m cdblock makes it.
HARNESS_FLAGS = binding.DEVICE_TIMING | binding.CLOCK
# The clock runs ITU4, which the firmware starts at φ/8 with its overflow interrupt, one every
# 524,288 cycles. Its handler takes 15 instructions.
OVI4, OVERFLOW_INSTRUCTIONS = 98, 15
# The sector provider's disc has no header, so E0, reading into CHECK_PARTITION, refuses it with
# result 2 and reads stay open. Each read takes the sector at PROVIDED_FRAME into partition 0.
CHECK_PARTITION, PROVIDED_FRAME_ADDRESS = 5, 1000
WRITTEN, REPLACED = b"written".ljust(DATA, b"\0"), b"replaced".ljust(DATA, b"\0")
# The subcode case's disc: a data track, then an audio track whose sectors hold generated R-W packs
# from SUBCODE_MARGIN sectors before the frames played to as many after them. A sector takes
# SECTOR_CYCLES at single speed, the speed of an audio play.
AUDIO_TRACK, AUDIO_START = 0x01, 20_000
PLAY_FRAME_ADDRESS, PLAY_SECTORS, SUBCODE_MARGIN = 20_100, 40, 16
SECTOR_CYCLES = 266_667
# What these plays return, pinned as the runs have it on the drive's default settings: four packs
# for each sector played, the first 11 packs before the one that starts the subcode of the play's
# first sector. The runs are deterministic.
FIRST_PACK = rw.SECTOR_PACKS * SUBCODE_MARGIN - 11
# The subcode provider's plays. Its replacing provider answers each sector with the subcode
# generated for the one PROVIDED_SHIFT sectors further on.
PROVIDED_SECTORS, PROVIDED_SHIFT = 8, 256


class ProviderStop(Exception):
    pass


def direct(image, extension, skip=True):
    """python -m cdblock on a known revision without a scenario: the board, its host and the result
    line. Without skip the host leaves the clock running at the idle loop."""
    profile = cli.PROFILES.get(hashlib.sha1(image).hexdigest())
    bd = board(image, card_rom=extension.card if extension else None, flags=HARNESS_FLAGS)
    strict.apply(bd, profile, extension)
    host = Host(bd, idle=getattr(profile, "IDLE", None) if skip else None)
    try:
        host.boot()
        return bd, host, cli.default_run(host, extension)
    except (RunError, EmulatorError) as error:
        return bd, host, f"failed: {error}"


def overflows(bd):
    """ITU4 overflow interrupts taken (OVI4), the only ITU4 interrupt these runs enable."""
    return len([e for e in bd.log(binding.LOG_INTERRUPT) if e.value == OVI4])


def steps(host):
    return [tuple(step) for step in host.steps]


def check_run(case, bd, host, result, extension):
    case.check(not result.startswith("failed"), result)
    case.equal(len(bd.log(binding.LOG_VIOLATION)), 0, "violations")
    if host.steps:
        last = host.steps[-1]
        case.equal(last.name, "01 hardware info", "last step")
        case.equal(last.answer[2] & 0xFF, extension.code[0] if extension else 0, "MPEG version")
    case.info["steps"] = [f"{s.name}: {s.instructions}" for s in host.steps]
    case.info["instructions"] = bd.instructions


def check_matrix(suite, dumps):
    routes = {}
    for name in ROMS:
        if name not in dumps.roms:
            continue
        image = dumps.rom(name)
        for card in [None, *sorted(dumps.cards)]:
            case = suite.case(f"{name} {card or 'no card'}")
            if card is None:
                # The command line runs a dump strict when a profile names it, as direct() does.
                case.check(hashlib.sha1(image).hexdigest() in cli.PROFILES, "a known revision")
            extension = extensions.from_card(dumps.cards[card]) if card else None
            bd, host, result = direct(image, extension)
            check_run(case, bd, host, result, extension)
            if host.steps:
                case.equal(host.steps[0].instructions, BOOT[name], "boot")
            words = bd.peek16(DIAGNOSTICS), bd.peek16(DIAGNOSTICS + 2)
            case.equal(words, (0, 0) if card else (0, 0xF0), "diagnostic words")
            if card == "vcd":
                routes[name] = bd.instructions, overflows(bd)
            bd.close()
    return routes


def check_without_skip(suite, dumps, routes):
    """CDB104 with the vcd card and the clock running at the idle loop: the same instructions and
    overflows as the run that skips, so the skip moves none of them."""
    case = suite.case("cdb104 vcd without the idle skip")
    extension = extensions.from_card(dumps.cards["vcd"])
    bd, host, result = direct(dumps.rom("cdb104"), extension, skip=False)
    check_run(case, bd, host, result, extension)
    case.equal((bd.instructions, overflows(bd)), routes.get("cdb104"), "the skipping run's instructions and overflows")
    bd.close()


def check_round_trips(suite, dumps):
    for name in ROMS:
        if name not in dumps.roms:
            continue
        case = suite.case(f"{name} put and get")
        image = dumps.rom(name)
        profile = PROFILES[name]
        bd = board(image, flags=HARNESS_FLAGS)
        strict.apply(bd, profile, None)
        host = Host(bd, idle=profile.IDLE)
        total = SECTORS * SECTOR_WORDS
        sectors = [(0x1000 + n) & 0xFFFF for n in range(total)]
        try:
            host.boot()
            case.equal(host.put(0, SECTORS, sectors), total, "words put")
            case.check(host.get(0, 0, SECTORS, total) == sectors, "the sectors read back differ")
            case.equal(sum(s.instructions for s in host.steps), bd.instructions, "steps back to back")
            # The firmware's count of each transfer, from 06.
            for step in host.steps:
                if step.name == "06 end transfer":
                    case.equal((step.answer[0] & 0xFF) << 16 | step.answer[1], total, "06 count")
                    case.equal(step.sectors, SECTORS, "06 sectors")
            dei1 = [e for e in bd.log(binding.LOG_INTERRUPT) if e.value == 74]
            case.equal(len(dei1), 2 * SECTORS, "DEI1 entries, one per sector each way")
        except (RunError, EmulatorError) as error:
            case.check(False, f"failed: {error}")
        case.equal(len(bd.log(binding.LOG_VIOLATION)), 0, "violations")
        case.info["steps"] = [f"{s.name}: {s.words} words, {s.instructions}" for s in host.steps]
        bd.close()


def extension_forms(source, out):
    """An extension assembled, then written as ELF, plain code and card ROM."""
    elf = extensions.assemble(source)
    _, code, _ = extensions.elf_image(elf)
    files = {".elf": out / f"{source.stem}.elf", ".bin": out / f"{source.stem}.bin"}
    files[".elf"].write_bytes(elf)
    files[".bin"].write_bytes(code)
    card = out / f"{source.stem}-card.bin"
    card.write_bytes(extensions.payload.card_rom(code))
    return [("source", extensions.from_code(source)), ("ELF", extensions.from_code(files[".elf"])),
            ("plain code", extensions.from_code(files[".bin"])), ("card ROM", extensions.from_card(card))]


def check_extension(suite, dumps, out, source, label):
    """The four forms on CDB105, then the card ROM on the other revisions. Returns each revision's
    instructions for the card ROM's run."""
    try:
        forms = extension_forms(source, out)
    except extensions.ExtensionError as error:
        suite.case(f"{label} extension").check(False, str(error))
        return {}
    routes, first = {}, None
    if "cdb105" in dumps.roms:
        for form, extension in forms:
            case = suite.case(f"cdb105 {label} as {form}")
            bd, host, result = direct(dumps.rom("cdb105"), extension)
            check_run(case, bd, host, result, extension)
            if first is None:
                first = steps(host)
            else:
                case.check(steps(host) == first, "steps differ from the source's")
            routes["cdb105"] = bd.instructions, overflows(bd)
            bd.close()
    card = forms[-1][1]
    for name in ROMS:
        if name == "cdb105" or name not in dumps.roms:
            continue
        case = suite.case(f"{name} {label} as card ROM")
        bd, host, result = direct(dumps.rom(name), card)
        check_run(case, bd, host, result, card)
        routes[name] = bd.instructions, overflows(bd)
        bd.close()
    return routes


def check_wait_limit(suite, dumps):
    """A wait on CDB105 for MPCM, which no card sets on a board without one: the clock skips on at
    the idle loop, so the wait ends on its cycles, within its instruction budget."""
    case = suite.case("cdb105 a wait that never ends")
    profile = PROFILES["cdb105"]
    bd = board(dumps.rom("cdb105"), flags=HARNESS_FLAGS)
    strict.apply(bd, profile, None)
    host = Host(bd, idle=profile.IDLE)
    try:
        host.boot()
        start = bd.instructions
        host.wait_for(MPCM, "MPCM")
        case.check(False, "the wait ended")
    except RunError as error:
        case.check(f"{WAIT_CYCLES:,} cycles passed" in str(error), f"failed otherwise: {error}")
        case.check(bd.instructions - start < host.budget, "within the instruction budget")
    case.info["cycles"] = bd.cycles
    bd.close()


def read_provided(bd, disc, idle):
    """The data at PROVIDED_FRAME as a Saturn reads it from bd, whose drive already holds disc."""
    host = Host(bd, idle=idle)
    drive = Drive(host)
    drive.disc = disc
    drive.read_sectors(0, PROVIDED_FRAME_ADDRESS, 1)
    return b"".join(word.to_bytes(2, "big") for word in host.get(0, 0, 1, DATA // 2))


def check_sector_provider(suite, dumps):
    """A provider that raises, set before the drive is selected again for the disc, so the check or
    a copy reaches it if selecting keeps it. Then each provider on its own copy of the board that
    checked the disc, as a board one fails needs a reset: one that leaves the sectors, one that
    replaces one, one dropped again, a malformed answer and an exception on a copy of the board it
    was set on."""
    case = suite.case("cdb105 sector provider")
    image, idle = dumps.rom("cdb105"), PROFILES["cdb105"].IDLE
    disc = Disc(image, header=None)
    disc.write(PROVIDED_FRAME_ADDRESS, WRITTEN)
    bd = board(image, flags=HARNESS_FLAGS)
    strict.apply(bd, PROFILES["cdb105"], None)
    boards = [bd]

    def provided(provider, base=bd):
        copy = base.clone()
        boards.append(copy)
        copy.drive_disc_sector_provider(provider)
        return copy

    def stop(frame_address, raw):
        raise ProviderStop(frame_address)

    try:
        host = Host(bd, idle=idle)
        host.boot()
        bd.drive_select_disc()
        bd.drive_disc_sector_provider(stop)
        drive = Drive(host)
        drive.insert(disc)
        case.equal(drive.authenticate(CHECK_PARTITION) & 0xFF, 2, "E1's result")

        # A copy takes no provider from a drive selected since.
        plain = bd.clone()
        boards.append(plain)
        case.check(read_provided(plain, disc, idle) == WRITTEN, "the disc's own sector differs")

        seen = {}
        left = provided(lambda frame_address, raw: seen.update({frame_address: raw}))
        case.check(read_provided(left, disc, idle) == WRITTEN, "a sector left alone differs from the disc's")
        case.check(seen.get(PROVIDED_FRAME_ADDRESS) == disc.sectors[PROVIDED_FRAME_ADDRESS],
                   "the provider saw another sector")
        case.info["frame_addresses"] = sorted(seen)

        def replace(frame_address, raw):
            return sector(frame_address, data=REPLACED) if frame_address == PROVIDED_FRAME_ADDRESS else None

        case.check(read_provided(provided(replace), disc, idle) == REPLACED, "the replaced sector did not arrive")

        dropped = provided(replace)
        dropped.drive_disc_sector_provider(None)
        case.check(read_provided(dropped, disc, idle) == WRITTEN, "the provider answered after it was dropped")

        malformed = provided(lambda frame_address, raw: b"short")
        try:
            read_provided(malformed, disc, idle)
            case.check(False, "the read with a malformed sector ended")
        except ValueError as error:
            case.check("2352 bytes" in str(error), f"raised {error}")

        # The copy's own run raises it, not the board it was copied from.
        stopped = provided(stop).clone()
        boards.append(stopped)
        try:
            read_provided(stopped, disc, idle)
            case.check(False, "the read with a raising provider ended")
        except ProviderStop:
            case.check("sector provider stopped the run" in stopped.error(), f"the board's error: {stopped.error()}")
    except (RunError, EmulatorError, ProviderStop) as error:
        case.check(False, f"failed: {error!r}")
    case.equal(len(bd.log(binding.LOG_VIOLATION)), 0, "violations")
    for each in boards:
        each.close()


def get_subcode(host):
    """20 type 1, Get Subcode R-W: None when it answers WAIT, or the error flags in CR4 and the
    pack's 24 symbols, moved through the FIFO and ended with 06, which sets no EHST here. A
    transfer that is not a pack's 12 words raises RunError before a word moves."""
    board, name = host.board, "20 get subcode"
    answer = host.command([0x2001, 0, 0, 0], name=name)
    status = answer[0] >> 8
    if status == REJECT:
        raise RunError(f"{name}: rejected")
    if status & WAIT:
        return None
    total, data = (answer[0] & 0xFF) << 16 | answer[1], []
    if total != rw.PACK // 2:
        raise RunError(f"{name}: a transfer of {total} words, not a pack's {rw.PACK // 2}")

    def move(_, n):
        got = board.host_transfer(n)
        data.extend(got)
        return len(got)

    host._transfer(name, move, total)
    host.command([0x0600, 0, 0, 0], name="06 end transfer")
    return answer[3], [symbol for word in data for symbol in (word >> 8, word & 0xFF)]


def subcode_board(image):
    """A board for a subcode play, with its host and the disc for its drive: a data track, then the
    audio track."""
    bd = board(image, flags=HARNESS_FLAGS)
    strict.apply(bd, PROFILES["cdb105"], None)
    host = Host(bd, idle=PROFILES["cdb105"].IDLE)
    return bd, host, Disc(image, header=None, tracks=((FIRST, DATA_TRACK), (AUDIO_START, AUDIO_TRACK)))


def insert_subcode(host, disc):
    """04's R-W setting on, then the disc in the drive with the generated packs around the frames
    played. The board takes the subcode after the insert, which selects the drive again."""
    host.command([0x0402, 0xFFFF, 0, 0xFFFF], name="04 R-W subcode on")
    Drive(host).insert(disc)
    for n in range(PLAY_SECTORS + 2 * SUBCODE_MARGIN):
        host.board.drive_disc_subcode(PLAY_FRAME_ADDRESS - SUBCODE_MARGIN + n, rw.sector(n))


def read_packs(host, packs):
    """Every pack 20 has for the host now, added to packs."""
    while True:
        got = get_subcode(host)
        if got is None:
            return
        packs.append(got)


def idle_for(host, cycles, what, flags=0):
    bd = host.board
    until = bd.cycles + cycles
    host.idle_until(lambda: bd.cycles >= until or bd.host_flags() & flags, what)


def play_subcode(host, sectors):
    """An audio play of sectors from PLAY_FRAME_ADDRESS, the host asking for the packs with 20 once
    a sector period. Returns the packs it got by two periods after the play, when the drive has
    paused. A play that does not end raises RunError as the host's waits do."""
    bd, packs = host.board, []
    bd.host_clear_flags(0xFFFF & ~PEND)
    answer = host.command([0x1080 | PLAY_FRAME_ADDRESS >> 16, PLAY_FRAME_ADDRESS & 0xFFFF,
                           0x0080 | sectors >> 16, sectors & 0xFFFF], name="10 play")
    if answer[0] >> 8 == REJECT:
        raise RunError("10 play: rejected")
    start = bd.cycles
    while not bd.host_flags() & PEND:
        if bd.cycles - start > WAIT_CYCLES:
            raise RunError(f"audio play: {WAIT_CYCLES:,} cycles passed")
        read_packs(host, packs)
        idle_for(host, SECTOR_CYCLES, "audio play", PEND)
    idle_for(host, 2 * SECTOR_CYCLES, "after the play")
    read_packs(host, packs)
    return packs


def check_packs(case, what, packs, sectors, shift=0):
    """The packs of a play of sectors: four a sector from FIRST_PACK, each once and in order, as the
    disc holds them shift sectors on, none with an error flag. what names the play in a failure."""
    first = FIRST_PACK + rw.SECTOR_PACKS * shift
    case.check(all(flags == 0 for flags, _ in packs),
               f"{what}: a pack came with the pack data error or the overrun flag")
    case.check([symbols for _, symbols in packs] == [rw.pack(first + k) for k in range(rw.SECTOR_PACKS * sectors)],
               f"{what}: not packs {first} to {first + rw.SECTOR_PACKS * sectors - 1}, each once and in order: "
               f"{[rw.number(symbols) for _, symbols in packs]}")


def check_subcode(suite, dumps):
    """An audio play with 04's R-W setting on. The firmware locks on to the drive's subcode frames
    and decodes the packs, and the host asks for them with 20 once a sector period. After the play
    the drive pauses, which ends the frames: the decoded packs still come, then none."""
    case = suite.case("cdb105 subcode")
    bd, host, disc = subcode_board(dumps.rom("cdb105"))
    try:
        host.boot()
        insert_subcode(host, disc)
        packs = play_subcode(host, PLAY_SECTORS)
        played = len(packs)
        idle_for(host, 4 * SECTOR_CYCLES, "paused")
        read_packs(host, packs)
        case.equal(len(packs), played, "packs once the drive paused")
        check_packs(case, "the play", packs, PLAY_SECTORS)
        case.info["packs"], case.info["first"] = len(packs), rw.number(packs[0][1]) if packs else None
        case.info["flags"] = sorted({flags for flags, _ in packs})
    except (RunError, EmulatorError) as error:
        case.check(False, f"failed: {error!r}")
    case.equal(len(bd.log(binding.LOG_VIOLATION)), 0, "violations")
    bd.close()


def check_subcode_provider(suite, dumps):
    """A provider that raises, set before the drive is selected again for the disc, so a copy
    reaches it if selecting keeps it. Then each provider on its own copy of the board that took the
    disc, as a board one fails needs a reset: one that leaves the subcode, one that replaces it, one
    dropped again, a malformed answer and an exception on a copy of the board it was set on."""
    case = suite.case("cdb105 subcode provider")
    bd, host, disc = subcode_board(dumps.rom("cdb105"))
    idle, boards, first = host.idle, [bd], PLAY_FRAME_ADDRESS - SUBCODE_MARGIN

    def provided(provider):
        copy = bd.clone()
        boards.append(copy)
        copy.drive_disc_subcode_provider(provider)
        return copy

    def played(copy):
        return play_subcode(Host(copy, idle=idle), PROVIDED_SECTORS)

    def stop(frame_address, symbols):
        raise ProviderStop(frame_address)

    try:
        host.boot()
        bd.drive_select_disc()
        bd.drive_disc_subcode_provider(stop)
        insert_subcode(host, disc)

        # A copy takes no provider from a drive selected since.
        plain = bd.clone()
        boards.append(plain)
        check_packs(case, "the disc's own subcode", played(plain), PROVIDED_SECTORS)

        seen = {}
        left = provided(lambda frame_address, symbols: seen.update({frame_address: symbols}))
        check_packs(case, "subcode left alone", played(left), PROVIDED_SECTORS)
        case.check(PLAY_FRAME_ADDRESS in seen and all(symbols == rw.sector(frame_address - first)
                                                      for frame_address, symbols in seen.items()),
                   "the provider saw other subcode than the disc's")
        case.info["frame_addresses"] = sorted(seen)

        def replace(frame_address, symbols):
            return rw.sector(frame_address - first + PROVIDED_SHIFT)

        check_packs(case, "replaced subcode", played(provided(replace)), PROVIDED_SECTORS, PROVIDED_SHIFT)

        dropped = provided(replace)
        dropped.drive_disc_subcode_provider(None)
        check_packs(case, "a provider dropped again", played(dropped), PROVIDED_SECTORS)

        try:
            played(provided(lambda frame_address, symbols: b"short"))
            case.check(False, "the play with malformed subcode ended")
        except ValueError as error:
            case.check("96 symbols" in str(error), f"raised {error}")

        # The copy's own run raises it, not the board it was copied from.
        stopped = provided(stop).clone()
        boards.append(stopped)
        try:
            played(stopped)
            case.check(False, "the play with a raising provider ended")
        except ProviderStop:
            case.check("subcode provider stopped the run" in stopped.error(), f"the board's error: {stopped.error()}")
    except (RunError, EmulatorError, ProviderStop) as error:
        case.check(False, f"failed: {error!r}")
    case.equal(len(bd.log(binding.LOG_VIOLATION)), 0, "violations")
    for each in boards:
        each.close()


def cli_status(argv):
    output, errors = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(output), contextlib.redirect_stderr(errors):
        status = cli.main([str(a) for a in argv])
    return status, output.getvalue(), errors.getvalue()


# A scenario that runs the firmware to its idle address after the host's helpers, which must leave
# no write stop behind.
SCENARIO = """\
from cdblock.host import run_to
host.hardware_info()
board.host_command([0x0100, 0, 0, 0])
run_to(board, [profile.IDLE])
"""


def check_cli(suite, dumps, out):
    firmware, vcd = dumps.roms["cdb105"], dumps.cards["vcd"]
    case = suite.case("command line")
    status, text, _ = cli_status(["--firmware", firmware, "--card", vcd])
    case.equal(status, 0, "run status")
    case.check("MPEG version 2" in text, "run result")
    case.check("checks    strict" in text, "a known revision runs strict")
    # CDB105 with two unused words changed and the checksum kept: no known revision.
    changed = bytearray(firmware.read_bytes())
    changed[0xD000:0xD008] = bytes.fromhex("fffffffe00000000")
    unknown = out / "unknown-firmware.bin"
    unknown.write_bytes(changed)
    status, text, _ = cli_status(["--firmware", unknown])
    case.check(status == 0 and "not a known revision" in text and "checks    lenient" in text
               and " 0 violations" in text, f"unknown dump: {status}, {text.strip()[-300:]}")
    short = out / "short-card.bin"
    short.write_bytes(vcd.read_bytes()[:4096])
    status, _, errors = cli_status(["--firmware", firmware, "--card", short])
    case.check(status == 2 and "check value" in errors, f"truncated card ROM: {status}, {errors.strip()}")
    elf = out / "lifecycle-extension.elf"
    if elf.exists():
        cut = out / "truncated.elf"
        cut.write_bytes(elf.read_bytes()[:60])
        status, _, errors = cli_status(["--firmware", firmware, "--extension", cut])
        case.check(status == 2 and "cdblock:" in errors, f"truncated ELF: {status}, {errors.strip()}")
    half = out / "half-firmware.bin"
    half.write_bytes(firmware.read_bytes()[:0x8000])
    status, _, _ = cli_status(["--firmware", half])
    case.equal(status, 2, "half a firmware dump")
    kept = os.environ.get("CDBLOCK_LIB")
    os.environ["CDBLOCK_LIB"] = str(out / "no-library")
    try:
        status, _, errors = cli_status(["--firmware", firmware])
    finally:
        if kept is None:
            del os.environ["CDBLOCK_LIB"]
        else:
            os.environ["CDBLOCK_LIB"] = kept
    case.check(status == 2 and "did not load" in errors, f"no library: {status}, {errors.strip()}")
    scenario = out / "idle-after-host.py"
    scenario.write_text(SCENARIO)
    status, text, _ = cli_status(["--firmware", firmware, scenario])
    case.check(status == 0 and "idle-after-host.py done" in text, f"scenario: {status}, {text.strip()[-300:]}")


def run(context):
    suite = Suite("package")
    dumps, out = context.dumps, context.out
    has = lambda *names: all(n in dumps.roms or n in dumps.cards for n in names)  # noqa: E731
    routes = check_matrix(suite, dumps)
    if has("cdb104", "vcd"):
        check_without_skip(suite, dumps, routes)
    else:
        suite.skip("cdb104 vcd without the idle skip: needs cdb104.bin and mpeg-card/vcd.bin")
    check_round_trips(suite, dumps)
    lifecycle = module = {}
    if context.binutils:
        lifecycle = check_extension(suite, dumps, out, LIFECYCLE, "lifecycle")
        module = check_extension(suite, dumps, out, MODULE, "module")
    else:
        suite.skip("the lifecycle and module extensions: no sh-elf- or sh2eb-elf- binutils")
    if has("cdb105", "vcd"):
        check_cli(suite, dumps, out)
    else:
        suite.skip("command line: needs cdb105.bin and mpeg-card/vcd.bin")
    if has("cdb105"):
        check_wait_limit(suite, dumps)
        check_sector_provider(suite, dumps)
        check_subcode(suite, dumps)
        check_subcode_provider(suite, dumps)
    else:
        suite.skip("cdb105 a wait that never ends, the providers and the subcode: need cdb105.bin")
    if dumps.complete and context.binutils:
        suite.pin("cases", len(suite.cases), 60)
    # Each profile's code: CDB105's blocks, 0x12 bytes more in CDB106's layout, the strap code's 22
    # for the 4 it replaces. The whole ROM, the code without a profile, fails it.
    for name in ROMS:
        suite.pin(f"codeBytes-{name}", sum(end - start for start, end in strict.firmware_code(PROFILES[name])),
                  0x9E44 if name in ("cdb106", "ygr022") else 0x9E32)
    # The default run's instructions, seven more in CDB106 and YGR022 as at boot, and the overflows
    # the clock adds over the card load. The skip at the idle loop moves none of them (CDB104's run
    # without it).
    for name in ROMS:
        if name not in dumps.roms:
            continue
        late = 7 if name in ("cdb106", "ygr022") else 0
        for route, results, before, count, ready in (("vcdRoute", routes, 11_048_274, 14, has("vcd")),
                                                      ("lifecycleRoute", lifecycle, 9_961_995, 12, context.binutils),
                                                      ("moduleRoute", module, 10_037_235, 12, context.binutils)):
            if not ready:
                continue
            instructions, taken = results.get(name, (None, None))
            suite.pin(f"{route}-{name}", instructions, before + late + OVERFLOW_INSTRUCTIONS * count)
            suite.pin(f"{route}Overflows-{name}", taken, count)
    return [suite]
