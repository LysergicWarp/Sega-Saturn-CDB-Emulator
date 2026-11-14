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
  the disc holds once, in order and without an error flag, until the pause ends them. The disc's
  symbols have bits 6 and 7 set, which the firmware drops. The disc refuses subcode that is not 96
  symbols, a number in their place and a frame address the drive does not reach.
- The drive's subcode provider in Python on CDB105, as the sector provider: what it leaves or
  replaces reaches the host as packs, and its exceptions come back from the run on the board that
  ran it. Neither provider sees what the disc was given after it went in.
- The drive's tray on CDB105: an opening gives the host status open with DCHG, a closing on another
  disc that disc's TOC through 02 and its sectors and none of the first disc's, 05 opens the tray
  through drive command 0c, and a closing on an empty tray gives the status no disc. An opening
  while the SH-1 still sends a seek passes. A disc changed with no link frame between the opening
  and the closing is seen as any other.
- The tray at boot and the host's requests on CDB105: a board booted without a disc gives the
  status no disc, where 02 sends the drive nothing and a play sends 0c, which opens the tray. A
  play asked for while the tray is open sends 03, which the drive refuses, and is carried out on
  the disc the tray closes on, with the drive connected to no filter. A board booted with its tray
  open gives the status open.
- Pregaps on CDB105: a read through 10 that runs from the data track into an audio track's pregap
  gives the host the data track's sectors, and the firmware plays on from the audio track's index 1.
  A play that runs from the audio track into a data track's pregap is read on from the frame before
  that track's start at double speed, and the host gets the data track's sectors. The disc's
  pregaps: the Q they hold, a long one before a data track, one merged with written subcode and
  the lengths and tracks refused.

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
from cdblock.disc import (DATA, DATA_TRACK, FIRST, LAST, LEAD_OUT, NO_DISC, OPEN, PAUSE, PREGAP, SUBCODE, Disc,
                          Drive, q_symbols, sector)
from cdblock.host import DCHG, DRDY, ESEL, MPCM, PEND, REJECT, WAIT, WAIT_CYCLES, Host, RunError

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
# The tray case's second disc: an audio track after the data track and an earlier lead-out. 02
# answers with the TOC's 102 entries, two words each: tracks 1 to 99, the first track, the last
# track and the lead-out.
CHANGED_TRACKS, CHANGED_LEAD_OUT = ((FIRST, DATA_TRACK), (30_000, AUDIO_TRACK)), 60_000
TOC_ENTRIES, NO_TRACK = 102, 0xFFFFFFFF
# The pregap case's disc: the data track, the audio track and a second data track, the later two
# with a pregap each. Each of its reads runs from PREGAP_BEFORE sectors before a pregap to
# PREGAP_AFTER sectors into the track.
LATER_DATA_START = 21_000
PREGAP_BEFORE, PREGAP_AFTER = 10, 40


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
        # The drive took the disc's sectors as the disc went in: a write after that does not reach
        # it.
        held = disc.sectors[PROVIDED_FRAME_ADDRESS]
        disc.write(PROVIDED_FRAME_ADDRESS, REPLACED)

        # A copy takes no provider from a drive selected since.
        plain = bd.clone()
        boards.append(plain)
        case.check(read_provided(plain, disc, idle) == WRITTEN, "the disc's own sector differs")

        seen = {}
        left = provided(lambda frame_address, raw: seen.update({frame_address: raw}))
        case.check(read_provided(left, disc, idle) == WRITTEN, "a sector left alone differs from the disc's")
        case.check(seen.get(PROVIDED_FRAME_ADDRESS) == held, "the provider saw another sector")
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
    name = "20 get subcode"
    answer = host.command([0x2001, 0, 0, 0], name=name)
    status = answer[0] >> 8
    if status == REJECT:
        raise RunError(f"{name}: rejected")
    if status & WAIT:
        return None
    total = (answer[0] & 0xFF) << 16 | answer[1]
    if total != rw.PACK // 2:
        raise RunError(f"{name}: a transfer of {total} words, not a pack's {rw.PACK // 2}")
    data = take_words(host, name, total)
    return answer[3], [symbol for word in data for symbol in (word >> 8, word & 0xFF)]


def take_words(host, name, total):
    """The total words of the transfer a command opened, moved through the FIFO and ended with 06."""
    data = []

    def move(_, n):
        got = host.board.host_transfer(n)
        data.extend(got)
        return len(got)

    host._transfer(name, move, total)
    host.command([0x0600, 0, 0, 0], name="06 end transfer")
    return data


def subcode_board(image):
    """A board for a subcode play, with its host and the disc for its drive: a data track, then the
    audio track, with the generated packs as its subcode around the frames played."""
    bd = board(image, flags=HARNESS_FLAGS)
    strict.apply(bd, PROFILES["cdb105"], None)
    host = Host(bd, idle=PROFILES["cdb105"].IDLE)
    disc = Disc(image, header=None, tracks=((FIRST, DATA_TRACK), (AUDIO_START, AUDIO_TRACK)))
    for n in range(PLAY_SECTORS + 2 * SUBCODE_MARGIN):
        disc.write_subcode(PLAY_FRAME_ADDRESS - SUBCODE_MARGIN + n, rw.sector(n))
    return bd, host, disc


def insert_subcode(host, disc):
    """04's R-W setting on, then the disc in the drive, which takes the disc's subcode with it."""
    host.command([0x0402, 0xFFFF, 0, 0xFFFF], name="04 R-W subcode on")
    Drive(host).insert(disc)


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


def refuses(case, call, error, text, what):
    """call raises error with text in its message. what names the call in a failure."""
    try:
        call()
        case.check(False, f"{what}: taken")
    except error as raised:
        case.check(text in str(raised), f"{what}: raised {raised}")


def check_subcode(suite, dumps):
    """An audio play with 04's R-W setting on. The firmware locks on to the drive's subcode frames
    and decodes the packs, and the host asks for them with 20 once a sector period. After the play
    the drive pauses, which ends the frames: the decoded packs still come, then none. The disc's
    symbols have bits 6 and 7 set, the Q and P channels, and the packs come without them. First
    what the disc refuses: subcode that is not 96 symbols, a number in their place, and a frame
    address the drive does not reach, for subcode and for a sector's data."""
    case = suite.case("cdb105 subcode")
    bd, host, disc = subcode_board(dumps.rom("cdb105"))
    for symbols in (b"", bytes(SUBCODE - 1), bytes(SUBCODE + 1)):
        refuses(case, lambda: disc.write_subcode(PLAY_FRAME_ADDRESS, symbols), ValueError, "96 symbols",
                f"subcode of {len(symbols)} symbols")
    refuses(case, lambda: disc.write_subcode(PLAY_FRAME_ADDRESS, SUBCODE), TypeError, "not the number",
            "a number as the disc's subcode")
    refuses(case, lambda: bd.drive_disc_subcode(PLAY_FRAME_ADDRESS, SUBCODE), TypeError, "not the number",
            "a number as the board's subcode")
    for frame_address in (-1, LAST + 1, 1 << 32, str(PLAY_FRAME_ADDRESS)):
        refuses(case, lambda: disc.write_subcode(frame_address, bytes(SUBCODE)), ValueError, "frame addresses",
                f"subcode at frame address {frame_address!r}")
        refuses(case, lambda: disc.write(frame_address, b""), ValueError, "frame addresses",
                f"a sector at frame address {frame_address!r}")
    refuses(case, lambda: disc.write(LAST, bytes(DATA + 1)), ValueError, "frame addresses",
            "two sectors from the last frame address")
    case.equal(disc.write(LAST, b""), 1, "sectors written at the last frame address")
    disc.write_subcode(LAST, bytes(SUBCODE))
    for frame_address, symbols in list(disc.subcodes.items()):
        disc.write_subcode(frame_address, bytes(symbol | 0xC0 for symbol in symbols))
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
        # The drive took the disc's subcode as the disc went in: subcode written after that does
        # not reach it.
        held = dict(disc.subcodes)
        disc.write_subcode(PLAY_FRAME_ADDRESS, rw.sector(PROVIDED_SHIFT))

        # A copy takes no provider from a drive selected since.
        plain = bd.clone()
        boards.append(plain)
        check_packs(case, "the disc's own subcode", played(plain), PROVIDED_SECTORS)

        seen = {}
        left = provided(lambda frame_address, symbols: seen.update({frame_address: symbols}))
        check_packs(case, "subcode left alone", played(left), PROVIDED_SECTORS)
        # The drive asks for the sector at each position it reports too, where this disc holds none
        # and the provider sees zeros.
        case.check(PLAY_FRAME_ADDRESS in seen and all(symbols == held.get(frame_address, bytes(SUBCODE))
                                                      for frame_address, symbols in seen.items()),
                   "the provider saw other subcode than the disc went in with")
        case.check(any(frame_address not in held for frame_address in seen),
                   "the provider saw no sector without subcode")
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


def get_toc(host):
    """02, Get TOC: its entries, two words each as one value, moved through the FIFO and ended with
    06, which sets no EHST here. A transfer that is not the TOC's 204 words raises RunError before
    a word moves."""
    name = "02 get TOC"
    total = host.command([0x0200, 0, 0, 0], DRDY, name)[1]
    if total != 2 * TOC_ENTRIES:
        raise RunError(f"{name}: a transfer of {total} words, not the TOC's {2 * TOC_ENTRIES}")
    data = take_words(host, name, total)
    return [data[k] << 16 | data[k + 1] for k in range(0, total, 2)]


def toc(tracks, lead_out):
    """The TOC of a disc with tracks and lead_out, as 02 returns it: each track's control/ADR byte
    and start, the first and the last track's number under its control/ADR byte, and the lead-out's
    start under the last track's."""
    entries = [control << 24 | start for start, control in tracks]
    entries += [NO_TRACK] * (TOC_ENTRIES - 3 - len(tracks))
    return entries + [tracks[0][1] << 24 | 1 << 16, tracks[-1][1] << 24 | len(tracks) << 16,
                      tracks[-1][1] << 24 | lead_out]


def check_tray(suite, dumps):
    """A disc in the drive, then its tray opened, closed on a second disc, opened by the host with
    05, closed on nothing and closed on the first disc again. The step of each opening and closing
    ends on the host status it names, so a status that never comes fails the case. The first
    opening comes while the SH-1 still sends its seek back from the disc check, which the open tray
    refuses. The first disc holds a sector the second does not, which must not stay in the drive.
    At last the disc changes in one go, the tray opened and closed with no link frame between: the
    drive reports the opening three times all the same."""
    case = suite.case("cdb105 tray and disc change")
    image, profile = dumps.rom("cdb105"), PROFILES["cdb105"]
    bd = board(image, flags=HARNESS_FLAGS)
    strict.apply(bd, profile, None)
    host = Host(bd, idle=profile.IDLE)
    drive = Drive(host)
    first, second = Disc(image, header=None), Disc(image, header=None, tracks=CHANGED_TRACKS,
                                                   lead_out=CHANGED_LEAD_OUT)
    first.write(PROVIDED_FRAME_ADDRESS, WRITTEN + WRITTEN)
    second.write(PROVIDED_FRAME_ADDRESS, REPLACED)

    def status():
        return bd.host_response()[0] >> 8 & 0x0F

    try:
        host.boot()
        drive.insert(first)
        case.equal(bd.drive_disc_state(), (False, True), "the tray open and a disc in, after insert")
        case.check(get_toc(host) == toc(first.tracks, LEAD_OUT), "the first disc's TOC differs")
        try:
            drive.change(second)
            case.check(False, "the disc changed with the tray closed")
        except EmulatorError as error:
            case.check("the tray is closed" in str(error), f"raised {error}")
        case.check(drive.disc is first, "a refused change took the disc")

        case.equal(drive.authenticate(CHECK_PARTITION) & 0xFF, 2, "E1's result on the first disc")
        mark = bd.log_count()
        step = drive.open_tray()
        case.check(any(e.text.endswith("the tray open, not obeyed") for e in bd.log(binding.LOG_DRIVE_COMMAND, mark)),
                   "the open tray refused no command of the SH-1's")
        case.check(step.flags & DCHG, "the opening set no DCHG")
        case.equal(bd.host_response()[1:], [0xFFFF] * 3, "the position of an open tray")
        case.equal(bd.drive_disc_state(), (True, True), "the tray open and a disc in, after the opening")
        drive.change(second)
        drive.close_tray()
        case.check(get_toc(host) == toc(CHANGED_TRACKS, CHANGED_LEAD_OUT), "the second disc's TOC differs")
        case.equal(drive.authenticate(CHECK_PARTITION) & 0xFF, 2, "E1's result")
        drive.read_sectors(0, PROVIDED_FRAME_ADDRESS, 2)
        got = b"".join(word.to_bytes(2, "big") for word in host.get(0, 0, 2, DATA))
        case.check(got == REPLACED + bytes(DATA), "not the second disc's sector and one it does not hold")

        mark = bd.log_count()
        host.command([0x0500, 0, 0, 0], name="05 open tray")
        host.idle_until(lambda: status() == OPEN, "05 open tray")
        case.equal([e.text[:2] for e in bd.log(binding.LOG_DRIVE_COMMAND, mark)], ["0c"], "the drive commands of 05")
        case.equal(bd.drive_disc_state(), (True, True), "the tray open and a disc in, after 05")
        drive.change(None)
        case.equal(bd.drive_disc_state(), (True, False), "the tray open and a disc in, the disc taken out")
        drive.close_tray()
        case.equal(status(), NO_DISC, "the status of an empty tray")
        case.equal(bd.host_response()[1:], [0xFFFF] * 3, "the position of an empty tray")
        case.equal(bd.drive_disc_state(), (False, False), "the tray open and a disc in, closed on nothing")

        drive.open_tray()
        drive.change(first)
        drive.close_tray()
        case.check(get_toc(host) == toc(first.tracks, LEAD_OUT), "the first disc's TOC differs after the changes")

        # The SH-1 takes no report while its seek back from the check is still to go out, which a
        # single report of the opening would fall to.
        case.equal(drive.authenticate(CHECK_PARTITION) & 0xFF, 2, "E1's result before the change in one go")
        bd.host_clear_flags(0xFFFF & ~DCHG)
        mark = bd.log_count()
        bd.drive_disc_tray(True)
        drive.change(second)
        bd.drive_disc_tray(False)
        case.equal(bd.drive_disc_state(), (False, True), "the tray open and a disc in, changed in one go")
        idle_for(host, 75 * SECTOR_CYCLES, "a change in one go")
        case.equal([e.value for e in bd.log(binding.LOG_DRIVE_REPORT, mark)].count(0x80), 3,
                   "reports of the open tray for a change in one go")
        case.check(bd.host_flags() & DCHG, "the change in one go set no DCHG")
        case.equal(status(), PAUSE, "the status a second after a change in one go")
        case.check(get_toc(host) == toc(CHANGED_TRACKS, CHANGED_LEAD_OUT),
                   "the second disc's TOC differs after the change in one go")
        case.info["steps"] = [f"{s.name}: {s.instructions}" for s in host.steps]
    except (RunError, EmulatorError) as error:
        case.check(False, f"failed: {error!r}")
    case.equal(len(bd.log(binding.LOG_VIOLATION)), 0, "violations")
    bd.close()


def check_tray_requests(suite, dumps):
    """A board booted with its tray closed on nothing: the host status is no disc, 02 sends the
    drive nothing, and a play makes the firmware send drive command 0c, which opens the tray. The
    caller sees that in the drive's state, puts a disc in and closes it. Then a play asked for
    while the tray is open: the firmware sends 03, which the open drive refuses, and carries the
    play out on the disc the tray closes on. The drive is connected to no filter by then, so the
    play's sectors reach no partition. At last a board booted with its tray open."""
    case = suite.case("cdb105 tray at boot and drive requests")
    image, profile = dumps.rom("cdb105"), PROFILES["cdb105"]
    disc = Disc(image, header=None)
    disc.write(PROVIDED_FRAME_ADDRESS, WRITTEN)
    play = [0x1080 | PROVIDED_FRAME_ADDRESS >> 16, PROVIDED_FRAME_ADDRESS & 0xFFFF, 0x0080, 1]
    boards = []

    def booted(opened):
        """A board whose drive has its tray open, or closed on nothing, from before its boot."""
        bd = board(image, flags=HARNESS_FLAGS)
        boards.append(bd)
        strict.apply(bd, profile, None)
        bd.drive_select_disc()
        bd.drive_disc_tray(True)
        if not opened:
            bd.drive_disc_present(False)
            bd.drive_disc_tray(False)
        host = Host(bd, idle=profile.IDLE)
        host.boot()
        return bd, host, lambda: bd.host_response()[0] >> 8 & 0x0F

    def sent(bd, mark):
        return [e.text for e in bd.log(binding.LOG_DRIVE_COMMAND, mark)]

    try:
        bd, host, status = booted(False)
        drive = Drive(host)
        host.idle_until(lambda: status() == NO_DISC, "the boot without a disc")
        case.equal(bd.drive_disc_state(), (False, False), "the tray open and a disc in, booted without a disc")
        mark = bd.log_count()
        get_toc(host)
        idle_for(host, 20 * SECTOR_CYCLES, "after 02 without a disc")
        case.equal(sent(bd, mark), [], "the drive commands of 02 without a disc")
        case.equal(status(), NO_DISC, "the status after 02 without a disc")
        host.command(play, name="10 play")
        host.idle_until(lambda: status() == OPEN, "the play without a disc")
        case.equal([text[:2] for text in sent(bd, mark)], ["0c"], "the drive commands of a play without a disc")
        case.equal(bd.drive_disc_state(), (True, False), "the tray open and a disc in, after that play")
        drive.change(disc)
        drive.close_tray()
        case.check(get_toc(host) == toc(disc.tracks, LEAD_OUT), "the TOC of the disc put in differs")
        case.equal(drive.authenticate(CHECK_PARTITION) & 0xFF, 2, "E1's result")

        drive.open_tray()
        host.command([0x3000, 0, 0, 0], ESEL, "30 connect drive")
        bd.host_clear_flags(0xFFFF & ~PEND)
        mark = bd.log_count()
        host.command(play, name="10 play")
        idle_for(host, 20 * SECTOR_CYCLES, "the play asked of an open tray")
        refused = sent(bd, mark)
        case.check(refused and all(text.startswith("03") and text.endswith("the tray open, not obeyed")
                                   for text in refused), f"the drive commands of a play asked of an open tray: {refused}")
        case.check(not bd.host_flags() & PEND, "the play ended while the tray was open")
        mark = bd.log_count()
        drive.close_tray()
        host.wait_for(PEND, "the play kept through the open tray")
        # The TOC, the seek to the first frame address, then the play and the pause at its end.
        case.equal([text[:2] for text in sent(bd, mark)], ["03", "09", "06", "08"],
                   "the drive commands after the closing")
        # The firmware connects the drive to no filter after the closing, so the play's sectors
        # reach no partition.
        case.equal(host.command([0x3200, 0, 0, 0], name="32 get connection")[2] >> 8, 0xFF,
                   "the filter the drive is connected to after the closing")
        case.equal(host.command([0x5100, 0, 0, 0], name="51 get sector number")[3], 0,
                   "sectors of the kept play in the partition")
        case.info["steps"] = [f"{s.name}: {s.instructions}" for s in host.steps]

        bd, host, status = booted(True)
        host.idle_until(lambda: status() == OPEN, "the boot with the tray open")
        case.equal(bd.drive_disc_state(), (True, True), "the tray open and a disc in, booted with the tray open")
        case.equal(sent(bd, 0), [], "the drive commands of a boot with the tray open")
        bd.drive_disc_tray(False)
        host.idle_until(lambda: status() == PAUSE, "the closing after a boot with the tray open")
        case.check(get_toc(host) == toc(((FIRST, DATA_TRACK),), LEAD_OUT), "the TOC after that closing differs")
    except (RunError, EmulatorError) as error:
        case.check(False, f"failed: {error!r}")
    for each in boards:
        case.equal(len(each.log(binding.LOG_VIOLATION)), 0, "violations")
        each.close()


def q_bytes(symbols):
    """The ten bytes of the Q in bit 6 of a sector's symbols, without its CRC."""
    return bytes(sum((symbol >> 6 & 1) << (7 - bit) for bit, symbol in enumerate(symbols[k * 8:k * 8 + 8]))
                 for k in range(10))


def check_pregap(suite, dumps):
    """Two reads through 10 that each run into a track of the other kind with a pregap. From the
    data track into the audio track: at the pregap's index 0 the firmware plays on from the audio
    track's index 1 at single speed, and the host gets the data track's sectors up to the pregap.
    From the audio track into the second data track: at the pregap's index 0 the firmware reads on
    from the frame before the data track's start at double speed, and the host gets that track's
    sectors. Each time it pauses the drive at the read's end."""
    case = suite.case("cdb105 pregap")
    image, profile = dumps.rom("cdb105"), PROFILES["cdb105"]
    bd = board(image, flags=HARNESS_FLAGS)
    strict.apply(bd, profile, None)
    host = Host(bd, idle=profile.IDLE)
    drive = Drive(host)
    disc = Disc(image, header=None, tracks=((FIRST, DATA_TRACK), (AUDIO_START, AUDIO_TRACK),
                                            (LATER_DATA_START, DATA_TRACK)))
    count = PREGAP_BEFORE + PREGAP + PREGAP_AFTER
    into_audio, into_data = AUDIO_START - PREGAP - PREGAP_BEFORE, LATER_DATA_START - PREGAP - PREGAP_BEFORE
    before = b"".join(f"sector {into_audio + k}".encode().ljust(DATA, b"\0") for k in range(PREGAP_BEFORE))
    after = b"".join(f"sector {LATER_DATA_START + k}".encode().ljust(DATA, b"\0") for k in range(PREGAP_AFTER))
    disc.write(into_audio, before)
    disc.write(LATER_DATA_START, after)
    # A Q of ADR 2, a catalogue number, in the first read's way. The firmware takes a position
    # from a Q of ADR 1 alone, so the read goes as without it.
    disc.write_subcode(into_audio + 3, q_symbols(bytes.fromhex("42123456789012300052")))
    refuses(case, lambda: disc.pregap(1), ValueError, "track 2", "a pregap of the first track")
    refuses(case, lambda: disc.pregap(4), ValueError, "track 2", "a pregap of a track the disc has not")
    refuses(case, lambda: disc.pregap(2, 0), ValueError, "does not fit", "a pregap of no frames")
    refuses(case, lambda: disc.pregap(2, AUDIO_START - FIRST), ValueError, "does not fit",
            "a pregap as long as the track before it")
    refuses(case, lambda: q_symbols(bytes(9)), ValueError, "ten bytes", "a Q of nine bytes")
    disc.pregap(2)
    disc.pregap(3)
    q = b"".join(q_bytes(disc.subcode()[frame]) for frame in (AUDIO_START - PREGAP, AUDIO_START - 1))
    case.equal(q.hex(" "), "01 02 00 00 01 74 00 04 24 50 01 02 00 00 00 00 00 04 26 49",
               "the Q of the audio track's pregap, its first and last frame")
    # Pregaps of three seconds. The data track's holds the audio track's control/ADR byte up to its
    # last two seconds, the audio track's its own throughout. The Q takes bit 6 alone of subcode
    # written there.
    longer = Disc(image, header=None, tracks=disc.tracks)
    longer.write_subcode(LATER_DATA_START - 1, bytes([0xBF]) * SUBCODE)
    longer.pregap(2, PREGAP + 75)
    longer.pregap(3, PREGAP + 75)
    held = longer.subcode()
    case.equal([q_bytes(held[start - before])[:3].hex() for start in (LATER_DATA_START, AUDIO_START)
                for before in (PREGAP + 75, PREGAP + 1, PREGAP, 1)],
               ["010300", "010300", "410300", "410300"] + ["010200"] * 4,
               "the control/ADR byte, track and index through the pregaps of three seconds")
    merged = held[LATER_DATA_START - 1]
    case.check(all(symbol | 0x40 == 0xFF for symbol in merged) and merged == bytes(
        symbol | 0xBF for symbol in q_symbols(q_bytes(merged))), "the pregap's Q changed other bits of written subcode")

    def crossed(partition, first, play, control, track, sectors, data, what):
        """A read of count sectors from first into partition, which the firmware takes up again
        with play: the frame address, index, track and speed of the drive command it sends."""
        mark = bd.log_count()
        drive.read_sectors(partition, first, count)
        host.idle_until(lambda: bd.host_response()[0] >> 8 & 0x0F == PAUSE, "the pause after the read")
        commands = [bytes.fromhex("".join(e.text.split()[:13])) for e in bd.log(binding.LOG_DRIVE_COMMAND, mark)]
        case.equal([command[0] for command in commands], [0x06, 0x06, 0x08], f"the drive commands of the read {what}")
        if len(commands) == 3:
            sent = commands[1]
            case.equal([int.from_bytes(sent[1:4], "big"), sent[4], sent[8], sent[10]], play,
                       f"the frame address, index, track and speed of the play sent at the pregap {what}")
        case.equal(bd.host_response()[1:], [control << 8 | track, 0x0100 | (first + count) >> 16,
                                            (first + count) & 0xFFFF], f"the position after the read {what}")
        case.equal(host.command([0x5100, 0, partition << 8, 0], name="51 get sector number")[3], sectors,
                   f"sectors in the partition {what}")
        got = b"".join(word.to_bytes(2, "big") for word in host.get(partition, 0, sectors, sectors * DATA // 2))
        case.check(got == data, f"the data track's sectors did not arrive {what}")

    try:
        host.boot()
        drive.insert(disc)
        case.equal(drive.authenticate(CHECK_PARTITION) & 0xFF, 2, "E1's result")
        mark = bd.log_count()
        crossed(0, into_audio, [AUDIO_START, 1, 2, 1], AUDIO_TRACK, 2, PREGAP_BEFORE, before, "into the audio track")
        case.check(any(e.text.startswith("36 42 12 34 56 78 90 12 30 00 52")
                       for e in bd.log(binding.LOG_DRIVE_REPORT, mark)), "no report carried the Q of ADR 2")
        crossed(1, into_data, [LATER_DATA_START - 1, 0, 0, 2], DATA_TRACK, 3, PREGAP_AFTER, after,
                "into the data track")
        case.info["steps"] = [f"{s.name}: {s.instructions}" for s in host.steps]
    except (RunError, EmulatorError) as error:
        case.check(False, f"failed: {error!r}")
    case.equal(len(bd.log(binding.LOG_VIOLATION)), 0, "violations")
    bd.close()


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
        check_tray(suite, dumps)
        check_tray_requests(suite, dumps)
        check_pregap(suite, dumps)
    else:
        suite.skip("cdb105 a wait that never ends, the providers, the subcode, the tray and the pregap: "
                   "need cdb105.bin")
    if dumps.complete and context.binutils:
        suite.pin("cases", len(suite.cases), 63)
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
