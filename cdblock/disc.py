"""A disc in the drive, for firmware the harness has a profile for.

The drive is the library's disc drive: it answers the SH-1's link commands by itself on the board's
clock, a link frame with a report each sector period, the sectors of a read through the decoder
input and DMA channel 0. The host waits on what the SH-1 writes, skipping the clock on at the
firmware's idle loop to the board's next event, so the idle time it skips costs no instructions.
While the drive reads, each subcode frame it sends is such an event and costs one pass of the idle
loop, 97 a sector besides the sector itself. Nothing here names a firmware address, but the idle
loop needs a profile: on any other dump the disc operations are not supported.

By default the disc has one data track from 00:02:00 with a Saturn header at its first frame
address. Its tracks, their pregaps, the header's frame address, sectors that are missing or
unreadable and each sector's subcode can be set. A pregap is the Q of its sectors, which the drive
reports: without one a track's reports start at index 1. It always holds disc authentication's ring
at every check position, the sectors that scripts/ring/make.py builds from the dump's own tables.
Which positions a check reads follows from the ITU4 count when the firmware recognizes the header,
so the ring is there for any count, as on a pressed disc. The drive answers each authentication
seek as Yabause's emulation does, with values that pass the seek checks, unless the disc has seek
answers of its own: failing, passing or scripted, below.

None of the drive's behaviour comes from a real drive. Its reports follow what the firmware
accepts, its seek answers are Yabause's or the disc's own and its timing is the library's defaults
unless settings change it. How often a real drive sends link frames, how far its reports lag its
sectors and whether a pressed disc holds these sectors at the check positions are unknown.

The tray opens and closes, and the disc changes while it is open: open_tray, change and close_tray
below. What a real drive reports after its tray closes, and what it obeys while the tray is open or
empty, is unconfirmed: here a closed tray reports paused or no disc from its first link frame.
"""
import hashlib

from . import _scripts, binding
from .host import EFLS, ESEL, PEND, REJECT, RunError

# make.py imports ring.py by its top-level name.
_ring = _scripts.load("ring", "ring")
_make = _scripts.load("ring", "make", ring=_ring)

SECTOR, DATA, SUBCODE = 2352, 2048, 96
FIRST = 150             # 00:02:00, the data track's first frame address
LEAD_OUT = 283_800      # 63:04:00, the latest Sega's disc format standard allows
DATA_TRACK = 0x41       # the control/ADR byte of a data track: bit 6 data, ADR 1
PREGAP = 150            # two seconds, a pregap's usual length
# The disc's Saturn header: the identifier, a third-party maker ID without a company code and a
# product number. Every other field is 00, so the header names no extension.
IDENTIFIER, MAKER, PRODUCT = b"SEGA SEGASATURN ", b"SEGA TP".ljust(16), b"T-0".ljust(10)
# The ring's grid: a check reads logo row zone + pass, for zones 0..12 and passes 0..2, in one of
# 70 columns. The special cases of zones 0 and 12, 42 frames for a low column in zone 0 and columns
# modulo 39 or 19 in zone 12, still land on its columns.
RING_ROWS, RING_COLUMNS = 15, 70
# The host status codes in the low bits of CR1's status byte: the drive paused, its tray open and
# its tray closed on no disc.
PAUSE, OPEN, NO_DISC = 1, 6, 7


class NotSupported(RunError):
    pass


def sector(frame_address, mode=1, submode=0, data=b""):
    """A raw sector: sync, header and data. Mode 1 data starts at byte 16, mode 2 data after the
    subheader at byte 24, which holds submode twice. The rest is 00."""
    raw = bytearray(SECTOR)
    raw[1:11] = b"\xff" * 10
    raw[12:16] = _make.bcd_address(frame_address) + bytes([mode])
    start = 16
    if mode == 2:
        raw[16:24] = bytes([0, 0, submode, 0]) * 2
        start = 24
    raw[start:start + len(data)] = data
    return bytes(raw)


def q_symbols(q):
    """A sector's 96 subcode symbols with q, a Q's ten bytes, in the Q channel: bit 6 of each
    symbol, the most significant bit first, followed by the CRC as a disc holds it, inverted. The
    other bits are 0."""
    q = bytes(q)
    if len(q) != 10:
        raise ValueError(f"a Q is ten bytes before its CRC, not {len(q)}")
    crc = 0
    for byte in q:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1 ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    bits = q + (crc ^ 0xFFFF).to_bytes(2, "big")
    return bytes((bits[k // 8] >> (7 - k % 8) & 1) << 6 for k in range(SUBCODE))


class SeekAnswers:
    """Answers to the drive's ring seeks, each a report word and four report bytes.
    answer(n, frame_address) gives the nth seek this provider answers, retries counted, on every
    board it answers: a board's clone and a disc inserted again share the count. The firmware
    retries a seek answered with word 0 or ffff at a frame address further on. asked holds (frame
    address, word, bytes) for each seek answered."""

    def __init__(self):
        self.asked = []

    def __call__(self, frame_address):
        word, report = self.answer(len(self.asked), frame_address)
        self.asked.append((frame_address, word, tuple(report)))
        return word, report

    def answer(self, n, frame_address):
        raise NotImplementedError


class Always(SeekAnswers):
    def __init__(self, word, report):
        super().__init__()
        self.word, self.report = word, tuple(report)

    def answer(self, n, frame_address):
        return self.word, self.report


def failing():
    """Word 0 for every seek: the firmware tries the first seek three times, each 206 frames on,
    then gives up with result 3."""
    return Always(0, (0, 0, 0, 0))


def passing(word=0x1000, report=(0x40, 0x20, 0x40, 0x20)):
    """The same answer for every seek. The defaults pass the seek checks as CDB105 makes them:
    the words of a pair agree, and each second byte is half the first."""
    return Always(word, report)


class Scripted(SeekAnswers):
    """answers, (word, four bytes) each, in the order the drive takes the seeks."""

    def __init__(self, answers):
        super().__init__()
        self.answers = [(word, tuple(report)) for word, report in answers]

    def answer(self, n, frame_address):
        if n >= len(self.answers):
            raise RunError(f"ring seek {n} to frame address {frame_address}: "
                           f"the script has {len(self.answers)} answers")
        return self.answers[n]


class Disc:
    """What the drive reads. rom is the dump, whose tables give the ring sectors. header is the
    frame address of the Saturn header, None for none. tracks are (start frame address, control/ADR
    byte) pairs, numbered from 1, and lead_out the lead-out's first frame address. seeks answers the
    ring seeks, as Board.drive_disc_ring_provider takes it: None for Yabause's answers. logo gives
    the good columns of each logo row, out of 106: an int for every row, or a list of 15 counts, one
    a row. The bad columns spread across the row. CDB105 scores a pass from 85 good columns and
    accepts two scoring passes."""

    def __init__(self, rom, header=FIRST, tracks=((FIRST, DATA_TRACK),), lead_out=LEAD_OUT, seeks=None,
                 logo=_ring.LOGO_COLUMNS):
        known = _ring.REVISIONS.get(hashlib.sha1(rom).hexdigest())
        if known is None:
            raise NotSupported("the ring sectors need the check tables of a known revision")
        tables = _ring.Tables(rom, known[1])
        counts = [logo] * RING_ROWS if isinstance(logo, int) else list(logo)
        if len(counts) != RING_ROWS or not all(0 <= c <= _ring.LOGO_COLUMNS for c in counts):
            raise ValueError(f"logo is a count of 0 to {_ring.LOGO_COLUMNS} good columns, or {RING_ROWS} of them")
        self.header, self.tracks, self.lead_out, self.seeks, self.logo = header, list(tracks), lead_out, seeks, counts
        # (frame address, count, state, decoder status) in order, as the drive takes them.
        self.states = []
        self.sectors = {}
        self.subcodes = {}
        # Each pregap's frames by its track's number.
        self.pregaps = {}
        if header is not None:
            self.sectors[header] = sector(header, data=IDENTIFIER + MAKER + PRODUCT)
        for row in range(RING_ROWS):
            # Past the header the sectors of a row are the same at every position.
            bodies = [raw[16:] for raw in _make.build(tables, 0, row, _make.spread(counts[row]))]
            for grid_column in range(RING_COLUMNS):
                position = _ring.FIRST_POSITION + _ring.ZONE_FRAMES * row + _ring.COLUMN_FRAMES * grid_column
                for s, body in enumerate(bodies):
                    self.sectors[position + s] = _make.header(position + s) + body

    def write(self, frame_address, data, mode=1, submode=0):
        """Data in sectors from frame_address on, 2048 bytes each, the last padded with zeros.
        Returns the sectors written, at least one. The drive takes the disc's sectors at insert, so
        a write after it does not reach the drive."""
        count = max(1, -(-len(data) // DATA))
        for k in range(count):
            self.sectors[frame_address + k] = sector(frame_address + k, mode, submode,
                                                     data[k * DATA:(k + 1) * DATA])
        return count

    def write_subcode(self, frame_address, symbols):
        """The subcode of the sector at frame_address: its 96 symbols in disc order, R to W in the
        low six bits of each and the Q channel in bit 6. The drive sends each symbol whole, and its
        position reports carry the Q when the Q's CRC holds. A sector without any sends zeros and
        reports the Q built from the tracks. A missing or unreadable sector's subcode is sent and
        reported too. Before insert, as write."""
        symbols = bytes(symbols)
        if len(symbols) != SUBCODE:
            raise ValueError(f"a sector's subcode is {SUBCODE} symbols, not {len(symbols)}")
        self.subcodes[frame_address] = symbols

    def pregap(self, track, frames=PREGAP):
        """Track number track, the second or a later one, gets a pregap: the frames before its
        start hold a Q that names the track at index 0, with its control/ADR byte and a relative
        time that counts down to the start. The drive reports that Q there, and the track at index
        1 from its start. The Q takes bit 6 of those sectors' symbols and leaves subcode written
        there its other bits. Before insert, as write."""
        if not 2 <= track <= len(self.tracks):
            raise ValueError(f"a pregap goes before track 2 to {len(self.tracks)}, not {track}")
        if not 0 < frames < self.tracks[track - 1][0] - self.tracks[track - 2][0]:
            raise ValueError(f"a pregap of {frames} frames does not fit in the track before track {track}")
        self.pregaps[track] = frames

    def subcode(self):
        """Each sector's subcode as the drive takes it: the symbols written, and the Q of the
        pregaps in bit 6."""
        merged = dict(self.subcodes)
        for track, frames in self.pregaps.items():
            start, control = self.tracks[track - 1]
            for frame_address in range(start - frames, start):
                q = (bytes([control, _make.bcd(track), 0]) + _make.bcd_address(start - frame_address) + b"\0"
                     + _make.bcd_address(frame_address))
                written = merged.get(frame_address, bytes(SUBCODE))
                merged[frame_address] = bytes(symbol & 0xBF | bit for symbol, bit in zip(written, q_symbols(q)))
        return merged

    def missing(self, frame_address, count=1):
        """count frames from frame_address on hold no sector: the drive passes them with nothing to
        read. A later missing or unreadable range wins where they overlap. Before insert, as
        write."""
        self.states.append((frame_address, count, binding.SECTOR_MISSING, 0))

    def unreadable(self, frame_address, count, status):
        """count frames from frame_address on hold sectors the decoder delivers with status as its
        decoder status. CDB105 reads 0x01, 0x02 and 0x04 as errors for mode 1, form 1 and form 2
        sectors, 0x20 for any. Before insert, as write."""
        self.states.append((frame_address, count, binding.SECTOR_UNREADABLE, status))

    def unreachable(self, frame_address, count):
        """count frames from frame_address on are a place the drive never finds: a seek there
        reports seeking until another command, and a read that reaches one loses its place the same
        way. Before insert, as write."""
        self.states.append((frame_address, count, binding.SECTOR_UNREACHABLE, 0))


class Drive:
    """The disc drive on host's board. Without the host's idle loop each disc operation raises
    NotSupported. settings name fields of the drive configuration, as Board.drive_select_disc
    takes them."""

    def __init__(self, host, **settings):
        self.host, self.board = host, host.board
        self.settings = settings
        self.disc = None

    def _ready(self, what, disc=True):
        if self.host.idle is None:
            raise NotSupported(f"{what}: disc operations need the firmware's idle loop from its profile, "
                               "and this firmware has none")
        if disc and self.disc is None:
            raise RunError(f"{what}: no disc in the drive")

    def _step(self, name, mark, start):
        """The step from log index mark, which fails on a link transfer the SH-1 missed or a
        command the drive did not obey: the firmware would retry either, hiding a link fault."""
        board = self.board
        if board.log(binding.LOG_LINK_MISSED, mark):
            raise RunError(f"{name}: the SH-1 missed a transfer of a link frame")
        refused = [e.text for e in board.log(binding.LOG_DRIVE_COMMAND, mark) if e.text.endswith("not obeyed")]
        if refused:
            raise RunError(f"{name}: the drive did not obey {refused[0]}")
        sectors = len(board.log(binding.LOG_DRIVE_SECTOR, mark))
        return self.host._step(name, None, [], mark, start, sectors=sectors)

    # What the host sees.

    def _load(self, disc):
        """The drive takes the disc's tracks, sectors, sector states, subcode and seek answers."""
        board = self.board
        board.drive_disc_tracks(disc.tracks, disc.lead_out)
        for frame_address, raw in disc.sectors.items():
            board.drive_disc_sector(frame_address, raw)
        for frame_address, count, state, status in disc.states:
            board.drive_disc_sector_state(frame_address, count, state, status)
        for frame_address, symbols in disc.subcode().items():
            board.drive_disc_subcode(frame_address, symbols)
        board.drive_disc_ring_provider(disc.seeks)
        self.disc = disc

    def _status(self, code, name, mark, start):
        """The step that ends once the status in CR1 is code, as a Saturn reads it whenever the
        SH-1 idles."""
        board = self.board
        self.host.idle_until(lambda: board.host_response()[0] >> 8 & 0x0F == code, name)
        return self._step(name, mark, start)

    def insert(self, disc):
        """The disc goes into the drive, which starts paused at frame address 150 whatever the
        tracks. The SH-1 reads the TOC and seeks to the first frame address, and the step ends once
        the status in CR1 is pause, as a Saturn reads it whenever the SH-1 idles."""
        self._ready("insert", False)
        board = self.board
        mark, start = board.log_count(), board.instructions
        board.drive_select_disc(**self.settings)
        self._load(disc)
        self._status(PAUSE, "drive startup", mark, start)

    def open_tray(self):
        """The tray opens, the disc still in it, and the drive stops where it is. The SH-1 drops
        what it knew of the disc and sets DCHG, and the step ends once the status in CR1 is open."""
        self._ready("open tray", False)
        board = self.board
        mark, start = board.log_count(), board.instructions
        board.drive_disc_tray(True)
        return self._status(OPEN, "tray open", mark, start)

    def change(self, disc):
        """With the tray open, disc takes the place of what is in it, None for an empty tray. The
        drive keeps its sector and subcode providers, and its seek answers are the new disc's."""
        self._ready("change", False)
        self.board.drive_disc_present(False)
        self.disc = None
        if disc is not None:
            self._load(disc)
            self.board.drive_disc_present(True)

    def close_tray(self):
        """The tray closes. On a disc the drive starts as insert has it, and the step ends once the
        status in CR1 is pause. On an empty tray it ends once the status is no disc."""
        self._ready("close tray", False)
        board = self.board
        mark, start = board.log_count(), board.instructions
        board.drive_disc_tray(False)
        if self.disc is None:
            return self._status(NO_DISC, "tray closed", mark, start)
        return self._status(PAUSE, "drive startup", mark, start)

    def authenticate(self, partition):
        """E0 subcode 0, the check reading into partition, then E1. The drive reads the header,
        makes the four seeks and reads the three passes, and the step ends when the SH-1 sets EFLS.
        Returns E1's result word, 4 in its low byte for an accepted disc."""
        self._ready("E0 check disc")
        host, board = self.host, self.board
        board.host_clear_flags(0xFFFF & ~EFLS)
        answer = host.command([0xE000, 0, partition << 8, 0], name="E0 check disc")
        if answer[0] >> 8 == REJECT:
            raise RunError("E0 check disc: rejected")
        mark, start = board.log_count(), board.instructions
        host.wait_for(EFLS, "drive check")
        self._step("drive check", mark, start)
        return host.command([0xE100, 0, 0, 0], name="E1 check result")[1]

    def read_sectors(self, filter_number, frame_address, count):
        """30 connects the drive to filter_number and 10 plays count frames from frame_address. The
        step ends when the SH-1 sets PEND. The drive lands landing_frames before the frame address
        of the read the SH-1 sends and hands those sectors over too, so the step's sector count
        includes them."""
        self._ready("10 play")
        host, board = self.host, self.board
        host.command([0x3000, 0, filter_number << 8, 0], ESEL, "30 connect drive")
        board.host_clear_flags(0xFFFF & ~PEND)
        answer = host.command([0x1080 | frame_address >> 16, frame_address & 0xFFFF,
                               0x0080 | count >> 16, count & 0xFFFF], name="10 play")
        if answer[0] >> 8 == REJECT:
            raise RunError("10 play: rejected")
        mark, start = board.log_count(), board.instructions
        host.wait_for(PEND, "drive read")
        self._step("drive read", mark, start)
