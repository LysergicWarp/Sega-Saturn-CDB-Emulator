"""The emulated CD block board as a Python object over the binding.

Failures raise EmulatorError with the library's reason. Addresses and values are integers, byte
strings are bytes.
"""
import ctypes
import weakref
from collections import namedtuple

from . import binding as b

LogEntry = namedtuple("LogEntry", "kind pc address value size instruction text")
# Bytes in a raw sector, sync and header included, and symbols in a sector's subcode.
RAW_SECTOR = 2352
SUBCODE = 96


class EmulatorError(Exception):
    pass


class Board:
    def __init__(self, rom, card_rom=None, flags=0, lib=None, cycles_per_instruction=0):
        self.lib = lib or b.load()
        options = b.Options(ctypes.sizeof(b.Options), flags, None, 0, cycles_per_instruction)
        if card_rom is not None:
            self._card = (b.u8 * len(card_rom)).from_buffer_copy(card_rom)
            options.card_rom = ctypes.cast(self._card, ctypes.POINTER(b.u8))
            options.card_rom_size = len(card_rom)
        image = (b.u8 * len(rom)).from_buffer_copy(rom)
        self.handle = self.lib.cdblock_create(image, len(rom), ctypes.byref(options))
        if not self.handle:
            raise EmulatorError(self.lib.cdblock_last_error(None).decode())
        self._no_providers()

    def clone(self):
        """A new board in this one's state: CPU, memory, devices, drive, checks, stops, watches,
        coverage and log. A trace stays with this board. The drive's providers answer the copy too,
        an exception from one raised by the copy."""
        handle = self.lib.cdblock_clone(self.handle)
        if not handle:
            raise EmulatorError(self.lib.cdblock_last_error(None).decode())
        copy = Board.__new__(Board)
        copy.lib, copy.handle = self.lib, handle
        copy._no_providers()
        if self._rings is not None:
            copy.drive_disc_ring_provider(self._rings)
        if self._sectors is not None:
            copy.drive_disc_sector_provider(self._sectors)
        if self._subcodes is not None:
            copy.drive_disc_subcode_provider(self._subcodes)
        return copy

    def close(self):
        if self.handle:
            self.lib.cdblock_destroy(self.handle)
            self.handle = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __del__(self):
        self.close()

    def error(self):
        return self.lib.cdblock_last_error(self.handle).decode()

    def _ok(self, result):
        if result < 0:
            self._raise_kept()
            raise EmulatorError(self.error())
        return result

    def _raise_kept(self):
        """Raises the exception a provider raised, which stopped the run."""
        error, self._provider_error = self._provider_error, None
        if error is not None:
            raise error

    # Running.

    def reset(self):
        self._ok(self.lib.cdblock_reset(self.handle))

    def run(self, budget, from_stop=False):
        """Runs at most budget boundaries. Returns the exit reason and the PC. An exception from
        a provider is raised instead."""
        pc = b.u32()
        run = self.lib.cdblock_run_from_stop if from_stop else self.lib.cdblock_run
        exit_reason = run(self.handle, budget, ctypes.byref(pc))
        if exit_reason == b.EXIT_ERROR:
            self._raise_kept()
        return exit_reason, pc.value

    def step(self):
        pc = b.u32()
        exit_reason = self.lib.cdblock_step(self.handle, ctypes.byref(pc))
        if exit_reason == b.EXIT_ERROR:
            self._raise_kept()
        return exit_reason, pc.value

    def add_stop(self, *pcs):
        for pc in pcs:
            self._ok(self.lib.cdblock_add_stop(self.handle, pc))

    def remove_stop(self, *pcs):
        for pc in pcs:
            self._ok(self.lib.cdblock_remove_stop(self.handle, pc))

    def clear_stops(self):
        self.lib.cdblock_clear_stops(self.handle)

    def add_write_stop(self, start, end):
        """A CPU write overlapping [start, end) ends the run with EXIT_WRITE at the next boundary."""
        self._ok(self.lib.cdblock_add_write_stop(self.handle, start, end))

    def clear_write_stops(self):
        self.lib.cdblock_clear_write_stops(self.handle)

    @property
    def instructions(self):
        return self.lib.cdblock_instructions(self.handle)

    # CPU, memory and registers.

    def cpu(self):
        state = b.Cpu()
        self.lib.cdblock_get_cpu(self.handle, ctypes.byref(state))
        return state

    def set_cpu(self, state):
        self._ok(self.lib.cdblock_set_cpu(self.handle, ctypes.byref(state)))

    def peek(self, address, count):
        data = (b.u8 * count)()
        self._ok(self.lib.cdblock_peek(self.handle, address, data, count))
        return bytes(data)

    def peek32(self, address):
        return int.from_bytes(self.peek(address, 4), "big")

    def peek16(self, address):
        return int.from_bytes(self.peek(address, 2), "big")

    def peek8(self, address):
        return self.peek(address, 1)[0]

    def poke(self, address, data):
        data = bytes(data)
        buffer = (b.u8 * len(data)).from_buffer_copy(data)
        self._ok(self.lib.cdblock_poke(self.handle, address, buffer, len(data)))

    def read(self, address, size):
        value = b.u32()
        self._ok(self.lib.cdblock_bus_read(self.handle, address, size, ctypes.byref(value)))
        return value.value

    def write(self, address, size, value):
        self._ok(self.lib.cdblock_bus_write(self.handle, address, size, value))

    # Checks.

    def set_lenient(self, lenient):
        self.lib.cdblock_set_lenient(self.handle, 1 if lenient else 0)

    def add_code_range(self, start, end):
        self._ok(self.lib.cdblock_add_code_range(self.handle, start, end))

    def add_write_area(self, start, end):
        self._ok(self.lib.cdblock_add_write_area(self.handle, start, end))

    def clear_checks(self):
        self.lib.cdblock_clear_checks(self.handle)

    def add_watch(self, start, end):
        """CPU accesses overlapping [start, end) go to the log as LOG_READ and LOG_WRITE."""
        self._ok(self.lib.cdblock_add_watch(self.handle, start, end))

    def clear_watches(self):
        self.lib.cdblock_clear_watches(self.handle)

    # Host interface.

    def host_command(self, cr):
        words = (b.u16 * 4)(*cr)
        self._ok(self.lib.cdblock_host_command(self.handle, words))

    def host_response(self):
        words = (b.u16 * 4)()
        self._ok(self.lib.cdblock_host_response(self.handle, words))
        return list(words)

    def host_response_word(self, word):
        """One response word, 0 to 3 for CR1..CR4. Reading CR4 marks the response read."""
        value = b.u16()
        self._ok(self.lib.cdblock_host_response_word(self.handle, word, ctypes.byref(value)))
        return value.value

    def host_flags(self):
        return self.lib.cdblock_host_flags(self.handle)

    def host_clear_flags(self, keep):
        self._ok(self.lib.cdblock_host_clear_flags(self.handle, keep))

    def host_port_push(self, words):
        words = list(words)
        self._ok(self.lib.cdblock_host_port_push(self.handle, (b.u16 * len(words))(*words), len(words)))

    def host_transfer(self, count, words_in=None):
        """Words on DMA channel 1: words_in into memory, or the words taken from memory. With device
        timing the host writes or reads the FIFO, as far as it and the channel allow."""
        source = (b.u16 * count)(*words_in) if words_in is not None else None
        out = (b.u16 * count)()
        moved = self._ok(self.lib.cdblock_host_transfer(self.handle, count, source, out))
        return list(out[:moved]) if words_in is None else moved

    def host_fifo(self):
        """The words the FIFO holds and the transfer control as the SH-1 wrote it, without moving
        words."""
        control = b.u16()
        words = self.lib.cdblock_host_fifo(self.handle, ctypes.byref(control))
        return words, control.value

    # Decoder input.

    def decoder_flags(self, bits):
        self._ok(self.lib.cdblock_decoder_flags(self.handle, bits))

    def decoder_load(self, data, status):
        data = bytes(data)
        self._ok(self.lib.cdblock_decoder_load(self.handle, (b.u8 * len(data)).from_buffer_copy(data), len(data), status))

    def decoder_transfer(self):
        return self._ok(self.lib.cdblock_decoder_transfer(self.handle))

    def decoder_subcode(self, symbol):
        self._ok(self.lib.cdblock_decoder_subcode(self.handle, symbol))

    # Drive.

    def drive_select(self, kind):
        self._ok(self.lib.cdblock_drive_select(self.handle, kind))
        self._no_providers()

    def drive_feed_link_frame(self, link_frame):
        """A whole link frame: 13 bytes, COMSYNC low on the first strobe and high after."""
        link_frame = bytes(link_frame)
        if len(link_frame) != 13:
            raise ValueError("a link frame is 13 bytes")
        self._ok(self.lib.cdblock_drive_feed_link_frame(self.handle, (b.u8 * 13).from_buffer_copy(link_frame)))

    def drive_feed_strobes(self, strobes):
        """A link frame as the drive's strobes: (comsync, byte) pairs, COMSYNC's level during the
        strobe (true high) and the byte the drive shifts out in the transfer the SH-1 starts in
        answer."""
        strobes = list(strobes)
        bytes(byte for _, byte in strobes)  # each byte 0..255, as drive_feed_link_frame checks
        array = (b.Strobe * len(strobes))(*(b.Strobe(1 if comsync else 0, byte) for comsync, byte in strobes))
        self._ok(self.lib.cdblock_drive_feed_strobes(self.handle, array, len(strobes)))

    def drive_feed_sector(self, data, status):
        data = bytes(data)
        self._ok(self.lib.cdblock_drive_feed_sector(self.handle, (b.u8 * len(data)).from_buffer_copy(data), len(data), status))

    def drive_turn(self):
        return self._ok(self.lib.cdblock_drive_turn(self.handle))

    def link_step(self):
        return self._ok(self.lib.cdblock_link_step(self.handle))

    def link_sent(self):
        count = self._ok(self.lib.cdblock_link_sent(self.handle, None, 0))
        data = (b.u8 * count)()
        self.lib.cdblock_link_sent(self.handle, data, count)
        return bytes(data)

    def drive_select_disc(self, **settings):
        """The disc drive, which answers link commands by itself on the clock. settings name fields
        of the drive configuration, sector_cycles, strobe_cycles, seek_cycles, scan_frames and
        landing_frames, the others keeping their defaults."""
        config = b.DriveConfig()
        self.lib.cdblock_drive_config_defaults(ctypes.byref(config))
        for name, value in settings.items():
            if name == "size" or not hasattr(config, name):
                raise ValueError(f"no drive setting {name}")
            setattr(config, name, value)
        self._ok(self.lib.cdblock_drive_select_disc(self.handle, ctypes.byref(config)))
        self._no_providers()

    def drive_disc_sector(self, frame_address, raw):
        """A raw sector of 2352 bytes, sync and header included, read at frame_address."""
        raw = bytes(raw)
        data = (b.u8 * len(raw)).from_buffer_copy(raw)
        self._ok(self.lib.cdblock_drive_disc_sector(self.handle, frame_address, data, len(raw)))

    def drive_disc_tracks(self, tracks, lead_out):
        """The disc's tracks as (start frame address, control/ADR byte) pairs, numbered from 1, and
        the lead-out's first frame address."""
        tracks = list(tracks)
        array = (b.Track * len(tracks))(*(b.Track(start, control) for start, control in tracks))
        self._ok(self.lib.cdblock_drive_disc_tracks(self.handle, array, len(tracks), lead_out))

    def drive_disc_sector_state(self, frame_address, count, state, status=0):
        """count frames from frame_address on hold state, SECTOR_PRESENT, SECTOR_MISSING or
        SECTOR_UNREADABLE with status as the decoder status, or SECTOR_UNREACHABLE."""
        self._ok(self.lib.cdblock_drive_disc_sector_state(self.handle, frame_address, count, state, status))

    def drive_disc_ring_provider(self, provider):
        """provider(frame_address) answers each ring seek the drive obeys, given its target, with
        (report word, four report bytes), or None for Yabause's answer. None for Yabause's answers
        throughout. An exception it raises fails the board, and the call that ran the drive raises
        it again. It runs inside that call and must not change the board. The provider goes with the
        drive, as in the library."""
        callback = b.RingProvider()
        if provider is not None:
            def answer(frame_address, out):
                given = provider(frame_address)
                if given is not None:
                    word, report = given
                    report = bytes(report)
                    if not 0 <= word <= 0xFFFF or len(report) != 4:
                        raise ValueError(f"a ring answer is a 16-bit word and four bytes, not {given!r}")
                    out.contents.word = word
                    out.contents.bytes[:] = report
            callback = b.RingProvider(self._provider_callback(answer))
        self._ok(self.lib.cdblock_drive_disc_ring_provider(self.handle, callback, None))
        # The library holds the callback, so it stays referenced here.
        self._rings, self._ring_callback = provider, callback

    def drive_disc_sector_provider(self, provider):
        """provider(frame_address, raw) sees each sector as the drive reads it, past the lead-out
        too. raw is the 2352 bytes it would read there, the sector written there or the sync pattern
        and a mode 1 header. It returns 2352 bytes to read instead, or None to leave them. The bytes
        from the header on reach the decoder input, and missing sectors ask nothing. None for the
        disc's own sectors throughout. An exception, the board and the drive as for
        drive_disc_ring_provider."""
        callback = b.SectorProvider()
        if provider is not None:
            def answer(frame_address, out):
                given = provider(frame_address, ctypes.string_at(out, RAW_SECTOR))
                if given is not None:
                    given = bytes(given)
                    if len(given) != RAW_SECTOR:
                        raise ValueError(f"a raw sector is {RAW_SECTOR} bytes, not {len(given)}")
                    ctypes.memmove(out, given, RAW_SECTOR)
            callback = b.SectorProvider(self._provider_callback(answer))
        self._ok(self.lib.cdblock_drive_disc_sector_provider(self.handle, callback, None))
        # The library holds the callback, so it stays referenced here.
        self._sectors, self._sector_callback = provider, callback

    def drive_disc_subcode(self, frame_address, symbols):
        """The subcode of the sector at frame_address: its 96 symbols in disc order, R to W in the
        low six bits of each and the Q channel in bit 6. The drive's position reports carry a
        sector's Q when its CRC holds, and one built from the tracks otherwise."""
        symbols = bytes(symbols)
        if len(symbols) != SUBCODE:
            raise ValueError(f"a sector's subcode is {SUBCODE} symbols, not {len(symbols)}")
        data = (b.u8 * SUBCODE).from_buffer_copy(symbols)
        self._ok(self.lib.cdblock_drive_disc_subcode(self.handle, frame_address, data))

    def drive_disc_subcode_provider(self, provider):
        """provider(frame_address, symbols) sees each sector's subcode as the drive reports a
        position in it, a paused drive's too, or starts its block, a missing sector's too: once for
        a sector it reports and then reads. symbols is the 96 bytes the disc holds there, zeros for a
        sector without any. It returns 96 bytes to send instead, or None to leave them. None for the
        disc's own subcode throughout. An exception, the board and the drive as for
        drive_disc_ring_provider."""
        callback = b.SubcodeProvider()
        if provider is not None:
            def answer(frame_address, out):
                given = provider(frame_address, ctypes.string_at(out, SUBCODE))
                if given is not None:
                    given = bytes(given)
                    if len(given) != SUBCODE:
                        raise ValueError(f"a sector's subcode is {SUBCODE} symbols, not {len(given)}")
                    ctypes.memmove(out, given, SUBCODE)
            callback = b.SubcodeProvider(self._provider_callback(answer))
        self._ok(self.lib.cdblock_drive_disc_subcode_provider(self.handle, callback, None))
        # The library holds the callback, so it stays referenced here.
        self._subcodes, self._subcode_callback = provider, callback

    def drive_disc_tray(self, opened):
        """The disc drive's tray opens or closes. Open, the drive stops where it is, reports status
        80 and obeys no command. Closed on a disc it starts as at power-on, paused at the first
        frame address, and closed on none it reports status 83."""
        self._ok(self.lib.cdblock_drive_disc_tray(self.handle, 1 if opened else 0))

    def drive_disc_present(self, present):
        """The disc goes into the open tray or comes out of it. Taking it out clears its contents:
        the tracks, sectors, subcode and sector states given. The providers stay with the drive."""
        self._ok(self.lib.cdblock_drive_disc_present(self.handle, 1 if present else 0))

    def _provider_callback(self, answer):
        """answer(frame_address, out) as the library calls a provider. An exception from it fails
        the board."""
        # Weak, so the board and its callback make no cycle. The call running the drive holds the
        # board.
        board = weakref.ref(self)

        def callback(_, frame_address, out):
            try:
                answer(frame_address, out)
                return 0
            except BaseException as error:
                # Kept and raised once the run returns, Ctrl-C included: ctypes would drop it.
                board()._provider_error = error
                return 1
        return callback

    def _no_providers(self):
        self._rings = self._ring_callback = self._sectors = self._sector_callback = None
        self._subcodes = self._subcode_callback = None
        self._provider_error = None

    # MPEG card.

    def card_request(self, line, count=1):
        return self._ok(self.lib.cdblock_card_request(self.handle, line, count))

    def card_interrupt(self, line):
        self._ok(self.lib.cdblock_card_interrupt(self.handle, line))

    def card_push_status(self, word):
        self._ok(self.lib.cdblock_card_push_status(self.handle, word))

    def card_set_picture(self, word):
        self._ok(self.lib.cdblock_card_set_picture(self.handle, word))

    def card_set_indexed(self, index, word):
        self._ok(self.lib.cdblock_card_set_indexed(self.handle, index, word))

    def card_memory(self, word, count):
        words = (b.u16 * count)()
        self._ok(self.lib.cdblock_card_memory(self.handle, word, words, count, 0))
        return list(words)

    def card_memory_write(self, word, words):
        words = list(words)
        self._ok(self.lib.cdblock_card_memory(self.handle, word, (b.u16 * len(words))(*words), len(words), 1))

    # Timers.

    def timer_counter(self, channel, value):
        self._ok(self.lib.cdblock_timer_counter(self.handle, channel, value))

    def timer_compare(self, channel, b_register=False):
        self._ok(self.lib.cdblock_timer_compare(self.handle, channel, 1 if b_register else 0))

    def timer_overflow(self, channel):
        self._ok(self.lib.cdblock_timer_overflow(self.handle, channel))

    # The TEST pad and SCI1's serial line.

    def test_pad(self, grounded):
        """TEST (TP3), PB10's pad: grounded, or released to its pull-up."""
        self._ok(self.lib.cdblock_test_pad(self.handle, 1 if grounded else 0))

    def serial_send(self, data):
        """Bytes the serial line's far end sends on TP3 (RxD1), back to back, with CLOCK."""
        data = bytes(data)
        self._ok(self.lib.cdblock_serial_send(self.handle, (b.u8 * len(data)).from_buffer_copy(data), len(data)))

    def serial_pending(self):
        """The bytes the far end has yet to deliver."""
        return self.lib.cdblock_serial_pending(self.handle)

    # The clock, with CLOCK.

    @property
    def cycles(self):
        return self.lib.cdblock_clock_cycles(self.handle)

    def advance(self, cycles):
        """Cycles pass without instructions, as while the CPU waits."""
        self._ok(self.lib.cdblock_clock_advance(self.handle, cycles))

    def next_event(self):
        """Cycles until the next timer event, SCI0 transfer end or SCI1 event whose interrupt is
        enabled, or the disc drive's next action, 0 for none."""
        return self.lib.cdblock_clock_next(self.handle)

    # Coverage, log and trace.

    def coverage(self, start=0, end=0x10000):
        total = self.lib.cdblock_coverage(self.handle, start, end, None, 0)
        pcs = (b.u32 * total)()
        self.lib.cdblock_coverage(self.handle, start, end, pcs, total)
        return list(pcs)

    def clear_coverage(self):
        self.lib.cdblock_clear_coverage(self.handle)

    def log_count(self):
        return self.lib.cdblock_log_count(self.handle)

    def log(self, kind=None, start=0):
        """The log's entries from index start, of one kind or all."""
        entries = []
        entry = b.LogEntry()
        for index in range(start, self.log_count()):
            self._ok(self.lib.cdblock_log_get(self.handle, index, ctypes.byref(entry)))
            if kind is None or entry.kind == kind:
                entries.append(LogEntry(entry.kind, entry.pc, entry.address, entry.value, entry.size,
                                        entry.instruction, (entry.text or b"").decode()))
        return entries

    def clear_log(self):
        self.lib.cdblock_clear_log(self.handle)

    def log_kinds(self, kinds=None):
        """The kinds the log keeps, as LOG_HOST_FLAGS, None for all of them. The trace still writes
        every kind."""
        self.lib.cdblock_log_kinds(self.handle, 0xFFFFFFFF if kinds is None else sum(1 << kind for kind in set(kinds)))

    def trace_open(self, path):
        self._ok(self.lib.cdblock_trace_open(self.handle, str(path).encode()))

    def trace_close(self):
        self.lib.cdblock_trace_close(self.handle)
