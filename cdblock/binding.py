"""ctypes binding of the Sega Saturn CD block emulator's C API (include/cdblock.h).

The library comes from CDBLOCK_LIB when it is set, otherwise from this repository's build/.
"""
import ctypes
import os
import pathlib
import sys

EXIT_ERROR, EXIT_STOP, EXIT_BUDGET, EXIT_SLEEP, EXIT_WRITE = 0, 1, 2, 3, 4
TEST_LOW, PC0_HIGH, PROBE_MIRROR, DEVICE_TIMING, CLOCK = 0x1, 0x2, 0x4, 0x8, 0x10
CARD_PERIOD, FIFO_WORDS = 64, 6
DRIVE_SILENT, DRIVE_FED, DRIVE_DISC = 0, 1, 2
SECTOR_PRESENT, SECTOR_MISSING, SECTOR_UNREADABLE, SECTOR_UNREACHABLE = 0, 1, 2, 3
DISC_TRAY_OPEN, DISC_PRESENT = 1, 2
CARD_VIDEO, CARD_AUDIO = 0, 1
(LOG_HOST_FLAGS, LOG_PORT_WRITE, LOG_LINK_SENT, LOG_LINK_MISSED, LOG_CARD_VIDEO, LOG_CARD_AUDIO,
 LOG_CARD_WRITE, LOG_INTERRUPT, LOG_EXCEPTION, LOG_VIOLATION, LOG_READ, LOG_WRITE,
 LOG_DRIVE_COMMAND, LOG_DRIVE_REPORT, LOG_DRIVE_SECTOR, LOG_SERIAL_SENT, LOG_SERIAL_LOST,
 LOG_DRIVE_AUDIO) = range(1, 19)

u8, u16, u32, u64 = ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint32, ctypes.c_uint64
size_t, c_int, c_long, c_char_p = ctypes.c_size_t, ctypes.c_int, ctypes.c_long, ctypes.c_char_p
P = ctypes.POINTER


class Options(ctypes.Structure):
    _fields_ = [("size", u32), ("flags", u32), ("card_rom", P(u8)), ("card_rom_size", size_t),
                ("cycles_per_instruction", u32)]


class Cpu(ctypes.Structure):
    _fields_ = [("r", u32 * 16), ("pc", u32), ("pr", u32), ("sr", u32), ("gbr", u32), ("vbr", u32),
                ("mach", u32), ("macl", u32), ("delay", u32)]


class Strobe(ctypes.Structure):
    _fields_ = [("comsync", u8), ("byte", u8)]


class DriveConfig(ctypes.Structure):
    _fields_ = [("size", u32), ("sector_cycles", u32), ("strobe_cycles", u32), ("seek_cycles", u32),
                ("scan_frames", u32), ("landing_frames", u32)]


class Track(ctypes.Structure):
    _fields_ = [("start", u32), ("control", u8)]


class RingAnswer(ctypes.Structure):
    _fields_ = [("word", u16), ("bytes", u8 * 4)]


# int provider(void *user, uint32_t frame_address, cdblock_ring_answer *answer)
RingProvider = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, u32, ctypes.POINTER(RingAnswer))
# int provider(void *user, uint32_t frame_address, uint8_t *raw)
SectorProvider = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, u32, ctypes.POINTER(u8))
# int provider(void *user, uint32_t frame_address, uint8_t symbols[96])
SubcodeProvider = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, u32, ctypes.POINTER(u8))
# int sink(void *user, uint32_t frame_address, const uint8_t *raw, int speed)
AudioSink = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, u32, ctypes.POINTER(u8), ctypes.c_int)


class LogEntry(ctypes.Structure):
    _fields_ = [("kind", u32), ("pc", u32), ("address", u32), ("value", u32), ("size", u32), ("instruction", u64),
                ("text", c_char_p)]


Board = ctypes.c_void_p

# Each function: name, result type, argument types.
FUNCTIONS = [
    ("cdblock_create", Board, [P(u8), size_t, P(Options)]),
    ("cdblock_destroy", None, [Board]),
    ("cdblock_clone", Board, [Board]),
    ("cdblock_last_error", c_char_p, [Board]),
    ("cdblock_reset", c_int, [Board]),
    ("cdblock_run", c_int, [Board, u64, P(u32)]),
    ("cdblock_run_from_stop", c_int, [Board, u64, P(u32)]),
    ("cdblock_step", c_int, [Board, P(u32)]),
    ("cdblock_add_stop", c_int, [Board, u32]),
    ("cdblock_remove_stop", c_int, [Board, u32]),
    ("cdblock_clear_stops", None, [Board]),
    ("cdblock_add_write_stop", c_int, [Board, u32, u32]),
    ("cdblock_clear_write_stops", None, [Board]),
    ("cdblock_instructions", u64, [Board]),
    ("cdblock_get_cpu", None, [Board, P(Cpu)]),
    ("cdblock_set_cpu", c_int, [Board, P(Cpu)]),
    ("cdblock_peek", c_int, [Board, u32, P(u8), size_t]),
    ("cdblock_poke", c_int, [Board, u32, P(u8), size_t]),
    ("cdblock_bus_read", c_int, [Board, u32, ctypes.c_uint, P(u32)]),
    ("cdblock_bus_write", c_int, [Board, u32, ctypes.c_uint, u32]),
    ("cdblock_set_lenient", None, [Board, c_int]),
    ("cdblock_add_code_range", c_int, [Board, u32, u32]),
    ("cdblock_add_write_area", c_int, [Board, u32, u32]),
    ("cdblock_clear_checks", None, [Board]),
    ("cdblock_add_watch", c_int, [Board, u32, u32]),
    ("cdblock_clear_watches", None, [Board]),
    ("cdblock_host_command", c_int, [Board, P(u16)]),
    ("cdblock_host_response", c_int, [Board, P(u16)]),
    ("cdblock_host_response_word", c_int, [Board, ctypes.c_uint, P(u16)]),
    ("cdblock_host_flags", u16, [Board]),
    ("cdblock_host_clear_flags", c_int, [Board, u16]),
    ("cdblock_host_port_push", c_int, [Board, P(u16), size_t]),
    ("cdblock_host_transfer", c_long, [Board, size_t, P(u16), P(u16)]),
    ("cdblock_host_fifo", size_t, [Board, P(u16)]),
    ("cdblock_decoder_flags", c_int, [Board, u16]),
    ("cdblock_decoder_load", c_int, [Board, P(u8), size_t, u16]),
    ("cdblock_decoder_transfer", c_long, [Board]),
    ("cdblock_decoder_subcode", c_int, [Board, u16]),
    ("cdblock_drive_select", c_int, [Board, c_int]),
    ("cdblock_drive_feed_link_frame", c_int, [Board, P(u8)]),
    ("cdblock_drive_feed_strobes", c_int, [Board, P(Strobe), size_t]),
    ("cdblock_drive_feed_sector", c_int, [Board, P(u8), size_t, u16]),
    ("cdblock_drive_turn", c_int, [Board]),
    ("cdblock_link_step", c_int, [Board]),
    ("cdblock_link_sent", c_int, [Board, P(u8), size_t]),
    ("cdblock_drive_config_defaults", None, [P(DriveConfig)]),
    ("cdblock_drive_select_disc", c_int, [Board, P(DriveConfig)]),
    ("cdblock_drive_disc_sector", c_int, [Board, u32, P(u8), size_t]),
    ("cdblock_drive_disc_tracks", c_int, [Board, P(Track), size_t, u32]),
    ("cdblock_drive_disc_sector_state", c_int, [Board, u32, u32, c_int, u16]),
    ("cdblock_drive_disc_ring_provider", c_int, [Board, RingProvider, ctypes.c_void_p]),
    ("cdblock_drive_disc_sector_provider", c_int, [Board, SectorProvider, ctypes.c_void_p]),
    ("cdblock_drive_disc_subcode", c_int, [Board, u32, P(u8)]),
    ("cdblock_drive_disc_subcode_provider", c_int, [Board, SubcodeProvider, ctypes.c_void_p]),
    ("cdblock_drive_disc_tray", c_int, [Board, c_int]),
    ("cdblock_drive_disc_present", c_int, [Board, c_int]),
    ("cdblock_drive_disc_state", c_int, [Board]),
    ("cdblock_set_audio_sink", c_int, [Board, AudioSink, ctypes.c_void_p]),
    ("cdblock_card_request", c_int, [Board, c_int, ctypes.c_uint]),
    ("cdblock_card_interrupt", c_int, [Board, c_int]),
    ("cdblock_card_push_status", c_int, [Board, u16]),
    ("cdblock_card_set_picture", c_int, [Board, u16]),
    ("cdblock_card_set_indexed", c_int, [Board, u16, u16]),
    ("cdblock_card_memory", c_int, [Board, u32, P(u16), size_t, c_int]),
    ("cdblock_timer_counter", c_int, [Board, ctypes.c_uint, u16]),
    ("cdblock_timer_compare", c_int, [Board, ctypes.c_uint, c_int]),
    ("cdblock_timer_overflow", c_int, [Board, ctypes.c_uint]),
    ("cdblock_test_pad", c_int, [Board, c_int]),
    ("cdblock_serial_send", c_int, [Board, P(u8), size_t]),
    ("cdblock_serial_pending", size_t, [Board]),
    ("cdblock_clock_cycles", u64, [Board]),
    ("cdblock_clock_advance", c_int, [Board, u64]),
    ("cdblock_clock_next", u64, [Board]),
    ("cdblock_coverage", size_t, [Board, u32, u32, P(u32), size_t]),
    ("cdblock_clear_coverage", None, [Board]),
    ("cdblock_log_count", size_t, [Board]),
    ("cdblock_log_get", c_int, [Board, size_t, P(LogEntry)]),
    ("cdblock_clear_log", None, [Board]),
    ("cdblock_log_kinds", None, [Board, u32]),
    ("cdblock_trace_open", c_int, [Board, c_char_p]),
    ("cdblock_trace_close", None, [Board]),
]


def library_path():
    explicit = os.environ.get("CDBLOCK_LIB")
    if explicit:
        return pathlib.Path(explicit)
    name = {"win32": "cdblock.dll", "darwin": "libcdblock.dylib"}.get(sys.platform, "libcdblock.so")
    return pathlib.Path(__file__).resolve().parents[1] / "build" / name


def load(path=None):
    """Loads the library and declares every function's types."""
    lib = ctypes.CDLL(str(path or library_path()))
    for name, result, arguments in FUNCTIONS:
        function = getattr(lib, name)
        function.restype = result
        function.argtypes = arguments
    return lib
