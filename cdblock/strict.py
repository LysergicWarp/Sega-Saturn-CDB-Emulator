"""The strict checks, for a dump of a known revision.

Any other dump runs lenient: reads of registers the emulator does not model are logged as
violations and the run goes on. Strict, such a read ends the run, and so does an instruction outside the declared
code: the firmware's, from its profile or else the whole ROM, and the extension's.
"""
import pathlib

HERE = pathlib.Path(__file__).resolve().parent


def firmware_code(profile):
    """The firmware's code ranges: its profile's code file, or the whole ROM."""
    if profile is None:
        return [(0, 0x10000)]
    lines = (HERE / profile.CODE).read_text().splitlines()
    return [tuple(int(value, 16) for value in line.split()) for line in lines if line and not line.startswith("#")]


def apply(board, profile, extension):
    """Sets the board strict with the code ranges of the firmware and the extension. Returns a line
    for the transcript."""
    board.set_lenient(False)
    ranges = firmware_code(profile) + (list(extension.code_ranges) if extension else [])
    for start, end in ranges:
        board.add_code_range(start, end)
    firmware = f"{profile.NAME}'s code blocks" if profile else "the whole ROM"
    return f"strict: code in {firmware}{' and the extension' if extension else ''}, unmodeled reads fail"
