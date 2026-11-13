"""python -m cdblock: runs a CD-block firmware dump, with card code, on the emulator.

usage:
  python -m cdblock --firmware DUMP [--extension CODE | --card ROM] [--trace FILE] [SCENARIO]

DUMP is a 64 KB CD-block ROM dump, any revision. CODE is extension code as .s, assembled with the
binutils CDBLOCK_BINUTILS names, the first of sh-elf- and sh2eb-elf- on PATH or the Dockerfile's in
build/binutils/, as .elf, or as a plain .bin, and payload.py encodes it into a card ROM. ROM is a card ROM already encoded, a known
card's included, loaded unchanged.

A dump of one of the four known revisions, CDB104, CDB105, CDB106 and YGR022, named by its SHA-1,
runs with the strict checks and its profile. Any other dump runs lenient.

Without a scenario the run boots the firmware and, with card code, loads it through E0 subcode 1,
turns it on with 93 and asks 01 for the MPEG version it set. A scenario is a Python file run with
the booted board: host, board, extension and profile are defined for it.

The run prints a transcript: each step's command words and answer, the interrupt flags the SH-1
set, the words and sectors moved and the instructions it took. It exits with 1 when a step fails.
"""
import argparse
import hashlib
import pathlib
import runpy
import sys

from . import binding, cdb104, cdb105, cdb106, strict, ygr022
from . import extension as extensions
from .board import Board, EmulatorError
from .host import Host, RunError, flag_names, words

# The known revisions, by SHA-1.
PROFILES = {profile.SHA1: profile for profile in (cdb104, cdb105, cdb106, ygr022)}


def transcript(host, out):
    print(f"{'step':18} {'sent':19}  {'answer':19}  {'flags set':30} {'words':>5} {'sectors':>7} "
          f"{'instructions':>12}", file=out)
    for step in host.steps:
        sent = words(step.sent) if step.sent else "-"
        print(f"{step.name:18} {sent:19}  {words(step.answer):19}  {flag_names(step.flags):30} {step.words:5} "
              f"{step.sectors:7} {step.instructions:12,}", file=out)


def default_run(host, extension):
    if extension is None:
        info = host.hardware_info()
        return f"firmware ready, CD block version {info['version']}, no card"
    status = host.load_card()
    info = host.hardware_info()
    if not info["mpeg"]:
        raise RunError(f"the card was not loaded: E1 card status {status[0]:04x} {status[1]:04x}, MPEG version 0")
    host.activate_card()
    info = host.hardware_info()
    return f"card code loaded and on, MPEG version {info['mpeg']}"


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m cdblock", description=__doc__.split("\n")[0])
    parser.add_argument("--firmware", required=True, type=pathlib.Path, help="the CD-block ROM dump")
    code = parser.add_mutually_exclusive_group()
    code.add_argument("--extension", type=pathlib.Path, help="extension code: .s, .elf or .bin")
    code.add_argument("--card", type=pathlib.Path, help="a card ROM already encoded")
    parser.add_argument("--trace", type=pathlib.Path, help="write every instruction to this file")
    parser.add_argument("scenario", nargs="?", type=pathlib.Path)
    args = parser.parse_args(argv)
    out = sys.stdout

    try:
        rom = args.firmware.read_bytes()
        if len(rom) != 0x10000:
            raise extensions.ExtensionError(f"{args.firmware.name} is {len(rom)} bytes, a CD-block ROM is 65536")
        extension = (extensions.from_code(args.extension) if args.extension
                     else extensions.from_card(args.card) if args.card else None)
    except (OSError, extensions.ExtensionError) as error:
        print(f"cdblock: {error}", file=sys.stderr)
        return 2
    try:
        lib = binding.load()
    except OSError as error:
        print(f"cdblock: the emulator library {binding.library_path()} did not load: build it with make, or "
              f"name one with CDBLOCK_LIB ({error})", file=sys.stderr)
        return 2

    sha1 = hashlib.sha1(rom).hexdigest()
    profile = PROFILES.get(sha1)
    name = f"{profile.NAME}, its profile" if profile else "not a known revision"
    print(f"firmware  {args.firmware.name}, SHA-1 {sha1}, {name}", file=out)
    if extension:
        print(f"card      {extension.name}, {len(extension.code):#x} bytes of code, version byte {extension.code[0]}", file=out)

    board = Board(rom, card_rom=extension.card if extension else None, flags=binding.DEVICE_TIMING | binding.CLOCK,
                  lib=lib)
    if profile:
        print(f"checks    {strict.apply(board, profile, extension)}", file=out)
    else:
        board.set_lenient(True)
        print("checks    lenient: unmodeled reads logged as violations", file=out)
    if args.trace:
        board.trace_open(args.trace)
    host = Host(board, idle=getattr(profile, "IDLE", None))
    status = 0
    try:
        host.boot()
        if args.scenario:
            runpy.run_path(str(args.scenario), init_globals={
                "host": host, "board": board, "extension": extension, "profile": profile})
            result = f"{args.scenario.name} done"
        else:
            result = default_run(host, extension)
    except (RunError, EmulatorError, AssertionError) as error:
        result = f"failed: {error}"
        status = 1
    finally:
        board.trace_close()
    print(file=out)
    transcript(host, out)
    violations = board.log(binding.LOG_VIOLATION)
    print(f"\n{result}. {board.instructions:,} instructions, {len(violations)} violations", file=out)
    for entry in violations[:20]:
        print(f"  violation at {entry.pc:08x}: {entry.text}", file=out)
    board.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
