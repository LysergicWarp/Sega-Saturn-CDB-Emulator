"""python -m tests.firmware: the firmware tests, on dumps and the emulator's shared library.

usage:
  python -m tests.firmware [--dumps DIR] [--out DIR] [TEST ...]

TEST is decoder, encoding, scripts, package or authentication, all five by default. --dumps names
the folder of dumps that dumps/README.md lists, the repository's dumps/ by default. The run names
each dump missing, then each test runs the parts its dumps allow. The extension cases assemble the
example extensions with the binutils CDBLOCK_BINUTILS names, the first of sh-elf- and sh2eb-elf- on
PATH or the Dockerfile's in build/binutils/, and are skipped without them. Each test writes its
results to results-TEST.json in the --out folder, build/firmware-tests by default, the decoder one
file for each revision. Exits with 1 when a case fails or a pinned total differs, and with 2 when
--dumps names no folder.
"""
import argparse
import importlib
import json
import pathlib
import sys
import time
from types import SimpleNamespace

from cdblock import extension as extensions

from .common import Dumps

REPOSITORY = pathlib.Path(__file__).resolve().parents[2]
TESTS = ["decoder", "encoding", "scripts", "package", "authentication"]


def binutils():
    try:
        return extensions.binutils_prefix()
    except extensions.ExtensionError:
        return None


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m tests.firmware", description=__doc__.split("\n")[0])
    parser.add_argument("--dumps", type=pathlib.Path, default=REPOSITORY / "dumps")
    parser.add_argument("--out", type=pathlib.Path, default=REPOSITORY / "build" / "firmware-tests")
    parser.add_argument("tests", nargs="*", metavar="TEST", help=", ".join(TESTS))
    args = parser.parse_args(argv)
    unknown = [name for name in args.tests if name not in TESTS]
    if unknown:
        parser.error(f"no test {', '.join(unknown)}: the tests are {', '.join(TESTS)}")
    if not args.dumps.is_dir():
        parser.error(f"--dumps {args.dumps}: no such folder")
    args.tests = args.tests or TESTS
    args.out.mkdir(parents=True, exist_ok=True)
    context = SimpleNamespace(dumps=Dumps(args.dumps), out=args.out, binutils=binutils())
    for line in context.dumps.missing:
        print(f"skipped: {line}", flush=True)
    failed = False
    for name in args.tests:
        module = importlib.import_module(f"{__package__}.{name}")
        started = time.monotonic()
        for suite in module.run(context):
            passed, mismatched = suite.summary()
            (args.out / f"results-{suite.name}.json").write_text(json.dumps(suite.record(), indent=1))
            print(f"{suite.name}: {passed}/{len(suite.cases)} cases, {time.monotonic() - started:.0f} s", flush=True)
            for case in suite.cases:
                for failure in case.failures[:4]:
                    print(f"  {case.name}: {failure}")
            for what, (actual, expected) in mismatched.items():
                print(f"  pin {what}: {actual}, expected {expected}")
            for what in suite.skipped:
                print(f"  skipped: {what}")
            failed |= passed != len(suite.cases) or bool(mismatched)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
