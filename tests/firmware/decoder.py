"""The firmware's payload decoder on the emulator: vectors.py's 147 cases on each CD-block revision,
the decoder entered at its start with every register set.

The decoder's code moves 16 bytes up in CDB106 and YGR022. A run fails on an instruction outside
the decoder's code and on a write outside the output, its 32-byte scratch area and a 20-byte stack
window. After the run the result, the check value, the output, the stack and the registers the
decoder must keep are compared.
"""
from . import vectors
from .common import ROMS, Suite, board

STOP = 0x09070000
SH_FLAGS, SH_I = 0x3F3, 0xF0
# The decoder's code, each range's end its last instruction.
RANGES = [(0x9904, 0x9968), (0x9970, 0x99DA), (0x99FC, 0x9B38), (0x9B3C, 0x9B7C), (0x9B80, 0x9C5C), (0x9C60, 0x9CE2)]


def run_rom(name, image, rows):
    suite = Suite(f"decoder-{name}")
    delta = 16 if name in ("cdb106", "ygr022") else 0
    total = 0
    covered = set()
    for row_name, source, dest, length, seed, ret, a, b, data, expected in rows:
        case = suite.case(row_name)
        bd = board(image)
        bd.poke(0x0F000000, b"\xa5" * 0x1000)
        bd.poke(0x09000000, b"\xa5" * 0x80000)
        case.equal(len(data), length, "input length")
        bd.poke(source & ~3, data)
        cpu = bd.cpu()
        for r in range(16):
            cpu.r[r] = (seed ^ (0x01010101 * r)) & 0xFFFFFFFF
        r14 = cpu.r[14]
        cpu.sr = (seed & SH_FLAGS) | SH_I
        sr = cpu.sr
        cpu.macl, cpu.mach = seed ^ 0x87654321, seed
        cpu.gbr, cpu.vbr = seed ^ 0x12345678, seed ^ 0xFEDCBA98
        cpu.r[1], cpu.r[2], cpu.r[3] = source, dest, length
        cpu.r[15], cpu.pr, cpu.pc, cpu.delay = 0x0F001000, STOP, 0x9904 + delta, 0
        bd.set_cpu(cpu)
        out_start = dest & ~3
        if expected:
            bd.add_write_area(out_start, out_start + len(expected))
        bd.add_write_area(0x0F00078C, 0x0F0007AC)
        bd.add_write_area(0x0F000FEC, 0x0F001000)
        for start, end in RANGES:
            bd.add_code_range(start + delta, end + delta + 2)
        bd.add_stop(STOP)
        exit_reason, pc = bd.run(20_000_000)
        if not case.equal(exit_reason, 1, f"exit ({bd.error()})"):
            continue
        after = bd.cpu()
        case.equal(after.r[0], ret, "return value")
        case.equal(bd.peek32(0x0F00079C), a, "accumulator A")
        case.equal(bd.peek32(0x0F0007A0), b, "accumulator B")
        case.equal(bd.peek(out_start, len(expected)), expected, "output")
        case.check(after.r[15] == 0x0F001000 and after.pr == STOP, "SP/PR restored")
        case.check(after.r[14] == r14 and after.mach == seed and after.gbr == seed ^ 0x12345678
                   and after.vbr == seed ^ 0xFEDCBA98, "preserved registers")
        case.equal(after.sr & ~1, sr & ~1, "non-T SR bits")
        case.info["instructions"] = bd.instructions
        total += bd.instructions
        covered.update(pc - delta for pc in bd.coverage(0x9904 + delta, 0x9CE4 + delta))
        bd.close()
    suite.pin("passed", sum(c.passed for c in suite.cases), 147)
    suite.pin("instructions", total, 2_288_731)
    suite.pin("distinctPcs", len(covered), 474)
    return suite


def run(context):
    rows = vectors.rows()
    suites = []
    for name in ROMS:
        if name in context.dumps.roms:
            suites.append(run_rom(name, context.dumps.rom(name), rows))
        else:
            suites.append(Suite(f"decoder-{name}"))
            suites[-1].skip(f"every case: needs {name}.bin")
    return suites
