"""Runs firmware on the board as a Saturn would.

Commands go through CR1..CR4, and the host waits on what the SH-1 writes: the response registers
and the host's interrupt flags. Data moves through the transfer FIFO. Nothing here names a firmware
address, so any CD-block dump runs the same way on a board with device timing on. Given the
firmware's idle loop, from its profile, each wait skips the clock on to the next event whenever the
SH-1 reaches that loop, as time passes there without anything to do.
"""
from collections import namedtuple

from . import binding


class RunError(Exception):
    pass


# The response registers hold NUL CDBLOCK once the firmware takes commands.
SIGNATURE = (0x0043, 0x4442, 0x4C4F, 0x434B)
CR4 = 0x0A000016
HIRQ = 0x0A00001E
TRANSFER_CONTROL = 0x0A000002
CHANNEL1 = (0x05FFFF50, 0x05FFFF60)  # SAR1..CHCR1, which the firmware rewrites for each sector
# The host's pace, a behavioural one: the SH-1 runs this many instructions for each word the host
# moves. Without a pace a whole sector would move between two instructions, and the next sector's
# end would interrupt the handler that armed it.
WORD_INSTRUCTIONS = 4
# A wait that the clock takes past a minute of a 20 MHz clock has hung.
WAIT_CYCLES = 1_200_000_000

# The host's interrupt flags, Sega's HIRQREQ bits.
FLAGS = ("CMOK", "DRDY", "CSCT", "BFUL", "PEND", "DCHG", "ESEL", "EHST", "ECPY", "EFLS", "SCDQ", "MPED",
         "MPCM", "MPST")
CMOK, DRDY, CSCT, BFUL, PEND, DCHG, ESEL, EHST, ECPY, EFLS, SCDQ, MPED, MPCM, MPST = (1 << n for n in range(14))
REJECT, WAIT = 0xFF, 0x80

# One step of a run: what the host sent and got back, the flags the SH-1 set meanwhile, the words
# moved through the FIFO, the sectors moved (for 61 those asked for) and the instructions it took.
Step = namedtuple("Step", "name sent answer flags words sectors instructions")


def flag_names(bits):
    return " ".join(name for n, name in enumerate(FLAGS) if bits >> n & 1) or "-"


def run_to(board, targets, events=None, budget=500_000_000, from_stop=False):
    """Runs until execution reaches one of targets and returns that PC. events maps PCs to functions
    called with the board when execution stops there, after which the run goes on. The stops this
    adds are removed again."""
    targets = set(targets)
    events = dict(events or {})
    board.add_stop(*targets, *events)
    start = board.instructions
    try:
        while True:
            left = budget - (board.instructions - start)
            if left <= 0:
                raise RunError(f"budget of {budget} instructions spent")
            exit_reason, pc = board.run(left, from_stop)
            if exit_reason != binding.EXIT_STOP:
                raise RunError(f"exit {exit_reason} at 0x{pc:08x}: {board.error()}")
            if pc in events:
                events[pc](board)
            if pc in targets:
                return pc
            from_stop = True
    finally:
        board.remove_stop(*targets, *events)


class Host:
    """The Saturn's side of the interface. Each step is kept in steps. idle is the firmware's idle
    loop, from its profile, or None."""

    def __init__(self, board, budget=400_000_000, idle=None):
        self.board = board
        self.budget = budget
        self.idle = idle
        self.steps = []

    def run_until(self, condition, what, stops=()):
        """Runs until condition holds at a write stop: HIRQ, and the extra ranges in stops. A stop
        address a scenario added only pauses the run. The write stops are removed again."""
        board = self.board
        board.clear_write_stops()
        board.add_write_stop(HIRQ, HIRQ + 2)
        for start, end in stops:
            board.add_write_stop(start, end)
        try:
            self._run(what, lambda reason, pc: reason == binding.EXIT_WRITE and condition())
        finally:
            board.clear_write_stops()

    def idle_until(self, condition, what):
        """Runs until condition holds when the SH-1 reaches its idle loop."""
        if self.idle is None:
            raise RunError(f"{what}: waiting for the idle loop needs it from the firmware's profile")
        self._run(what, lambda reason, pc: reason == binding.EXIT_STOP and pc == self.idle and condition())

    def _run(self, what, done):
        """Runs until done(reason, pc) holds where a run ends. At the idle loop the clock skips to
        the next event, and with none to come the wait fails, as nothing would wake the SH-1. The
        idle loop's stop is removed again, a stop the caller had there with it."""
        board = self.board
        if self.idle is not None:
            board.add_stop(self.idle)
        start, cycles, from_stop = board.instructions, board.cycles, False
        try:
            while True:
                left = self.budget - (board.instructions - start)
                if left <= 0:
                    raise RunError(f"{what}: budget of {self.budget:,} instructions spent")
                if board.cycles - cycles > WAIT_CYCLES:
                    raise RunError(f"{what}: {WAIT_CYCLES:,} cycles passed")
                reason, pc = board.run(left, from_stop)
                if reason == binding.EXIT_ERROR:
                    raise RunError(f"{what}: {board.error()} (PC 0x{pc:08x})")
                if reason == binding.EXIT_SLEEP:
                    raise RunError(f"{what}: the SH-1 sleeps at 0x{pc:08x} with nothing to wake it")
                if done(reason, pc):
                    return
                if reason == binding.EXIT_STOP and pc == self.idle:
                    skip = board.next_event()
                    if not skip:
                        raise RunError(f"{what}: the SH-1 idles with no timer event or drive action to come")
                    board.advance(skip)
                from_stop = reason == binding.EXIT_STOP
        finally:
            if self.idle is not None:
                board.remove_stop(self.idle)

    def wait_for(self, flags, what):
        """Runs until the SH-1 has set one of flags, unless it already has."""
        board = self.board
        if not board.host_flags() & flags:
            self.run_until(lambda: board.host_flags() & flags, what)

    def _step(self, name, sent, answer, mark, start, words=0, sectors=0):
        flags = 0
        for entry in self.board.log(binding.LOG_HOST_FLAGS, mark):
            flags |= entry.value
        step = Step(name, sent, answer, flags, words, sectors, self.board.instructions - start)
        self.steps.append(step)
        return step

    def boot(self):
        """Resets the board and runs until the firmware publishes its signature in CR1..CR4."""
        board = self.board
        board.reset()
        mark = board.log_count()
        answer = []

        def ready():
            nonlocal answer
            answer = board.host_response()
            return tuple(answer) == SIGNATURE

        # Only CR4 stops the run here: the signature's last word.
        self.run_until(ready, "boot", [(CR4, CR4 + 2)])
        self._step("boot", None, answer, mark, 0)
        return board.instructions

    def command(self, cr, wait=0, name=None):
        """Sends a command and returns its answer. With wait, also runs until the SH-1 sets one of
        those flags, and a command that is rejected or told to wait raises RunError."""
        board = self.board
        name = name or f"{cr[0] >> 8:02X}"
        mark, start = board.log_count(), board.instructions
        board.host_clear_flags(0xFFFF & ~(CMOK | wait))
        board.host_command(cr)
        self.wait_for(CMOK, name)
        answer = board.host_response()
        status = answer[0] >> 8
        if wait:
            if status == REJECT or status & WAIT:
                raise RunError(f"{name}: answered {'REJECT' if status == REJECT else 'WAIT'}, {words(answer)}")
            self.wait_for(wait, name)
        self._step(name, list(cr), answer, mark, start)
        return answer

    # The MPEG card.

    def load_card(self):
        """E0 subcode 1: the firmware's card checks, then its load of the card's code. Returns E1's
        two card status words."""
        self.command([0xE000, 0x0001, 0, 0], MPED, "E0 load card")
        answer = self.command([0xE100, 0x0001, 0, 0], name="E1 card result")
        return answer[1], answer[3]

    def activate_card(self):
        """93 with CR2 bit 0: the loaded card turned on."""
        return self.command([0x9300, 0x0001, 0, 0], MPED, "93 card on")

    def hardware_info(self):
        """01: the hardware flag, the CD block version, the MPEG version (byte 0 of a loaded card's
        code), the drive information and the CD block revision."""
        answer = self.command([0x0100, 0, 0, 0], name="01 hardware info")
        return {"flag": answer[1] >> 8, "version": answer[1] & 0xFF, "mpeg": answer[2] & 0xFF,
                "drive": answer[3] >> 8, "revision": answer[3] & 0xFF}

    # Data transfers.

    def _end_transfer(self, name, mark, start, moved, sectors):
        """06 ends the open transfer, whose step counts the words and sectors moved and the
        instructions since the command that opened it answered. Returns the words the firmware
        counts as moved."""
        board = self.board
        cr = [0x0600, 0, 0, 0]
        board.host_clear_flags(0xFFFF & ~(CMOK | EHST))
        board.host_command(cr)
        self.wait_for(CMOK, f"{name}, 06")
        answer = board.host_response()
        self.wait_for(EHST, f"{name}, 06")
        self._step("06 end transfer", cr, answer, mark, start, moved, sectors)
        return (answer[0] & 0xFF) << 16 | answer[1]

    def _pace(self, name, instructions):
        """Runs the SH-1 for the time the host took, unless it sleeps. A stop address a scenario
        added only pauses it."""
        board = self.board
        end, from_stop = board.instructions + instructions, False
        while board.instructions < end:
            reason, pc = board.run(end - board.instructions, from_stop)
            if reason == binding.EXIT_ERROR:
                raise RunError(f"{name}: {board.error()} (PC 0x{pc:08x})")
            if reason != binding.EXIT_STOP:
                return
            from_stop = True

    def _transfer(self, name, move, total):
        """Moves total words through the FIFO, a FIFO's worth at a time at the host's pace. When
        nothing moves, the SH-1 runs to its next write of channel 1, the transfer control or HIRQ,
        as it rewrites channel 1 for each sector."""
        board = self.board
        moved, start = 0, board.instructions
        while moved < total:
            if board.instructions - start > self.budget:
                raise RunError(f"{name}: {moved} of {total} words moved within {self.budget:,} instructions")
            step = move(moved, min(total - moved, binding.FIFO_WORDS))
            moved += step
            if step:
                self._pace(name, step * WORD_INSTRUCTIONS)
            else:
                self.run_until(lambda: True, name, [CHANNEL1, (TRANSFER_CONTROL, TRANSFER_CONTROL + 2)])
        return moved

    def get(self, partition, position, count, total):
        """61: count sectors from position in partition, total words. Its step shows count as asked,
        ffff for the rest of the partition. Returns the words."""
        name = "61 get sectors"
        self.command([0x6100, position, partition << 8, count], DRDY, name)
        mark, start = self.board.log_count(), self.board.instructions
        data = []

        def move(_, n):
            got = self.board.host_transfer(n)
            data.extend(got)
            return len(got)

        moved = self._transfer(name, move, total)
        self._end_transfer(name, mark, start, moved, count)
        return data

    def put(self, filter_number, count, data):
        """64: count sectors through filter_number, data as words. Returns the words moved."""
        name = "64 put sectors"
        board = self.board
        self.command([0x6400, 0, filter_number << 8, count], DRDY | EHST, name)
        mark, start = board.log_count(), board.instructions
        if not board.host_flags() & DRDY:
            # No free slots: the answer opened a transfer that nothing runs, and 06 closes it.
            self.command([0x0600, 0, 0, 0], name=f"{name} 06")
            raise RunError(f"{name}: no free slots for {count} sectors")
        data = list(data)
        moved = self._transfer(name, lambda at, n: board.host_transfer(n, data[at:at + n]), len(data))
        self._end_transfer(name, mark, start, moved, count)
        return moved


def words(values):
    return " ".join(f"{v:04x}" for v in values)
