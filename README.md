# Sega Saturn CD block emulator

The CD block is the Saturn's CD-ROM subsystem, with its SH-1 between the Saturn's SH-2 and the
optical drive. The SH-2 writes host commands to the interface chip's registers CR1..CR4 and reads
the answers there. The SH-1 sends the drive its commands over a serial link and takes its reports.
Sectors the drive reads pass through the decoder into the interface chip, and DMA channel 0 moves
them into the buffer DRAM. The firmware files them there until DMA channel 1 moves them out through
the chip's FIFO to the SH-2. An optional MPEG card adds its own ROM, registers and memory.

This is a C++ library with a C API that runs the CD block's firmware on an emulation of its SH-1
board: the SH7034 CPU with its on-chip modules, the buffer DRAM, the interface chip, the MPEG card
connector and the SH-1 side of the optical drive link. It contains no firmware and runs any CD-block
ROM dump the caller supplies.

- One board object behind a C API, built with no outside dependencies. A Saturn emulator can use it
  as its CD block, and a test can drive it event by event.
- The SH7034 follows the Renesas hardware and programming manuals, exceptions included: CPU and
  DMA address errors, user breaks, undefined code as illegal instructions, and the boundaries where
  no interrupt is accepted.
- It models all four DMA channels and the five timer channels, timer-requested transfers
  included, and both serial channels: the drive link and the serial line on the board's test pads.
- With an MPEG card ROM, the firmware loads the card's SH-1 code and runs it against the card's
  registers, memory, data requests and interrupts.
- The drive is a separate component on the serial link, as on the board. The included disc drive
  answers the firmware's link commands by itself and can take each sector it reads from the
  caller.
- Stops, write stops, watches, coverage and an instruction trace show what the firmware does.
- Strict checks end a run with an error on a read of a register the library does not model, and on
  code outside the ranges the caller declares.
- Behaviour the manuals leave open is marked unconfirmed where it applies.
- A Python harness ([`cdblock/`](#the-python-harness)) runs a dump as a Saturn would, with example
  card extensions, and [firmware tests](#firmware-tests) of the four known firmware revisions and
  eight MPEG cards run on dumps you supply.

It is not cycle accurate: there is no instruction or bus timing, and the disc drive's timing is a
guess. MPEG decoding, CD audio and disc images are not modeled
([what it does not model](#what-it-does-not-model)).

By default the board has no clock. Devices act only when the caller raises their events, and the
CPU runs until a stop address, a write stop, an instruction budget, or a SLEEP with no interrupt to
accept. That makes every run reproducible and leaves the timing of outside events to the caller.
Two board options act on their own: [device timing](#device-timing) lets the MPEG card and the
transfer FIFO act, counted in instructions, and [the clock](#the-clock) lets the timers count.

## Quick start

With GNU Make, a C++20 compiler and Python 3.9 or later, and a CDB105 dump at `dumps/cdb105.bin`
([dumps/README.md](dumps/README.md)), from this folder:

```
make
python -m cdblock --firmware dumps/cdb105.bin
```

The second command boots the firmware and prints a transcript of the boot and of host command
`01`, Get Hardware Info. [Building](#building) covers Windows and Docker, [using it](#using-it) the
C API, and [the Python harness](#the-python-harness) the rest of `cdblock`.

## Terms

Most come from the firmware, which the
[firmware documentation](https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation)
covers in full.

- Host: the Saturn's side of the interface chip, which sends commands.
- Host command: what the host writes to CR1..CR4, such as `01` (Get Hardware Info), `93`
  (Initialize MPEG) or `E0` (Authenticate Device).
- Drive command: byte 0 of what the SH-1 sends the drive over the link, such as `03` (Read TOC),
  `06` (Play) or `09` (Seek). Drive commands are numbered apart from host commands.
- CDB104, CDB105, CDB106 and YGR022: the four known firmware revisions. YGR022 is one chip that
  combines the interface chip, the SH-1 and CDB106's firmware.
- Task: one job of the firmware's small kernel, with its own stack. Task 6 runs the drive, and task
  14 is the idle loop.
- Extension: SH-1 code from outside the ROM that the firmware loads and runs, from a disc or, as a
  card extension, from the MPEG card's ROM.
- Payload: an extension as stored, encoded, ending in a check value the firmware's decoder tests.
- Expansion loader: the firmware's loader of a card extension. After the card checks pass it
  decodes the payload in the card's ROM into the expansion area at `0x0907b000` and calls its entry
  ([expansion extension loader](https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/extensions.md#expansion-extension-loader)).
- Module:
  - An on-chip module is one of the SH7034's peripherals, such as the DMAC or an SCI, in the
    manual's term.
  - A module of `module-extension.s` is SH-1 code that extension installs and runs
    ([examples](examples/README.md)).
- Disc authentication: the firmware's check that a disc is a genuine Saturn disc, by special seeks
  and sector checks.
- Ring seek: drive command `02`, the seek disc authentication makes. The drive's report answers it
  with values the firmware compares.
- Ring: the frame addresses disc authentication's sector checks read. Published observations place
  a ring visible on the data side of Saturn discs around them. Whether the two are the same is
  unknown.
- Frame, in four senses:
  - A link frame is 13 transfers on the drive link.
  - A frame address is a position on the disc, 75 to the second. The distance between two is
    counted in frames.
  - A serial frame is one character on SCI1's line.
  - A small frame is 24 bytes of a sector, 98 to the sector.
- Status, in three senses:
  - A report's status is byte 0 of the drive's report.
  - Host status is the drive state the firmware gives the host, in Sega's numbering.
  - Decoder status is the word that comes with a sector.
- Turn: a drive's chance to act, for a drive that does not keep its own time. The caller gives it
  with `cdblock_drive_turn`, and the drive may start a link frame and hand a sector to the decoder
  input.

## How it fits together

`Board` is the library. The C API in `src/api.cpp` is a thin layer over it, and each part below
belongs to one board and reaches the others through it.

| Part | Holds | Reaches the board through |
| --- | --- | --- |
| `Board`, `src/board/board.h` | ROM, RAM, buffer DRAM, the address decode, coverage and the parts below | |
| `Sh1`, `src/cpu/` | CPU state, the instruction loop, exception and interrupt entry | `Sh1Port`: memory accesses and hooks at each instruction |
| `Sh7034`, `src/sh7034/` | The on-chip modules and the clock's cycle count | A board pointer, for the DMAC's bus cycles, the link and checks |
| `Interface`, `Card`, `Link`, `SerialLine`, `src/board/` | The interface chip, the MPEG card, the running link frame, the far end of SCI1's line | A board pointer |
| `Drive`, `src/drive/` | The drive, silent, fed by the caller or the disc drive | Turns the caller gives, or `DrivePort` for a drive that keeps its own time |
| `RunControl`, `Logbook`, `src/board/` | Stops and the strict checks' ranges, the log and the trace | Called by the board |

One instruction, from the loop in `Sh1::run`:

1. `Sh1::before_instruction` calls `Board::boundary`, where `RunControl` says whether the run ends
   here. With device timing the card takes its tick there too.
2. A pending address error is taken, then the opcode is fetched. A user break on the fetch,
   undefined code or a branch in a delay slot is taken here too. Each of these enters its handler
   and throws `Sh1::Redirect`, so the loop starts again there.
3. `Board::executing` checks the PC against the declared code, counts the instruction, marks its
   coverage, passes its cycles and writes the trace.
4. PC moves on, to the branch destination in a delay slot, and `execute_one` dispatches to the
   handler. Its accesses reach `Board::load` and `Board::store`, which decode the address to a
   device.
5. A device that changed an interrupt source asked for a check with `Sh1::request_check`. Outside a
   delay slot `Sh1::accept_interrupt` runs now, or one instruction later after LDC, STC, LDS or STS.

Time passes in one place, `Board::advance`:

- Without the clock no cycles pass. Instructions are counted, and the card's ticks follow that
  count.
- With the clock, `Board::executing` advances `cycles_per_instruction` as each instruction starts,
  before its handler runs. `cdblock_clock_advance` advances what the caller gives.
- Most advances only run the timers, in `Sh7034::advance`. When the next device event falls inside
  the step, `Board::advance_events` runs the events in time order, with the timers brought up to
  each: SCI0's transfer end, SCI1's transmitter, the serial line's far end and the drive's `act`.
- A device that moves its next event calls `Board::events_moved`, and `advance` looks again.
- There are no threads: everything runs on the caller's.

`Sh1::run` never returns. It runs inside a `try` in `Board::run`, and a run ends with an exception:

- `Sh1::Stop` from `Board::boundary`, at a stop address, after a write stop, out of budget or in a
  SLEEP with nothing to accept. `Board::run` returns the exit reason.
- `std::runtime_error` from a failed check or the drive. `Board::run` keeps the text and marks the
  board failed until its reset.

`Sh1::Redirect` stays inside `Sh1::run` and ends no run.

To read the code, start from `include/cdblock.h`, then:

1. `src/board/board.h`, for the parts and what each asks of the board.
2. `src/cpu/sh1.h`, `src/cpu/sh1.cpp` and the top of `src/cpu/instructions.cpp`, for the loop.
3. `src/board/board.cpp`: `decode`, `Board::run`, `Board::boundary`, `Board::executing` and
   `Board::advance_events`.
4. `src/sh7034/registers.h` and `src/sh7034/sh7034.h`, for the on-chip modules by name.
5. `src/board/link.h`, `src/drive/drive.h` and `src/drive/disc.h`, for the drive and its wire
   format.
6. `tests/emulator/program.h`, the small assembler the tests are written in.

## What it models

The CPU core's instruction handlers, dispatch and fetch/delay/execute loop derive from MAME's SH
interpreter (tag `mame0289`), in `src/cpu/instructions.cpp`. Around them the library implements
the SH7034's rules from the
[SH7032/SH7034 Hardware Manual](https://www.renesas.com/en/document/mah/sh7032-sh7034-hardware-manual)
(Renesas REJ09B0272, Rev. 7.00) and the SH-1/SH-2 Programming Manual. Section, table and figure
numbers in this README and in the sources are the Hardware Manual's unless they name the
Programming Manual. The CPU:

- Reset from vectors 0 and 1, with SR.I at 15 and VBR at 0.
- Interrupt acceptance:
  - The highest level among pending requests is accepted when it is above SR.I. Ties go in the
    default order of Table 5.3.
  - None is accepted in a delay slot, nor at the boundary right after LDC, STC, LDS or STS
    (Table 4.9).
  - On-chip requests and level-sensed IRQs stay pending while their source holds them. An
    edge-sensed IRQ is held once.
- General and slot illegal instructions, with the pushed PC of sections 4.5.3 and 4.5.4.
- Undefined code: every code outside the SH-1's forms in the Programming Manual's Table A.50.
  - That leaves out the nine SH-2 instructions the SH-1 lacks: BF/S, BT/S, BRAF, BSRF, DMULS.L,
    DMULU.L, DT, MAC.L and MUL.L.
  - It is refused at the fetch and never executes, so `cdblock_instructions`, coverage, the trace
    and the declared code check leave it out.
  - The manual lists no exception for these codes. That each of them traps on the chip is
    unconfirmed.
- Inside a delay slot, MOV.W, MOV.L and MOVA with `@(disp,PC)` take the branch destination + 2 as
  PC. Elsewhere it is their own address + 4 (Programming Manual 6.33 and 6.36).
- CPU address errors (Table 4.5): misaligned accesses, longwords in the 8-bit module space, and
  fetches from odd addresses or the on-chip modules.
  - Each is taken after its instruction, at the first boundary outside a delay slot and ahead of an
    interrupt there. It pushes the next instruction's address.
  - One raised while an address error's own exception stacks is ignored (section 4.8.3).
  - The access goes ahead as an aligned one at the address with its low bits cleared, byte by byte
    in the 8-bit module space. This is unconfirmed: the manual gives the bus cycle only for an
    address error while an exception stacks, which completes with the address unchanged and the data
    undefined (section 4.8.3).
- MAC.W's 42-bit result on the SH-1 when S is clear.
- The address decode of section 8.3: A31..A28 are ignored, and A27 selects an area's 8- or 16-bit
  half. The mask ROM repeats through area 0, the on-chip modules through `0x05000000..0x05ffffff`
  and the on-chip RAM through `0x0f000000..0x0fffffff`.

On-chip modules:

- INTC: IPRA..IPRE and ICR, with IRQ6 and IRQ7 from the interface chip. NMI is tied high on the
  board.
- UBC: BARH, BARL, BAMRH, BAMRL and BBR break on CPU or DMA cycles, fetches or data, reads or
  writes and an operand size. The user break interrupt is level 15.
  - A break on a fetch is taken before that instruction, where an interrupt can be. When the fetch
    is an address error, the break follows the error's handling.
  - A break on data is taken after its instruction.
  - Fetches are checked an instruction at a time. The on-chip memories' paired fetches, the overrun
    fetches past a taken branch and the request's latency are left out, unconfirmed.
- DMAC: all four channels, with their address modes, transfer sizes, TCR, TE with its
  read-before-clear rule, AE and NMIF, DEI, and fixed priority.
  - Requests come from DREQ0 and DREQ1 in dual and single address mode, from ITU IMIA0..IMIA3,
    whose transfer clears IMFA, and from auto-request.
  - On an ITU request, cycle steal moves one unit and burst mode the whole count. A burst that
    writes its own CHCR to another request source ends there, unconfirmed.
  - A word unit at an odd address is a DMA address error (Table 4.5). The unit completes, and SAR,
    DAR and TCR count it. Then AE stops every channel with TE not set, and the CPU takes vector 10,
    which ends SLEEP (sections 9.3.6 and 19.3.2). The unit's cycles at the address with its low bit
    cleared, and TE clear on the last unit, are unconfirmed.
- ITU: all five channels, with input capture on the edges TIOR selects, flags with their
  read-before-clear rule, and interrupts. A capture clears the counter when CCLR selects its
  register, and with buffer operation the register's last value moves to its buffer register.
  - With the clock: counting, compare matches with counter clear, overflows, buffer operation on
    channels 3 and 4, and PWM mode's compare registers.
  - Without it, the caller raises compare matches and overflows.
- SCI0: clocked synchronous transfers on its internal clock, for the drive link. A transfer in
  another mode is a strict check.
- SCI1: asynchronous and clocked synchronous serial frames on its internal clock, on the test pads
  ([the serial line](#the-serial-line)).
- Both SCIs have TDRE, RDRF, ORER and TEND, with RXI, TXI, TEI and ERI. They are identical on the
  chip (section 13.1). Here each models what the board does with it: the firmware runs SCI0 only as
  the clocked drive link, and SCI1 in both modes on the test pads.
- Ports: PB10 (TEST, or RxD1), PB2 (COMSYNC) and PA3 (the CS7 pin, tied high) as inputs, and the
  Port C pins.
- BSC and WDT: registers only, with the password write formats of sections 8.2.11 and 12.2.4.

Board devices, wired as the SH-1 board in the Saturn PAL service manual
([sources](CREDITS.md#documents)):

- Buffer DRAM: 512 KB at `0x09000000`.
- Interface chip, from `0x0a000000`:
  - CR1..CR4, with separate latches for the command and the response.
  - The command flags (`0x0a000004`) and their enables, which raise IRQ6.
  - The sector header, subcode sync and subcode block start flags (`0x0a000006`) and their enables,
    which raise IRQ7.
  - The host interrupt flags (`0x0a00001e`).
  - The FIFO the SH-1 reads at the data port (`0x0a000000`), and host transfers on DMA channel 1.
  - Sector bytes on DMA channel 0 with the decoder status (`0x0a00001c`), and the subcode symbol
    (`0x0a000018`).
- MPEG card:
  - Its ROM through CS6, repeating every 512 KB.
  - The status, picture and indexed registers, and the video and audio data ports.
  - The card memory behind `0x0a100030`. Its status always reads ready, unconfirmed: when a real
    card is busy is not known.
  - The card memory takes the five commands the firmware uses. The CD block's
    [memory check](https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/extensions.md#card-memory)
    writes words with `0xfe00` and reads groups of four with `0xfa04`. The card's
    [jobs](https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/mpeg-card/task-9.md#card-memory)
    read and write groups with `0xfa04` and `0xfe04`, and sectors with `0xfb00` and `0xff00`.
  - Data requests on TIOCA1 and TIOCA0, and interrupts on TIOCB2 (MPEG V IRQ) and TIOCA2 (MPEG A
    IRQ).
- Drive link: link frames as the drive's COMREQ strobes on TIOCB3, each with the COMSYNC level the
  drive sets, and their bytes through SCI0. A whole link frame is 13 transfers, COMSYNC low on the
  first.
- Test pads: TEST on TP3, grounded or released to its pull-up, and SCI1's line, with TxD1 on TP2,
  RxD1 on TP3 and SCK1 on TP1.
- PC0, a drive strap: tied low, or high with `CDBLOCK_PC0_HIGH`, as on CDB106 boards fitted with
  the 20-pin drive. CDB106 reads it at reset and writes `0x1100` rather than 0 to the interface
  chip's `0x0a00001a` when it is high
  ([drive strap](https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/firmware-revisions.md#drive-strap)).
  What that register's bits 8 and 12 do is unknown, so it is plain storage here.

The drive is a separate component behind one interface (`src/drive/drive.h`), as the real drive is
a separate unit on the serial link. Three implementations come with the library:

- A silent drive, with no disc and no link frames.
- A drive the caller feeds with link frames or other sequences of strobes, and sectors. Which
  sequences other than whole link frames a real drive produces is unknown.
- [The disc drive](#the-disc-drive), which answers the link by itself on the clock.

## What it does not model

- No instruction timing and no bus timing. With the clock each instruction takes the same number
  of cycles. Without it, counters change only when the CPU or the caller writes them.
- The SH-1's MACH outside MAC.W with S clear. LDS and LDS.L store all 32 bits where the SH-1 keeps
  10, sign-extended, and a saturating MAC.W does not set bit 0 on an overflow (Programming Manual
  6.28 and 6.30).
- A delayed branch to address 0. Its slot runs and execution falls through.
- Without device timing or the clock nothing happens unless the caller raises it.
- ITU external clocks, phase counting, synchronous operation and complementary or
  reset-synchronized PWM.
- The optical drive's own electronics, disc images, CD audio, MPEG decoding, the A/D converter, the
  timer pulse output and the watchdog's counting.
- SCI0 in asynchronous mode, with an external clock, and receiving without transmitting. A transfer
  ends whole, so TDR's next byte and TEND come at its end, not as bit 7 goes out.
- SCI0 with TE cleared, which on the chip sets TDRE and TEND and holds TDRE at 1 (sections 13.2.6
  and 13.2.7), and with ORER set, which stops its transfers (sections 13.2.7 and 13.5). SCI1 models
  TE cleared.
- SCI1 with an external clock, in the 7-bit and multiprocessor formats, and receiving in clocked
  synchronous mode.
- A break on SCI1's line. With TP3 grounded, PB10 as RxD1 and the receiver on, the chip sees the
  line held low and sets FER (section 13.5). Here the far end's byte is logged as lost and no error
  flag is set.
- SCI and A/D requests to the DMAC, and with them the clearing of TDRE and RDRF by the transfers
  they start. Round-robin channel priority.

Some of the board's behaviour is unknown:

- The interface chip's `0x0a00000c` and `0x0a00001a` are plain storage.
- No register is known at `0x0a00000e`, which the firmware does not use. A read of it is a read of
  an unmodeled register.
- Bit 2 of the transfer control register (`0x0a000002`) reads clear while the FIFO holds words.
  Every other bit reads as written.
- The firmware's startup probe of the buffer, a word every `0x200` bytes, touches the two bytes
  past it at `0x09080000`. How the board decodes them is unknown, so it is a board option: two bytes
  of their own (the default) or a repeat of the buffer's first bytes.

With no MPEG card, reads from its windows and ROM return all ones, as a bus floating high would.
What a board without a card reads there is unconfirmed.

## Using it

```c
#include "cdblock.h"

/* rom is a 64 KB CD-block dump, and card_rom an MPEG card's ROM, or NULL with a size of 0. */
cdblock_options options = {sizeof options, CDBLOCK_DEVICE_TIMING | CDBLOCK_CLOCK,
                           card_rom, card_rom_size, 0};
cdblock_board *board = cdblock_create(rom, 65536, &options);
cdblock_add_stop(board, idle_pc); /* the firmware's idle loop, below */
if (cdblock_run(board, 50000000, NULL) != CDBLOCK_EXIT_STOP) puts(cdblock_last_error(board));
/* booted: CR1..CR4 read NUL CDBLOCK */

uint16_t command[4] = {0x0100, 0x0000, 0x0000, 0x0000}; /* host command 01, Get Hardware Info */
cdblock_host_command(board, command);       /* IRQ6, once the firmware has enabled it */
cdblock_run_from_stop(board, 1000000, NULL); /* steps off the stop, then runs to the next */
uint16_t response[4];
cdblock_host_response(board, response);
cdblock_destroy(board);
```

- The idle loop is `0x1be6` in CDB104 and CDB105, `0x1bf6` in CDB106 and YGR022. The Python
  harness's profiles, `cdblock/cdb105.py` and the others, give it as `IDLE`.
- Real firmware needs `CDBLOCK_DEVICE_TIMING`. Without it a startup check fails: `01` answers with
  bit 7 of the hardware flag set, Sega's hardware error bit, and the firmware finds no card.
- `CDBLOCK_CLOCK` lets the timers count, which the disc drive and the firmware's timeouts need.
- The boot takes about 3.6 million instructions to the idle loop on each known revision. A host
  waiting at the idle loop can skip the clock to the next event with `cdblock_clock_next` and
  `cdblock_clock_advance`, as the Python harness does.
- A C program that links `build/libcdblock.a` links the C++ runtime too: `-lstdc++`, or link with
  `g++`.

`include/cdblock.h` gives each call's arguments, results and refusals. In short:

- A run that starts on a stop returns at once from `cdblock_run`, unless an interrupt is accepted
  there first: the run then goes on from its handler. `cdblock_run_from_stop` steps off the stop.
  Fewer, longer runs cost less than many short ones.
- An event raised at a stop is accepted before the next instruction, unless the stop follows an
  LDC, STC, LDS or STS or sits in a delay slot, where no interrupt is accepted. Then the
  instruction at the stop runs first.
- Device events: the host interface (`cdblock_host_*`), the decoder input (`cdblock_decoder_*`),
  the drive (`cdblock_drive_*`, `cdblock_link_*`), the MPEG card (`cdblock_card_*`), the timers
  (`cdblock_timer_*`), the TEST pad (`cdblock_test_pad`) and SCI1's line (`cdblock_serial_*`).
- `cdblock_peek` and `cdblock_poke` reach memory without side effects. `cdblock_bus_read` and
  `cdblock_bus_write` behave as the CPU's accesses, devices included.
- The log records what devices see that the CPU does not report itself: host interrupt flags set,
  link bytes sent, words reaching the card's ports, the disc drive's commands, reports and sectors,
  bytes SCI1 sent and bytes from its line it did not take, interrupts and exceptions entered, and
  the violations of a lenient run. `cdblock_add_watch` adds the CPU's accesses to a range.
- `cdblock_trace_open` writes a text trace: a row for each instruction as it starts, and one for
  each log entry, interrupt, exception and the error that fails the board.
- `cdblock_log_kinds` chooses the kinds the log keeps. Every word the SH-1 sends the host is an
  entry, so a host that runs a game keeps only the kinds it reads. The trace still writes every
  kind.
- `cdblock_host_response_word` lets a host read the response registers one at a time.
- `cdblock_coverage` lists the PCs that have executed in a range, by canonical address.
- `cdblock_clone` makes a new board in another's state, so a caller can go on from one state more
  than once, mainly for tests that start from a booted board.
- `cdblock_add_write_stop` ends a run after a CPU write to a range, for a caller that waits on what
  the SH-1 writes, as a host waits on CR1..CR4 and its interrupt flags.

## Device timing

`CDBLOCK_DEVICE_TIMING` lets two devices act on their own, counted in instructions, so that a
caller can write a command and wait for the answer with no event of its own.

- The MPEG card pulses its video and audio data requests, TIOCA1 and TIOCA0, every
  `CDBLOCK_CARD_PERIOD` instructions, as a card that always has room does. The firmware's checks of
  the card wait for these edges.
- After each reset the card's video interrupt on TIOCB2 falls once, one to two periods after ITU2
  is set to capture that edge. The firmware's startup check waits for it.
- The transfer FIFO holds `CDBLOCK_FIFO_WORDS` words for either direction. The interface requests
  DMA channel 1 while the transfer is enabled and the FIFO can serve it, with room toward the host
  or words from it.
- The SH-1 reads the FIFO at the data port, whichever side wrote it.
- `cdblock_host_transfer` moves the host's words through the FIFO rather than on channel 1
  directly.

Both are behavioural, since the card's timing and the FIFO's control bits are not documented. Ymir
gives the FIFO's depth. The timing stands still while the CPU sleeps, and the caller's other device
events work as without it.

The transfer control register (`0x0a000002`) sets the transfer up. In the emulator's model bit 2
enables it, bit 0 sets its direction from the host, and bit 1 resets the FIFO. The bits are
unconfirmed on a real chip.

With or without device timing, the FIFO keeps its last `CDBLOCK_FIFO_WORDS` words by position, as
Ymir and Yabause model the chip. This is unconfirmed on a real chip.

- The FIFO reset empties the FIFO and moves both positions back to the start, keeping the words.
- The SH-1's read of the empty FIFO gives the kept word at the read position and moves it on.
- The firmware resets the FIFO before each transfer, which drops the words a transfer the host
  ended early left behind.
- Its startup test of the interface resets the FIFO between sending four words and reading them
  back, so it reads them from storage.

A host that holds an access to the data port until the FIFO can serve it asks `cdblock_host_fifo`,
which moves no words.

## The clock

`CDBLOCK_CLOCK` gives the board a system clock, φ, counted in cycles from reset. Each instruction
takes `cycles_per_instruction` cycles (an option, 1 by default), and `cdblock_clock_advance` adds
cycles without instructions, as time that passes while the CPU waits.

- Each ITU channel started in TSTR counts φ, φ/2, φ/4 or φ/8 as its TCR selects. The prescaler
  is taken to run from reset, so a channel's first count comes at the next multiple of its
  divisor. The manual does not give the prescaler's phase.
- A compare match happens as TCNT leaves the value of GRA or GRB. The counter clears in its place
  when CCLR selects that register.
- TCNT wrapping past `0xffff` sets OVF. A compare clear at `0xffff` sets none, unconfirmed: the
  manual says so only for a clear by input capture.
- A flag set during an instruction is accepted at the boundary after it.
- The clock stands still while the CPU sleeps, until the caller advances it.
- A compare match the caller raises with `cdblock_timer_compare` sets TCNT to the register's value
  and its flag, without counter clear or buffer transfer. With the clock on, the next count then
  matches again.
- SCI1's serial frames take their bit times ([the serial line](#the-serial-line)).
- `cdblock_clock_next` gives the cycles until the next event that can raise an enabled interrupt,
  or the disc drive's next action. A caller that knows the CPU is idle can advance by that much
  instead of running instructions. The library never decides that by itself. Each subcode frame
  of a read is a drive action, 98 a sector period, whether or not flag 5 is enabled.
- A channel started with an external clock, in phase counting, synchronous operation, or
  complementary or reset-synchronized PWM counts nothing and is a strict check.

Real instructions take one to several cycles depending on the instruction and the memory they use,
so timers on this clock drift from the firmware's real timing.

## The serial line

SCI1 reaches the board's test pads: TxD1 on TP2, SCK1 on TP1 and RxD1 on TP3, the pad of PB10 that
is also the TEST input. `cdblock_test_pad` grounds or releases TP3, and `CDBLOCK_TEST_LOW` grounds
it when the board is made. A reset leaves the pad as it is.

With TEST grounded at reset the firmware reports its self-tests on SCI1, then runs a small monitor
that reads and writes memory over the line, instead of starting its kernel
([serial monitor](https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/structure.md#serial-monitor)).
The report goes out clocked synchronous at 50,000 bit/s, and the monitor runs asynchronous at about
9470 bit/s.

Sending:

- Clearing TDRE after it read 1 moves TDR into an idle shift register at once and sets TDRE
  again. The byte is logged with SMR1, BRR1 and SCR1. With TE clear, TDRE stays set.
- On the clock a serial frame takes its bits at the rate SMR1 and BRR1 set, from the formulas of
  section 13.2.8: a start bit, 8 data bits, the parity bit when set and one or two stop bits, or 8
  bits in clocked synchronous mode.
- At 20 MHz, BRR1 `0x41` gives 9470 bit/s asynchronous, 21120 cycles a serial frame. BRR1 99 gives
  50,000 bit/s synchronous, 3200 cycles.
- A byte written meanwhile goes into the shift register as the stop bit goes out, or bit 7 in
  clocked synchronous mode. With TDR empty, TEND sets there instead, and a byte written after that
  goes as the serial frame ends.
- Clearing TE ends the serial frame. Without the clock a serial frame ends as it starts.

Receiving:

- The line's far end sends with `cdblock_serial_send`, which needs the clock. Its bytes go back to
  back, each in SCI1's asynchronous format and at its rate as they stand when the byte starts.
- A byte reaches RDR1 at its stop bit's middle, while TP3 is released, PB10 is RxD1 and the
  receiver is on.
- RDRF still set makes it an overrun, and a set error flag holds reception off until it is cleared.
  A byte RDR1 does not take is logged with the reason.
- `cdblock_serial_pending` gives the bytes the far end has yet to deliver. A reset drops them.

Unconfirmed on a real board:

- How soon an idle shift register takes TDR, which the manual does not give.
- A byte written after the stop bit or bit 7 started waiting for the serial frame's end. The manual
  loads TDR when the transmitter is idle and does not say when that is after its TDRE check.
- TEND's exact moment. The manual's text sets it as the stop bit or bit 7 starts, and its figures
  13.6 and 13.15 draw it inside that bit.
- RDRF at the stop bit's middle. The manual latches each bit at its centre and does not say when
  RDRF follows.
- The receiver's conditions taken at the sample. A real receiver first has to see the start bit, so
  TP3, PBCR1 or RE changed during a serial frame may lose or spoil it.
- A far end at SCI1's rate and format. A real one runs at its own nominal rate and format.
- A change of mode or rate while TE is set taking effect from the next serial frame. The manual has
  TE and RE cleared first, so what an SCI does then is not documented.

## The disc drive

`CDBLOCK_DRIVE_DISC`, or `cdblock_drive_select_disc` with a configuration, selects a drive that
answers the SH-1's link commands by itself on the clock, so a Saturn emulator needs only to run the
board. It needs `CDBLOCK_CLOCK`, and takes no turns or link steps from the caller. Its disc has one
data track from 00:02:00 (frame address 150) to the lead-out at 63:04:00 (283800), the latest Sega's
Disc Format Standard Specifications allow. `cdblock_drive_disc_tracks` gives other tracks and
lead-out: up to 99 tracks, each with its start and control/ADR byte.

Link frames:

- Every sector period, 1/75 s at single speed and half that at double, the drive starts a link
  frame with its report. Its strobes come a fixed number of cycles apart.
- Link frames go on in every state, so the SH-1 can always send a command.
- The drive starts paused at frame address 150 at double speed. Its first link frame comes a sector
  period after it is selected or the board resets.
- SCI0's transfers take their bits at the rate SMR and BRR set, 3200 cycles a byte at the
  firmware's 50,000 bit/s.
- A transfer still running when the drive strobes again, or ends the link frame, is logged as
  missed. The drive's byte changed under it, so the SH-1 receives `ff` and its own byte leaves the
  link frame. A link frame short of a byte carries no command.

Drive commands, each taking effect when its link frame ends:

- `03` reads the TOC, `09` seeks and pauses, `06` plays, reading sectors on from its frame address,
  `02` is the ring seek, `08` pauses, `04` stops, and `0a` and `0b` scan forward and backward.
- `05` (standby) stops as `04` does, and `0c` (open tray) opens the tray, which stays open until
  the caller closes it. What the two do in a real drive is unknown, so both are unconfirmed.
- Byte 10 at 1 or 2 sets single or double speed. Whether a real drive takes the byte as its speed
  is unknown, so this is unconfirmed.
- A command with a bad sum is logged and not obeyed, as is any command while the tray is open and
  any but `0c` while the drive has no disc. An unknown command fails the board.

Seeks and reads:

- Seeks, reads and ring seeks report seeking for a fixed latency first.
- They reach past the lead-out up to frame address 449999 (99:59:74). Disc authentication's ring,
  from 299300 to 321349, lies past the default lead-out.
- A read lands a few frames before its frame address, 4 by default, and reads on into it. It never
  lands below 0 or in an unreachable sector.
- A TOC read, pause, stop, scan or tray opening during a seek leaves the position where the seek
  started, unconfirmed.

Reports:

- The TOC lists each track's start, then points A0 with the first track and disc type 0, A1 with
  the last track and A2 with the lead-out, over and over. A0 carries the first track's control byte
  and A1 and A2 the last track's, unconfirmed.
- A position report names the track holding the position, its control byte, index 1 and the time
  in the track. Past the lead-out the track is `aa`.
- Index 0 counts down only before the first track: there are no pregaps between tracks.
- The ring seek answers as Yabause's drive emulation does: status `b6`, report word `44f1` and bytes
  `09 09 09 09`. A provider set with `cdblock_drive_disc_ring_provider` can answer each seek
  instead, given its target, or fail the board.

Sectors:

- While reading, each sector period hands the sector at the position to the decoder input with
  decoder status 0. The drive raises its header flag and moves it on DMA channel 0, then reports in
  the next link frame.
- A sector the caller did not write with `cdblock_drive_disc_sector` has only a mode 1 header.
  Audio sectors are delivered as data sectors are.
- `cdblock_drive_disc_sector_state` makes sectors missing, unreadable or unreachable. A missing
  sector reads as nothing, and an unreadable one comes with a decoder status the caller gives. At
  an unreachable one a seek never arrives and a read loses its place, both reporting seeking.
- A sector provider set with `cdblock_drive_disc_sector_provider` serves the disc's sectors from
  the caller, past the lead-out too, as a Saturn emulator serves its disc image. It gets the 2352
  bytes the drive would read, the written sector or the sync pattern and a mode 1 header, and may
  change them or fail the board. Missing sectors ask nothing.

Subcode:

- Each sector period the drive reads, it sends the sector's
  [subcode block](https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/subcode.md#frames)
  to the interface chip: 98 frames, evenly spaced over the period, 7350 a second at single speed.
- Each frame sets the symbol word (`0x0a000018`) and raises decoder flag 5. The first frame of a
  block raises flag 6 with it. IRQ7 follows the enables the firmware wrote.
- `cdblock_drive_disc_subcode` gives a sector its 96 symbols in disc order, R to W in the low six
  bits of each byte. A sector without any sends zeros.
- Frames 0 to 95 carry the symbols, each byte whole. Frames 96 and 97, the sync frames, leave the
  last symbol in the word.
- The stream runs on data reads as on audio, and for missing and unreadable sectors too. It stops
  when the drive leaves the read: a pause, seek, stop or scan ends the block being sent.
- A subcode provider set with `cdblock_drive_disc_subcode_provider` serves the symbols from the
  caller as each block starts, under the sector provider's rules. It is asked for missing sectors
  too.
- The P and Q channels are not in the stream: the drive reports them over the link.
- The fed drive sends no subcode. Its caller raises the flags and sets the symbol word.
- Unconfirmed: whether a real drive streams while paused, seeking or scanning, whether the
  interface chip sets flag 5 while its enable is clear, where a block's first frame falls against
  the sector's header flag (here at the same time), what the sync frames carry, and what the symbol
  word holds above the symbol.

The tray and the disc:

- The drive starts with its tray closed and a disc in. `cdblock_drive_disc_tray` opens and closes
  the tray, and drive command `0c` opens it too.
- An open tray stops the drive where it is. It hands over no more sectors or subcode, reports
  status `80` and obeys no command until the caller closes it.
- `cdblock_drive_disc_present` takes the disc out or puts one in, and fails while the tray is
  closed. Taking the disc out clears its contents: the tracks, back to the one data track, and the
  sectors, subcode and sector states given. The providers stay with the drive.
- What the caller gives while the tray is open is the new disc's.
- Closing on a disc starts the drive as it starts when selected, paused at frame address 150 at
  double speed. Closing on an empty tray reports status `83`, and the drive then obeys only `0c`.
- The link frames keep their times through an opening and a closing.
- The tray and the disc stay through the board's reset, an open tray open, and a clone has both.
- The CDB105 firmware answers status `80` by dropping what it knew of the disc, raising DCHG and
  giving the host status 6, OPEN. After the close it reads the TOC again, or gives the host status
  7, NODISC, for status `83`
  ([tray and disc states](https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/drive.md#tray-and-disc-states)).
- A play the host asks for while the tray is open makes the firmware send `03`, which the open
  drive does not obey. The firmware keeps the request and carries it out on the disc the tray
  closes on. Without a disc a play makes the firmware send `0c`, which opens the tray here.
- Unconfirmed: the reports a real drive sends after its tray closes, their order and how long each
  lasts. A drive has to spin up and look for a disc first, and here the first link frame after the
  close already reports paused or no disc. Also what a drive obeys with its tray open or without a
  disc, the bytes after the status in reports `80` and `83` (zeros here), whether a drive with an
  open tray reports it from its first link frame after a reset, and what `0c` does in a drive whose
  lid opens by hand.

The landing:

- A sector reaches DMA channel 0 as it arrives. The CDB105 firmware arms the channel only after the
  first two sectors of a read, and counts up to the frame address it asked for.
- The landing gives it those sectors. With `landing_frames` at 0 the first two sectors pass before
  the channel is armed, so it misses that address and sends the read again, giving the host status
  8, RETRY. It sends the retries from the same frame address, then from earlier ones, and the
  fourth try starts early enough to reach it
  ([ranges in theory](https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/drive.md#ranges-in-theory)).
- The default of 4 frames is the landing of Ymir's drive emulation, a guess for stability and
  speed. Where a real drive lands is unconfirmed.

The reports follow what the firmware accepts. The timing defaults count the SH-1's 20 MHz clock and
are guesses: 4000 cycles between strobes, above the 3389 that a byte's 3200 and the SH-1's answer
189 instructions after the strobe need, 1/15 s to seek, 15 frames a period while scanning, and a
read landing 4 frames early. A real drive's timing and answers
are not confirmed. The log holds each command but 00, Null, and each report and sector. A link frame
the SH-1 did not answer in full carries no command, and the link logs the strobes it missed.

Not modeled: the drive's final byte asking for a repeat, which is always `00`, statuses `81` and
`82`, which this drive never sends, the index and track bytes of a command, the TOC end report, a
second session, pregaps, and the decoder's handling of audio and its status bits beyond the one the
caller gives.

Bit 3 of a report's status, whose meaning is unknown, is always clear. Disc authentication sets its
30-track flag only for a TOC of exactly 30 tracks whose lead-in reports are at least a quarter with
the bit set and a quarter with it clear
([request](https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/disc-authentication.md#request)).
Here the flag never sets, so a `SEGASYSTEM` disc never opens the any-ID gate that the flag adds to
its result
([classification](https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/disc-authentication.md#classification)).

## Checks

Always on: unmapped accesses, writes to the mask ROM or the card ROM, instruction fetches from
anywhere but the mask ROM, the buffer, RAM and a card's ROM, and through `cdblock_bus_read` and
`cdblock_bus_write` misaligned accesses and longwords in the 8-bit module space. The CPU's are
address errors, as is a fetch from the on-chip modules. A real board goes on past an unmapped
access or a write to ROM, but how is not known yet, so the board fails there rather than guess.

Strict unless `cdblock_set_lenient` is set:

- PCs outside the code ranges the caller declares.
- CPU writes to RAM and the buffer outside the areas the caller declares.
- Reads of registers the library does not model.
- With device timing, data port writes while the FIFO is full.
- With the clock, an ITU channel started in a mode the library does not model.
- SCI0 transferring outside clocked synchronous mode on its internal clock.
- SCI1 sending or receiving in a mode the library does not model.
- Access widths the manual disallows, such as a word access to a byte-only ITU register.
- With an MPEG card, a card memory command other than the five the firmware uses.

A check with no ranges declared is off. A strict failure ends the run with `CDBLOCK_EXIT_ERROR`
and the board needs `cdblock_reset`. Lenient logs each as a violation, and an unmodeled read returns
the register's last written value, or 0 for the card's picture and indexed registers when the caller
has set no word there.

A failed board, by a check or a drive error, refuses runs, clock advances, bus accesses and device
events until its reset, as the state the error left is not a real board's. Queries still work, to
see what happened.

## Layout

| Folder | Holds |
| --- | --- |
| `include/`, `src/` | The library: the C API in `cdblock.h`, its implementation by hardware in `cpu/`, `sh7034/`, `board/` and `drive/` |
| `tests/emulator/` | The library's tests, hand-encoded SH-1 programs that need no ROM (`make test`) |
| `cdblock/` | The [Python harness](#the-python-harness): the library's binding and the harness on it, with a profile for each known revision |
| `examples/` | Card extensions in SH-1 assembly and the scenarios that run them ([examples](examples/README.md)) |
| `scripts/` | `payload/payload.py`, the extension payload encoder and decoder, and `ring/`, which replays disc authentication's checks and builds the ring's sectors |
| `tests/firmware/` | The [firmware tests](#firmware-tests), on dumps you supply (`make firmware-test`) |
| `dumps/` | The dumps you supply ([dumps/README.md](dumps/README.md)) |

## Building

GNU Make and a C++20 compiler (GCC 11 or later, Clang 14 or later).

```
make                 # build/libcdblock.a and the shared library
make test            # the tests under AddressSanitizer, UBSan and libstdc++ assertions
make sanitized       # the shared library under the same sanitizers, in build/sanitized/
make firmware-test   # the shared library, then the firmware tests on the dumps in dumps/
make format          # the C and C++ sources through clang-format 23, in place
```

Warnings are errors, with `-Wall -Wextra -Wpedantic`, the conversion and shadowing warnings and
more.

On Windows build with MSYS2 or w64devkit. The DLL links libstdc++ and libgcc statically, so it
loads without them. MinGW has no sanitizers: run `make test SANITIZE=0`.

The `Dockerfile` builds what the harness needs without a local toolchain, on x86-64:
`libcdblock.so` with GCC 14 and `cdblock.dll` with llvm-mingw into `build/`, and GNU binutils 2.47's
`as`, `ld` and `objcopy` for `sh-elf`, which assemble card code, into `build/binutils/linux/` and
`windows/`. The libraries link the C++ runtime statically. The Linux files need glibc 2.38 or
later, the Windows ones the Universal C Runtime of Windows 10 and later. On macOS build with
`make`.

```
docker build -t cdblock-build .
docker run --rm -v "$PWD:/src" --user "$(id -u):$(id -g)" cdblock-build   # Linux
docker run --rm -v "${PWD}:/src" cdblock-build                            # Windows, PowerShell
```

The image has clang-format too, so a `make format` needs no local copy:

```
docker run --rm -v "$PWD:/src" --user "$(id -u):$(id -g)" cdblock-build make format   # Linux
docker run --rm -v "${PWD}:/src" cdblock-build make format                            # Windows, PowerShell
```

The tests run in it the same way, under the sanitizers:

```
docker run --rm -v "$PWD:/src" --user "$(id -u):$(id -g)" cdblock-build make test   # Linux
docker run --rm -v "${PWD}:/src" cdblock-build make test                            # Windows, PowerShell
```

Its objects go to `build/sanitized/` in the mounted folder, where a sanitized build outside the
image would take them for its own. Run `make clean` before that build.

It has clang-tidy as well, with its checks in `.clang-tidy`. On Linux and in PowerShell alike:

```
docker run --rm -v "${PWD}:/src" cdblock-build sh -c 'clang-tidy --quiet src/*.cpp src/*/*.cpp tests/emulator/*.cpp -- -std=c++20 -Iinclude -DCDBLOCK_BUILD -stdlib=libstdc++ --gcc-install-dir=/usr/local/lib/gcc/x86_64-linux-gnu/14.4.0'
```

A CMake project (3.16 or later) can add this directory instead. `add_subdirectory` gives the static
library target `cdblock`, alias `cdblock::cdblock`, with `include/` public. It builds the library
alone, without the Makefile's warning flags.

The tests in `tests/emulator/` run hand-encoded SH-1 programs and need no ROM.

## The Python harness

The `cdblock` package drives the library through ctypes. It needs Python 3.9 or later and nothing
beyond its standard library. It loads the shared library from `build/`, or the file `CDBLOCK_LIB`
names. The commands here run from this folder.

`cdblock.board.Board` is the C API as a Python object. This boots CDB105 to its idle loop and reads
its answer to host command `01`, Get Hardware Info:

```python
import pathlib

from cdblock import binding
from cdblock.board import Board

rom = pathlib.Path("dumps/cdb105.bin").read_bytes()
board = Board(rom, flags=binding.DEVICE_TIMING | binding.CLOCK)
board.add_stop(0x1BE6)                   # CDB105's idle loop
board.run(50_000_000)
board.host_command([0x0100, 0, 0, 0])    # host command 01
board.run(50_000_000, from_stop=True)
print([f"{word:04x}" for word in board.host_response()])
board.close()
```

`python -m cdblock` runs a dump as a Saturn would and prints a transcript: each command and its
answer, the interrupt flags the SH-1 set, the words and sectors moved and the instructions each
step took.

```
python -m cdblock --firmware dumps/cdb105.bin --card dumps/mpeg-card/vcd.bin
```

- Without a scenario it boots the firmware and, with card code, loads it through host command `E0`
  (Authenticate Device) subcode 1, turns it on with `93` (Initialize MPEG) and asks `01` (Get
  Hardware Info) for the MPEG version. A scenario is a Python file run with the booted board
  ([examples](examples/README.md)). It finds `host`, the `cdblock.host.Host` that booted it,
  `board`, `extension`, the card code given or None, and `profile`, the revision's profile module or
  None, already defined.
- `--extension` takes card code as `.s`, assembled with the binutils `CDBLOCK_BINUTILS` names, the
  first of `sh-elf-` and `sh2eb-elf-` on PATH or the Dockerfile's in `build/binutils/`, as `.elf`
  or as `.bin`, and encodes it into a card ROM. `--card` takes a card ROM already encoded.
- `--trace FILE` writes every instruction the SH-1 runs to FILE.
- In the transcript, `words` counts the words a transfer moved through the FIFO, on the `06` step
  that ends it. `sectors` on that step is the transfer's sector count as its command gave it, and
  on a `cdblock.disc` drive step the sectors the drive handed over.
- A dump of CDB104, CDB105, CDB106 or YGR022, known by its SHA-1, runs strict with its profile: its
  code ranges, and the idle loop where the host lets the clock skip ahead while the firmware
  waits. Any other dump runs lenient.
- `cdblock.disc` puts a disc in the disc drive for a known revision, with disc authentication's
  ring built from the dump's own tables. Its drive opens and closes the tray and changes the disc
  while it is open.

[`payload.py`](scripts/payload/payload.py) encodes and decodes extension payloads and card ROMs.
In [`scripts/ring/`](scripts/ring/), `ring.py` replays disc authentication's checks and `make.py`
builds the sectors they read. Both print positions as MM:SS:FF and as an LBA, the frame address
less 150.

## Firmware tests

`make firmware-test` builds the shared library and runs `tests/firmware/` on the dumps in `dumps/`
([dumps/README.md](dumps/README.md)). It runs `python3`: give `PYTHON=python` where the interpreter
has that name, as on Windows.

- `decoder`: the firmware's payload decoder on 147 cases from its equations, on each revision.
- `encoding`: `payload.py` on the same cases, round trips, and each known card's code encoded back
  to the payload in its ROM, its length word and payload byte for byte.
- `scripts`: `ring.py` on its example files but `sectors.txt`, which needs a disc image, and
  `make.py`'s sectors, and `payload.py`'s card ROM and back, through their command lines.
- `package`: `python -m cdblock` on each revision with no card and with each card, sectors put and
  got, the lifecycle and module extensions as source, ELF, plain code and card ROM, the command
  line, the R-W subcode of an audio play through `20` (Get Subcode), and the tray opened, closed on
  another disc and closed on none.
- `authentication`: disc authentication on the disc drive at the edges of the drive timings the
  firmware accepts, and on discs that fail it.

The run first names each dump that is missing or not the known one, then each test runs what the
dumps there allow. The extension cases need the binutils `--extension` uses. Results go to
`build/firmware-tests/`. `DUMPS=DIR` takes the dumps from another folder and
`TESTS="decoder encoding"` runs some of the tests. Without Make, `python -m tests.firmware` runs
them on the library `build/` holds, however it was built, with `--dumps DIR`, `--out DIR` and the
tests' names. `make test` needs neither Python nor dumps.

## Licence

BSD-3-Clause ([LICENSE](LICENSE)). `src/cpu/instructions.cpp` derives from MAME and is
BSD-3-Clause under its authors' copyright ([MAME-NOTICE.txt](MAME-NOTICE.txt)). The manuals and the
other emulators whose models the library follows are credited in [CREDITS.md](CREDITS.md).
