# Credits

## Code

- MAME (BSD-3-Clause, https://github.com/mamedev/mame), tag `mame0289`. `src/cpu/instructions.cpp`
  derives from its SH core: the instruction handlers and their dispatch from
  `src/devices/cpu/sh/sh.cpp` (David Haywood), the fetch/delay/execute loop, SR loads, RTE and
  TRAPA from `src/devices/cpu/sh/sh2.cpp` (Juergen Buchmueller, R. Belmont). Notice and terms in
  [MAME-NOTICE.txt](MAME-NOTICE.txt).

## Models

The library follows these emulators' models where no documentation gives the behaviour. No code
is copied from them.

- Ymir (GPL-3.0, https://github.com/ymir-emu/Ymir):
  - The interface chip's transfer FIFO (`ygr.cpp`): six words deep, the transfer control's
    direction and enable bits, and bit 1 resetting the FIFO's positions with the stored words
    kept.
  - Host reads of the response registers, where only reading CR4 marks the response read.
  - The disc drive's default read landing of 4 frames before the read's frame address
    (`cd_drive.cpp`).
- Yabause (GPL-2.0, https://github.com/Yabause/yabause):
  - The transfer FIFO's stored words, read by position as in Ymir.
  - Its drive emulation (`cd_drive.c`, James Laird-Wah and Theo Berkau): the disc drive's default
    answer to a ring seek (status `b6`, word `44f1`, report bytes `09`) and the status bytes of
    its reports (`04`, `12`, `22`, `36`, `46`, `80`, `b6`).
- Mednafen (GPL-2.0, https://mednafen.github.io/): the control bytes of TOC points A0, the first
  track's, and A1 and A2, the last track's.

## Formats

- PRS as fuzziqer's compressor and decompressor write and read it, archived in libyaul's
  `libbcl/prs.c` (https://github.com/yaul-org/libyaul): the stream the example decompress extensions
  decode and `examples/prs.py` writes. No code is copied.

## Documents

- SH7032/SH7034 Hardware Manual, Renesas `REJ09B0272-0700`, Rev. 7.00, 31 January 2006. Section,
  table and figure numbers in this repository are this manual's unless they name another.
  https://www.renesas.com/en/document/mah/sh7032-sh7034-hardware-manual
- SuperH RISC Engine SH-1/SH-2 Programming Manual, Hitachi, 3 September 1996.
- Sega Saturn (PAL) Service Manual 013-1, June 1995, Schematic Diagram-7, "SH-1 C.B" (printed pages
  25-26): the board's wiring, pin and signal names and test pads.
  https://segaretro.org/images/4/4e/Sega_Service_Manual_-_Sega_Saturn_(PAL)_-_013-1_-_June_1995.pdf
- Sega Saturn Disc Format Standard Specifications, Ver. 1.0, Sega `ST-040-R4-051795`, 17 May 1995,
  section 1.2: the latest lead-out start, 63:04:00, the default disc's lead-out.
- The firmware documentation:
  https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation
