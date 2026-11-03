# Examples

Proofs of concept in SH-1 assembly for a card extension of one's own, written for CDB105 (SHA-1
`eb8393058f324e922c11b43709b64fc6ca94ab86`), and the scenarios that run them on the emulator.

| File | What it shows |
| --- | --- |
| [`lifecycle-extension.s`](lifecycle-extension.s) | A resident extension going through the [card extension lifecycle][docs-lifecycle]: loaded by the card checks and the expansion loader, activated by `93`, its own identity on `AD`, and jobs submitted with `AB`, run in task 9 and reported by `AC`. Linked by [`lifecycle-extension.ld`](lifecycle-extension.ld) |
| [`decompress-extension.s`](decompress-extension.s) | The same extension with a job that decompresses data: the host puts a PRS stream into a partition with `64`, and task 9 decodes it one output sector per pass into sectors filed through a filter, which the host gets with `61`. The CD tasks run between and during the passes, and `93` ends the job between passes. Linked by [`decompress-extension.ld`](decompress-extension.ld) |
| [`disc-decompress-extension.s`](disc-decompress-extension.s) | The decompress extension with its input read from a disc: `30` connects the drive to a filter, `10` reads sectors that hold a PRS stream into its partition, and task 9 decodes them there, so only the output reaches the host. Each input sector's data starts by its mode, at `+16` for mode 1 and `+24` otherwise, as `61` reads it. A mode 2 form 2 sector fails the job. `AD` answers protocol version 3. Linked by [`disc-decompress-extension.ld`](disc-decompress-extension.ld) |
| [`module-extension.s`](module-extension.s) | The disc decompress extension with its decoder taken out and a loader in its place. The host puts a module's package into a partition with `64`, or reads it from a disc, and an `AB` load checks its header, sizes, imports and CRC, then installs it in a fixed slot after the extension, `0x0907c000..0x0907ffff`. The load gives a handle, and `AB` with the handle submits the module's jobs, which task 9 runs a call per pass. The module calls the extension's services to find, take and file sectors. A package that fails leaves the installed module callable. `AD` answers protocol version 4. Linked by [`module-extension.ld`](module-extension.ld) |
| [`decompress-module.s`](decompress-module.s) | The disc decompress extension's decoder as a module of `module-extension.s`. Linked into the slot by [`decompress-module.ld`](decompress-module.ld) |
| [`transform-module.s`](transform-module.s) | A second module: a fixed-point matrix transform of vertices, one sector per pass, using `muls.w`. Loading it replaces the decompress module without reloading the extension. Linked into the slot by [`transform-module.ld`](transform-module.ld) |

| Scenario | What it runs |
| --- | --- |
| [`lifecycle.py`](lifecycle.py) | The lifecycle extension through load, activation, a job, a stop, activation again, a reset and a reload |
| [`prs.py`](prs.py) | Not a scenario: compresses a file into the PRS stream the decompress scenarios read, `decompress-input.prs` |
| [`decompress.py`](decompress.py) | A job of the decompress extension on that stream, the output in `decompress-output.bin` |
| [`disc-decompress.py`](disc-decompress.py) | The stream on a disc in the library's disc drive: disc authentication, then a job of the disc decompress extension on the sectors it reads |
| [`module.py`](module.py) | Both modules uploaded to the module extension in turn, a job of each |

## Running one

From the repository's top folder, with the dump in `dumps/` and binutils for the assembly
(`CDBLOCK_BINUTILS` names their prefix, or `sh-elf-` or `sh2eb-elf-` on PATH, or the ones the
[`Dockerfile`](../README.md#building) builds in `build/binutils/`):

```
python -m cdblock --firmware dumps/cdb105.bin --extension examples/lifecycle-extension.s examples/lifecycle.py
python examples/prs.py original.bin
python -m cdblock --firmware dumps/cdb105.bin --extension examples/decompress-extension.s examples/decompress.py
```

The harness assembles the extension, encodes it into a card ROM and boots the firmware with the
card. A scenario then runs with `host`, the `cdblock.host.Host` that booted it, `board`,
`extension` and `profile` defined ([the Python harness](../README.md#the-python-harness)). Each
scenario's header says what it checks and the files it reads and writes.

## Status

These examples have run only in emulation. Each extension runs on this emulator through its
scenario on CDB105, the two modules through `module.py`, and each extension's card ROM loads and
turns on with CDB104, CDB106 and YGR022 too. `module-extension.s` and its two modules have also run
from SH-2 code in an emulated Saturn, with the same images on CDB104, CDB105, CDB106 and YGR022.
None has run on a console.

## Turning one into card firmware

1. Assemble for big-endian SH-1 and link at `0x0907b000`, where the expansion loader decodes.
   The other examples build the same way as `lifecycle-extension.s`, from the repository's top
   folder too:

   ```
   sh-elf-as --big --isa=sh -o lifecycle-extension.o examples/lifecycle-extension.s
   sh-elf-ld --no-relax -T examples/lifecycle-extension.ld -o lifecycle-extension.elf lifecycle-extension.o
   sh-elf-objcopy -O binary lifecycle-extension.elf lifecycle-extension.bin
   ```

2. Build the card ROM with [`payload.py`](../scripts/payload/payload.py): the encoded length as a
   big-endian longword at offset 0, then the payload the firmware's decoder accepts, encoded as the
   [decoder guide][docs-encoding] describes, at most `0x5000` bytes.

   ```
   python scripts/payload/payload.py card lifecycle-extension.bin lifecycle-card.bin
   ```

3. Put it on card hardware the firmware accepts: present at reset, passing the
   [card checks][docs-card-checks], serving the ROM at `0x0e000000`, and taking the register
   accesses of [activation][docs-activation], `AE` and `AF`.
4. From the SH-2, send `E0` subcode 1, then `93` with CR2 bit 0, then the extension's commands.

A module of `module-extension.s` builds the same way with its own linker script, which links it at
the slot. Its package is the binary with the last longword set to the CRC-32 of the bytes before
it, as zlib computes it. The SH-2 puts the package with `64` and loads it with `AB` operation 1.

Both decompress extensions decode PRS as fuzziqer's compressor writes it, and `prs.py` writes it
too. Their header comments give the format.

[docs-activation]: https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/extensions.md#card-activation
[docs-card-checks]: https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/extensions.md#card-checks
[docs-encoding]: https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/cdblock/decoder.md#encoding
[docs-lifecycle]: https://github.com/LysergicWarp/Sega-Saturn-CDblock-Reverse-Engineering-Documentation/blob/main/docs/sh1-execution/lifecycle.md
