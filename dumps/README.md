# Dumps

`make firmware-test` runs on dumps you supply. Put them in the files below, or give another folder
laid out the same way with `make firmware-test DUMPS=DIR`. Each dump is known by its SHA-1, and the
tests skip what a missing one would need.

## CD block

The SH-1's 64 KB mask ROM.

| File | Revision | SHA-1 |
| --- | --- | --- |
| `cdb104.bin` | CDB104 | `fb0993246ed0ad27ca086854295eccd412196254` |
| `cdb105.bin` | CDB105 | `eb8393058f324e922c11b43709b64fc6ca94ab86` |
| `cdb106.bin` | CDB106 | `b3c20fbe57cd2eb595e9edac86817e5948dccae4` |
| `ygr022.bin` | YGR022 | `f4f6c2aac68c352814d396ae41f81f54ad228e68` |

## MPEG cards

The card's 512 KB ROM, each file named by its MAME set.

| File | Card | SHA-1 |
| --- | --- | --- |
| `mpeg-card/ssmovie.bin` | Sega Saturn Movie Card HSS-0119, one dot on the back | `f7171b2a8f1c464696771941225c32d636b59349` |
| `mpeg-card/vcd.bin` | Sega Video CD Card MK-80310 | `2f5ebd434f6d5b82e93d03e5626007bd5380f652` |
| `mpeg-card/hivcd1d.bin` | Hitachi Video CD card, one dot on the front | `40f284e79f205728b72c7d7c0cf0bae0ade38f76` |
| `mpeg-card/hivcd2d.bin` | Hitachi Video CD card, two dots on the front | `88aa35e48e655540e87ebeae10220413a8feee81` |
| `mpeg-card/jvcvc1.bin` | Victor RG-VC1 Video CD Decoder | `eaac576b8b5e79e7df5e3efd70275b220ff70d78` |
| `mpeg-card/jvcvc2a.bin` | Victor RG-VC2 Twin Operator, version 2.000 | `567f82a31d5f93676120d699869b3ca64995e167` |
| `mpeg-card/jvcvc2.bin` | Victor RG-VC2 Twin Operator, version 2.001 | `286f2c75b1a5bb9d7df23db9f0a381e61b1ac76c` |
| `mpeg-card/jvcvc20.bin` | Victor RG-VC20 | `bb6459da484864226616c5d45b779d4b1644c527` |
