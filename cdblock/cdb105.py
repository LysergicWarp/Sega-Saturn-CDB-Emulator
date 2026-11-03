"""CDB105: what the harness knows of this revision beyond the hardware.

The harness runs any CD-block dump without a profile: the board's device timing answers the
firmware, and the host waits on what the SH-1 writes. A profile, chosen by the dump's SHA-1, adds
the checks that need the firmware's layout. Every address here is CDB105's.
"""
NAME = "CDB105"
SHA1 = "eb8393058f324e922c11b43709b64fc6ca94ab86"
# Where strict runs allow instructions: the 243 CODE_BLOCKs of the firmware documentation's Ghidra
# annotation of CDB105, ghidra/cdb105.xml.
CODE = "cdb105-code.txt"

# Task 14's idle loop: every other task waits for a message.
IDLE = 0x1BE6
