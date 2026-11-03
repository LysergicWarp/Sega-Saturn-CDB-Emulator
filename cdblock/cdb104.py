"""CDB104: what the harness knows of this revision beyond the hardware.

CDB104 has CDB105's layout and differs from it in one branch target, the version number and the
checksum word, so CDB105's code blocks and idle loop are its own.
"""
from . import cdb105

NAME = "CDB104"
SHA1 = "fb0993246ed0ad27ca086854295eccd412196254"
CODE = "cdb104-code.txt"
IDLE = cdb105.IDLE
