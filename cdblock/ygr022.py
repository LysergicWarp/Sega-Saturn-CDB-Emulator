"""YGR022: what the harness knows of this revision beyond the hardware.

YGR022 is CDB106's firmware with 512 bytes of unused space zero instead of ff, so CDB106's code
blocks and idle loop are its own.
"""
from . import cdb106

NAME = "YGR022"
SHA1 = "f4f6c2aac68c352814d396ae41f81f54ad228e68"
CODE = "ygr022-code.txt"
IDLE = cdb106.IDLE
