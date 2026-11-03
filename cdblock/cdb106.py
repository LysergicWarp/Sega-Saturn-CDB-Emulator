"""CDB106: what the harness knows of this revision beyond the hardware.

CDB106 is CDB105 with the drive strap code at 0x07fc..0x0811, which moves the code after it by
0x12 and then 0x10. Its code blocks are CDB105's, moved to this layout, with the strap code in place
of the two instructions it replaces.
"""
NAME = "CDB106"
SHA1 = "b3c20fbe57cd2eb595e9edac86817e5948dccae4"
CODE = "cdb106-code.txt"
# Task 14's idle loop, CDB105's moved by 0x10.
IDLE = 0x1BF6
