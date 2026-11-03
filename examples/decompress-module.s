! A module for module-extension.s, in this folder, that decompresses data: the decoder of
! disc-decompress-extension.s, through the resident's services. A job of it decodes a PRS stream in a
! partition into sectors it files through a filter, one output sector per pass. The stream can come
! from 64 or from a disc read. Its package is its image, linked into the module slot by
! decompress-module.ld.
!
! The argument: CR3's high byte the input partition and its low byte the output filter, both up to
! 23, and CR4 the most output sectors, from 1. The result is the output's length in bytes.
!
! - The input is the partition's sectors in order, up to 32, 2048 bytes each from where their mode
!   puts their data. A mode 2 form 2 sector is refused.
! - Each output sector reads as a mode 0 sector, so 61 with get sector length 0 sends its 2048
!   bytes. The last one is padded with zeros.
! - A failed job's result is the reason in the high byte and the bytes filed below it. Reasons: 1 no
!   input sector or more than 32, 2 the input ends before the end code, 3 a copy from before the
!   output's start, 4 more output than the limit, 5 no free slot, 6 a mode 2 form 2 input sector,
!   7 an argument out of range.
! - The input stays in its partition, and the sectors filed stay in the filter's, whether the job is
!   done or failed. The host leaves both partitions alone until MPED, then deletes them.
!
! The PRS stream: control bits, taken lowest first from control bytes, each read from the stream when
! the one before is used up.
! - 1: a literal byte follows.
! - 0, 0, then bits b1 and b2: a copy of b1 * 2 + b2 + 2 bytes from 256 less the next byte back.
! - 0, 1, then a little-endian word w: 0 ends the stream. Otherwise the copy reads from 8192 less
!   w >> 3 back, for (w & 7) + 2 bytes, or the next byte + 1 when w & 7 is 0.

    .equ    input_most, 32                  ! the input sectors a job takes
    .equ    import_count, 5
    .equ    stack_depth, 12                 ! PR at the entry, in decompress_peek and decompress_bit

! The services, by offset in the import table once the load has bound it.
    .equ    slot_at, 0
    .equ    sector_data, 4
    .equ    take, 8
    .equ    file, 12
    .equ    give_back, 16

! The job, in the BSS after the image, by offset from its start.
    .equ    job_result_at, 0                ! the result word's address, from the entry's r6
    .equ    job_limit, 4                    ! the most output sectors
    .equ    job_filed, 8                    ! the output sectors filed
    .equ    job_input_at, 12                ! the next input sector's entry in job_input
    .equ    job_input_end, 16               ! the end of the entries
    .equ    job_in, 20                      ! the input pointer between passes
    .equ    job_in_end, 24                  ! the end of its sector
    .equ    job_control, 28                 ! the control bits not yet used, above a marker bit
    .equ    job_copy, 32                    ! the bytes of a copy still to go
    .equ    job_distance, 36                ! how far back the copy reads
    .equ    job_filter, 40                  ! the output filter
    .equ    job_slot, 44                    ! the pass's slot
    .equ    job_partition, 48               ! the input partition
    .equ    job_count, 52                   ! the input sectors found
    .equ    job_ended, 56                   ! nonzero once the end code is read
    .equ    job_result, 60                  ! the output's length, once ended
    .equ    job_output, 64                  ! output sector k's data address at entry k & 7
    .equ    job_input, 96                   ! each input sector's data address
    .equ    job_size, job_input + 4 * input_most

    .text
    .global module_start
module_start:
! The package header.
    .long   0x444f534d                      ! +0 "DOSM"
    .word   1                               ! +4 ABI version
    .word   import_count                    ! +6
    .long   module_end - module_start       ! +8 the image's bytes
    .long   job_size                        ! +12 BSS
    .long   stack_depth                     ! +16
    .long   module_entry                    ! +20
    .long   module_imports                  ! +24
    .long   0                               ! +28

! The services by number. The load replaces each with its address.
module_imports:
    .long   1                               ! slot_at
    .long   2                               ! sector_data
    .long   3                               ! take
    .long   4                               ! file
    .long   5                               ! give_back

! The entry, once per pass: r4 the pass, r5 the argument, r6 the result word's address. Pass 0 is
! the job's start, every later pass fills one output sector.
module_entry:
    sts.l   pr,@-r15
    mov.l   job_address,r13
    mov.l   r6,@(job_result_at,r13)
    tst     r4,r4
    bt      decompress_start
    bra     decompress_pass
    nop

! The job's start: the argument checked and kept, then the input.
decompress_start:
    mov     r5,r0
    shlr16  r0
    mov     r0,r1
    shlr8   r1
    extu.b  r0,r0
    mov     #24,r2
    cmp/hs  r2,r1
    bt      decompress_argument_refused
    cmp/hs  r2,r0
    bt      decompress_argument_refused
    mov.l   r0,@(job_filter,r13)
    mov.l   r1,@(job_partition,r13)
    extu.w  r5,r0
    tst     r0,r0
    bt      decompress_argument_refused
    mov.l   r0,@(job_limit,r13)
    mov     #0,r0
    mov.l   r0,@(job_filed,r13)
    mov.l   r0,@(job_count,r13)
    mov.l   r0,@(job_copy,r13)
    mov.l   r0,@(job_ended,r13)
    mov     #1,r0
    mov.l   r0,@(job_control,r13)

! The slot at each position of the input partition, one at a time, and its data.
decompress_input:
    mov.l   @(job_partition,r13),r4
    mov.l   @(job_count,r13),r5
    mov.l   imports_address,r1
    mov.l   @(slot_at,r1),r0
    jsr     @r0
    nop
    cmp/pz  r0
    bf      decompress_input_end
    mov     r0,r4
    mov.l   @(job_count,r13),r0
    cmp/eq  #input_most,r0
    bt      decompress_input_refused
    mov.l   imports_address,r1
    mov.l   @(sector_data,r1),r0
    jsr     @r0
    nop
    tst     r0,r0
    bt      decompress_input_form2
    mov     r0,r1
    mov.l   @(job_count,r13),r0
    shll2   r0
    add     #job_input,r0
    mov.l   r1,@(r0,r13)
    mov.l   @(job_count,r13),r0
    add     #1,r0
    bra     decompress_input
    mov.l   r0,@(job_count,r13)

! No input sector, or more than input_most: reason 1.
decompress_input_refused:
    bra     decompress_fail
    mov     #1,r4

! A mode 2 form 2 input sector: reason 6.
decompress_input_form2:
    bra     decompress_fail
    mov     #6,r4

! An argument out of range: reason 7.
decompress_argument_refused:
    mov.l   job_address,r13
    mov     #0,r0
    mov.l   r0,@(job_filed,r13)
    bra     decompress_fail
    mov     #7,r4

! The input found: decoding starts at the first sector's data with the next pass.
decompress_input_end:
    mov.l   @(job_count,r13),r0
    tst     r0,r0
    bt      decompress_input_refused
    shll2   r0
    add     #job_input,r0
    add     r13,r0
    mov.l   r0,@(job_input_end,r13)
    mov     r13,r1
    add     #job_input,r1
    mov.l   @r1+,r8
    mov.l   r1,@(job_input_at,r13)
    mov.l   r8,@(job_in,r13)
    mov     #8,r0
    shll8   r0
    add     r8,r0
    mov.l   r0,@(job_in_end,r13)

! Called again: r0 0.
module_more:
    mov     #0,r0
    lds.l   @r15+,pr
    rts
    nop

! The job's end: r4 the result into the result word, r0 r5, 1 done or 2 failed.
module_end_job:
    mov.l   job_address,r13
    mov.l   @(job_result_at,r13),r1
    mov.l   r4,@r1
    mov     r5,r0
    lds.l   @r15+,pr
    rts
    nop

! A failed job, r4 the reason: the result is the reason in its high byte and the bytes filed below.
decompress_fail:
    mov.l   job_address,r13
    mov.l   @(job_filed,r13),r0
    shll8   r0
    shll2   r0
    shll    r0
    shll16  r4
    shll8   r4
    or      r0,r4
    bra     module_end_job
    mov     #2,r5

    .balign 4
job_address:
    .long   module_end
imports_address:
    .long   module_imports

! A pass: one output sector. With no copy under way the stream can end here, at a sector's end, and
! the job is done without taking a slot.
decompress_pass:
    mov.l   pass_job_address,r13
    mov.l   @(job_in,r13),r8
    mov.l   @(job_in_end,r13),r9
    mov.l   @(job_control,r13),r10
    mov.l   @(job_copy,r13),r0
    tst     r0,r0
    bf      decompress_room
    bsr     decompress_peek
    nop
    cmp/pz  r0
    bt      decompress_peeked
    bra     decompress_fail
    mov     #2,r4

decompress_peeked:
    tst     r0,r0
    bt      decompress_room
    mov.l   @(job_filed,r13),r4
    shll8   r4
    shll2   r4
    shll    r4
    bra     module_end_job
    mov     #1,r5

! More output than the limit allows: reason 4.
decompress_room:
    mov.l   @(job_filed,r13),r0
    mov.l   @(job_limit,r13),r1
    cmp/hs  r1,r0
    bf      decompress_slot
    bra     decompress_fail
    mov     #4,r4

! A free slot that reads as a mode 0 sector, with its data from +24. No free slot: reason 5.
decompress_slot:
    mov.l   pass_imports_address,r1
    mov.l   @(take,r1),r0
    jsr     @r0
    nop
    cmp/eq  #-1,r0
    bf      decompress_slot_taken
    bra     decompress_fail
    mov     #5,r4

decompress_slot_taken:
    mov.l   r0,@(job_slot,r13)
    mov     r0,r4
    mov.l   pass_imports_address,r1
    mov.l   @(sector_data,r1),r0
    jsr     @r0
    nop
! r11 and r12, the sector's data and its end. Its address goes to output entry filed & 7 for the
! copies that read it later.
    mov     r0,r11
    mov.l   @(job_filed,r13),r0
    and     #7,r0
    shll2   r0
    add     #job_output,r0
    mov.l   r11,@(r0,r13)
    mov     #8,r12
    shll8   r12
    add     r11,r12
    mov.l   @(job_copy,r13),r4
    tst     r4,r4
    bt      decompress_decode
    bra     decompress_copy
    mov.l   @(job_distance,r13),r5

! Decodes into the sector until it is full or the stream ends. r8 is the input pointer, r9 the end
! of its sector, r10 the control bits, r11 the output pointer, r12 the end of the sector and r13 the
! job. decompress_bit and decompress_byte change r0 and r1 only.
decompress_decode:
    cmp/eq  r11,r12
    bt      decompress_full
    bsr     decompress_bit
    nop
    cmp/pz  r0
    bf      decompress_truncated
    tst     r0,r0
    bt      decompress_copy_command
! 1: a literal byte.
    bsr     decompress_byte
    nop
    cmp/pz  r0
    bf      decompress_truncated
    mov.b   r0,@r11
    bra     decompress_decode
    add     #1,r11

decompress_copy_command:
    bsr     decompress_bit
    nop
    cmp/pz  r0
    bf      decompress_truncated
    tst     r0,r0
    bf      decompress_long
! 0, 0, then two bits and a byte: r4 the length, r5 the distance.
    bsr     decompress_bit
    nop
    cmp/pz  r0
    bf      decompress_truncated
    mov     r0,r4
    add     r4,r4
    bsr     decompress_bit
    nop
    cmp/pz  r0
    bf      decompress_truncated
    add     r0,r4
    add     #2,r4
    bsr     decompress_byte
    nop
    cmp/pz  r0
    bf      decompress_truncated
    mov     #1,r5
    shll8   r5
    bra     decompress_copy
    sub     r0,r5

! 0, 1, then the word w, and for a length of 0 in it the byte after.
decompress_long:
    bsr     decompress_byte
    nop
    cmp/pz  r0
    bf      decompress_truncated
    mov     r0,r5
    bsr     decompress_byte
    nop
    cmp/pz  r0
    bf      decompress_truncated
    shll8   r0
    or      r0,r5
    tst     r5,r5
    bt      decompress_end
    mov     r5,r0
    and     #7,r0
    mov     r0,r4
    shlr2   r5
    shlr    r5
    mov     #32,r0
    shll8   r0
    sub     r5,r0
    mov     r0,r5
    tst     r4,r4
    bf      decompress_long_short
    bsr     decompress_byte
    nop
    cmp/pz  r0
    bf      decompress_truncated
    mov     r0,r4
    bra     decompress_copy
    add     #1,r4

decompress_long_short:
    add     #2,r4

! A copy of r4 bytes from r5 back, which must not reach before the output's start. Output position
! p is in output sector p >> 11, whose data address is at output entry (p >> 11) & 7: r6 is the
! source pointer, r7 the end of its sector and r3 its entry.
decompress_copy:
    bsr     decompress_position
    nop
    cmp/hs  r5,r0
    bf      decompress_before_start
    sub     r5,r0
    mov     r0,r6
    shlr8   r0
    shlr2   r0
    shlr    r0
    and     #7,r0
    shll2   r0
    mov     r0,r3
    add     #job_output,r0
    mov.l   @(r0,r13),r7
    mov     #8,r0
    shll8   r0
    add     #-1,r0
    and     r0,r6
    add     r7,r6
    add     #1,r0
    add     r0,r7

decompress_copy_byte:
    cmp/eq  r11,r12
    bt      decompress_full_copy
    mov.b   @r6+,r0
    mov.b   r0,@r11
    add     #1,r11
    cmp/eq  r6,r7
    bf      decompress_copy_counted
! The source's next sector.
    mov     r3,r0
    add     #4,r0
    and     #28,r0
    mov     r0,r3
    add     #job_output,r0
    mov.l   @(r0,r13),r6
    mov     #8,r0
    shll8   r0
    mov     r6,r7
    add     r0,r7

decompress_copy_counted:
    add     #-1,r4
    tst     r4,r4
    bf      decompress_copy_byte
    bra     decompress_decode
    nop

! The sector full. The input and a copy under way wait for the next pass.
decompress_full:
    mov     #0,r4

decompress_full_copy:
    mov.l   r4,@(job_copy,r13)
    mov.l   r5,@(job_distance,r13)
    mov.l   r8,@(job_in,r13)
    mov.l   r9,@(job_in_end,r13)
    bra     decompress_file
    mov.l   r10,@(job_control,r13)

! The end code: the output's length is the result, and the rest of the sector is cleared.
decompress_end:
    bsr     decompress_position
    nop
    mov.l   r0,@(job_result,r13)
    mov     #1,r0
    mov.l   r0,@(job_ended,r13)
    mov     #0,r0

decompress_end_clear:
    cmp/eq  r11,r12
    bt      decompress_file
    mov.b   r0,@r11
    bra     decompress_end_clear
    add     #1,r11

! The input ended before the end code, or a copy reached before the output's start: the slot goes
! back to the pool.
decompress_truncated:
    bsr     decompress_slot_back
    nop
    bra     decompress_fail
    mov     #2,r4

decompress_before_start:
    bsr     decompress_slot_back
    nop
    bra     decompress_fail
    mov     #3,r4

! The sector done and filed through the job's filter. A job that is no longer its own has had the
! slot returned, and ends with nothing published.
decompress_file:
    mov.l   @(job_slot,r13),r4
    mov.l   @(job_filter,r13),r5
    mov.l   pass_imports_address,r1
    mov.l   @(file,r1),r0
    jsr     @r0
    nop
    tst     r0,r0
    bt      decompress_dropped
    mov.l   @(job_filed,r13),r0
    add     #1,r0
    mov.l   r0,@(job_filed,r13)
    mov.l   @(job_ended,r13),r0
    tst     r0,r0
    bt      decompress_file_more
    mov.l   @(job_result,r13),r4
    bra     module_end_job
    mov     #1,r5

decompress_file_more:
    bra     module_more
    nop

decompress_dropped:
    mov     #0,r4
    bra     module_end_job
    mov     #2,r5

! Returns the pass's slot to the pool.
decompress_slot_back:
    sts.l   pr,@-r15
    mov.l   @(job_slot,r13),r4
    mov.l   pass_imports_address,r1
    mov.l   @(give_back,r1),r0
    jsr     @r0
    nop
    lds.l   @r15+,pr
    rts
    nop

! r0 = the output written so far: the sectors filed, then the offset in this one. Uses r1.
decompress_position:
    mov.l   @(job_filed,r13),r0
    shll8   r0
    shll2   r0
    shll    r0
    add     r11,r0
    sub     r12,r0
    mov     #8,r1
    shll8   r1
    rts
    add     r1,r0

! Whether the next command is the end code, without taking it: r0 1 when it is, 0 when it is not
! and -1 when the input ends first. Uses r4..r7.
decompress_peek:
    sts.l   pr,@-r15
    mov     r8,r4
    mov     r9,r5
    mov     r10,r6
    mov.l   @(job_input_at,r13),r7
    bsr     decompress_bit
    nop
    cmp/pz  r0
    bf      decompress_peek_return
    tst     r0,r0
    bf      decompress_peek_not
    bsr     decompress_bit
    nop
    cmp/pz  r0
    bf      decompress_peek_return
    tst     r0,r0
    bt      decompress_peek_not
    bsr     decompress_byte
    nop
    cmp/pz  r0
    bf      decompress_peek_return
    tst     r0,r0
    bf      decompress_peek_not
    bsr     decompress_byte
    nop
    cmp/pz  r0
    bf      decompress_peek_return
    tst     r0,r0
    bf      decompress_peek_not
    bra     decompress_peek_return
    mov     #1,r0

decompress_peek_not:
    mov     #0,r0

decompress_peek_return:
    mov     r4,r8
    mov     r5,r9
    mov     r6,r10
    mov.l   r7,@(job_input_at,r13)
    lds.l   @r15+,pr
    rts
    nop

! r0 = the next control bit, or -1 past the input's last sector. r10 holds the control bits not yet
! used above a marker bit, so at 1 the next control byte comes from the stream. Uses r1.
decompress_bit:
    mov     r10,r0
    cmp/eq  #1,r0
    bf      decompress_bit_take
    sts.l   pr,@-r15
    bsr     decompress_byte
    nop
    lds.l   @r15+,pr
    cmp/pz  r0
    bf      decompress_bit_none
    mov     #1,r10
    shll8   r10
    or      r0,r10

decompress_bit_take:
    shlr    r10
    rts
    movt    r0

decompress_bit_none:
    rts
    nop

! r0 = the next input byte, or -1 past the input's last sector. r8 is the input pointer and r9 the
! end of its sector. Uses r1.
decompress_byte:
    cmp/eq  r8,r9
    bt      decompress_byte_sector

decompress_byte_take:
    mov.b   @r8+,r0
    rts
    extu.b  r0,r0

decompress_byte_sector:
    mov.l   @(job_input_at,r13),r1
    mov.l   @(job_input_end,r13),r0
    cmp/eq  r0,r1
    bt      decompress_byte_none
    mov.l   @r1+,r8
    mov.l   r1,@(job_input_at,r13)
    mov     #8,r0
    shll8   r0
    mov     r8,r9
    bra     decompress_byte_take
    add     r0,r9

decompress_byte_none:
    rts
    mov     #-1,r0

    .balign 4
pass_job_address:
    .long   module_end
pass_imports_address:
    .long   module_imports

! The image's last longword, the CRC the package tool writes.
    .long   0
    .global module_end
module_end:
