! A card extension for the CD-block SH-1, CDB105 (SHA-1 eb8393058f324e922c11b43709b64fc6ca94ab86),
! that decompresses data for the host. The host puts a PRS stream into a partition with 64, AB
! submits it, and task 9 decodes it into sectors it files through a filter, which the host gets with
! 61. The CD tasks outrank task 9 and go on meanwhile. It is lifecycle-extension.s, in this folder,
! with this operation added and its jobs run in passes. README.md in this folder says what it has
! run on and what turning it into card firmware takes.
!
! Commands, once 93 has activated the card:
! - AD, identity: CR1 low byte the protocol version, CR2 the activations since the load, CR3:CR4
!   0x444f5345 ("DOSE"). The known cards answer AD with ffff in CR1 to CR4.
! - AB submits a job: CR1 low byte the operation, CR2 the job ID, CR3:CR4 the argument. An unknown
!   operation, an argument the operation refuses, or a job already submitted or running rejects it
!   and nothing is posted.
! - AC reports the job: CR1 low byte the state, CR2 the job ID, CR3:CR4 the result. States: 0 none
!   since activation, 1 submitted, 2 running, 3 done, 4 failed.
! - The end of each job writes MPED to 0x0a00001e.
! - 90, 91 and 92 stay the ROM's, and 93, AE and AF have handlers of their own. Every other MPEG
!   command is rejected with status ff.
!
! Operations:
! - 0, ping: the result is the argument inverted.
! - 1, decompress: CR3's high byte the input partition and its low byte the output filter, both up
!   to 23, and CR4 the most output sectors, from 1. The result is the output's length in bytes.
!
! Decompress:
! - The input is the partition's sectors in order, up to 32, each 2048 bytes from +24 of its slot,
!   where 64 with put sector length 0 writes them.
! - Each output sector is a free slot that reads as a mode 0 sector, so 61 with get sector length 0
!   sends its 2048 bytes. The last one is padded with zeros.
! - A failed job's result is the reason in the high byte and the bytes filed below it. Reasons: 1 no
!   input sector or more than 32, 2 the input ends before the end code, 3 a copy from before the
!   output's start, 4 more output than the limit, 5 no free slot.
! - The input stays in its partition, and the sectors filed stay in the filter's, whether the job is
!   done or failed. The host leaves both partitions alone until MPED, then deletes them.
!
! The PRS stream: control bits, taken lowest first from control bytes, each read from the stream when
! the one before is used up.
! - 1: a literal byte follows.
! - 0, 0, then bits b1 and b2: a copy of b1 * 2 + b2 + 2 bytes from 256 less the next byte back.
! - 0, 1, then a little-endian word w: 0 ends the stream. Otherwise the copy reads from 8192 less
!   w >> 3 back, for (w & 7) + 2 bytes, or the next byte + 1 when w & 7 is 0.
!
! Stopping. Task 9 runs a decompress job in passes, one output sector each. 93 clears the work
! request, which ends the job between passes, as on the known cards: a pass that finds the request
! cleared, or the record since cleared by an activation, returns its slot to the pool and publishes
! nothing. Task 9 writes the stop's MPED after that pass, and the host waits for it before its next
! command. Until then an accepted disc can clear the expansion area under the pass, and an activation
! can clear the slot that entry 87 is about to file. 04 restarts task 9, which abandons the job.
!
! Activation clears the job and counts itself. A reload decodes the image again, which resets the
! record, since everything the extension keeps is in the image.

    .equ    protocol_version, 2
    .equ    operation_count, 2
    .equ    input_most, 32                  ! the input sectors a job takes
    .equ    work_request, 0x09075388        ! bit 0 makes task 9's card work loop call entry 54
    .equ    next_request, 0x09075389        ! bits 0..3 make the loop jump to entries 78, 77, 80 and 79
    .equ    rom_filter, 0x09075399          ! the filter entry 87 files through
    .equ    rom_slot, 0x0907539a            ! the slot entry 87 files, a word
    .equ    interrupt_flags, 0x0a00001e
    .equ    sector_lock_task9, 0x0b090000   ! the sector lock, owner task 9
    .equ    submit_message, 0x09810001      ! task 9, class 0x81, payload 1
    .equ    rom_entry_34, 0xf9c0            ! the ROM table's entry 34
    .equ    stub_task8_wait, 0xf80a         ! entry 32, task 8's wait
    .equ    stub_recheck, 0xf8ba            ! entry 76, the card work loop's recheck
    .equ    stub_reject, 0xf892             ! entry 66, status ff with the MPEG status
    .equ    stub_slot_take, 0xf8ce          ! entry 81, a free slot under the sector lock
    .equ    stub_slot_clear, 0xf8d2         ! entry 82, clear 24 or 280 bytes
    .equ    stub_slot_address, 0xf8da       ! entry 84, a slot's address
    .equ    stub_slot_file, 0xf8e6          ! entry 87, file a slot through a filter
    .equ    stub_status_byte, 0xf8ee        ! service 89, the status byte in r12
    .equ    stub_slot_at, 0xf902            ! service 94, the slot at a position of a partition
    .equ    stub_slot_return, 0xf90e        ! service 97, return a slot to the pool

! The decompress job, which only task 9 uses, by offset from resident_job.
    .equ    job_limit, 0                    ! the most output sectors
    .equ    job_filed, 4                    ! the output sectors filed
    .equ    job_input_at, 8                 ! the next input sector's entry in job_input
    .equ    job_input_end, 12               ! the end of the entries
    .equ    job_in, 16                      ! the input pointer between passes
    .equ    job_in_end, 20                  ! the end of its sector
    .equ    job_control, 24                 ! the control bits not yet used, above a marker bit
    .equ    job_copy, 28                    ! the bytes of a copy still to go
    .equ    job_distance, 32                ! how far back the copy reads
    .equ    job_filter, 36                  ! the output filter
    .equ    job_slot, 40                    ! the pass's slot
    .equ    job_partition, 44               ! the input partition
    .equ    job_count, 48                   ! the input sectors found
    .equ    job_ended, 52                   ! nonzero once the end code is read
    .equ    job_result, 56                  ! the output's length, once ended
    .equ    job_output, 60                  ! output sector k's data address at entry k & 7
    .equ    job_input, 92                   ! each input sector's data address
    .equ    job_size, job_input + 4 * input_most

    .text
    .global resident_start
resident_start:
! Expansion header, 0x16c bytes.
    .byte   1                       ! +0 MPEG version, nonzero for 93
    .byte   1                       ! +1 compared with a disc extension's +0x45
    .word   0
    .long   resident_load           ! +4 entry
! +8 expansion entries 0..88. The entry fills each 0 from the ROM table. Entries 3, 30 and 31 stay
! empty, as on the known cards: 93, AE and AF have handlers of their own. Every other entry the ROM
! table leaves empty, and the known cards fill, gets a filler so that no stub jumps to address 0:
! - resident_ignore returns. ROM entry 35 calls 43..49, 73..75 and 85. No ROM code calls 50, 52, 55,
!   62, 63, 65 or 71.
! - resident_task8_ignore goes to task 8's wait, for task 8's message handlers 51, 64 and 72.
! - resident_request_ignore clears the work request's bits 1..3 for 53, 69 and 70.
! - resident_jump_ignore clears bits 0..3 of the next request byte for 77..80, then goes to entry 76.
    .fill   4,4,0                   ! 0..3, commands 90..93
    .rept   23
    .long   resident_reject         ! 4..26, commands 94..AA
    .endr
    .long   resident_submit         ! 27, command AB
    .long   resident_status         ! 28, command AC
    .long   resident_identity       ! 29, command AD
    .fill   2,4,0                   ! 30, 31, commands AE and AF
    .fill   2,4,0                   ! 32, 33
    .long   resident_activate       ! 34, from task 8's activation
    .fill   8,4,0                   ! 35..42
    .rept   8
    .long   resident_ignore         ! 43..50
    .endr
    .long   resident_task8_ignore   ! 51
    .long   resident_ignore         ! 52
    .long   resident_request_ignore ! 53
    .long   resident_run            ! 54, from task 9's card work loop
    .long   resident_ignore         ! 55
    .fill   6,4,0                   ! 56..61
    .long   resident_ignore         ! 62
    .long   resident_ignore         ! 63
    .long   resident_task8_ignore   ! 64
    .long   resident_ignore         ! 65
    .fill   3,4,0                   ! 66..68
    .long   resident_request_ignore ! 69
    .long   resident_request_ignore ! 70
    .long   resident_ignore         ! 71
    .long   resident_task8_ignore   ! 72
    .rept   3
    .long   resident_ignore         ! 73..75
    .endr
    .long   0                       ! 76
    .rept   4
    .long   resident_jump_ignore    ! 77..80
    .endr
    .fill   4,4,0                   ! 81..84
    .long   resident_ignore         ! 85
    .fill   3,4,0                   ! 86..88

! Addresses for tools that read the image: the code, its end and the record. The firmware does not
! use them.
    .long   resident_code           ! +0x16c
    .long   resident_end            ! +0x170
    .long   resident_state          ! +0x174

! Entry, called from 0x1b42 in task 9 with r0 the entry address, r1 the header 0x0907b000 and PR
! 0x1b46. The loader ignores the return value. Fills each empty entry of the table from the ROM
! table at 0xf938, as the known cards' entries do, so ROM routines and services are reachable
! through the stubs. Uses r0 and r2..r4 only.
    .balign 4
resident_code:
resident_load:
    mov.l   load_rom_table,r2
    mov     r1,r3
    add     #8,r3
    mov     #89,r4

resident_load_next:
    mov.l   @r3,r0
    tst     r0,r0
    bf      resident_load_kept
    mov.l   @r2,r0
    mov.l   r0,@r3

resident_load_kept:
    add     #4,r2
    add     #4,r3
    add     #-1,r4
    tst     r4,r4
    bf      resident_load_next
    rts
    nop

! Entry 34, called by task 8's activation (0xa41c) through the stub 0xf812 with PR 0xa438, after
! the MPEG state is cleared. Clears the job and counts the activation, then continues in the ROM
! table's entry 34, as the known cards do. It sets the MPEG state, the card bit that opens groups 9
! and A among it, and returns.
resident_activate:
    mov.l   state_address,r1
    mov     #0,r0
    mov.l   r0,@r1
    mov.l   r0,@(4,r1)
    mov.l   r0,@(8,r1)
    mov.w   @(12,r1),r0
    add     #1,r0
    mov.w   r0,@(12,r1)
    mov.l   rom_activate,r0
    mov.l   @r0,r0
    jmp     @r0
    nop

! Entry 27, command AB, inside IRQ6 with r1 = CR1:CR2 and r2 = CR3:CR4. The IRQ6 exit writes r12
! and r13 to CR1..CR4 and posts a nonzero r11.
resident_submit:
    mov.l   state_address,r8
    mov.b   @(7,r8),r0
    cmp/eq  #1,r0
    bt      resident_reject
    cmp/eq  #2,r0
    bt      resident_reject
! The operation from the CR1 low byte.
    swap.w  r1,r0
    extu.b  r0,r4
    mov     #operation_count,r3
    cmp/hs  r3,r4
    bt      resident_reject
! Decompress takes an input partition and an output filter up to 23, and a limit from 1.
    mov     r4,r0
    cmp/eq  #1,r0
    bf      resident_accept
    mov     r2,r0
    shlr16  r0
    mov     r0,r5
    shlr8   r5
    extu.b  r0,r0
    mov     #24,r3
    cmp/hs  r3,r5
    bt      resident_reject
    cmp/hs  r3,r0
    bt      resident_reject
    extu.w  r2,r0
    tst     r0,r0
    bt      resident_reject
! Keep the job, set bit 0 of the work request and post the job to task 9. Task 9 changes the work
! request only with interrupts masked or in one instruction, and IRQ6 cannot come between this read
! and write.
resident_accept:
    mov.l   r2,@r8
    mov     r1,r0
    mov.w   r0,@(4,r8)
    mov     r4,r0
    mov.b   r0,@(6,r8)
    mov     #1,r0
    mov.b   r0,@(7,r8)
    mov     #0,r0
    mov.l   r0,@(8,r8)
    mov.l   work_request_address,r3
    mov.b   @r3,r0
    or      #1,r0
    mov.b   r0,@r3
    mov.l   submit_message_word,r11

! The response of AB and AC: the state and the job ID in r12, the result in r13, then the status byte
! through service 89, which returns to the IRQ6 handler.
resident_respond:
    mov.l   state_address,r8
    mov.b   @(7,r8),r0
    extu.b  r0,r12
    shll16  r12
    mov.w   @(4,r8),r0
    extu.w  r0,r0
    or      r0,r12
    mov.l   @(8,r8),r13
    mov.l   status_byte_stub,r0
    jmp     @r0
    nop

! Entry 28, command AC, posts nothing.
resident_status:
    bra     resident_respond
    xor     r11,r11

! Entry 29, command AD: the protocol version and the activations in r12, the identity in r13.
resident_identity:
    xor     r11,r11
    mov.l   state_address,r8
    mov.w   @(12,r8),r0
    extu.w  r0,r12
    mov.l   version_word,r0
    or      r0,r12
    mov.l   identity_word,r13
    mov.l   status_byte_stub,r0
    jmp     @r0
    nop

! Every rejected command: entry 66 answers status ff, and nothing is posted.
resident_reject:
    mov.l   reject_stub,r0
    jmp     @r0
    xor     r11,r11

! The fillers. Called with a return address, return.
resident_ignore:
    rts
    nop

! Jumped to from task 8's message dispatch, go to its wait.
resident_task8_ignore:
    mov.l   task8_wait_stub,r0
    jmp     @r0
    nop

! Called by task 9's card work loop for bit 1, 2 or 3 of the work request: clear all three, or the
! loop calls again. and.b is one instruction, so a submit inside IRQ6 cannot come between the read
! and the write and lose bit 0.
resident_request_ignore:
    stc     gbr,r2
    mov.l   work_request_address,r1
    ldc     r1,gbr
    mov     #0,r0
    and.b   #241,@(r0,gbr)
    ldc     r2,gbr
    rts
    nop

! Jumped to by the loop for bit 0, 1, 2 or 3 of the next request byte: clear all four, then the
! loop's recheck through entry 76. No interrupt handler sets bits in this byte.
resident_jump_ignore:
    mov.l   next_request_address,r1
    mov.b   @r1,r0
    and     #240,r0
    mov.b   r0,@r1
    mov.l   recheck_stub,r0
    jmp     @r0
    nop

! Entry 54, called by task 9's card work loop (0xba5e) with PR 0xba6e while bit 0 of the work
! request is set. Each call is one step of the job: its start, or a pass that fills one output
! sector. The loop calls again until the job's end clears the bit. The record is read with
! interrupts masked, so that no submit or activation comes between reading it and acting on it.
resident_run:
    sts.l   pr,@-r15
    stc     sr,r3
    mov     r3,r0
    or      #240,r0
    ldc     r0,sr
    mov.l   state_address,r1
    mov.b   @(7,r1),r0
    cmp/eq  #1,r0
    bt      resident_start_job
    cmp/eq  #2,r0
    bt      resident_running
! No job submitted or running: a request bit left from elsewhere. Clear it and return.
    mov.l   work_request_address,r2
    mov.b   @r2,r0
    and     #254,r0
    bra     resident_unmask_return
    mov.b   r0,@r2

resident_running:
    ldc     r3,sr
    bra     decompress_pass
    nop

! The job's start: the activations it runs in, its operation and argument, and state 2.
resident_start_job:
    mov.w   @(12,r1),r0
    mov.w   r0,@(14,r1)
    mov.l   @r1,r4
    mov.b   @(6,r1),r0
    extu.b  r0,r5
    mov     #2,r0
    mov.b   r0,@(7,r1)
    ldc     r3,sr
    tst     r5,r5
    bf      decompress_start
! Operation 0, ping: the argument inverted.
    not     r4,r4
    bra     resident_publish
    mov     #3,r5

! Masks interrupts, keeping the SR to restore in r3, and sets T when the job is still its
! activation's and its request still set: bit 0 of the work request, the record in state 2 and its
! activations those the job started in. Returns r1 the record and r2 the work request.
resident_owned:
    stc     sr,r3
    mov     r3,r0
    or      #240,r0
    ldc     r0,sr
    mov.l   state_address,r1
    mov.l   work_request_address,r2
    mov.b   @r2,r0
    tst     #1,r0
    bt      resident_owned_not
    mov.b   @(7,r1),r0
    cmp/eq  #2,r0
    bf      resident_owned_not
    mov.w   @(12,r1),r0
    mov     r0,r6
    mov.w   @(14,r1),r0
    rts
    cmp/eq  r0,r6

resident_owned_not:
    rts
    clrt

! The job's end: r4 the result, r5 the state, 3 or 4. The record takes both, bit 0 is cleared and
! MPED written, all with interrupts masked and only while the job is its own. A stop or an
! activation before that ends the job with nothing published.
resident_publish:
    bsr     resident_owned
    nop
    bf      resident_unmask_return
    mov.l   r4,@(8,r1)
    mov     r5,r0
    mov.b   r0,@(7,r1)
    mov.b   @r2,r0
    and     #254,r0
    mov.b   r0,@r2
    mov.l   interrupt_flags_address,r1
    mov     #8,r0
    shll8   r0
    mov.w   r0,@r1

resident_unmask_return:
    ldc     r3,sr

resident_return:
    lds.l   @r15+,pr
    rts
    nop

    .balign 4
load_rom_table:
    .long   0x0000f938
state_address:
    .long   resident_state
rom_activate:
    .long   rom_entry_34
work_request_address:
    .long   work_request
next_request_address:
    .long   next_request
task8_wait_stub:
    .long   stub_task8_wait
recheck_stub:
    .long   stub_recheck
submit_message_word:
    .long   submit_message
status_byte_stub:
    .long   stub_status_byte
reject_stub:
    .long   stub_reject
interrupt_flags_address:
    .long   interrupt_flags
version_word:
    .long   protocol_version << 16
identity_word:
    .long   0x444f5345

! Decompress's start, with r4 the argument: the argument kept, then the input.
decompress_start:
    mov.l   job_address,r13
    extu.w  r4,r0
    mov.l   r0,@(job_limit,r13)
    shlr16  r4
    extu.b  r4,r0
    mov.l   r0,@(job_filter,r13)
    shlr8   r4
    mov.l   r4,@(job_partition,r13)
    mov     #0,r0
    mov.l   r0,@(job_filed,r13)
    mov.l   r0,@(job_count,r13)
    mov.l   r0,@(job_copy,r13)
    mov.l   r0,@(job_ended,r13)
    mov     #1,r0
    mov.l   r0,@(job_control,r13)

! Under the sector lock, the slot at each position of the input partition, one at a time. A
! contended acquire resumes with only PC and SR, so what is used after one is read again from
! memory.
decompress_input:
    mov.l   sector_lock,r11
    trapa   #42
    mov.l   job_address,r13
    mov.l   @(job_partition,r13),r1
    mov.l   @(job_count,r13),r2
    mov.l   slot_at_stub,r0
    jsr     @r0
    nop
    mov.l   sector_lock,r11
    trapa   #39
    exts.b  r1,r0
    cmp/eq  #-1,r0
    bt      decompress_input_end
    mov.l   job_address,r13
    mov.l   @(job_count,r13),r0
    cmp/eq  #input_most,r0
    bt      decompress_input_refused
! The sector's data, 24 bytes into the slot.
    mov.l   slot_address_stub,r0
    jsr     @r0
    nop
    add     #24,r1
    mov.l   job_address,r13
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

! The input found: decoding starts at the first sector's data with the next call.
decompress_input_end:
    mov.l   job_address,r13
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
    bra     resident_return
    mov.l   r0,@(job_in_end,r13)

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
    bra     resident_publish
    mov     #4,r5

! A pass: one output sector. With no copy under way the stream can end here, at a sector's end, and
! the job is done without taking a slot.
decompress_pass:
    mov.l   job_address,r13
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
    bra     resident_publish
    mov     #3,r5

! More output than the limit allows: reason 4.
decompress_room:
    mov.l   @(job_filed,r13),r0
    mov.l   @(job_limit,r13),r1
    cmp/hs  r1,r0
    bf      decompress_slot
    bra     decompress_fail
    mov     #4,r4

! A free slot from entry 81, whose first 24 bytes entry 82 clears, so it reads as a mode 0 sector
! with its data from +24. No free slot: reason 5.
decompress_slot:
    mov.l   slot_take_stub,r0
    jsr     @r0
    nop
    cmp/eq  #-1,r0
    bf      decompress_slot_taken
    bra     decompress_fail
    mov     #5,r4

decompress_slot_taken:
    extu.b  r0,r1
    mov.l   job_address,r13
    mov.l   r1,@(job_slot,r13)
    mov.l   slot_address_stub,r0
    jsr     @r0
    nop
    mov     #0,r5
    mov.l   slot_clear_stub,r0
    jsr     @r0
    nop
! r11 and r12, the sector's data and its end. Its address goes to output entry filed & 7 for the
! copies that read it later.
    mov     r1,r11
    mov.l   job_address,r13
    mov.l   @(job_filed,r13),r0
    and     #7,r0
    shll2   r0
    add     #job_output,r0
    mov.l   r11,@(r0,r13)
    mov     #8,r12
    shll8   r12
    add     r11,r12
    mov.l   @(job_in,r13),r8
    mov.l   @(job_in_end,r13),r9
    mov.l   @(job_control,r13),r10
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

! The sector done: the 280 bytes after its data cleared, then filed through the job's filter by
! entry 87, which takes the slot and the filter from RAM. A job that is no longer its own returns
! the slot to the pool instead.
decompress_file:
    mov     r12,r1
    mov     #1,r5
    mov.l   slot_clear_stub,r0
    jsr     @r0
    nop
    bsr     resident_owned
    nop
    bf      decompress_dropped
    mov.l   job_address,r13
    mov.l   @(job_filter,r13),r0
    mov.l   rom_filter_address,r1
    mov.b   r0,@r1
    mov.l   @(job_slot,r13),r0
    mov.l   rom_slot_address,r1
    mov.w   r0,@r1
    ldc     r3,sr
    mov.l   slot_file_stub,r0
    jsr     @r0
    nop
    mov.l   job_address,r13
    mov.l   @(job_filed,r13),r0
    add     #1,r0
    mov.l   r0,@(job_filed,r13)
    mov.l   @(job_ended,r13),r0
    tst     r0,r0
    bt      decompress_file_return
    mov.l   @(job_result,r13),r4
    bra     resident_publish
    mov     #3,r5

decompress_file_return:
    bra     resident_return
    nop

decompress_dropped:
    ldc     r3,sr
    bsr     decompress_slot_back
    nop
    bra     resident_return
    nop

! Returns the pass's slot to the pool through service 97, under the sector lock.
decompress_slot_back:
    sts.l   pr,@-r15
    mov.l   sector_lock,r11
    trapa   #42
    mov.l   job_address,r13
    mov.l   @(job_slot,r13),r1
    mov.l   slot_return_stub,r0
    jsr     @r0
    nop
    mov.l   sector_lock,r11
    trapa   #39
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
job_address:
    .long   resident_job
sector_lock:
    .long   sector_lock_task9
slot_at_stub:
    .long   stub_slot_at
slot_address_stub:
    .long   stub_slot_address
slot_take_stub:
    .long   stub_slot_take
slot_clear_stub:
    .long   stub_slot_clear
slot_file_stub:
    .long   stub_slot_file
slot_return_stub:
    .long   stub_slot_return
rom_filter_address:
    .long   rom_filter
rom_slot_address:
    .long   rom_slot

! The record: +0 the argument, +4 the job ID, +6 the operation, +7 the state, +8 the result, +12
! the activations since the load, +14 the activations when the job started.
    .balign 4
resident_state:
    .long   0
    .word   0
    .byte   0
    .byte   0
    .long   0
    .word   0
    .word   0

! The decompress job, from the offsets above.
    .balign 4
resident_job:
    .fill   job_size,1,0

! The encoder takes whole eight-byte blocks.
    .balign 8
    .global resident_end
resident_end:
