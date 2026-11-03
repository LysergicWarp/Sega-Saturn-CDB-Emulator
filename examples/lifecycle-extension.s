! A minimal resident card extension for the CD-block SH-1, CDB105 (SHA-1
! eb8393058f324e922c11b43709b64fc6ca94ab86). It goes through the lifecycle of a card extension of
! one's own: the card checks and the expansion loader load it as they load the known cards, 93
! activates it, it answers commands of its own inside IRQ6 and runs one job at a time in task 9. It
! uses only the interface page and the RAM the known cards use. README.md in this folder says what it
! has run on and what turning it into card firmware takes.
!
! Commands, once 93 has activated the card:
! - AD, identity: CR1 low byte the protocol version, CR2 the activations since the load, CR3:CR4
!   0x444f5345 ("DOSE"). The known cards answer AD with ffff in CR1 to CR4.
! - AB submits a job: CR1 low byte the operation, CR2 the job ID, CR3:CR4 the argument. An unknown
!   operation, or a job already submitted or running, rejects it and nothing is posted.
! - AC reports the job: CR1 low byte the state, CR2 the job ID, CR3:CR4 the result. States: 0 none
!   since activation, 1 submitted, 2 running, 3 done.
! - The end of each job writes MPED to 0x0a00001e.
! - 90, 91 and 92 stay the ROM's, and 93, AE and AF have handlers of their own. Every other MPEG
!   command is rejected with status ff.
!
! Operations: 0, ping, answers the argument inverted.
!
! Activation clears the job and counts itself. A reload decodes the image again, which resets the
! record, since everything the extension keeps is in the image.

    .equ    protocol_version, 1
    .equ    operation_count, 1
    .equ    work_request, 0x09075388        ! bit 0 makes task 9's card work loop call entry 54
    .equ    next_request, 0x09075389        ! bits 0..3 make the loop jump to entries 78, 77, 80 and 79
    .equ    interrupt_flags, 0x0a00001e
    .equ    submit_message, 0x09810001      ! task 9, class 0x81, payload 1
    .equ    rom_entry_34, 0xf9c0            ! the ROM table's entry 34
    .equ    stub_task8_wait, 0xf80a         ! entry 32, task 8's wait
    .equ    stub_recheck, 0xf8ba            ! entry 76, the card work loop's recheck
    .equ    stub_reject, 0xf892             ! entry 66, status ff with the MPEG status
    .equ    stub_status_byte, 0xf8ee        ! service 89, the status byte in r12

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
! Keep the job, set bit 0 of the work request and post the job to task 9. Task 9 only clears bits of
! the work request, and IRQ6 cannot come between this read and write.
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
! request is set. Runs the job's operation with r1 the argument, keeps its r0 as the result and
! returns. The operation runs in task 9, so every CD task and interrupt comes before it.
resident_run:
    sts.l   pr,@-r15
    stc     gbr,r2
    mov.l   work_request_address,r1
    ldc     r1,gbr
    mov     #0,r0
    and.b   #254,@(r0,gbr)
    ldc     r2,gbr
    mov.l   state_address,r8
    mov     #2,r0
    mov.b   r0,@(7,r8)
    mov.b   @(6,r8),r0
    extu.b  r0,r0
    shll2   r0
    mov.l   operations_address,r1
    mov.l   @(r0,r1),r2
    mov.l   @r8,r1
    jsr     @r2
    nop
    mov.l   state_address,r8
    mov.l   r0,@(8,r8)
    mov     #3,r0
    mov.b   r0,@(7,r8)
! MPED for the host.
    mov.l   interrupt_flags_address,r1
    mov     #8,r0
    shll8   r0
    mov.w   r0,@r1
    lds.l   @r15+,pr
    rts
    nop

! Operation 0, ping: the argument inverted.
resident_ping:
    rts
    not     r1,r0

    .balign 4
load_rom_table:
    .long   0x0000f938
state_address:
    .long   resident_state
operations_address:
    .long   resident_operations
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

! The operations by number.
resident_operations:
    .long   resident_ping

! The record: +0 the argument, +4 the job ID, +6 the operation, +7 the state, +8 the result, +12
! the activations since the load.
    .balign 4
resident_state:
    .long   0
    .word   0
    .byte   0
    .byte   0
    .long   0
    .word   0
    .word   0

! The encoder takes whole eight-byte blocks.
    .balign 8
    .global resident_end
resident_end:
