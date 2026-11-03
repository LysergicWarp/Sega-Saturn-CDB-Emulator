! A card extension for the CD-block SH-1, CDB105 (SHA-1 eb8393058f324e922c11b43709b64fc6ca94ab86),
! that runs modules of SH-1 code the host uploads. The host puts a module's package into a partition
! with 64, and a load job checks it and installs it in the module slot. AB then submits jobs to the
! module by the handle the load gave, and task 9 calls it once per pass while the CD tasks go on. It
! is disc-decompress-extension.s, in this folder, with the decoder moved out into
! decompress-module.s and a loader in its place. README.md in this folder says what it has run on
! and what turning it into card firmware takes.
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
! - 1, load: CR3's high byte the partition that holds the package, up to 23. The result is the
!   installed module's handle.
! - 2, unload: no module afterwards. The result is 0.
! - The installed module's handle, 3 to 255: a job of the module, with CR3:CR4 its argument and its
!   result. Any other number is rejected, so a handle from before a later load or an unload is,
!   until the handles come round again after 253 loads. A reload starts them again at 3, so the
!   host drops its handles then.
!
! Packages. A module is linked to run in the module slot, 0x0907c000..0x0907ffff, and its package is
! its image from there. 64 puts it, or a disc read files it, and each sector's data is found by its
! mode as 61 finds it. The image starts with a header:
! - +0 0x444f534d ("DOSM"), +4 the ABI version, 1, a word, +6 the number of imports, a word.
! - +8 the image's bytes, a multiple of 4, from the header through the CRC.
! - +12 the BSS bytes after the image, a multiple of 4, which the load clears.
! - +16 the stack bytes the module uses below the SP it is called with, up to 32.
! - +20 the entry, +24 the import table, +28 0.
! - The image's last longword is the CRC-32 of the bytes before it, as zlib computes it.
! - The import table holds a longword per import, the service's number. The load replaces each with
!   the service's address.
!
! Loading runs in passes of task 9. The first finds the package's sectors and checks the header, one
! pass per sector then checks the CRC, and the last installs the module. The installed module is
! left alone until then, so a package that fails leaves it callable. A failed load's result holds the
! reason in its high byte: 1 no sector or more than 8, 2 a mode 2 form 2 sector, 3 not a package of
! this ABI, 4 a size or address outside the slot or the sectors, a size or the import table not a
! multiple of 4, an odd entry or more than 32 stack bytes, 5 an import with no service, 6 the CRC.
! The package stays in its partition, and the host deletes it after MPED. The later passes work
! from the sizes and addresses the first one checked, so a host that changes the package's sectors
! meanwhile can install wrong bytes but not make the load write outside the slot.
!
! The module's entry is called once per pass of its job, on task 9's stack, with r4 the pass, 0 at
! the job's start, r5 the argument and r6 the address of the result word. It returns r0 0 to be
! called again, 1 when done and 2 when failed, with the result word set. It may change any register
! but r14, r15 and GBR, which tasks resumed with only PC and SR take as they find it, and leaves SR
! as it found it. Each call is meant to be one bounded step, since a stop takes effect between
! calls.
!
! Services, by number, called with jsr: the arguments from r4, the result in r0. They keep r8..r14
! and may change r0..r7, PR, MACH and MACL. Each one that takes the sector lock itself works when the
! lock is contended. take and file go through entries 81 and 87, which take it themselves and push PR
! after their acquire, so those two need it uncontended.
! - 1 slot_at, r4 a partition and r5 a position: the slot there, or -1.
! - 2 sector_data, r4 a slot: the address of its data by its mode, +16 for mode 1 and +24 for any
!   other mode, or 0 for a mode 2 form 2 sector.
! - 3 take: a free slot, its first 24 bytes cleared so that it reads as a mode 0 sector, or -1.
! - 4 file, r4 a slot from take and r5 a filter: the 280 bytes after its data cleared, then the slot
!   filed through the filter. 1 when filed, or 0 with the slot back in the pool when the job is no
!   longer its own: stopped, or cleared by an activation.
! - 5 give_back, r4 a slot from take: the slot back in the pool.
!
! Stopping. 93 clears the work request, which ends the job between passes, as on the known cards. A
! module's next file then returns 0, and the job publishes nothing. Task 9 writes the stop's MPED
! after that pass, and the host waits for it before its next command. Until then an accepted disc can
! clear the expansion area under the pass, and an activation can clear the slot that entry 87 is about
! to file. 04 restarts task 9, which abandons the job.
!
! Lifetime. Activation clears the job and counts itself. The module stays installed through 93's
! stop, start and reset, and through 04, and its next job starts at pass 0. A reload decodes this
! image again, which resets the record and forgets the module. A load stopped once its install has
! begun leaves no module.

    .equ    protocol_version, 4
    .equ    operation_count, 3              ! the operations before the handles: ping, load, unload
    .equ    handle_first, 3
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

! The module slot and the packages.
    .equ    slot_start, 0x0907c000
    .equ    slot_bytes, 0x4000
    .equ    package_most, 8                 ! the sectors a package takes
    .equ    header_bytes, 32
    .equ    module_abi, 1
    .equ    stack_most, 32                  ! the stack bytes a module may use
    .equ    service_count, 5

! The job, which only task 9 uses, by offset from resident_job.
    .equ    job_pass, 0                     ! the passes so far
    .equ    job_result, 4                   ! the module's result word
    .equ    job_partition, 8                ! the package's partition
    .equ    job_count, 12                   ! the package's sectors
    .equ    job_done, 16                    ! the bytes the CRC has taken
    .equ    job_crc, 20                     ! the CRC so far
    .equ    job_image, 24                   ! the header's fields as the first pass checked them
    .equ    job_bss, 28
    .equ    job_table, 32
    .equ    job_imports, 36
    .equ    job_entry, 40
    .equ    job_data, 44                    ! each package sector's data address
    .equ    job_size, job_data + 4 * package_most

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

! Addresses for tools that read the image: the code, its end, the record and the module record. The
! firmware does not use them.
    .long   resident_code           ! +0x16c
    .long   resident_end            ! +0x170
    .long   resident_state          ! +0x174
    .long   resident_module         ! +0x178

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
! and A among it, and returns. The module record is left alone.
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
    bt      resident_submit_module
! A load takes a partition up to 23.
    mov     r4,r0
    cmp/eq  #1,r0
    bf      resident_accept
    mov     r2,r0
    shlr16  r0
    shlr8   r0
    mov     #24,r3
    cmp/hs  r3,r0
    bt      resident_reject
    bra     resident_accept
    nop

! Any other operation is a job of the installed module, named by its handle.
resident_submit_module:
    mov.l   module_address,r3
    mov.b   @r3,r0
    extu.b  r0,r0
    cmp/eq  r4,r0
    bf      resident_reject

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
! request is set. Each call is one pass of the job, the first its start. The loop calls again until
! the job's end clears the bit. The record is read with interrupts masked, so that no submit or
! activation comes between reading it and acting on it.
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
    mov.b   @(6,r1),r0
    ldc     r3,sr
    cmp/eq  #1,r0
    bf      module_call
    bra     load_pass
    nop

! The job's start: the activations it runs in, its argument in r4, state 2 and no passes yet.
resident_start_job:
    mov.w   @(12,r1),r0
    mov.w   r0,@(14,r1)
    mov.l   @r1,r4
    mov.b   @(6,r1),r0
    extu.b  r0,r5
    mov     #2,r0
    mov.b   r0,@(7,r1)
    ldc     r3,sr
    mov.l   job_address,r13
    mov     #0,r0
    mov.l   r0,@(job_pass,r13)
    mov     r5,r0
    tst     r0,r0
    bt      resident_ping
    cmp/eq  #2,r0
    bt      resident_unload
    cmp/eq  #1,r0
    bf      module_call
    bra     load_start
    nop

! Operation 0, ping: the argument inverted.
resident_ping:
    not     r4,r4
    bra     resident_publish
    mov     #3,r5

! Operation 2, unload: the module forgotten, while the job is its own.
resident_unload:
    bsr     resident_owned
    nop
    bf      resident_unmask_return
    mov.l   module_address,r4
    mov     #0,r0
    mov.b   r0,@r4
    mov     #0,r4
    bra     resident_publish_owned
    mov     #3,r5

! A pass of the module's job: its entry with r4 the pass, r5 the argument and r6 the result word.
! It returns r0 0 to be called again, 1 done or 2 failed. Any other value counts as failed.
module_call:
    mov.l   job_address,r13
    mov.l   @(job_pass,r13),r4
    mov     r13,r6
    add     #job_result,r6
    mov.l   state_address,r1
    mov.l   @r1,r5
    mov.l   module_address,r1
    mov.l   @(4,r1),r0
    jsr     @r0
    nop
    mov.l   job_address,r13
    tst     r0,r0
    bf      module_ended
    mov.l   @(job_pass,r13),r0
    add     #1,r0
    bra     resident_return
    mov.l   r0,@(job_pass,r13)

module_ended:
    mov.l   @(job_result,r13),r4
    cmp/eq  #1,r0
    mov     #3,r5
    bt      resident_publish
    bra     resident_publish
    mov     #4,r5

! Masks interrupts, keeping the SR to restore in r3, and sets T when the job is still its
! activation's and its request still set: bit 0 of the work request, the record in state 2 and its
! activations those the job started in. Returns r1 the record and r2 the work request. Uses r0 and
! r6.
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

! The same with resident_owned's check made and its registers.
resident_publish_owned:
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
module_address:
    .long   resident_module
job_address:
    .long   resident_job
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

! The services, for modules and for the loader. Each keeps r8..r13 and its caller's PR in the save
! area while it runs, since a contended lock acquire resumes with only PC and SR, and restores them
! as it returns. Their arguments wait in service_args across an acquire.
service_slot_at:
    sts     pr,r3
    bsr     service_save
    nop
    mov.l   service_args_address,r1
    mov.l   r4,@r1
    mov.l   r5,@(4,r1)
    mov.l   sector_lock_service,r11
    trapa   #42
    mov.l   service_args_address,r1
    mov.l   @(4,r1),r2
    mov.l   @r1,r1
    mov.l   slot_at_stub,r0
    jsr     @r0
    nop
    mov.l   sector_lock_service,r11
    trapa   #39
    exts.b  r1,r0
    cmp/eq  #-1,r0
    bt      service_return
    bra     service_return
    extu.b  r1,r0

! The data by the mode byte at +15, as 61 reads it: 16 bytes into the slot for mode 1 and 24 for any
! other mode. Mode 2 form 2, submode bit 5 at +18, has none.
service_sector_data:
    sts     pr,r3
    bsr     service_save
    nop
    mov     r4,r1
    mov.l   slot_address_stub,r0
    jsr     @r0
    nop
    mov.b   @(15,r1),r0
    add     #16,r1
    cmp/eq  #1,r0
    bt      service_sector_data_at
    cmp/eq  #2,r0
    bf      service_sector_data_other
    mov.b   @(2,r1),r0
    tst     #32,r0
    bf      service_sector_data_form2

service_sector_data_other:
    add     #8,r1

service_sector_data_at:
    bra     service_return
    mov     r1,r0

service_sector_data_form2:
    bra     service_return
    mov     #0,r0

! A free slot from entry 81, whose first 24 bytes entry 82 clears.
service_take:
    sts     pr,r3
    bsr     service_save
    nop
    mov.l   slot_take_stub,r0
    jsr     @r0
    nop
    cmp/eq  #-1,r0
    bt      service_return
    extu.b  r0,r1
    mov.l   service_args_address,r2
    mov.l   r1,@r2
    mov.l   slot_address_stub,r0
    jsr     @r0
    nop
    mov     #0,r5
    mov.l   slot_clear_stub,r0
    jsr     @r0
    nop
    mov.l   service_args_address,r1
    bra     service_return
    mov.l   @r1,r0

! The 280 bytes after the data cleared, then the slot filed through the filter by entry 87, which
! takes both from RAM. A job that is no longer its own returns the slot to the pool instead.
service_file:
    sts     pr,r3
    bsr     service_save
    nop
    mov.l   service_args_address,r1
    mov.l   r4,@r1
    mov.l   r5,@(4,r1)
    mov     r4,r1
    mov.l   slot_address_stub,r0
    jsr     @r0
    nop
    mov.w   data_end,r0
    add     r0,r1
    mov     #1,r5
    mov.l   slot_clear_stub,r0
    jsr     @r0
    nop
    bsr     resident_owned
    nop
    bf      service_file_dropped
    mov.l   service_args_address,r1
    mov.l   @(4,r1),r0
    mov.l   rom_filter_address,r2
    mov.b   r0,@r2
    mov.l   @r1,r0
    mov.l   rom_slot_address,r2
    mov.w   r0,@r2
    ldc     r3,sr
    mov.l   slot_file_stub,r0
    jsr     @r0
    nop
    bra     service_return
    mov     #1,r0

service_file_dropped:
    ldc     r3,sr
    mov.l   service_args_address,r1
    bsr     service_slot_back
    mov.l   @r1,r4
    bra     service_return
    mov     #0,r0

service_give_back:
    sts     pr,r3
    bsr     service_save
    nop
    bsr     service_slot_back
    nop
    bra     service_return
    mov     #0,r0

! Returns slot r4 to the pool through service 97, under the sector lock.
service_slot_back:
    sts.l   pr,@-r15
    mov.l   service_args_address,r1
    mov.l   r4,@(8,r1)
    mov.l   sector_lock_service,r11
    trapa   #42
    mov.l   service_args_address,r1
    mov.l   @(8,r1),r1
    mov.l   slot_return_stub,r0
    jsr     @r0
    nop
    mov.l   sector_lock_service,r11
    trapa   #39
    lds.l   @r15+,pr
    rts
    nop

! Keeps r8..r13 and r3, the caller's PR, in the save area. Uses r1.
service_save:
    mov.l   save_end_address,r1
    mov.l   r3,@-r1
    mov.l   r13,@-r1
    mov.l   r12,@-r1
    mov.l   r11,@-r1
    mov.l   r10,@-r1
    mov.l   r9,@-r1
    rts
    mov.l   r8,@-r1

! Restores them and returns to the service's caller with r0 the result. Uses r1.
service_return:
    mov.l   save_address,r1
    mov.l   @r1+,r8
    mov.l   @r1+,r9
    mov.l   @r1+,r10
    mov.l   @r1+,r11
    mov.l   @r1+,r12
    mov.l   @r1+,r13
    lds.l   @r1+,pr
    rts
    nop

data_end:
    .word   24 + 2048
    .balign 4
service_args_address:
    .long   service_args
save_address:
    .long   service_save_area
save_end_address:
    .long   service_save_area + 28
sector_lock_service:
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

! The load's first pass, with r4 the argument: the package's sectors, then its header.
load_start:
    mov.l   load_job_address,r13
    shlr16  r4
    shlr8   r4
    mov.l   r4,@(job_partition,r13)
    mov     #0,r0
    mov.l   r0,@(job_count,r13)
    mov.l   r0,@(job_done,r13)
    mov     #-1,r0
    mov.l   r0,@(job_crc,r13)

! Each sector of the partition in turn, with its data by its mode.
load_find:
    mov.l   @(job_partition,r13),r4
    mov.l   @(job_count,r13),r5
    bsr     service_slot_at
    nop
    cmp/pz  r0
    bf      load_found
    mov     r0,r4
    mov.l   @(job_count,r13),r0
    cmp/eq  #package_most,r0
    bt      load_no_sectors
    bsr     service_sector_data
    nop
    tst     r0,r0
    bt      load_form2
    mov     r0,r1
    mov.l   @(job_count,r13),r0
    shll2   r0
    add     #job_data,r0
    mov.l   r1,@(r0,r13)
    mov.l   @(job_count,r13),r0
    add     #1,r0
    bra     load_find
    mov.l   r0,@(job_count,r13)

load_no_sectors:
    bra     load_fail
    mov     #1,r4

load_form2:
    bra     load_fail
    mov     #2,r4

load_bad_header:
    bra     load_fail
    mov     #3,r4

load_bounds:
    bra     load_fail
    mov     #4,r4

! The header, from r8 the first sector's data: this ABI's package.
load_found:
    mov.l   @(job_count,r13),r0
    tst     r0,r0
    bt      load_no_sectors
    mov.l   @(job_data,r13),r8
    mov.l   @r8,r0
    mov.l   module_magic,r1
    cmp/eq  r1,r0
    bf      load_bad_header
    mov.w   @(4,r8),r0
    cmp/eq  #module_abi,r0
    bf      load_bad_header
    mov.l   @(28,r8),r0
    tst     r0,r0
    bf      load_bad_header
! r9 the image's bytes: the header and the CRC at least, a multiple of 4, within the slot and
! exactly the sectors put.
    mov.l   @(8,r8),r9
    mov     #header_bytes + 4,r0
    cmp/hs  r0,r9
    bf      load_bounds
    mov     r9,r0
    tst     #3,r0
    bf      load_bounds
    mov.w   slot_size,r1
    cmp/hi  r1,r9
    bt      load_bounds
    mov.w   sector_less_one,r0
    add     r9,r0
    shlr8   r0
    shlr2   r0
    shlr    r0
    mov.l   @(job_count,r13),r2
    cmp/eq  r2,r0
    bf      load_bounds
    mov.l   r9,@(job_image,r13)
! r10 the BSS bytes: a multiple of 4, and the image with them within the slot.
    mov.l   @(12,r8),r10
    mov     r10,r0
    tst     #3,r0
    bf      load_bounds
    cmp/hi  r1,r10
    bt      load_bounds
    mov     r9,r0
    add     r10,r0
    cmp/hi  r1,r0
    bt      load_bounds
    mov.l   r10,@(job_bss,r13)
! The stack within the budget.
    mov.l   @(16,r8),r0
    mov     #stack_most,r1
    cmp/hi  r1,r0
    bt      load_bounds
! r12 the CRC's address. The entry even, after the header and before the CRC.
    mov.l   load_slot_start,r11
    mov     r11,r12
    add     r9,r12
    add     #-4,r12
    mov     r11,r2
    add     #header_bytes,r2
    mov.l   @(20,r8),r0
    tst     #1,r0
    bf      load_bounds
    cmp/hs  r2,r0
    bf      load_bounds
    cmp/hs  r12,r0
    bt      load_bounds
    mov.l   r0,@(job_entry,r13)
! The import table aligned, after the header, and ending at the CRC at the latest.
    mov.l   @(24,r8),r1
    mov     r1,r0
    tst     #3,r0
    bf      load_bounds
    cmp/hs  r2,r1
    bf      load_bounds
    cmp/hi  r12,r1
    bt      load_bounds
    mov.w   @(6,r8),r0
    extu.w  r0,r10
    mov     r10,r0
    shll2   r0
    add     r1,r0
    cmp/hi  r12,r0
    bt      load_bounds
    mov.l   r1,@(job_table,r13)
    mov.l   r10,@(job_imports,r13)
! Each import a service, read from the sectors: the image's byte o is byte o & 2047 of sector
! o >> 11. r1 runs through the table by its offset in the image.
    sub     r11,r1

load_import:
    tst     r10,r10
    bt      load_next
    mov     r1,r0
    shlr8   r0
    shlr2   r0
    shlr    r0
    shll2   r0
    add     #job_data,r0
    mov.l   @(r0,r13),r2
    mov.w   sector_less_one,r0
    and     r1,r0
    add     r0,r2
    mov.l   @r2,r2
    add     #-1,r2
    mov     #service_count,r0
    cmp/hs  r0,r2
    bt      load_unknown
    add     #4,r1
    bra     load_import
    add     #-1,r10

load_unknown:
    bra     load_fail
    mov     #5,r4

load_crc_bad:
    mov     #6,r4

! A failed load, r4 the reason: the result is the reason in its high byte.
load_fail:
    shll16  r4
    shll8   r4
    mov     #4,r5

load_publish:
    mov.l   load_publish_address,r0
    jmp     @r0
    nop

! The pass done: the next one comes with the loop's next call.
load_next:
    mov.l   load_job_address,r13
    mov.l   @(job_pass,r13),r0
    add     #1,r0
    mov.l   r0,@(job_pass,r13)
    mov.l   load_return_address,r0
    jmp     @r0
    nop

! Every later pass: pass n of the package's count checks the CRC of sector n - 1, and the pass
! after them installs the module.
load_pass:
    mov.l   load_job_address,r13
    mov.l   @(job_pass,r13),r0
    mov.l   @(job_count,r13),r1
    cmp/hi  r1,r0
    bt      load_install
! r8 the sector's data, r9 its bytes for the CRC: up to 2048, of those before the CRC that are left.
! The sizes come from the job, as the first pass checked them, not from the sectors again.
    add     #-1,r0
    shll2   r0
    add     #job_data,r0
    mov.l   @(r0,r13),r8
    mov.l   @(job_image,r13),r9
    add     #-4,r9
    mov.l   @(job_done,r13),r0
    sub     r0,r9
    mov.w   sector_bytes,r1
    cmp/hi  r1,r9
    bf      load_crc_counted
    mov     r1,r9

load_crc_counted:
    add     r9,r0
    mov.l   r0,@(job_done,r13)
    mov.l   @(job_crc,r13),r10
    mov.l   crc_table_address,r11

! CRC-32, reflected, four bits at a time from the table.
load_crc_byte:
    tst     r9,r9
    bt      load_crc_sector
    mov.b   @r8+,r0
    extu.b  r0,r0
    xor     r0,r10
    mov     r10,r0
    and     #15,r0
    shll2   r0
    mov.l   @(r0,r11),r1
    shlr2   r10
    shlr2   r10
    xor     r1,r10
    mov     r10,r0
    and     #15,r0
    shll2   r0
    mov.l   @(r0,r11),r1
    shlr2   r10
    shlr2   r10
    xor     r1,r10
    bra     load_crc_byte
    add     #-1,r9

! After the last sector's bytes r8 points at the CRC, the image's last longword.
load_crc_sector:
    mov.l   r10,@(job_crc,r13)
    mov.l   @(job_pass,r13),r0
    mov.l   @(job_count,r13),r1
    cmp/eq  r1,r0
    bf      load_next
    not     r10,r10
    mov.l   @r8,r0
    cmp/eq  r10,r0
    bf      load_crc_bad
    bra     load_next
    nop

! The install, while the job is its own: the module forgotten first, so that a stop from here on
! leaves none. The image is copied sector by sector, the BSS cleared and the imports bound, all by
! the sizes and addresses the first pass checked, so that nothing is written outside the slot even
! if the package's sectors have changed since.
load_install:
    mov.l   load_owned_address,r0
    jsr     @r0
    nop
    bf      load_unmask_return
    mov.l   load_module_address,r1
    mov     #0,r0
    mov.b   r0,@r1
    ldc     r3,sr
    mov.l   @(job_image,r13),r9
    mov.l   @(job_bss,r13),r10
    mov.l   load_slot_start,r11
    mov     r13,r12
    add     #job_data,r12
    mov.w   sector_bytes,r2

load_copy_sector:
    tst     r9,r9
    bt      load_clear
    mov.l   @r12+,r8
    mov     r9,r1
    cmp/hi  r2,r1
    bf      load_copy_counted
    mov     r2,r1

load_copy_counted:
    sub     r1,r9
    shlr2   r1

load_copy_long:
    mov.l   @r8+,r0
    mov.l   r0,@r11
    add     #4,r11
    add     #-1,r1
    tst     r1,r1
    bf      load_copy_long
    bra     load_copy_sector
    nop

load_clear:
    mov     #0,r0
    shlr2   r10

load_clear_long:
    tst     r10,r10
    bt      load_bind
    mov.l   r0,@r11
    add     #4,r11
    bra     load_clear_long
    add     #-1,r10

! Each import's service number replaced by the service's address, in the slot's copy.
load_bind:
    mov.l   @(job_table,r13),r1
    mov.l   @(job_imports,r13),r2
    mov.l   service_table_address,r3

load_bind_next:
    tst     r2,r2
    bt      load_installed
    mov.l   @r1,r0
    add     #-1,r0
    shll2   r0
    mov.l   @(r0,r3),r0
    mov.l   r0,@r1
    add     #4,r1
    bra     load_bind_next
    add     #-1,r2

! Installed while the job is still its own: the next handle and the entry, then the handle published.
! The handles run from 3 to 255 and start again at 3.
load_installed:
    mov.l   load_owned_address,r0
    jsr     @r0
    nop
    bf      load_unmask_return
    mov.l   load_module_address,r4
    mov.b   @(1,r4),r0
    extu.b  r0,r5
    mov     r5,r0
    mov.b   r0,@r4
    add     #1,r0
    extu.b  r0,r0
    tst     r0,r0
    bf      load_handle_next
    mov     #handle_first,r0

load_handle_next:
    mov.b   r0,@(1,r4)
    mov.l   @(job_entry,r13),r0
    mov.l   r0,@(4,r4)
    mov     r5,r4
    mov     #3,r5
    mov.l   load_publish_owned_address,r0
    jmp     @r0
    nop

load_unmask_return:
    mov.l   load_unmask_return_address,r0
    jmp     @r0
    nop

slot_size:
    .word   slot_bytes
sector_bytes:
    .word   2048
sector_less_one:
    .word   2047
    .balign 4
load_job_address:
    .long   resident_job
load_module_address:
    .long   resident_module
load_slot_start:
    .long   slot_start
module_magic:
    .long   0x444f534d
load_owned_address:
    .long   resident_owned
load_publish_address:
    .long   resident_publish
load_publish_owned_address:
    .long   resident_publish_owned
load_unmask_return_address:
    .long   resident_unmask_return
load_return_address:
    .long   resident_return
service_table_address:
    .long   service_table
crc_table_address:
    .long   crc_table

! Each service's address by its number, from 1.
service_table:
    .long   service_slot_at
    .long   service_sector_data
    .long   service_take
    .long   service_file
    .long   service_give_back

! CRC-32's reflected polynomial 0xedb88320 applied to each four-bit value.
crc_table:
    .long   0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c
    .long   0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c

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

! The module record: +0 the installed module's handle, 0 for none, +1 the next handle, +4 its entry.
resident_module:
    .byte   0
    .byte   handle_first
    .word   0
    .long   0

! The job, from the offsets above.
    .balign 4
resident_job:
    .fill   job_size,1,0

! r8..r13 and PR while a service runs, and its arguments across a lock acquire.
service_save_area:
    .fill   7,4,0
service_args:
    .fill   3,4,0

! The encoder takes whole eight-byte blocks.
    .balign 8
    .global resident_end
resident_end:
