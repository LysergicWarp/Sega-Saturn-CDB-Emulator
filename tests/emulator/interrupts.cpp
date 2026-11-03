// Interrupt acceptance: the stack frame, IRQ6's two interface flags, the boundaries that hold a request,
// priority and SLEEP.
#include "test.h"

namespace
{

// IRQ6 at level 6 from the interface's command flag, with SR.I lowered to 0. The NOP after LDC
// keeps the hold of Table 4.9 off the stops that follow.
void irq6_setup(Program &p)
{
    p.poke_w(0x05ffff86, 0x0060); // IPRB: IRQ6 at 6
    p.poke_w(0x0a000008, 0x0001); // command flag enabled
    p.mov(0, 0);
    p.ldc_sr(0);
    p.nop();
}

void irq6_acceptance()
{
    Program p;
    irq6_setup(p);
    const uint32_t loop = p.loop();
    p.pool();
    const uint32_t handler = p.here();
    p.poke_w(0x0a000004, 0x0000); // clear the command flag
    const uint32_t back = p.here();
    p.rte();
    p.nop();
    p.pool();
    p.vector(70, handler);
    Board b(p);
    cdblock_add_stop(b, loop);
    cdblock_add_stop(b, back);
    CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == loop);
    CHECK(cdblock_host_command(b, command) == 0);
    CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == back);
    const auto c = b.cpu();
    CHECK(((c.sr >> 4) & 15) == 6 && c.r[15] == 0x0f001000 - 8);
    CHECK(b.peek(c.r[15], 4) == loop && b.peek(c.r[15] + 4, 4) == 0);
    CHECK(b.log(CDBLOCK_LOG_INTERRUPT).size() == 1 && b.log(CDBLOCK_LOG_INTERRUPT)[0].value == 70);
    // The handler cleared the flag, so the line is down after RTE.
    CHECK(b.resume() == CDBLOCK_EXIT_STOP && b.cpu().pc == loop);
    CHECK(b.log(CDBLOCK_LOG_INTERRUPT).size() == 1 && b.cpu().r[15] == 0x0f001000);
}

void irq6_on_response_read()
{
    // The response-read flag raises IRQ6 as the command flag does, when enabled alone: the host's
    // read of the fourth response word sets it, its reads of the first three do not.
    Program p;
    p.poke_w(0x05ffff86, 0x0060); // IPRB: IRQ6 at 6
    p.poke_w(0x0a000008, 0x0002); // response-read flag enabled
    p.mov(0, 0);
    p.ldc_sr(0);
    p.nop();
    const uint32_t loop = p.loop();
    p.pool();
    const uint32_t handler = p.here();
    p.poke_w(0x0a000004, 0x0000); // clear the flags
    const uint32_t back = p.here();
    p.rte();
    p.nop();
    p.pool();
    p.vector(70, handler);
    Board b(p);
    cdblock_add_stop(b, loop);
    cdblock_add_stop(b, back);
    CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == loop);
    uint16_t word = 0;
    for (unsigned i = 0; i < 3; ++i)
        CHECK(cdblock_host_response_word(b, i, &word) == 0);
    CHECK(b.resume() == CDBLOCK_EXIT_STOP && b.cpu().pc == loop && b.log(CDBLOCK_LOG_INTERRUPT).empty());
    CHECK(cdblock_host_response_word(b, 3, &word) == 0);
    CHECK(b.resume() == CDBLOCK_EXIT_STOP && b.cpu().pc == back);
    CHECK(b.log(CDBLOCK_LOG_INTERRUPT).size() == 1 && b.log(CDBLOCK_LOG_INTERRUPT)[0].value == 70);
    // The handler cleared the flag, so the line is down after RTE.
    CHECK(b.resume() == CDBLOCK_EXIT_STOP && b.cpu().pc == loop);
    // A command raises nothing with only the response-read flag enabled.
    CHECK(cdblock_host_command(b, command) == 0);
    CHECK(b.resume() == CDBLOCK_EXIT_STOP && b.cpu().pc == loop && b.log(CDBLOCK_LOG_INTERRUPT).size() == 1);
}

void delay_slot_holds_interrupts()
{
    Program p;
    irq6_setup(p);
    const uint32_t loop = p.loop();
    p.pool();
    const uint32_t handler = p.here();
    p.loop();
    p.vector(70, handler);
    Board b(p);
    cdblock_add_stop(b, loop);
    CHECK(b.run() == CDBLOCK_EXIT_STOP);
    // Execute the branch: the next boundary is its slot.
    CHECK(b.resume(1) == CDBLOCK_EXIT_BUDGET && b.cpu().delay == loop);
    cdblock_host_command(b, command);
    CHECK(b.resume(1) == CDBLOCK_EXIT_BUDGET);
    // The slot ran first, then the interrupt, with the branch target as its return address.
    CHECK(b.cpu().pc == handler && b.pushed_pc() == loop && b.cpu().delay == 0);
}

void ldc_holds_interrupts()
{
    for (bool ldc : {true, false})
    {
        Program p;
        irq6_setup(p);
        p.mov(0, 2);
        if (ldc)
            p.ldc_gbr(2);
        else
            p.nop();
        const uint32_t after = p.here();
        p.nop();
        p.loop();
        p.pool();
        const uint32_t handler = p.here();
        p.loop();
        p.vector(70, handler);
        Board b(p);
        cdblock_add_stop(b, after);
        CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == after);
        cdblock_host_command(b, command);
        CHECK(b.resume(1) == CDBLOCK_EXIT_BUDGET);
        // After LDC the boundary that follows accepts nothing (Table 4.9).
        CHECK(b.pushed_pc() == (ldc ? after + 2 : after));
    }
}

void interrupt_priority()
{
    Program p;
    p.poke_w(0x05ffff86, 0x0066); // IRQ6 and IRQ7 both at 6
    p.poke_w(0x0a000008, 0x0001);
    p.poke_w(0x0a00000a, 0x0010);
    p.mov(0, 0);
    p.ldc_sr(0);
    p.nop();
    const uint32_t loop = p.loop();
    p.pool();
    const uint32_t h6 = p.loop(), h7 = p.loop();
    p.vector(70, h6);
    p.vector(71, h7);
    for (uint32_t ipr : {0x0066u, 0x0067u})
    {
        Board b(p);
        cdblock_add_stop(b, loop);
        cdblock_add_stop(b, h6);
        cdblock_add_stop(b, h7);
        CHECK(b.run() == CDBLOCK_EXIT_STOP);
        b.write(0x05ffff86, 2, ipr);
        cdblock_host_command(b, command);
        cdblock_decoder_flags(b, 0x0010);
        CHECK(b.run() == CDBLOCK_EXIT_STOP);
        // Equal levels go by Table 5.3's order, IRQ6 first. A higher level wins outright.
        CHECK(b.cpu().pc == (ipr == 0x0066 ? h6 : h7));
    }
}

void sleep_until_interrupt()
{
    Program p;
    irq6_setup(p);
    const uint32_t sleeping = p.here();
    p.sleep();
    const uint32_t woken = p.loop();
    p.pool();
    const uint32_t handler = p.here();
    p.poke_w(0x0a000004, 0x0000);
    p.rte();
    p.nop();
    p.pool();
    p.vector(70, handler);
    Board b(p);
    cdblock_add_stop(b, woken);
    CHECK(b.run() == CDBLOCK_EXIT_SLEEP && b.cpu().pc == sleeping);
    const uint64_t count = cdblock_instructions(b);
    CHECK(b.run() == CDBLOCK_EXIT_SLEEP && cdblock_instructions(b) == count);
    cdblock_host_command(b, command);
    // The handler returns to SLEEP, which then falls through.
    CHECK(b.run() == CDBLOCK_EXIT_STOP && b.cpu().pc == woken);
}

}

const std::vector<Test> interrupt_tests = {
    {"IRQ6 acceptance", irq6_acceptance},
    {"IRQ6 on a response read", irq6_on_response_read},
    {"delay slot holds interrupts", delay_slot_holds_interrupts},
    {"LDC holds interrupts", ldc_holds_interrupts},
    {"interrupt priority", interrupt_priority},
    {"sleep until interrupt", sleep_until_interrupt},
};
