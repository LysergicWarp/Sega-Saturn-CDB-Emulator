#pragma once
// The SH-1 CPU core. The instruction handlers, their dispatch and the fetch/delay/execute loop in
// instructions.cpp derive from MAME mame0289's SH core. Around them this class supplies memory
// access through the board, the instruction boundary hook, and the SH7034's exception and
// interrupt acceptance rules (Hardware Manual sections 4 and 5).
#include <cstdint>

namespace cdblock
{

// What the core needs from the board it sits on.
class Sh1Port
{
  public:
    virtual ~Sh1Port() = default;
    virtual uint8_t read8(uint32_t address) = 0;
    virtual uint16_t read16(uint32_t address) = 0;
    virtual uint32_t read32(uint32_t address) = 0;
    virtual void write8(uint32_t address, uint8_t value) = 0;
    virtual void write16(uint32_t address, uint16_t value) = 0;
    virtual void write32(uint32_t address, uint32_t value) = 0;
    virtual uint16_t fetch(uint32_t address) = 0;
    // Each instruction boundary, before the fetch. Throws Sh1::Stop to end the run there.
    virtual void boundary(uint32_t pc) = 0;
    // The instruction about to execute.
    virtual void executing(uint32_t pc, uint16_t opcode, bool slot) = 0;
    // The interrupt controller's choice: the request's priority level (0 for none) and vector.
    virtual unsigned interrupt_request(unsigned &vector) = 0;
    virtual void interrupt_accepted(unsigned vector, unsigned level) = 0;
    // Exceptions other than interrupts: traps, illegal instructions and address errors.
    virtual void exception_entered(unsigned vector) = 0;
    // The address error to take at the boundary at pc, 9 (CPU) or 10 (DMA), 0 for none: one an
    // access raised since, or the fetch from pc itself. Taking it clears it.
    virtual unsigned address_error(uint32_t pc) = 0;
    // Whether the fetch from pc meets the user break condition, which then requests the interrupt.
    virtual bool fetch_break(uint32_t pc) = 0;
};

class Sh1
{
  public:
    // SLEEP repeats itself while waiting. An exception wakes it, and once the handler returns it
    // falls through to the next instruction.
    enum class Sleep : uint8_t
    {
        Awake,
        Waiting,
        Woken
    };
    struct State
    {
        uint32_t pc = 0, pr = 0, sr = 0, mach = 0, macl = 0, r[16] = {}, gbr = 0, vbr = 0;
        uint32_t ea = 0; // the handlers' scratch for an operand's address, as in MAME
        // The branch target while the next instruction is in its delay slot, else 0, as in MAME. A
        // delayed branch to 0 is not modeled: its slot runs and execution falls through.
        uint32_t delay = 0;
        Sleep sleep = Sleep::Awake;
    };
    // Thrown by the board's boundary hook to end a run before the instruction at that boundary.
    struct Stop
    {
    };

    // SR bits.
    static constexpr uint32_t SH_T = 0x00000001;
    static constexpr uint32_t SH_S = 0x00000002;
    static constexpr uint32_t SH_I = 0x000000f0;
    static constexpr uint32_t SH_Q = 0x00000100;
    static constexpr uint32_t SH_M = 0x00000200;
    static constexpr uint32_t SH_FLAGS = SH_M | SH_Q | SH_I | SH_S | SH_T;

    explicit Sh1(Sh1Port &owner) : port(&owner)
    {
    }
    // A copy runs on the board it was copied into, with its own state.
    void rebind(Sh1Port &owner)
    {
        port = &owner;
    }
    State state;

    // Power-on reset: PC and SP from vectors 0 and 1, SR.I at 15, VBR 0 (section 4.2).
    void reset();
    // Executes until the board's boundary hook throws Stop.
    void run();
    // The board changed an interrupt source: evaluate requests at the next boundary.
    void request_check()
    {
        test_irq = true;
    }
    // SLEEP waiting for an interrupt.
    bool sleeping() const
    {
        return state.sleep == Sleep::Waiting;
    }

  private:
    // Thrown at a boundary where an address error, a user break or an illegal instruction is taken
    // instead of the instruction there: the loop starts again at the handler.
    struct Redirect
    {
    };
    Sh1Port *port;
    // The opcode the boundary fetched, which the loop's own fetch then takes instead of reading
    // again while fetched is set.
    uint16_t fetched_opcode = 0;
    bool fetched = false;
    // The instruction this boundary fetched is LDC, STC, LDS or STS, so no interrupt is accepted at
    // the boundary after it (Table 4.9).
    bool holding = false;
    // An interrupt check that holding put off, made after the next instruction.
    bool recheck = false;
    bool test_irq = false; // interrupts to evaluate after the instruction, outside a delay slot

    void before_instruction(uint32_t pc);
    void check_interrupts();
    bool accept_interrupt();
    bool take_address_error();
    void enter_exception(unsigned vector, uint32_t pushed_pc);

    uint8_t read_byte(uint32_t address)
    {
        return port->read8(address);
    }
    uint16_t read_word(uint32_t address);
    uint32_t read_long(uint32_t address)
    {
        return port->read32(address);
    }
    void write_byte(uint32_t address, uint8_t value)
    {
        port->write8(address, value);
    }
    void write_word(uint32_t address, uint16_t value)
    {
        port->write16(address, value);
    }
    void write_long(uint32_t address, uint32_t value)
    {
        port->write32(address, value);
    }

    // Dispatch.
    void execute_one(uint16_t opcode);
    void execute_one_0000(uint16_t opcode);
    void op0010(uint16_t opcode);
    void op0011(uint16_t opcode);
    void execute_one_4000(uint16_t opcode);
    void op0110(uint16_t opcode);
    void op1000(uint16_t opcode);
    void op1100(uint16_t opcode);

    // Instructions.
    void ADD(uint32_t m, uint32_t n);
    void ADDI(uint32_t i, uint32_t n);
    void ADDC(uint32_t m, uint32_t n);
    void ADDV(uint32_t m, uint32_t n);
    void AND(uint32_t m, uint32_t n);
    void ANDI(uint32_t i);
    void ANDM(uint32_t i);
    void BF(uint32_t d);
    void BRA(uint32_t d);
    void BSR(uint32_t d);
    void BT(uint32_t d);
    void CLRMAC();
    void CLRT();
    void CMPEQ(uint32_t m, uint32_t n);
    void CMPGE(uint32_t m, uint32_t n);
    void CMPGT(uint32_t m, uint32_t n);
    void CMPHI(uint32_t m, uint32_t n);
    void CMPHS(uint32_t m, uint32_t n);
    void CMPPL(uint32_t n);
    void CMPPZ(uint32_t n);
    void CMPSTR(uint32_t m, uint32_t n);
    void CMPIM(uint32_t i);
    void DIV0S(uint32_t m, uint32_t n);
    void DIV0U();
    void DIV1(uint32_t m, uint32_t n);
    void EXTSB(uint32_t m, uint32_t n);
    void EXTSW(uint32_t m, uint32_t n);
    void EXTUB(uint32_t m, uint32_t n);
    void EXTUW(uint32_t m, uint32_t n);
    void JMP(uint32_t m);
    void JSR(uint32_t m);
    void LDCSR(uint16_t opcode);
    void LDCGBR(uint32_t m);
    void LDCVBR(uint32_t m);
    void LDCMSR(uint16_t opcode);
    void LDCMGBR(uint32_t m);
    void LDCMVBR(uint32_t m);
    void LDSMACH(uint32_t m);
    void LDSMACL(uint32_t m);
    void LDSPR(uint32_t m);
    void LDSMMACH(uint32_t m);
    void LDSMMACL(uint32_t m);
    void LDSMPR(uint32_t m);
    void MAC_W(uint32_t m, uint32_t n);
    void MOV(uint32_t m, uint32_t n);
    void MOVBS(uint32_t m, uint32_t n);
    void MOVWS(uint32_t m, uint32_t n);
    void MOVLS(uint32_t m, uint32_t n);
    void MOVBL(uint32_t m, uint32_t n);
    void MOVWL(uint32_t m, uint32_t n);
    void MOVLL(uint32_t m, uint32_t n);
    void MOVBM(uint32_t m, uint32_t n);
    void MOVWM(uint32_t m, uint32_t n);
    void MOVLM(uint32_t m, uint32_t n);
    void MOVBP(uint32_t m, uint32_t n);
    void MOVWP(uint32_t m, uint32_t n);
    void MOVLP(uint32_t m, uint32_t n);
    void MOVBS0(uint32_t m, uint32_t n);
    void MOVWS0(uint32_t m, uint32_t n);
    void MOVLS0(uint32_t m, uint32_t n);
    void MOVBL0(uint32_t m, uint32_t n);
    void MOVWL0(uint32_t m, uint32_t n);
    void MOVLL0(uint32_t m, uint32_t n);
    void MOVI(uint32_t i, uint32_t n);
    void MOVWI(uint32_t d, uint32_t n);
    void MOVLI(uint32_t d, uint32_t n);
    void MOVBLG(uint32_t d);
    void MOVWLG(uint32_t d);
    void MOVLLG(uint32_t d);
    void MOVBSG(uint32_t d);
    void MOVWSG(uint32_t d);
    void MOVLSG(uint32_t d);
    void MOVBS4(uint32_t d, uint32_t n);
    void MOVWS4(uint32_t d, uint32_t n);
    void MOVLS4(uint32_t m, uint32_t d, uint32_t n);
    void MOVBL4(uint32_t m, uint32_t d);
    void MOVWL4(uint32_t m, uint32_t d);
    void MOVLL4(uint32_t m, uint32_t d, uint32_t n);
    void MOVA(uint32_t d);
    void MOVT(uint32_t n);
    void MULS(uint32_t m, uint32_t n);
    void MULU(uint32_t m, uint32_t n);
    void NEG(uint32_t m, uint32_t n);
    void NEGC(uint32_t m, uint32_t n);
    void NOP();
    void NOT(uint32_t m, uint32_t n);
    void OR(uint32_t m, uint32_t n);
    void ORI(uint32_t i);
    void ORM(uint32_t i);
    void ROTCL(uint32_t n);
    void ROTCR(uint32_t n);
    void ROTL(uint32_t n);
    void ROTR(uint32_t n);
    void RTE();
    void RTS();
    void SETT();
    void SHAL(uint32_t n);
    void SHAR(uint32_t n);
    void SHLL(uint32_t n);
    void SHLL2(uint32_t n);
    void SHLL8(uint32_t n);
    void SHLL16(uint32_t n);
    void SHLR(uint32_t n);
    void SHLR2(uint32_t n);
    void SHLR8(uint32_t n);
    void SHLR16(uint32_t n);
    void SLEEP();
    void STCSR(uint32_t n);
    void STCGBR(uint32_t n);
    void STCVBR(uint32_t n);
    void STCMSR(uint32_t n);
    void STCMGBR(uint32_t n);
    void STCMVBR(uint32_t n);
    void STSMACH(uint32_t n);
    void STSMACL(uint32_t n);
    void STSPR(uint32_t n);
    void STSMMACH(uint32_t n);
    void STSMMACL(uint32_t n);
    void STSMPR(uint32_t n);
    void SUB(uint32_t m, uint32_t n);
    void SUBC(uint32_t m, uint32_t n);
    void SUBV(uint32_t m, uint32_t n);
    void SWAPB(uint32_t m, uint32_t n);
    void SWAPW(uint32_t m, uint32_t n);
    void TAS(uint32_t n);
    void TRAPA(uint32_t i);
    void TST(uint32_t m, uint32_t n);
    void TSTI(uint32_t i);
    void TSTM(uint32_t i);
    void XOR(uint32_t m, uint32_t n);
    void XORI(uint32_t i);
    void XORM(uint32_t i);
    void XTRCT(uint32_t m, uint32_t n);
};

}
