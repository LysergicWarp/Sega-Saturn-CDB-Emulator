#pragma once
// SH-1 test programs encoded by hand: a 64 KB ROM image with the reset vectors set, a few
// instructions and a literal pool for 32-bit constants.
#include <cstdint>
#include <stdexcept>
#include <vector>

struct Program
{
    std::vector<uint8_t> rom = std::vector<uint8_t>(0x10000, 0);
    uint32_t at = 0x400;

    Program()
    {
        set_long(0, 0x400);      // reset PC
        set_long(4, 0x0f001000); // reset SP
    }

    void set_long(uint32_t address, uint32_t value)
    {
        for (unsigned i = 0; i < 4; ++i)
            rom[address + i] = uint8_t(value >> (24 - 8 * i));
    }
    void vector(unsigned number, uint32_t handler)
    {
        set_long(4 * number, handler);
    }
    void org(uint32_t address)
    {
        at = address;
    }
    uint32_t here() const
    {
        return at;
    }
    void op(uint16_t code)
    {
        rom[at] = uint8_t(code >> 8);
        rom[at + 1] = uint8_t(code);
        at += 2;
    }

    // MOV.L @(disp,PC),Rn with the constant placed by the next pool().
    void li(unsigned n, uint32_t value)
    {
        literals.push_back({at, value});
        op(uint16_t(0xd000 | n << 8));
    }
    void pool()
    {
        if (at & 2)
            nop();
        for (const auto &literal : literals)
        {
            const uint32_t base = (literal.instruction + 4) & ~3u, disp = (at - base) / 4;
            if (disp > 255)
                throw std::runtime_error("literal out of reach");
            rom[literal.instruction + 1] = uint8_t(disp);
            set_long(at, literal.value);
            at += 4;
        }
        literals.clear();
    }

    void mov(int imm, unsigned n)
    {
        op(uint16_t(0xe000 | n << 8 | (imm & 0xff)));
    }
    void add(int imm, unsigned n)
    {
        op(uint16_t(0x7000 | n << 8 | (imm & 0xff)));
    }
    void store_l(unsigned m, unsigned n)
    {
        op(uint16_t(0x2002 | n << 8 | m << 4));
    } // MOV.L Rm,@Rn
    void store_w(unsigned m, unsigned n)
    {
        op(uint16_t(0x2001 | n << 8 | m << 4));
    } // MOV.W Rm,@Rn
    void load_l(unsigned m, unsigned n)
    {
        op(uint16_t(0x6002 | n << 8 | m << 4));
    } // MOV.L @Rm,Rn
    void load_w(unsigned m, unsigned n)
    {
        op(uint16_t(0x6001 | n << 8 | m << 4));
    } // MOV.W @Rm,Rn
    void ldc_sr(unsigned m)
    {
        op(uint16_t(0x400e | m << 8));
    }
    void ldc_gbr(unsigned m)
    {
        op(uint16_t(0x401e | m << 8));
    }
    void lds_mach(unsigned m)
    {
        op(uint16_t(0x400a | m << 8));
    }
    void lds_macl(unsigned m)
    {
        op(uint16_t(0x401a | m << 8));
    }
    void sts_macl(unsigned n)
    {
        op(uint16_t(0x001a | n << 8));
    }
    void mac_w(unsigned m, unsigned n)
    {
        op(uint16_t(0x400f | n << 8 | m << 4));
    } // MAC.W @Rm+,@Rn+
    void mulu(unsigned m, unsigned n)
    {
        op(uint16_t(0x200e | n << 8 | m << 4));
    }
    void trapa(unsigned imm)
    {
        op(uint16_t(0xc300 | imm));
    }
    void nop()
    {
        op(0x0009);
    }
    void rte()
    {
        op(0x002b);
    }
    void sleep()
    {
        op(0x001b);
    }
    void bra(uint32_t target)
    {
        const int32_t disp = (int32_t(target) - int32_t(at + 4)) / 2;
        op(uint16_t(0xa000 | (disp & 0xfff)));
    }
    // A branch to itself with a NOP in its slot. Returns its address.
    uint32_t loop()
    {
        const uint32_t address = at;
        bra(address);
        nop();
        return address;
    }
    // Writes value to the register at address as a word, through r1 and r0.
    void poke_w(uint32_t address, uint32_t value)
    {
        li(1, address);
        li(0, value);
        store_w(0, 1);
    }

  private:
    struct Literal
    {
        uint32_t instruction, value;
    };
    std::vector<Literal> literals;
};
