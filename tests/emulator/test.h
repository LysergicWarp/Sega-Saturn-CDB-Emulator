#pragma once
// Shared by the test files: the check macro, a board wrapper and each file's table of tests.
#include "cdblock.h"
#include "program.h"

#include <cstdio>
#include <string>
#include <vector>

extern int failures, assertions;

#define CHECK(condition)                                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        ++assertions;                                                                                                  \
        if (!(condition))                                                                                              \
        {                                                                                                              \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                                         \
            ++failures;                                                                                                \
        }                                                                                                              \
    } while (0)

struct Board
{
    cdblock_board *b = nullptr;
    explicit Board(const Program &p, const cdblock_options *options = nullptr)
    {
        b = cdblock_create(p.rom.data(), p.rom.size(), options);
        if (!b)
            std::printf("  create failed: %s\n", cdblock_last_error(nullptr));
    }
    // Takes over a board the library made, a clone.
    explicit Board(cdblock_board *board) : b(board)
    {
    }
    ~Board()
    {
        cdblock_destroy(b);
    }
    Board(const Board &) = delete;
    Board &operator=(const Board &) = delete;
    operator cdblock_board *() const
    {
        return b;
    }

    cdblock_cpu cpu() const
    {
        cdblock_cpu c;
        cdblock_get_cpu(b, &c);
        return c;
    }
    int run(uint64_t budget = 100000) const
    {
        return cdblock_run(b, budget, nullptr);
    }
    int resume(uint64_t budget = 100000) const
    {
        return cdblock_run_from_stop(b, budget, nullptr);
    }
    uint32_t read(uint32_t address, unsigned size) const
    {
        uint32_t value = 0;
        cdblock_bus_read(b, address, size, &value);
        return value;
    }
    void write(uint32_t address, unsigned size, uint32_t value) const
    {
        cdblock_bus_write(b, address, size, value);
    }
    uint32_t peek(uint32_t address, unsigned size) const
    {
        uint8_t bytes[4] = {};
        cdblock_peek(b, address, bytes, size);
        uint32_t value = 0;
        for (unsigned i = 0; i < size; ++i)
            value = value << 8 | bytes[i];
        return value;
    }
    std::vector<cdblock_log_entry> log(uint32_t kind) const
    {
        std::vector<cdblock_log_entry> out;
        for (size_t i = 0; i < cdblock_log_count(b); ++i)
        {
            cdblock_log_entry e;
            cdblock_log_get(b, i, &e);
            if (e.kind == kind)
                out.push_back(e);
        }
        return out;
    }
    std::string error() const
    {
        return cdblock_last_error(b);
    }
    // The word pushed at the top of the stack, the return PC of the last exception.
    uint32_t pushed_pc() const
    {
        return peek(cpu().r[15], 4);
    }
};

inline bool contains(const std::string &text, const char *part)
{
    return text.find(part) != std::string::npos;
}

// A host command for the tests that only need the interface's command flag raised.
inline constexpr uint16_t command[4] = {0x0100, 0, 0, 0};

struct Test
{
    const char *name;
    void (*run)();
};

extern const std::vector<Test> cpu_tests, interrupt_tests, exception_tests, sh7034_tests, serial_tests, board_tests,
    timing_tests, drive_tests, api_tests;
