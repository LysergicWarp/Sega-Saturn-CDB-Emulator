#include "state.h"

#include <cstring>

namespace cdblock
{

namespace
{

// CRC-32's tables for eight bytes a step: table k carries a byte k places further through the
// polynomial division than table 0 does.
constexpr auto crc_tables = [] {
    std::array<std::array<uint32_t, 256>, 8> tables{};
    for (uint32_t i = 0; i < 256; ++i)
    {
        uint32_t crc = i;
        for (int bit = 0; bit < 8; ++bit)
            crc = crc & 1 ? crc >> 1 ^ 0xedb88320u : crc >> 1;
        tables[0][i] = crc;
    }
    for (size_t k = 1; k < tables.size(); ++k)
        for (size_t i = 0; i < 256; ++i)
            tables[k][i] = tables[k - 1][i] >> 8 ^ tables[0][tables[k - 1][i] & 0xff];
    return tables;
}();

}

uint32_t crc32(const uint8_t *bytes, size_t count)
{
    const auto &t = crc_tables;
    uint32_t crc = 0xffffffff;
    for (; count >= 8; count -= 8, bytes += 8)
    {
        const uint32_t low =
            crc ^ (uint32_t(bytes[0]) | uint32_t(bytes[1]) << 8 | uint32_t(bytes[2]) << 16 | uint32_t(bytes[3]) << 24);
        crc = t[7][low & 0xff] ^ t[6][low >> 8 & 0xff] ^ t[5][low >> 16 & 0xff] ^ t[4][low >> 24] ^ t[3][bytes[4]] ^
              t[2][bytes[5]] ^ t[1][bytes[6]] ^ t[0][bytes[7]];
    }
    for (; count; --count, ++bytes)
        crc = crc >> 8 ^ t[0][(crc ^ *bytes) & 0xff];
    return ~crc;
}

void StateIo::put(const uint8_t *data, size_t count)
{
    if (out && count)
        std::memcpy(out + at, data, count);
    at += count;
}

void StateIo::take(uint8_t *data, size_t count)
{
    if (count > left())
        throw StateError("the state ends before its last field");
    if (count)
        std::memcpy(data, in + at, count);
    at += count;
}

void StateIo::number(uint64_t &value, unsigned width)
{
    uint8_t raw[8] = {};
    if (load)
    {
        take(raw, width);
        value = 0;
        for (unsigned i = width; i-- > 0;)
            value = value << 8 | raw[i];
    }
    else
    {
        for (unsigned i = 0; i < width; ++i)
            raw[i] = uint8_t(value >> (8 * i));
        put(raw, width);
    }
}

void StateIo::u8(uint8_t &value)
{
    uint64_t wide = value;
    number(wide, 1);
    if (load)
        value = uint8_t(wide);
}

void StateIo::u16(uint16_t &value)
{
    uint64_t wide = value;
    number(wide, 2);
    if (load)
        value = uint16_t(wide);
}

void StateIo::u32(uint32_t &value)
{
    uint64_t wide = value;
    number(wide, 4);
    if (load)
        value = uint32_t(wide);
}

void StateIo::u64(uint64_t &value)
{
    number(value, 8);
}

void StateIo::flag(bool &value)
{
    uint8_t byte = value ? 1 : 0;
    u8(byte);
    require(byte <= 1, "flag");
    if (load)
        value = byte != 0;
}

void StateIo::bytes(uint8_t *data, size_t count)
{
    if (load)
        take(data, count);
    else
        put(data, count);
}

void StateIo::words(uint16_t *data, size_t count)
{
    if (load)
    {
        if (count > left() / 2)
            throw StateError("the state ends before its last field");
        const uint8_t *from = in + at;
        for (size_t i = 0; i < count; ++i)
            data[i] = uint16_t(from[2 * i] | from[2 * i + 1] << 8);
    }
    else if (out)
    {
        uint8_t *to = out + at;
        for (size_t i = 0; i < count; ++i)
        {
            to[2 * i] = uint8_t(data[i]);
            to[2 * i + 1] = uint8_t(data[i] >> 8);
        }
    }
    at += 2 * count;
}

void StateIo::bytes(std::vector<uint8_t> &data)
{
    const size_t loaded = count(data.size(), 1);
    if (load)
        data.resize(loaded);
    bytes(data.data(), data.size());
}

void StateIo::text(std::string &value)
{
    const size_t loaded = count(value.size(), 1);
    if (load)
        value.resize(loaded);
    for (char &character : value)
    {
        auto byte = uint8_t(character);
        u8(byte);
        if (load)
            character = char(byte);
    }
}

void StateIo::offset(size_t &value, size_t last, const char *what)
{
    auto narrow = uint32_t(value);
    u32(narrow);
    require(narrow <= last, what);
    if (load)
        value = narrow;
}

size_t StateIo::count(size_t current, size_t item_size)
{
    auto items = uint32_t(current);
    u32(items);
    if (load && items > left() / item_size)
        throw StateError("the state has a count larger than the state");
    return items;
}

void StateIo::same(uint32_t mine, const char *other)
{
    uint32_t value = mine;
    u32(value);
    if (value != mine)
        throw StateError(std::string("the state is of a board with ") + other);
}

}
