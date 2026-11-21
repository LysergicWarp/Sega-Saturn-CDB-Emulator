#pragma once
// A board's state as bytes. Each part lists its fields once, in exchange(), and a StateIo either
// writes them or reads them back, so a save and a load cannot differ in what they hold. Numbers
// have a fixed width with the least significant byte first, and the order is fixed, so the bytes
// are the same on every platform.
//
// A load trusts nothing it reads. A count is checked against the bytes left before anything is
// sized by it, and a part refuses a field outside its range, both with StateError. That keeps a
// made-up state from taking the board outside its memory and tables, and no more: the board goes
// on from what it was given.
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cdblock
{

// Why a state cannot be loaded.
struct StateError : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

// CRC-32 as zlib computes it.
uint32_t crc32(const uint8_t *bytes, size_t count);

class StateIo
{
  public:
    // Writes to out, or with none only counts the bytes.
    static StateIo writer(uint8_t *out)
    {
        return StateIo(out, nullptr, 0, false);
    }
    static StateIo reader(const uint8_t *in, size_t size)
    {
        return StateIo(nullptr, in, size, true);
    }
    // A save leaves every field as it is.
    bool loading() const
    {
        return load;
    }
    // The bytes written or read so far, and for a load those still to read.
    size_t position() const
    {
        return at;
    }
    size_t left() const
    {
        return size - at;
    }

    void u8(uint8_t &value);
    void u16(uint16_t &value);
    void u32(uint32_t &value);
    void u64(uint64_t &value);
    void flag(bool &value);
    void bytes(uint8_t *data, size_t count);
    void words(uint16_t *data, size_t count);
    template <size_t N> void bytes(std::array<uint8_t, N> &data)
    {
        bytes(data.data(), N);
    }
    // Bytes and text with their count.
    void bytes(std::vector<uint8_t> &data);
    void text(std::string &value);
    // A position of at most last, kept in 32 bits.
    void offset(size_t &value, size_t last, const char *what);
    // One of count values, from 0.
    template <class Enum> void choice(Enum &value, unsigned count, const char *what)
    {
        auto byte = uint8_t(value);
        u8(byte);
        require(byte < count, what);
        if (load)
            value = Enum(byte);
    }

    // The count of a sequence whose items take at least item_size bytes each. A load refuses one
    // the rest of the state cannot hold.
    size_t count(size_t current, size_t item_size);
    // A sequence: its count, then each item through item.
    template <class Sequence, class Item> void each(Sequence &sequence, size_t item_size, Item item)
    {
        const size_t loaded = count(sequence.size(), item_size);
        if (load)
            sequence.resize(loaded);
        for (auto &element : sequence)
            item(element);
    }
    // An ordered map: its count, then each key and value through item, written in key order.
    template <class Map, class Item> void map(Map &entries, size_t item_size, Item item)
    {
        const size_t loaded = count(entries.size(), item_size);
        if (!load)
        {
            for (auto &[key, value] : entries)
            {
                auto copy = key;
                item(copy, value);
            }
            return;
        }
        entries.clear();
        for (size_t i = 0; i < loaded; ++i)
        {
            typename Map::key_type key{};
            typename Map::mapped_type value{};
            item(key, value);
            // In key order and each key once, as a save writes them, so the loaded map saves the
            // same bytes.
            require(entries.empty() || entries.rbegin()->first < key, "order of entries");
            entries.emplace_hint(entries.end(), key, std::move(value));
        }
    }

    // A load refuses the state unless holds, naming what.
    void require(bool holds, const char *what) const
    {
        if (load && !holds)
            throw StateError(std::string("the state has an invalid ") + what);
    }
    // A value of what the state was saved from: written, or read and compared. A load refuses the
    // state of a board with another, which other names.
    void same(uint32_t mine, const char *other);

  private:
    StateIo(uint8_t *to, const uint8_t *from, size_t available, bool reading)
        : out(to), in(from), size(available), load(reading)
    {
    }
    uint8_t *out;
    const uint8_t *in;
    size_t size, at = 0;
    bool load;

    void put(const uint8_t *data, size_t count);
    void take(uint8_t *data, size_t count);
    void number(uint64_t &value, unsigned width);
};

}
