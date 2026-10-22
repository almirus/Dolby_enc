#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>

namespace atmos {

class ParseError final : public std::runtime_error {
public:
    explicit ParseError(const std::string& message) : std::runtime_error(message) {}
};

class BitReader final {
public:
    explicit BitReader(std::span<const std::uint8_t> data) : data_(data) {}

    [[nodiscard]] std::size_t position() const noexcept { return position_; }
    [[nodiscard]] std::size_t remaining() const noexcept {
        return data_.size() * 8U - position_;
    }

    std::uint32_t read(unsigned count) {
        if (count > 32U || remaining() < count) {
            throw ParseError("truncated bitstream");
        }
        std::uint32_t value = 0;
        for (unsigned i = 0; i < count; ++i) {
            const std::size_t byte = position_ >> 3U;
            const unsigned shift = 7U - static_cast<unsigned>(position_ & 7U);
            value = (value << 1U) | ((data_[byte] >> shift) & 1U);
            ++position_;
        }
        return value;
    }

    bool read_bit() { return read(1U) != 0U; }

    std::int32_t read_signed(unsigned count) {
        if (count == 0U || count > 32U) {
            throw ParseError("invalid signed bit width");
        }
        const std::uint32_t value = read(count);
        if (count == 32U) {
            return static_cast<std::int32_t>(value);
        }
        const std::uint32_t sign = 1U << (count - 1U);
        return static_cast<std::int32_t>((value ^ sign) - sign);
    }

    void skip(std::size_t count) {
        if (remaining() < count) {
            throw ParseError("truncated bitstream while skipping bits");
        }
        position_ += count;
    }

private:
    std::span<const std::uint8_t> data_;
    std::size_t position_ = 0;
};

} // namespace atmos
