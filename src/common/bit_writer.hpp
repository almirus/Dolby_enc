#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace atmos {

// MSB-first counterpart of BitReader. EVO metadata is serialized in this
// order, including fields that are not byte aligned.
class BitWriter final {
public:
    void write(std::uint32_t value, unsigned count) {
        if (count > 32U) return;
        for (unsigned bit = 0; bit < count; ++bit) {
            if ((position_ & 7U) == 0U) data_.push_back(0U);
            const unsigned shift = count - bit - 1U;
            if (((value >> shift) & 1U) != 0U) {
                data_.back() |= static_cast<std::uint8_t>(
                    1U << (7U - static_cast<unsigned>(position_ & 7U)));
            }
            ++position_;
        }
    }

    void write_bit(bool value) { write(value ? 1U : 0U, 1U); }

    void align() {
        if ((position_ & 7U) != 0U) {
            position_ = (position_ + 7U) & ~std::size_t(7U);
        }
    }

    [[nodiscard]] std::size_t position() const noexcept { return position_; }
    [[nodiscard]] const std::vector<std::uint8_t>& bytes() const noexcept {
        return data_;
    }
    [[nodiscard]] std::vector<std::uint8_t> take_bytes() && {
        return std::move(data_);
    }

private:
    std::vector<std::uint8_t> data_;
    std::size_t position_ = 0;
};

} // namespace atmos
