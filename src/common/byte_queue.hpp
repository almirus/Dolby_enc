#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace atmos {

class ByteQueue final {
public:
    void append(std::span<const std::uint8_t> input) {
        compact();
        bytes_.insert(bytes_.end(), input.begin(), input.end());
    }

    [[nodiscard]] std::span<const std::uint8_t> readable() const noexcept {
        return std::span<const std::uint8_t>(bytes_).subspan(position_);
    }

    void consume(std::size_t count) noexcept {
        position_ += std::min(count, bytes_.size() - position_);
    }

    void clear() noexcept {
        bytes_.clear();
        position_ = 0;
    }

private:
    void compact() {
        if (position_ == 0) {
            return;
        }
        bytes_.erase(bytes_.begin(), bytes_.begin() + static_cast<std::ptrdiff_t>(position_));
        position_ = 0;
    }

    std::vector<std::uint8_t> bytes_;
    std::size_t position_ = 0;
};

} // namespace atmos
