#pragma once

#include <cstdint>
#include <span>

namespace atmos {

std::uint16_t crc16_ansi(std::span<const std::uint8_t> bytes,
    std::uint16_t initial = 0U) noexcept;

} // namespace atmos
