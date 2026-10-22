#include "crc.hpp"

namespace atmos {

std::uint16_t crc16_ansi(std::span<const std::uint8_t> bytes,
    std::uint16_t initial) noexcept {
    std::uint16_t crc = initial;
    for (const std::uint8_t byte : bytes) {
        crc ^= static_cast<std::uint16_t>(byte) << 8U;
        for (unsigned bit = 0; bit < 8U; ++bit) {
            crc = static_cast<std::uint16_t>((crc << 1U) ^
                ((crc & 0x8000U) != 0U ? 0x8005U : 0U));
        }
    }
    return crc;
}

} // namespace atmos
