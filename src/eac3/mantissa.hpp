#pragma once

#include <array>
#include <cstdint>
#include <span>

namespace atmos {
class BitReader;
}

namespace atmos::eac3 {

class DitherGenerator final {
public:
    DitherGenerator() noexcept { reset(); }
    void reset() noexcept;
    [[nodiscard]] std::uint32_t next() noexcept;

private:
    std::array<std::uint32_t, 64> state_{};
    std::uint32_t index_ = 0;
};

class MantissaDecoder final {
public:
    explicit MantissaDecoder(DitherGenerator& dither) noexcept : dither_(dither) {}
    void reset_groups() noexcept;
    void decode(BitReader& bits, std::span<const std::uint8_t> bit_allocation,
        std::span<const std::int8_t> exponents,
        std::span<std::int32_t> coefficients, bool dither);

private:
    std::int32_t decode_one(BitReader& bits, std::uint8_t bap, bool dither);
    std::array<std::int32_t, 2> bap1_group_{};
    std::array<std::int32_t, 2> bap2_group_{};
    std::int32_t bap4_group_ = 0;
    std::uint8_t bap1_left_ = 0;
    std::uint8_t bap2_left_ = 0;
    bool bap4_left_ = false;
    DitherGenerator& dither_;
};

} // namespace atmos::eac3
