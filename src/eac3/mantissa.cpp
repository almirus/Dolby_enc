#include "mantissa.hpp"

#include "common/bit_reader.hpp"

#include <array>

namespace atmos::eac3 {
namespace {

constexpr std::array<std::uint32_t, 64> ffmpeg_lfg_seed_zero = {
    0x00000000U,0x00000000U,0x00000000U,0x00000000U,
    0x00000000U,0x00000000U,0x00000000U,0x00000000U,
    0x3B376B5EU,0xF56BE763U,0xCBA2A7FCU,0x9751B2C7U,
    0xFB0663DAU,0xC93C5120U,0x4CE49EBDU,0xC007A223U,
    0x378C8CDFU,0x2541B628U,0xE77D6BFBU,0x7E5DD113U,
    0x97838D2BU,0xB07A7700U,0xE375BF71U,0xF8F65272U,
    0x6EB97DB1U,0x30B2367EU,0xA8D3DF2AU,0xBF52464FU,
    0x3688C130U,0xF4760AE7U,0x47553A63U,0xF2CC2686U,
    0x13B5F6B2U,0x2163FC9BU,0x0B30CF93U,0x7DC16C37U,
    0x8AFF33E3U,0xF23C1A02U,0xCDFC8D83U,0xEDA3244AU,
    0xD52E81A6U,0xD8E9D76CU,0xA479C41FU,0xEE5E6611U,
    0xA851D875U,0x6D1F2709U,0x2E54BD86U,0x053DCBD1U,
    0x98CC2C41U,0xF9390099U,0x7764853BU,0x0275E470U,
    0xA4471CCAU,0x05DD687DU,0x4138F04FU,0x56823E49U,
    0xEEC155ECU,0xA7B31734U,0x5687C108U,0x70FDDAA9U,
    0x26FEEEE7U,0x6323287DU,0x8EF89ECCU,0xE1ECE72EU
};

constexpr std::array<unsigned, 16> quantizer_bits = {
    0U, 3U, 5U, 7U, 11U, 15U, 5U, 6U,
    7U, 8U, 9U, 10U, 11U, 12U, 14U, 16U
};

std::int32_t symmetric_dequant(unsigned code, unsigned levels) {
    return static_cast<std::int32_t>(
        (static_cast<std::int64_t>(static_cast<int>(code) -
            static_cast<int>(levels / 2U)) * (1LL << 24U)) /
        static_cast<std::int64_t>(levels));
}

} // namespace

void DitherGenerator::reset() noexcept {
    state_ = ffmpeg_lfg_seed_zero;
    index_ = 0;
}

std::uint32_t DitherGenerator::next() noexcept {
    const std::uint32_t value = state_[index_ & 63U] =
        state_[(index_ - 24U) & 63U] + state_[(index_ - 55U) & 63U];
    ++index_;
    return value;
}

void MantissaDecoder::reset_groups() noexcept {
    bap1_left_ = 0;
    bap2_left_ = 0;
    bap4_left_ = false;
}

std::int32_t MantissaDecoder::decode_one(
    BitReader& bits, std::uint8_t bap, bool dither) {
    if (bap > 15U) throw ParseError("bit allocation pointer out of range");
    if (bap == 0U) {
        if (!dither) return 0;
        const std::uint32_t random = dither_.next();
        return static_cast<std::int32_t>((((random >> 8U) * 181U) >> 8U) - 5931008U);
    }
    if (bap == 1U) {
        if (bap1_left_ != 0U) return bap1_group_[--bap1_left_];
        const unsigned code = bits.read(5U);
        if (code >= 27U) throw ParseError("invalid bap 1 mantissa group");
        bap1_group_[1] = symmetric_dequant((code % 9U) / 3U, 3U);
        bap1_group_[0] = symmetric_dequant(code % 3U, 3U);
        bap1_left_ = 2U;
        return symmetric_dequant(code / 9U, 3U);
    }
    if (bap == 2U) {
        if (bap2_left_ != 0U) return bap2_group_[--bap2_left_];
        const unsigned code = bits.read(7U);
        if (code >= 125U) throw ParseError("invalid bap 2 mantissa group");
        bap2_group_[1] = symmetric_dequant((code % 25U) / 5U, 5U);
        bap2_group_[0] = symmetric_dequant(code % 5U, 5U);
        bap2_left_ = 2U;
        return symmetric_dequant(code / 25U, 5U);
    }
    if (bap == 3U) {
        const unsigned code = bits.read(3U);
        if (code >= 7U) throw ParseError("invalid bap 3 mantissa");
        return symmetric_dequant(code, 7U);
    }
    if (bap == 4U) {
        if (bap4_left_) {
            bap4_left_ = false;
            return bap4_group_;
        }
        const unsigned code = bits.read(7U);
        if (code >= 121U) throw ParseError("invalid bap 4 mantissa group");
        bap4_group_ = symmetric_dequant(code % 11U, 11U);
        bap4_left_ = true;
        return symmetric_dequant(code / 11U, 11U);
    }
    if (bap == 5U) {
        const unsigned code = bits.read(4U);
        if (code >= 15U) throw ParseError("invalid bap 5 mantissa");
        return symmetric_dequant(code, 15U);
    }
    const unsigned width = quantizer_bits[bap];
    return static_cast<std::int32_t>(
        static_cast<std::int64_t>(bits.read_signed(width)) *
        (1LL << (24U - width)));
}

void MantissaDecoder::decode(BitReader& bits,
    std::span<const std::uint8_t> bit_allocation,
    std::span<const std::int8_t> exponents,
    std::span<std::int32_t> coefficients, bool dither) {
    if (exponents.size() < bit_allocation.size() ||
        coefficients.size() < bit_allocation.size()) {
        throw ParseError("mantissa buffer is too small");
    }
    for (std::size_t bin = 0; bin < bit_allocation.size(); ++bin) {
        if (exponents[bin] < 0 || exponents[bin] > 24) {
            throw ParseError("mantissa exponent out of range");
        }
        coefficients[bin] = decode_one(bits, bit_allocation[bin], dither) /
            static_cast<std::int32_t>(1U << static_cast<unsigned>(exponents[bin]));
    }
}

} // namespace atmos::eac3
