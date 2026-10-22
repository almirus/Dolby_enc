#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace atmos::eac3 {

constexpr std::size_t coefficient_count = 253U;
constexpr std::size_t critical_band_count = 50U;

enum class DeltaBitAllocationMode : std::uint8_t {
    None = 0,
    Reuse = 1,
    New = 2
};

struct DeltaBitAllocation final {
    DeltaBitAllocationMode mode = DeltaBitAllocationMode::None;
    std::uint8_t segment_count = 0;
    std::array<std::uint8_t, 8> offsets{};
    std::array<std::uint8_t, 8> lengths{};
    std::array<std::uint8_t, 8> values{};
};

struct BitAllocationParameters final {
    std::int16_t slow_decay = 0x13;
    std::int16_t fast_decay = 0x53;
    std::int16_t slow_gain = 0x4D8;
    std::int16_t db_per_bit = 0x900;
    std::int16_t floor = -2048;
    std::int16_t coupling_fast_leak = 0;
    std::int16_t coupling_slow_leak = 0;
    std::uint8_t sample_rate_code = 0;
    std::uint8_t sample_rate_shift = 0;
};

struct BitAllocationWorkspace final {
    std::array<std::int16_t, coefficient_count> psd{};
    std::array<std::int16_t, critical_band_count> band_psd{};
    std::array<std::int16_t, critical_band_count> mask{};
};

void calculate_psd(std::span<const std::int8_t> exponents,
    std::size_t start, std::size_t end, BitAllocationWorkspace& workspace);

void calculate_mask(const BitAllocationParameters& parameters,
    std::size_t start, std::size_t end, std::int16_t fast_gain, bool lfe,
    const DeltaBitAllocation& delta, BitAllocationWorkspace& workspace);

void calculate_bap(const BitAllocationParameters& parameters,
    std::size_t start, std::size_t end, std::int16_t snr_offset,
    const BitAllocationWorkspace& workspace, std::span<std::uint8_t> bap,
    bool adaptive_hybrid = false);

} // namespace atmos::eac3
