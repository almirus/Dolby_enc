#include "bit_allocation.hpp"

#include "common/bit_reader.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace atmos::eac3 {
namespace {

constexpr std::array<std::uint8_t, critical_band_count + 1U> band_start = {
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
     10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
     20, 21, 22, 23, 24, 25, 26, 27, 28, 31,
     34, 37, 40, 43, 46, 49, 55, 61, 67, 73,
     79, 85, 97,109,121,133,157,181,205,229,253
};

constexpr std::array<std::array<std::int16_t, 3>, critical_band_count>
    hearing_threshold{{
    {0x04D0,0x04F0,0x0580}, {0x04D0,0x04F0,0x0580},
    {0x0440,0x0460,0x04B0}, {0x0400,0x0410,0x0450},
    {0x03E0,0x03E0,0x0420}, {0x03C0,0x03D0,0x03F0},
    {0x03B0,0x03C0,0x03E0}, {0x03B0,0x03B0,0x03D0},
    {0x03A0,0x03B0,0x03C0}, {0x03A0,0x03A0,0x03B0},
    {0x03A0,0x03A0,0x03B0}, {0x03A0,0x03A0,0x03B0},
    {0x03A0,0x03A0,0x03A0}, {0x0390,0x03A0,0x03A0},
    {0x0390,0x0390,0x03A0}, {0x0390,0x0390,0x03A0},
    {0x0380,0x0390,0x03A0}, {0x0380,0x0380,0x03A0},
    {0x0370,0x0380,0x03A0}, {0x0370,0x0380,0x03A0},
    {0x0360,0x0370,0x0390}, {0x0360,0x0370,0x0390},
    {0x0350,0x0360,0x0390}, {0x0350,0x0360,0x0390},
    {0x0340,0x0350,0x0380}, {0x0340,0x0350,0x0380},
    {0x0330,0x0340,0x0380}, {0x0320,0x0340,0x0370},
    {0x0310,0x0320,0x0360}, {0x0300,0x0310,0x0350},
    {0x02F0,0x0300,0x0340}, {0x02F0,0x02F0,0x0330},
    {0x02F0,0x02F0,0x0320}, {0x02F0,0x02F0,0x0310},
    {0x0300,0x02F0,0x0300}, {0x0310,0x0300,0x02F0},
    {0x0340,0x0320,0x02F0}, {0x0390,0x0350,0x02F0},
    {0x03E0,0x0390,0x0300}, {0x0420,0x03E0,0x0310},
    {0x0460,0x0420,0x0330}, {0x0490,0x0450,0x0350},
    {0x04A0,0x04A0,0x03C0}, {0x0460,0x0490,0x0410},
    {0x0440,0x0460,0x0470}, {0x0440,0x0440,0x04A0},
    {0x0520,0x0480,0x0460}, {0x0800,0x0630,0x0440},
    {0x0840,0x0840,0x0450}, {0x0840,0x0840,0x04E0}
}};

constexpr std::array<std::uint8_t, 64> bap_table = {
    0,1,1,1,1,1,2,2,3,3,3,4,4,5,5,6,
    6,6,6,7,7,7,7,8,8,8,8,9,9,9,9,10,
    10,10,10,11,11,11,11,12,12,12,12,13,13,13,13,14,
    14,14,14,14,14,14,14,15,15,15,15,15,15,15,15,15
};

std::size_t band_for_bin(std::size_t bin) {
    const auto next = std::upper_bound(band_start.begin(), band_start.end(), bin);
    return static_cast<std::size_t>(next - band_start.begin() - 1);
}

int logarithmic_add(unsigned address) {
    address = std::min(address, 255U);
    int value = static_cast<int>(64.0 * std::log2(
        1.0 + std::pow(2.0, -static_cast<double>(address) / 32.0)));
    // Values rounded upward by the normative AC-3 log-add table.
    constexpr std::array<unsigned, 19> rounded_up = {
        13U, 19U, 23U, 27U, 36U, 41U, 48U, 50U, 52U, 54U,
        71U, 74U, 77U, 111U, 125U, 144U, 158U, 177U, 209U
    };
    if (std::binary_search(rounded_up.begin(), rounded_up.end(), address)) ++value;
    return value;
}

int low_compensation_step(int value, int first, int second, int reset) {
    if (first + 256 == second) return reset;
    if (first > second) return std::max(value - 64, 0);
    return value;
}

int low_compensation(int value, int first, int second, std::size_t band) {
    if (band < 7U) return low_compensation_step(value, first, second, 384);
    if (band < 20U) return low_compensation_step(value, first, second, 320);
    return std::max(value - 128, 0);
}

} // namespace

void calculate_psd(std::span<const std::int8_t> exponents,
    std::size_t start, std::size_t end, BitAllocationWorkspace& workspace) {
    if (start >= end || end > coefficient_count || exponents.size() < end) {
        throw ParseError("invalid bit allocation frequency range");
    }
    for (std::size_t bin = start; bin < end; ++bin) {
        if (exponents[bin] < 0 || exponents[bin] > 24) {
            throw ParseError("bit allocation exponent out of range");
        }
        workspace.psd[bin] = static_cast<std::int16_t>(
            3072 - static_cast<int>(exponents[bin]) * 128);
    }
    std::size_t bin = start;
    std::size_t band = band_for_bin(start);
    do {
        int integrated = workspace.psd[bin++];
        const std::size_t end_of_band = std::min<std::size_t>(band_start[band + 1U], end);
        while (bin < end_of_band) {
            const int next = workspace.psd[bin++];
            const int maximum = std::max(integrated, next);
            const unsigned address = static_cast<unsigned>(std::min(
                maximum - ((integrated + next + 1) >> 1), 255));
            integrated = maximum + logarithmic_add(address);
        }
        workspace.band_psd[band++] = static_cast<std::int16_t>(integrated);
    } while (band < critical_band_count && end > band_start[band]);
}

void calculate_mask(const BitAllocationParameters& parameters,
    std::size_t start, std::size_t end, std::int16_t fast_gain, bool lfe,
    const DeltaBitAllocation& delta, BitAllocationWorkspace& workspace) {
    if (start >= end || end > coefficient_count || parameters.sample_rate_code > 2U ||
        parameters.sample_rate_shift > 1U) {
        throw ParseError("invalid masking parameters");
    }
    std::array<std::int16_t, critical_band_count> excitation{};
    const std::size_t first_band = band_for_bin(start);
    const std::size_t final_band = band_for_bin(end - 1U) + 1U;
    std::size_t begin = first_band;
    int fast_leak = 0;
    int slow_leak = 0;
    if (first_band == 0U) {
        int low_comp = 0;
        low_comp = low_compensation_step(low_comp,
            workspace.band_psd[0], workspace.band_psd[1], 384);
        excitation[0] = static_cast<std::int16_t>(workspace.band_psd[0] - fast_gain - low_comp);
        low_comp = low_compensation_step(low_comp,
            workspace.band_psd[1], workspace.band_psd[2], 384);
        excitation[1] = static_cast<std::int16_t>(workspace.band_psd[1] - fast_gain - low_comp);
        begin = 7U;
        for (std::size_t band = 2U; band < 7U; ++band) {
            if (!(lfe && band == 6U)) {
                low_comp = low_compensation_step(low_comp,
                    workspace.band_psd[band], workspace.band_psd[band + 1U], 384);
            }
            fast_leak = workspace.band_psd[band] - fast_gain;
            slow_leak = workspace.band_psd[band] - parameters.slow_gain;
            excitation[band] = static_cast<std::int16_t>(fast_leak - low_comp);
            if (!(lfe && band == 6U) &&
                workspace.band_psd[band] <= workspace.band_psd[band + 1U]) {
                begin = band + 1U;
                break;
            }
        }
        const std::size_t low_end = std::min<std::size_t>(final_band, 22U);
        for (std::size_t band = begin; band < low_end; ++band) {
            if (!(lfe && band == 6U)) {
                low_comp = low_compensation(low_comp,
                    workspace.band_psd[band], workspace.band_psd[band + 1U], band);
            }
            fast_leak = std::max(fast_leak - parameters.fast_decay,
                workspace.band_psd[band] - fast_gain);
            slow_leak = std::max(slow_leak - parameters.slow_decay,
                workspace.band_psd[band] - parameters.slow_gain);
            excitation[band] = static_cast<std::int16_t>(
                std::max(fast_leak - low_comp, slow_leak));
        }
        begin = 22U;
    } else {
        fast_leak = parameters.coupling_fast_leak * 256 + 768;
        slow_leak = parameters.coupling_slow_leak * 256 + 768;
    }
    for (std::size_t band = begin; band < final_band; ++band) {
        fast_leak = std::max(fast_leak - parameters.fast_decay,
            workspace.band_psd[band] - fast_gain);
        slow_leak = std::max(slow_leak - parameters.slow_decay,
            workspace.band_psd[band] - parameters.slow_gain);
        excitation[band] = static_cast<std::int16_t>(std::max(fast_leak, slow_leak));
    }
    for (std::size_t band = first_band; band < final_band; ++band) {
        int value = excitation[band];
        const int compensation = parameters.db_per_bit - workspace.band_psd[band];
        if (compensation > 0) value += compensation >> 2;
        const std::size_t threshold_band = band >> parameters.sample_rate_shift;
        if (threshold_band >= critical_band_count) {
            throw ParseError("hearing threshold band out of range");
        }
        workspace.mask[band] = static_cast<std::int16_t>(std::max<int>(
            hearing_threshold[threshold_band][parameters.sample_rate_code], value));
    }
    if (delta.mode == DeltaBitAllocationMode::Reuse ||
        delta.mode == DeltaBitAllocationMode::New) {
        if (delta.segment_count > delta.offsets.size()) {
            throw ParseError("too many delta bit allocation segments");
        }
        std::size_t band = first_band;
        for (std::size_t segment = 0; segment < delta.segment_count; ++segment) {
            band += delta.offsets[segment];
            if (band >= critical_band_count ||
                delta.lengths[segment] > critical_band_count - band) {
                throw ParseError("delta bit allocation segment out of range");
            }
            const int code = delta.values[segment];
            const int adjustment = (code >= 4 ? code - 3 : code - 4) * 128;
            for (unsigned i = 0; i < delta.lengths[segment]; ++i) {
                workspace.mask[band] = static_cast<std::int16_t>(
                    workspace.mask[band] + adjustment);
                ++band;
            }
        }
    }
}

void calculate_bap(const BitAllocationParameters& parameters,
    std::size_t start, std::size_t end, std::int16_t snr_offset,
    const BitAllocationWorkspace& workspace, std::span<std::uint8_t> bap,
    bool adaptive_hybrid) {
    if (start >= end || end > coefficient_count || bap.size() < end) {
        throw ParseError("invalid BAP output range");
    }
    if (snr_offset == -960) {
        std::fill(bap.begin() + static_cast<std::ptrdiff_t>(start),
            bap.begin() + static_cast<std::ptrdiff_t>(end), std::uint8_t{0});
        return;
    }
    std::size_t bin = start;
    std::size_t band = band_for_bin(start);
    do {
        const int mask = (std::max<int>(workspace.mask[band] - snr_offset -
            parameters.floor, 0) & 0x1FE0) + parameters.floor;
        const std::size_t end_of_band = std::min<std::size_t>(band_start[++band], end);
        while (bin < end_of_band) {
            const int raw = (workspace.psd[bin] - mask) >> 5;
            const unsigned address = static_cast<unsigned>(std::clamp(raw, 0, 63));
            static constexpr std::array<std::uint8_t, 64> hybrid_bap_table = {
                0,1,2,3,4,5,6,7,8,8,8,8,9,9,9,10,
                10,10,10,11,11,11,11,12,12,12,12,13,13,13,13,14,
                14,14,14,15,15,15,15,16,16,16,16,17,17,17,17,18,
                18,18,18,18,18,18,18,19,19,19,19,19,19,19,19,19
            };
            bap[bin++] = adaptive_hybrid ? hybrid_bap_table[address] : bap_table[address];
        }
    } while (end > bin);
}

} // namespace atmos::eac3
