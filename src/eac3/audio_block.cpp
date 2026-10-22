#include "audio_block.hpp"

#include "aht_tables.hpp"
#include "common/bit_reader.hpp"
#include "exponents.hpp"
#include "mantissa.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>

namespace atmos::eac3 {
namespace {

constexpr std::array<std::int16_t, 4> slow_decay = {0x0F,0x11,0x13,0x15};
constexpr std::array<std::int16_t, 4> fast_decay = {0x3F,0x53,0x67,0x7B};
constexpr std::array<std::int16_t, 4> slow_gain = {0x540,0x4D8,0x478,0x410};
constexpr std::array<std::int16_t, 4> db_per_bit = {0x000,0x700,0x900,0xB00};
constexpr std::array<std::int16_t, 8> floor_values = {
    0x2F0,0x2B0,0x270,0x230,0x1F0,0x170,0x0F0,-2048
};
constexpr std::array<std::int16_t, 8> fast_gain = {
    0x080,0x100,0x180,0x200,0x280,0x300,0x380,0x400
};
constexpr std::array<bool, 17> default_spx_bands = {
    false,false,false,false,false,false,false,false,true,
    false,true,false,true,false,true,false,true
};
constexpr std::array<bool, 18> default_coupling_bands = {
    false,false,false,false,false,false,false,false,true,
    false,true,true,false,true,true,true,true,true
};
constexpr std::array<std::size_t, 23> ecpl_subband_start = {
    13U,19U,25U,31U,37U,49U,61U,73U,85U,97U,109U,121U,
    133U,145U,157U,169U,181U,193U,205U,217U,229U,241U,253U
};
constexpr std::array<bool, 22> default_ecpl_bands = {
    false,false,false,false,false,false,false,false,false,true,false,
    true,false,true,false,true,true,true,false,true,true,true
};
constexpr std::array<std::uint8_t, 32> ecpl_amplitude_exponent = {
    0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,
    3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,0
};
constexpr std::array<std::uint8_t, 32> ecpl_amplitude_mantissa = {
    0x20,0x1b,0x17,0x13,0x10,0x1b,0x17,0x13,
    0x10,0x1b,0x17,0x13,0x10,0x1b,0x17,0x13,
    0x10,0x1b,0x17,0x13,0x10,0x1b,0x17,0x13,
    0x10,0x1b,0x17,0x13,0x10,0x1b,0x17,0x00
};

struct EcplStrategy final {
    std::uint8_t raw_begin = 0;
    std::uint8_t begin = 0;
    std::uint8_t end = 0;
    std::uint8_t band_count = 0;
    std::array<bool, 22> band_structure = default_ecpl_bands;
};

struct EcplChannelParameters final {
    std::array<std::uint8_t, 22> amplitude{};
    std::array<std::uint8_t, 22> angle{};
    std::array<std::uint8_t, 22> chaos{};
    bool transient = false;
};

struct EcplBlockParameters final {
    EcplStrategy strategy{};
    std::array<EcplChannelParameters, AudioFrameSyntax::max_channels> channels{};
    std::array<bool, AudioFrameSyntax::max_channels> channel_in_coupling{};
    bool enabled = false;
    bool angle_interpolation = false;
};

struct DecodeState final {
    std::array<std::array<std::int8_t, coefficient_count>,
        AudioFrameSyntax::max_channels> exponents{};
    std::array<std::array<std::uint8_t, coefficient_count>,
        AudioFrameSyntax::max_channels> bap{};
    std::array<std::size_t, AudioFrameSyntax::max_channels> start{};
    std::array<std::size_t, AudioFrameSyntax::max_channels> end{};
    std::array<std::size_t, AudioFrameSyntax::max_channels> exponent_groups{};
    std::array<std::int16_t, AudioFrameSyntax::max_channels> snr{};
    std::array<std::int16_t, AudioFrameSyntax::max_channels> fast_gain_value{};
    std::array<DeltaBitAllocation, AudioFrameSyntax::max_channels> delta{};
    std::array<bool, AudioFrameSyntax::max_channels> dither{};
    std::array<bool, AudioFrameSyntax::max_channels> uses_spx{};
    std::array<bool, AudioFrameSyntax::max_channels> first_spx_coordinates{};
    std::array<bool, AudioFrameSyntax::max_channels> channel_in_coupling{};
    std::array<bool, AudioFrameSyntax::max_channels> first_coupling_coordinates{};
    std::array<std::array<double, 22>, AudioFrameSyntax::max_channels>
        coupling_coordinates{};
    std::array<std::array<double, 17>, AudioFrameSyntax::max_channels>
        spx_noise_blend{};
    std::array<std::array<double, 17>, AudioFrameSyntax::max_channels>
        spx_signal_blend{};
    std::array<bool, 22> coupling_phase{};
    std::array<bool, 4> rematrix{};
    std::array<bool, 18> coupling_band_structure = default_coupling_bands;
    std::array<std::uint8_t, 22> coupling_band_sizes{};
    std::array<bool, 17> spx_band_structure = default_spx_bands;
    std::array<std::uint8_t, 17> spx_band_sizes{};
    BitAllocationParameters allocation{};
    std::size_t spx_source_start = 0;
    std::size_t spx_copy_start = 0;
    std::size_t spx_destination_end = 0;
    std::uint8_t spx_band_count = 0;
    std::uint8_t coupling_band_count = 0;
    EcplStrategy ecpl_strategy{};
    std::array<EcplChannelParameters, AudioFrameSyntax::max_channels>
        ecpl_channels{};
    std::array<EcplBlockParameters, AudioFrameSyntax::max_blocks> ecpl_blocks{};
    bool spx_in_use = false;
    bool ecpl_in_use = false;
    bool ecpl_strategy_seen = false;
    bool phase_flags_in_use = false;
    bool first_coupling_leak = true;
    std::array<unsigned, AudioFrameSyntax::max_channels> bandwidth_code{};
    std::size_t bandwidth_start_bit = 0;
    DitherGenerator* dither_generator = nullptr;
};

unsigned ecpl_begin_subband(unsigned code) noexcept {
    if (code < 3U) return code * 2U;
    if (code < 13U) return code + 2U;
    return code * 2U - 10U;
}

unsigned ecpl_spx_begin_code(const DecodeState& state) noexcept {
    const unsigned subband = static_cast<unsigned>(
        (state.spx_source_start - 25U) / 12U);
    return subband <= 7U ? subband - 2U : (subband + 3U) / 2U;
}

double ecpl_amplitude(std::uint8_t code) noexcept {
    const unsigned index = code & 31U;
    if (index == 31U) return 0.0;
    return std::ldexp(static_cast<double>(ecpl_amplitude_mantissa[index]) / 32.0,
        -static_cast<int>(ecpl_amplitude_exponent[index]));
}

void parse_ecpl_strategy(BitReader& bits, DecodeState& state) {
    EcplStrategy strategy;
    strategy.raw_begin = static_cast<std::uint8_t>(bits.read(4U));
    const unsigned begin = ecpl_begin_subband(strategy.raw_begin);
    unsigned end = 0U;
    if (state.spx_in_use) {
        const unsigned spx_begin = ecpl_spx_begin_code(state);
        end = spx_begin < 6U ? spx_begin + 5U : spx_begin * 2U;
    } else {
        end = bits.read(4U) + 7U;
    }
    if (begin >= end || end > 22U) throw ParseError("invalid enhanced coupling range");
    strategy.begin = static_cast<std::uint8_t>(begin);
    strategy.end = static_cast<std::uint8_t>(end);
    const bool explicit_structure = bits.read_bit();
    if (explicit_structure) {
        strategy.band_structure.fill(false);
        for (unsigned subband = std::max(9U, begin + 1U); subband < end; ++subband) {
            strategy.band_structure[subband] = bits.read_bit();
        }
    } else if (state.ecpl_strategy_seen) {
        strategy.band_structure = state.ecpl_strategy.band_structure;
    }
    for (unsigned subband = 0U; subband <= std::max(begin, 8U); ++subband) {
        strategy.band_structure[subband] = false;
    }
    unsigned bands = end - begin;
    for (unsigned subband = begin; subband < end; ++subband) {
        if (strategy.band_structure[subband]) --bands;
    }
    if (bands == 0U || bands > 22U) throw ParseError("invalid enhanced coupling bands");
    strategy.band_count = static_cast<std::uint8_t>(bands);
    state.ecpl_strategy = strategy;
    state.ecpl_strategy_seen = true;
    state.start[0] = ecpl_subband_start[begin];
    state.end[0] = ecpl_subband_start[end];
    state.coupling_band_count = strategy.band_count;
    state.coupling_band_sizes.fill(0U);
    unsigned band = 0U;
    for (unsigned subband = begin; subband < end; ++subband) {
        if (subband != begin && !strategy.band_structure[subband]) ++band;
        state.coupling_band_sizes[band] = static_cast<std::uint8_t>(
            state.coupling_band_sizes[band] +
            ecpl_subband_start[subband + 1U] - ecpl_subband_start[subband]);
    }
}

void parse_ecpl_coordinates(BitReader& bits, unsigned block,
    const AudioFrameSyntax& syntax, DecodeState& state) {
    auto& snapshot = state.ecpl_blocks[block];
    snapshot.enabled = true;
    snapshot.strategy = state.ecpl_strategy;
    snapshot.angle_interpolation = bits.read_bit();
    snapshot.channel_in_coupling = state.channel_in_coupling;
    bool found_first = false;
    for (unsigned channel = 1U; channel <= syntax.full_bandwidth_channels; ++channel) {
        if (!state.channel_in_coupling[channel]) {
            state.first_coupling_coordinates[channel] = true;
            continue;
        }
        const bool first = !found_first;
        found_first = true;
        bool amplitude_present = true;
        bool angle_present = !first;
        if (!state.first_coupling_coordinates[channel]) {
            amplitude_present = bits.read_bit();
            angle_present = !first && bits.read_bit();
        }
        state.first_coupling_coordinates[channel] = false;
        auto& parameters = state.ecpl_channels[channel];
        if (amplitude_present) {
            for (unsigned band = 0U; band < state.ecpl_strategy.band_count; ++band) {
                parameters.amplitude[band] = static_cast<std::uint8_t>(bits.read(5U));
            }
        }
        if (angle_present) {
            for (unsigned band = 0U; band < state.ecpl_strategy.band_count; ++band) {
                parameters.angle[band] = static_cast<std::uint8_t>(bits.read(6U));
                parameters.chaos[band] = static_cast<std::uint8_t>(bits.read(3U));
            }
        }
        parameters.transient = !first && bits.read_bit();
        for (unsigned band = 0U; band < state.ecpl_strategy.band_count; ++band) {
            double amplitude = ecpl_amplitude(parameters.amplitude[band]);
            if (!first && !parameters.transient) {
                amplitude *= 1.0 - 0.38 *
                    static_cast<double>(parameters.chaos[band]) / 7.0;
            }
            state.coupling_coordinates[channel][band] = amplitude;
        }
    }
    snapshot.channels = state.ecpl_channels;
}

std::span<const std::array<std::int16_t, 6>> aht_vq_table(unsigned hebap) {
    switch (hebap) {
    case 1U: return aht_tables::vq_hebap1;
    case 2U: return aht_tables::vq_hebap2;
    case 3U: return aht_tables::vq_hebap3;
    case 4U: return aht_tables::vq_hebap4;
    case 5U: return aht_tables::vq_hebap5;
    case 6U: return aht_tables::vq_hebap6;
    case 7U: return aht_tables::vq_hebap7;
    default: throw ParseError("invalid AHT VQ pointer");
    }
}

void aht_idct6(std::array<std::int32_t, 6>& values) {
    constexpr std::int64_t coefficient0 = 10273905LL;
    constexpr std::int64_t coefficient1 = 11863283LL;
    constexpr std::int64_t coefficient2 = 3070444LL;
    const std::int64_t odd1 = static_cast<std::int64_t>(values[1]) - values[3] - values[5];
    std::int64_t even2 = (static_cast<std::int64_t>(values[2]) * coefficient0) >> 23U;
    std::int64_t temporary = (static_cast<std::int64_t>(values[4]) * coefficient1) >> 23U;
    std::int64_t odd0 = ((static_cast<std::int64_t>(values[1]) + values[5]) *
        coefficient2) >> 23U;
    std::int64_t even0 = values[0] + (temporary >> 1U);
    const std::int64_t even1 = values[0] - temporary;
    temporary = even0;
    even0 = temporary + even2;
    even2 = temporary - even2;
    temporary = odd0;
    odd0 = temporary + values[1] + values[3];
    const std::int64_t odd2 = temporary + values[5] - values[3];
    const std::array<std::int64_t, 6> result = {
        even0 + odd0, even1 + odd1, even2 + odd2,
        even2 - odd2, even1 - odd1, even0 - odd0
    };
    for (unsigned index = 0; index < values.size(); ++index) {
        values[index] = static_cast<std::int32_t>(std::clamp(result[index],
            static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min()),
            static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())));
    }
}

void decode_aht_channel(BitReader& bits, unsigned channel, DecodeState& state,
    SpectralFrame& output) {
    const unsigned gaq_mode = bits.read(2U);
    const unsigned gain_end = gaq_mode < 2U ? 12U : 17U;
    std::array<unsigned, coefficient_count> gains{};
    std::size_t gain_count = 0U;
    if (gaq_mode == 1U || gaq_mode == 2U) {
        for (std::size_t bin = state.start[channel]; bin < state.end[channel]; ++bin) {
            const unsigned hebap = state.bap[channel][bin];
            if (hebap > 7U && hebap < gain_end) gains[gain_count++] =
                bits.read_bit() ? 1U << (gaq_mode - 1U) : 0U;
        }
    } else if (gaq_mode == 3U) {
        std::array<unsigned, 3> group{};
        unsigned group_index = 3U;
        for (std::size_t bin = state.start[channel]; bin < state.end[channel]; ++bin) {
            const unsigned hebap = state.bap[channel][bin];
            if (hebap <= 7U || hebap >= 17U) continue;
            if (group_index == 3U) {
                const unsigned code = std::min(bits.read(5U), 26U);
                group = {code / 9U, (code % 9U) / 3U, code % 3U};
                group_index = 0U;
            }
            gains[gain_count++] = group[group_index++];
        }
    }
    std::size_t gain_index = 0U;
    for (std::size_t bin = state.start[channel]; bin < state.end[channel]; ++bin) {
        const unsigned hebap = state.bap[channel][bin];
        if (hebap >= aht_tables::bits_vs_hebap.size()) throw ParseError("invalid AHT hebap");
        const unsigned width = aht_tables::bits_vs_hebap[hebap];
        std::array<std::int32_t, 6> mantissas{};
        if (hebap == 0U) {
            for (auto& mantissa : mantissas) {
                mantissa = static_cast<std::int32_t>(
                    (state.dither_generator->next() & 0x007FFFFFU) - 0x00400000U);
            }
        } else if (hebap < 8U) {
            const auto table = aht_vq_table(hebap);
            const unsigned vector = bits.read(width);
            if (vector >= table.size()) throw ParseError("invalid AHT vector");
            for (unsigned block = 0; block < 6U; ++block) {
                mantissas[block] = static_cast<std::int32_t>(table[vector][block]) * 256;
            }
        } else {
            const unsigned log_gain = gaq_mode != 0U && hebap < gain_end ?
                gains[gain_index++] : 0U;
            const unsigned gain_width = width - log_gain;
            for (auto& mantissa : mantissas) {
                std::int32_t value = bits.read_signed(gain_width);
                if (log_gain != 0U && value == -(1 << (gain_width - 1U))) {
                    const unsigned mantissa_width = width - (2U - log_gain);
                    value = bits.read_signed(mantissa_width);
                    value = static_cast<std::int32_t>(static_cast<std::uint32_t>(value) <<
                        (23U - (mantissa_width - 1U)));
                    const std::int32_t offset = value >= 0 ? 1 << (23U - log_gain) :
                        static_cast<std::int32_t>(aht_tables::gaq_remap_2_4_b[
                            hebap - 8U][log_gain - 1U]) * 256;
                    value += static_cast<std::int32_t>((static_cast<std::int64_t>(
                        aht_tables::gaq_remap_2_4_a[hebap - 8U][log_gain - 1U]) * value) >>
                        15U) + offset;
                } else {
                    value *= 1 << (24U - width);
                    if (log_gain == 0U) value += static_cast<std::int32_t>((
                        static_cast<std::int64_t>(aht_tables::gaq_remap_1[hebap - 8U]) *
                        value) >> 15U);
                }
                mantissa = value;
            }
        }
        aht_idct6(mantissas);
        const unsigned exponent = static_cast<unsigned>(state.exponents[channel][bin]);
        for (unsigned block = 0; block < 6U; ++block) {
            output.blocks[block].coefficients[channel][bin] = mantissas[block] >> exponent;
        }
    }
}

void parse_rematrix_strategy(BitReader& bits, unsigned block,
    const AudioFrameSyntax& syntax, DecodeState& state) {
    if (block != 0U && !bits.read_bit()) return;
    static constexpr std::array<std::size_t, 4> band_start = {13U,25U,37U,61U};
    const std::size_t end = syntax.coupling[block][0] ? state.start[0] : 253U;
    for (unsigned band = 0; band < band_start.size(); ++band) {
        if (band_start[band] >= end) break;
        state.rematrix[band] = bits.read_bit();
    }
}

void decode_spx_band_structure(BitReader& bits, unsigned block,
    unsigned start_subband, unsigned end_subband, DecodeState& state) {
    const unsigned subbands = end_subband - start_subband;
    if (block == 0U) state.spx_band_structure = default_spx_bands;
    if (bits.read_bit()) {
        for (unsigned index = 0; index + 1U < subbands; ++index) {
            state.spx_band_structure[start_subband + 1U + index] = bits.read_bit();
        }
    }
    unsigned band = 0;
    state.spx_band_sizes.fill(0U);
    state.spx_band_sizes[0] = 12U;
    for (unsigned subband = 1; subband < subbands; ++subband) {
        if (state.spx_band_structure[start_subband + subband]) {
            state.spx_band_sizes[band] = static_cast<std::uint8_t>(
                state.spx_band_sizes[band] + 12U);
        } else {
            state.spx_band_sizes[++band] = 12U;
        }
    }
    state.spx_band_count = static_cast<std::uint8_t>(band + 1U);
}

void decode_coupling_band_structure(BitReader& bits, unsigned block,
    unsigned start_subband, unsigned end_subband, DecodeState& state) {
    const unsigned subbands = end_subband - start_subband;
    if (block == 0U) state.coupling_band_structure = default_coupling_bands;
    if (bits.read_bit()) {
        for (unsigned index = 0; index + 1U < subbands; ++index) {
            state.coupling_band_structure[start_subband + 1U + index] = bits.read_bit();
        }
    }
    unsigned band = 0;
    state.coupling_band_sizes.fill(0U);
    state.coupling_band_sizes[0] = 12U;
    for (unsigned subband = 1; subband < subbands; ++subband) {
        if (state.coupling_band_structure[start_subband + subband]) {
            state.coupling_band_sizes[band] = static_cast<std::uint8_t>(
                state.coupling_band_sizes[band] + 12U);
        } else {
            state.coupling_band_sizes[++band] = 12U;
        }
    }
    state.coupling_band_count = static_cast<std::uint8_t>(band + 1U);
}

void parse_coupling(BitReader& bits, unsigned block, const BitstreamInfo& info,
    const AudioFrameSyntax& syntax, DecodeState& state) {
    const bool in_use = syntax.coupling[block][0];
    if (syntax.coupling_strategy_present[block]) {
        if (in_use) {
            state.ecpl_in_use = bits.read_bit();
            if (info.audio_coding_mode == 2U) {
                state.channel_in_coupling[1] = true;
                state.channel_in_coupling[2] = true;
            } else {
                for (unsigned channel = 1; channel <= syntax.full_bandwidth_channels;
                    ++channel) {
                    state.channel_in_coupling[channel] = bits.read_bit();
                }
            }
            if (state.ecpl_in_use) {
                state.phase_flags_in_use = false;
                parse_ecpl_strategy(bits, state);
            } else {
            state.phase_flags_in_use = info.audio_coding_mode == 2U && bits.read_bit();
            const unsigned start_subband = bits.read(4U);
            const unsigned end_subband = state.spx_in_use ?
                static_cast<unsigned>((state.spx_source_start - 37U) / 12U) :
                bits.read(4U) + 3U;
            if (start_subband >= end_subband || end_subband > 18U) {
                throw ParseError("invalid coupling range");
            }
            state.start[0] = start_subband * 12U + 37U;
            state.end[0] = end_subband * 12U + 37U;
            decode_coupling_band_structure(bits, block, start_subband,
                end_subband, state);
            }
        } else {
            state.channel_in_coupling.fill(false);
            state.first_coupling_coordinates.fill(true);
            state.phase_flags_in_use = false;
            state.first_coupling_leak = true;
            state.ecpl_in_use = false;
        }
    }
    if (!in_use) return;
    if (state.ecpl_in_use) {
        parse_ecpl_coordinates(bits, block, syntax, state);
        return;
    }
    bool coordinates_changed = false;
    for (unsigned channel = 1; channel <= syntax.full_bandwidth_channels; ++channel) {
        if (!state.channel_in_coupling[channel]) {
            state.first_coupling_coordinates[channel] = true;
            continue;
        }
        const bool present = state.first_coupling_coordinates[channel] || bits.read_bit();
        if (!present) continue;
        state.first_coupling_coordinates[channel] = false;
        coordinates_changed = true;
        const unsigned master = bits.read(2U) * 3U;
        for (unsigned band = 0; band < state.coupling_band_count; ++band) {
            const unsigned exponent = bits.read(4U);
            const unsigned mantissa = bits.read(4U);
            const std::uint32_t fixed = exponent == 15U ? mantissa << 22U :
                (mantissa + 16U) << 21U;
            state.coupling_coordinates[channel][band] =
                std::ldexp(static_cast<double>(fixed),
                    -static_cast<int>(23U + exponent + master));
        }
    }
    if (info.audio_coding_mode == 2U && coordinates_changed) {
        for (unsigned band = 0; band < state.coupling_band_count; ++band) {
            state.coupling_phase[band] = state.phase_flags_in_use && bits.read_bit();
        }
    }
}

void parse_spx_strategy(BitReader& bits, unsigned block,
    const BitstreamInfo& info, const AudioFrameSyntax& syntax, DecodeState& state) {
    const bool strategy_present = block == 0U || bits.read_bit();
    if (strategy_present) {
        state.spx_in_use = bits.read_bit();
        if (state.spx_in_use) {
            if (info.audio_coding_mode == 1U) {
                state.uses_spx[1] = true;
            } else {
                const unsigned flags = bits.read(syntax.full_bandwidth_channels);
                for (unsigned channel = 1; channel <= syntax.full_bandwidth_channels; ++channel) {
                    const unsigned shift = syntax.full_bandwidth_channels - channel;
                    state.uses_spx[channel] = ((flags >> shift) & 1U) != 0U;
                }
            }
            const unsigned destination_code = bits.read(2U);
            unsigned start_subband = bits.read(3U) + 2U;
            if (start_subband > 7U) start_subband += start_subband - 7U;
            unsigned end_subband = bits.read(3U) + 5U;
            if (end_subband > 7U) end_subband += end_subband - 7U;
            const std::size_t destination_start = destination_code * 12U + 25U;
            state.spx_copy_start = destination_start;
            state.spx_source_start = start_subband * 12U + 25U;
            state.spx_destination_end = end_subband * 12U + 25U;
            if (start_subband >= end_subband ||
                destination_start >= state.spx_source_start) {
                throw ParseError("invalid spectral extension range");
            }
            decode_spx_band_structure(bits, block, start_subband, end_subband, state);
        }
    }
    if (!state.spx_in_use) {
        state.uses_spx.fill(false);
        state.first_spx_coordinates.fill(true);
        return;
    }
    for (unsigned channel = 1; channel <= syntax.full_bandwidth_channels; ++channel) {
        if (!state.uses_spx[channel]) {
            state.first_spx_coordinates[channel] = true;
            continue;
        }
        const bool coordinates_present = state.first_spx_coordinates[channel] || bits.read_bit();
        if (coordinates_present) {
            state.first_spx_coordinates[channel] = false;
            const double blend = static_cast<double>(bits.read(5U)) / 32.0;
            const unsigned master = bits.read(2U) * 3U;
            std::size_t bin = state.spx_source_start;
            for (unsigned band = 0; band < state.spx_band_count; ++band) {
                const unsigned size = state.spx_band_sizes[band];
                const double noise_ratio = std::clamp(
                    static_cast<double>(bin + size / 2U) /
                        static_cast<double>(state.spx_destination_end) - blend,
                    0.0, 1.0);
                const double noise = std::sqrt(3.0 * noise_ratio);
                const double signal = std::sqrt(1.0 - noise_ratio);
                bin += size;
                const unsigned exponent = bits.read(4U);
                unsigned mantissa = bits.read(2U);
                if (exponent == 15U) mantissa <<= 1U;
                else mantissa += 4U;
                const double coordinate = std::ldexp(static_cast<double>(mantissa),
                    2 - static_cast<int>(exponent) - static_cast<int>(master));
                state.spx_noise_blend[channel][band] = noise * coordinate;
                state.spx_signal_blend[channel][band] = signal * coordinate;
            }
        }
    }
}

void apply_spectral_extension(unsigned channel, const AudioFrameSyntax& syntax,
    DecodeState& state, SpectralBlock& output) {
    if (!state.uses_spx[channel]) return;
    std::array<bool, 17> wrap{};
    wrap[0] = true;
    std::size_t copy = state.spx_copy_start;
    std::size_t destination = state.spx_source_start;
    for (unsigned band = 0; band < state.spx_band_count; ++band) {
        const std::size_t size = state.spx_band_sizes[band];
        if (copy + size > state.spx_source_start) {
            copy = state.spx_copy_start;
            wrap[band] = true;
        }
        for (std::size_t index = 0; index < size; ++index) {
            if (copy == state.spx_source_start) {
                copy = state.spx_copy_start;
                wrap[band] = true;
            }
            output.coefficients[channel][destination++] =
                output.coefficients[channel][copy++];
        }
    }
    std::array<double, 17> rms{};
    std::size_t bin = state.spx_source_start;
    for (unsigned band = 0; band < state.spx_band_count; ++band) {
        double energy = 0.0;
        for (unsigned index = 0; index < state.spx_band_sizes[band]; ++index) {
            const double coefficient = output.coefficients[channel][bin++];
            energy += coefficient * coefficient;
        }
        rms[band] = std::sqrt(energy / state.spx_band_sizes[band]);
    }
    const int attenuation = syntax.spectral_extension_attenuation[channel];
    if (attenuation >= 0) {
        bin = state.spx_source_start - 2U;
        for (unsigned band = 0; band < state.spx_band_count; ++band) {
            if (wrap[band]) {
                const double a0 = std::pow(2.0, -static_cast<double>(attenuation + 1) / 15.0);
                const double a1 = a0 * a0;
                const double a2 = a1 * a0;
                output.coefficients[channel][bin] = static_cast<std::int32_t>(
                    output.coefficients[channel][bin] * a0);
                output.coefficients[channel][bin + 1U] = static_cast<std::int32_t>(
                    output.coefficients[channel][bin + 1U] * a1);
                output.coefficients[channel][bin + 2U] = static_cast<std::int32_t>(
                    output.coefficients[channel][bin + 2U] * a2);
                output.coefficients[channel][bin + 3U] = static_cast<std::int32_t>(
                    output.coefficients[channel][bin + 3U] * a1);
                output.coefficients[channel][bin + 4U] = static_cast<std::int32_t>(
                    output.coefficients[channel][bin + 4U] * a0);
            }
            bin += state.spx_band_sizes[band];
        }
    }
    bin = state.spx_source_start;
    for (unsigned band = 0; band < state.spx_band_count; ++band) {
        const double noise_scale = state.spx_noise_blend[channel][band] * rms[band] /
            static_cast<double>(std::numeric_limits<std::int32_t>::min());
        const double signal_scale = state.spx_signal_blend[channel][band];
        for (unsigned index = 0; index < state.spx_band_sizes[band]; ++index) {
            const auto noise = static_cast<std::int32_t>(state.dither_generator->next());
            const double value = output.coefficients[channel][bin] * signal_scale +
                noise_scale * noise;
            output.coefficients[channel][bin++] = static_cast<std::int32_t>(std::clamp(value,
                static_cast<double>(std::numeric_limits<std::int32_t>::min()),
                static_cast<double>(std::numeric_limits<std::int32_t>::max())));
        }
    }
}

void parse_exponents(BitReader& bits, unsigned block, const BitstreamInfo& info,
    const AudioFrameSyntax& syntax, DecodeState& state) {
    const unsigned channels = syntax.full_bandwidth_channels + (info.lfe ? 1U : 0U);
    state.bandwidth_start_bit = bits.position();
    for (unsigned channel = 1; channel <= syntax.full_bandwidth_channels; ++channel) {
        state.start[channel] = 0;
        const ExponentStrategy strategy = syntax.exponent_strategy[block][channel];
        if (strategy == ExponentStrategy::Reuse) continue;
        if (state.channel_in_coupling[channel]) {
            state.end[channel] = state.start[0];
        } else if (state.uses_spx[channel]) {
            state.end[channel] = state.spx_source_start;
        } else {
            const unsigned bandwidth_code = bits.read(6U);
            state.bandwidth_code[channel] = bandwidth_code;
            if (bandwidth_code > 60U) throw ParseError("channel bandwidth out of range");
            state.end[channel] = bandwidth_code * 3U + 73U;
        }
        const unsigned group_size = 3U << (static_cast<unsigned>(strategy) - 1U);
        state.exponent_groups[channel] =
            (state.end[channel] + group_size - 4U) / group_size;
    }
    if (syntax.coupling[block][0] &&
        syntax.exponent_strategy[block][0] != ExponentStrategy::Reuse) {
        const auto strategy = syntax.exponent_strategy[block][0];
        const unsigned group_size = 3U << (static_cast<unsigned>(strategy) - 1U);
        state.exponent_groups[0] = (state.end[0] - state.start[0]) / group_size;
    }
    const unsigned first_channel = syntax.coupling[block][0] ? 0U : 1U;
    for (unsigned channel = first_channel; channel <= channels; ++channel) {
        const ExponentStrategy strategy = syntax.exponent_strategy[block][channel];
        if (strategy == ExponentStrategy::Reuse) continue;
        const std::uint8_t absolute = static_cast<std::uint8_t>(
            bits.read(4U) * (channel == 0U ? 2U : 1U));
        state.exponents[channel][0] = static_cast<std::int8_t>(absolute);
        const std::size_t count = decoded_exponent_count(
            strategy, state.exponent_groups[channel]);
        try {
            decode_exponents(bits, strategy, state.exponent_groups[channel], absolute,
                std::span(state.exponents[channel]).subspan(
                    channel == 0U ? state.start[0] : 1U, count));
        } catch (const ParseError& exception) {
            throw ParseError("block " + std::to_string(block) + " channel " +
                std::to_string(channel) + " end " + std::to_string(state.end[channel]) +
                " bw-start " + std::to_string(state.bandwidth_start_bit) +
                " bw " + std::to_string(state.bandwidth_code[channel]) +
                " groups " + std::to_string(state.exponent_groups[channel]) +
                " absolute " + std::to_string(absolute) + " exponent at bit " +
                std::to_string(bits.position()) + ": " + exception.what());
        }
        if (channel != 0U && channel != syntax.lfe_channel) bits.skip(2U);
    }
}

void parse_allocation_data(BitReader& bits, unsigned block,
    const BitstreamInfo& info, const AudioFrameSyntax& syntax, DecodeState& state) {
    const unsigned channels = syntax.full_bandwidth_channels + (info.lfe ? 1U : 0U);
    const unsigned first_channel = syntax.coupling[block][0] ? 0U : 1U;
    if (syntax.bit_allocation_syntax) {
        if (bits.read_bit()) {
            state.allocation.slow_decay = slow_decay[bits.read(2U)];
            state.allocation.fast_decay = fast_decay[bits.read(2U)];
            state.allocation.slow_gain = slow_gain[bits.read(2U)];
            state.allocation.db_per_bit = db_per_bit[bits.read(2U)];
            state.allocation.floor = floor_values[bits.read(3U)];
        } else if (block == 0U) {
            throw ParseError("missing initial bit allocation parameters");
        }
    }
    if (block == 0U && syntax.snr_offset_strategy != 0U) {
        if (bits.read_bit()) {
            const int coarse = (static_cast<int>(bits.read(6U)) - 15) * 16;
            int value = 0;
            for (unsigned channel = first_channel; channel <= channels; ++channel) {
                if (channel == first_channel || syntax.snr_offset_strategy == 2U) {
                    value = (coarse + static_cast<int>(bits.read(4U))) * 4;
                }
                state.snr[channel] = static_cast<std::int16_t>(value);
            }
        }
    }
    if (syntax.fast_gain_syntax && bits.read_bit()) {
        for (unsigned channel = first_channel; channel <= channels; ++channel) {
            state.fast_gain_value[channel] = fast_gain[bits.read(3U)];
        }
    } else if (block == 0U) {
        for (unsigned channel = first_channel; channel <= channels; ++channel) {
            state.fast_gain_value[channel] = fast_gain[4];
        }
    }
    if (info.stream_type == 0U && bits.read_bit()) bits.skip(10U);

    if (syntax.coupling[block][0]) {
        if (state.first_coupling_leak || bits.read_bit()) {
            state.allocation.coupling_fast_leak = static_cast<std::int16_t>(bits.read(3U));
            state.allocation.coupling_slow_leak = static_cast<std::int16_t>(bits.read(3U));
        }
        state.first_coupling_leak = false;
    }

    if (syntax.delta_bit_allocation_syntax && bits.read_bit()) {
        for (unsigned channel = first_channel; channel <= syntax.full_bandwidth_channels; ++channel) {
            const unsigned mode = bits.read(2U);
            if (mode == 3U) throw ParseError("reserved delta bit allocation mode");
            state.delta[channel].mode = mode == 0U ? DeltaBitAllocationMode::Reuse :
                mode == 1U ? DeltaBitAllocationMode::New : DeltaBitAllocationMode::None;
        }
        for (unsigned channel = first_channel; channel <= syntax.full_bandwidth_channels; ++channel) {
            auto& delta = state.delta[channel];
            if (delta.mode != DeltaBitAllocationMode::New) continue;
            delta.segment_count = static_cast<std::uint8_t>(bits.read(3U) + 1U);
            for (unsigned segment = 0; segment < delta.segment_count; ++segment) {
                delta.offsets[segment] = static_cast<std::uint8_t>(bits.read(5U));
                delta.lengths[segment] = static_cast<std::uint8_t>(bits.read(4U));
                delta.values[segment] = static_cast<std::uint8_t>(bits.read(3U));
            }
        }
    } else if (block == 0U) {
        for (auto& delta : state.delta) delta.mode = DeltaBitAllocationMode::None;
    }
}

void allocate_and_decode(BitReader& bits, const BitstreamInfo& info,
    unsigned block, const AudioFrameSyntax& syntax, DecodeState& state,
    SpectralFrame& frame_output, std::vector<std::uint8_t>& auxiliary_data) {
    SpectralBlock& output = frame_output.blocks[block];
    const unsigned channels = syntax.full_bandwidth_channels + (info.lfe ? 1U : 0U);
    const unsigned first_channel = syntax.coupling[block][0] ? 0U : 1U;
    std::array<BitAllocationWorkspace, AudioFrameSyntax::max_channels> workspace{};
    for (unsigned channel = first_channel; channel <= channels; ++channel) {
        calculate_psd(state.exponents[channel], state.start[channel],
            state.end[channel], workspace[channel]);
        calculate_mask(state.allocation, state.start[channel], state.end[channel],
            state.fast_gain_value[channel], channel == syntax.lfe_channel,
            state.delta[channel], workspace[channel]);
        calculate_bap(state.allocation, state.start[channel], state.end[channel],
            state.snr[channel], workspace[channel], state.bap[channel],
            syntax.adaptive_hybrid_transform[channel]);
    }
    if (syntax.skip_syntax) {
        const auto start = bits.position();
        if (bits.read_bit()) {
            const unsigned length = bits.read(9U);
            for (unsigned index = 0; index < length; ++index)
                auxiliary_data.push_back(static_cast<std::uint8_t>(bits.read(8U)));
        }
        frame_output.skip_ranges.emplace_back(start, bits.position());
    }
    MantissaDecoder mantissas(*state.dither_generator);
    mantissas.reset_groups();
    bool coupling_decoded = false;
    for (unsigned channel = 1; channel <= channels; ++channel) {
        if (syntax.adaptive_hybrid_transform[channel]) {
            if (block == 0U) decode_aht_channel(bits, channel, state, frame_output);
        } else {
            const std::size_t length = state.end[channel] - state.start[channel];
            mantissas.decode(bits,
                std::span(state.bap[channel]).subspan(state.start[channel], length),
                std::span(state.exponents[channel]).subspan(state.start[channel], length),
                std::span(output.coefficients[channel]).subspan(state.start[channel], length),
                state.dither[channel]);
        }
        if (state.channel_in_coupling[channel] && !coupling_decoded) {
            if (syntax.adaptive_hybrid_transform[0]) {
                if (block == 0U) decode_aht_channel(bits, 0U, state, frame_output);
            } else {
                const std::size_t coupling_length = state.end[0] - state.start[0];
                mantissas.decode(bits,
                    std::span(state.bap[0]).subspan(state.start[0], coupling_length),
                    std::span(state.exponents[0]).subspan(state.start[0], coupling_length),
                    std::span(output.coefficients[0]).subspan(state.start[0], coupling_length),
                    true);
            }
            coupling_decoded = true;
        }
    }
    if (coupling_decoded) {
        for (unsigned channel = 1; channel <= syntax.full_bandwidth_channels; ++channel) {
            if (!state.channel_in_coupling[channel]) continue;
            std::size_t bin = state.start[0];
            for (unsigned band = 0; band < state.coupling_band_count; ++band) {
                const double phase = info.audio_coding_mode == 2U && channel == 2U &&
                    state.coupling_phase[band] ? -1.0 : 1.0;
                const double coordinate = state.coupling_coordinates[channel][band] * phase;
                const std::size_t band_end = bin + state.coupling_band_sizes[band];
                while (bin < band_end) {
                    if (state.dither[channel] || state.bap[0][bin] != 0U) {
                        output.coefficients[channel][bin] = static_cast<std::int32_t>(
                            static_cast<double>(output.coefficients[0][bin]) * coordinate);
                    }
                    ++bin;
                }
            }
        }
    }
    if (info.audio_coding_mode == 2U) {
        static constexpr std::array<std::size_t, 5> band_edge = {13U,25U,37U,61U,253U};
        const std::size_t end = syntax.coupling[block][0] ? state.start[0] :
            std::min(state.end[1], state.end[2]);
        for (unsigned band = 0; band < state.rematrix.size(); ++band) {
            if (!state.rematrix[band] || band_edge[band] >= end) continue;
            const std::size_t band_end = std::min(band_edge[band + 1U], end);
            for (std::size_t bin = band_edge[band]; bin < band_end; ++bin) {
                const std::int32_t left = output.coefficients[1][bin];
                const std::int32_t right = output.coefficients[2][bin];
                const auto clamp_coefficient = [](std::int64_t value) {
                    return static_cast<std::int32_t>(std::clamp(value,
                        static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min()),
                        static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())));
                };
                output.coefficients[1][bin] = clamp_coefficient(
                    static_cast<std::int64_t>(left) + right);
                output.coefficients[2][bin] = clamp_coefficient(
                    static_cast<std::int64_t>(left) - right);
            }
        }
    }
    for (unsigned channel = 1; channel <= syntax.full_bandwidth_channels; ++channel) {
        apply_spectral_extension(channel, syntax, state, output);
    }
}

double ecpl_bessel_i0(double value) {
    const double half = value * 0.5;
    double sum = 1.0;
    double term = 1.0;
    for (unsigned order = 1U; order < 40U; ++order) {
        term *= (half * half) /
            (static_cast<double>(order) * static_cast<double>(order));
        sum += term;
        if (term < sum * 1.0e-16) break;
    }
    return sum;
}

const std::array<double, 256>& ecpl_window() {
    static const auto window = [] {
        std::array<double, 256> result{};
        double total = 0.0;
        for (unsigned sample = 0U; sample < result.size(); ++sample) {
            const double position = 2.0 * static_cast<double>(sample) / 256.0 - 1.0;
            total += ecpl_bessel_i0(std::numbers::pi * 5.0 *
                std::sqrt(std::max(0.0, 1.0 - position * position)));
            result[sample] = total;
        }
        total += 1.0;
        for (double& value : result) value = std::sqrt(value / total);
        return result;
    }();
    return window;
}

const std::vector<std::array<double, 256>>& ecpl_imdct_basis() {
    static const auto basis = [] {
        std::vector<std::array<double, 256>> result(512U);
        for (unsigned sample = 0U; sample < result.size(); ++sample) {
            const double phase = std::numbers::pi / 256.0 *
                (static_cast<double>(sample) + 128.5);
            for (unsigned bin = 0U; bin < 256U; ++bin) {
                result[sample][bin] = -std::cos(phase *
                    (static_cast<double>(bin) + 0.5));
            }
        }
        return result;
    }();
    return basis;
}

std::array<double, 512> ecpl_windowed_imdct(
    const std::array<std::int32_t, 256>* coefficients) {
    std::array<double, 512> result{};
    if (coefficients == nullptr) return result;
    const auto& basis = ecpl_imdct_basis();
    const auto& window = ecpl_window();
    for (unsigned sample = 0U; sample < 512U; ++sample) {
        double value = 0.0;
        for (unsigned bin = 0U; bin < 256U; ++bin) {
            value += static_cast<double>((*coefficients)[bin]) * basis[sample][bin];
        }
        result[sample] = value * window[sample < 256U ? sample : 511U - sample];
    }
    return result;
}

double ecpl_angle(std::uint8_t code) noexcept {
    return code < 32U ? static_cast<double>(code) / 32.0 :
        -1.0 + static_cast<double>(code - 32U) / 32.0;
}

double ecpl_uniform(std::uint32_t& generator) noexcept {
    generator ^= generator << 13U;
    generator ^= generator >> 17U;
    generator ^= generator << 5U;
    if (generator == 0U) generator = 1U;
    return static_cast<double>(generator) /
        static_cast<double>(std::numeric_limits<std::uint32_t>::max()) * 2.0 - 1.0;
}

std::vector<unsigned> ecpl_bin_bands(const EcplStrategy& strategy) {
    std::vector<unsigned> result;
    result.reserve(ecpl_subband_start[strategy.end] -
        ecpl_subband_start[strategy.begin]);
    unsigned band = 0U;
    for (unsigned subband = strategy.begin; subband < strategy.end; ++subband) {
        if (subband != strategy.begin && !strategy.band_structure[subband]) ++band;
        const unsigned count = static_cast<unsigned>(
            ecpl_subband_start[subband + 1U] - ecpl_subband_start[subband]);
        result.insert(result.end(), count, band);
    }
    return result;
}

std::vector<double> ecpl_bin_angles(const EcplStrategy& strategy,
    const EcplChannelParameters& parameters, bool interpolate) {
    const auto bin_bands = ecpl_bin_bands(strategy);
    std::vector<double> result(bin_bands.size());
    if (!interpolate || strategy.band_count < 2U) {
        for (unsigned bin = 0U; bin < result.size(); ++bin) {
            result[bin] = ecpl_angle(parameters.angle[bin_bands[bin]]);
        }
        return result;
    }
    std::vector<unsigned> sizes(strategy.band_count);
    std::vector<double> values(strategy.band_count);
    for (unsigned band : bin_bands) ++sizes[band];
    for (unsigned band = 0U; band < strategy.band_count; ++band) {
        values[band] = ecpl_angle(parameters.angle[band]);
    }
    const auto wrap = [](double value) {
        while (value > 1.0) value -= 2.0;
        while (value < -1.0) value += 2.0;
        return value;
    };
    std::size_t bin = 0U;
    for (unsigned band = 1U; band < strategy.band_count; ++band) {
        const unsigned previous_size = sizes[band - 1U];
        const unsigned current_size = sizes[band];
        const double previous_angle = values[band - 1U];
        double current_angle = values[band];
        while (current_angle - previous_angle > 1.0) current_angle -= 2.0;
        while (previous_angle - current_angle > 1.0) current_angle += 2.0;
        const double slope = (current_angle - previous_angle) /
            (static_cast<double>(previous_size + current_size) * 0.5);
        if (band == 1U && previous_size > 1U) {
            double value = previous_angle;
            std::size_t down_bin = 0U;
            if ((previous_size & 1U) == 0U) {
                value -= slope * 0.5;
                down_bin = previous_size / 2U - 1U;
            } else {
                value -= slope;
                down_bin = (previous_size - 3U) / 2U;
            }
            const std::size_t count = down_bin + 1U;
            for (std::size_t index = 0U; index < count; ++index) {
                result[down_bin] = wrap(value);
                if (down_bin != 0U) --down_bin;
                value -= slope;
            }
            bin = count;
        }
        double value = previous_angle;
        std::size_t count = 0U;
        if ((previous_size & 1U) == 0U) {
            value += slope * 0.5;
            count = current_size / 2U + previous_size / 2U;
        } else {
            count = current_size / 2U + (previous_size + 1U) / 2U;
        }
        for (std::size_t index = 0U; index < count && bin < result.size(); ++index) {
            result[bin++] = wrap(value);
            value += slope;
        }
        if (band + 1U == strategy.band_count) {
            const std::size_t closing = (current_size & 1U) == 0U ?
                current_size / 2U : current_size / 2U + 1U;
            for (std::size_t index = 0U; index < closing && bin < result.size(); ++index) {
                result[bin++] = wrap(value);
                value += slope;
            }
        }
    }
    return result;
}

void synthesize_ecpl(SpectralFrame& frame, const DecodeState& state,
    unsigned blocks, unsigned full_bandwidth_channels, EcplDecodeState* decode_state) {
    std::array<std::array<double, 512>, AudioFrameSyntax::max_blocks> time{};
    for (unsigned block = 0U; block < blocks; ++block) {
        if (!state.ecpl_blocks[block].enabled) continue;
        time[block] = ecpl_windowed_imdct(&frame.blocks[block].coefficients[0]);
    }
    std::array<double, 512> previous_frame_time{};
    if (decode_state != nullptr && decode_state->previous_frame_mantissa) {
        previous_frame_time = ecpl_windowed_imdct(
            &*decode_state->previous_frame_mantissa);
    }
    std::uint32_t local_transient_generator = 0x2545F491U;
    std::uint32_t& transient_generator = decode_state != nullptr ?
        decode_state->transient_generator : local_transient_generator;
    for (unsigned block = 0U; block < blocks; ++block) {
        const auto& parameters = state.ecpl_blocks[block];
        if (!parameters.enabled) continue;
        const auto& previous = block != 0U && state.ecpl_blocks[block - 1U].enabled ?
            time[block - 1U] : block == 0U ? previous_frame_time :
                std::array<double, 512>{};
        const auto& next = block + 1U < blocks && state.ecpl_blocks[block + 1U].enabled ?
            time[block + 1U] : std::array<double, 512>{};
        std::array<double, 512> real{};
        std::array<double, 512> imaginary{};
        const auto& window = ecpl_window();
        for (unsigned sample = 0U; sample < 256U; ++sample) {
            const double lower = 2.0 * (previous[256U + sample] + time[block][sample]);
            const double upper = 2.0 * (time[block][256U + sample] + next[sample]);
            const double lower_phase = std::numbers::pi * sample / 512.0;
            const double upper_phase = std::numbers::pi * (256U + sample) / 512.0;
            real[sample] = lower * window[sample] * std::cos(lower_phase);
            imaginary[sample] = -lower * window[sample] * std::sin(lower_phase);
            real[256U + sample] = upper * window[255U - sample] * std::cos(upper_phase);
            imaginary[256U + sample] = -upper * window[255U - sample] *
                std::sin(upper_phase);
        }
        const unsigned first = static_cast<unsigned>(std::distance(
            parameters.channel_in_coupling.begin(), std::find(
                parameters.channel_in_coupling.begin() + 1U,
                parameters.channel_in_coupling.begin() +
                    full_bandwidth_channels + 1U, true)));
        const auto bin_bands = ecpl_bin_bands(parameters.strategy);
        const unsigned begin_bin = static_cast<unsigned>(
            ecpl_subband_start[parameters.strategy.begin]);
        for (unsigned channel = 1U; channel <= full_bandwidth_channels; ++channel) {
            if (!parameters.channel_in_coupling[channel]) continue;
            auto angles = channel == first ? std::vector<double>(bin_bands.size()) :
                ecpl_bin_angles(parameters.strategy, parameters.channels[channel],
                    parameters.angle_interpolation);
            std::uint32_t fixed_generator = 0x9E3779B9U ^
                ((channel - 1U) * 0x01000193U + 1U);
            std::array<double, 256> fixed_random{};
            for (double& value : fixed_random) value = ecpl_uniform(fixed_generator);
            std::array<double, 22> transient_random{};
            if (parameters.channels[channel].transient) {
                for (unsigned band = 0U; band < parameters.strategy.band_count; ++band) {
                    transient_random[band] = ecpl_uniform(transient_generator);
                }
            }
            for (unsigned offset = 0U; offset < bin_bands.size(); ++offset) {
                const unsigned bin = begin_bin + offset;
                const unsigned band = bin_bands[offset];
                if (channel != first) {
                    const double chaos = -static_cast<double>(
                        parameters.channels[channel].chaos[band]) / 7.0;
                    angles[offset] += chaos * (parameters.channels[channel].transient ?
                        transient_random[band] : fixed_random[bin]);
                    if (angles[offset] < -1.0) angles[offset] += 2.0;
                    else if (angles[offset] >= 1.0) angles[offset] -= 2.0;
                }
                double carrier_real = 0.0;
                double carrier_imaginary = 0.0;
                for (unsigned sample = 0U; sample < 512U; ++sample) {
                    const double phase = 2.0 * std::numbers::pi * bin * sample / 512.0;
                    carrier_real += real[sample] * std::cos(phase) +
                        imaginary[sample] * std::sin(phase);
                    carrier_imaginary += imaginary[sample] * std::cos(phase) -
                        real[sample] * std::sin(phase);
                }
                carrier_real /= 512.0;
                carrier_imaginary /= 512.0;
                double amplitude = ecpl_amplitude(
                    parameters.channels[channel].amplitude[band]);
                if (channel != first && !parameters.channels[channel].transient) {
                    amplitude *= 1.0 - 0.38 * static_cast<double>(
                        parameters.channels[channel].chaos[band]) / 7.0;
                }
                const double phase = std::numbers::pi * angles[offset];
                const double rotated_real = amplitude * (carrier_real * std::cos(phase) -
                    carrier_imaginary * std::sin(phase));
                const double rotated_imaginary = amplitude *
                    (carrier_imaginary * std::cos(phase) + carrier_real * std::sin(phase));
                const auto synthesis = [](unsigned index) {
                    return std::cos(2.0 * std::numbers::pi * 128.5 / 512.0 *
                        (static_cast<double>(index) + 0.5));
                };
                const double coefficient = -2.0 * (synthesis(bin) * rotated_real +
                    synthesis(255U - bin) * rotated_imaginary);
                frame.blocks[block].coefficients[channel][bin] =
                    static_cast<std::int32_t>(std::clamp(coefficient,
                        static_cast<double>(std::numeric_limits<std::int32_t>::min()),
                        static_cast<double>(std::numeric_limits<std::int32_t>::max())));
            }
        }
    }
    if (decode_state != nullptr) {
        if (blocks != 0U && state.ecpl_blocks[blocks - 1U].enabled) {
            decode_state->previous_frame_mantissa =
                frame.blocks[blocks - 1U].coefficients[0];
        } else {
            decode_state->previous_frame_mantissa.reset();
        }
    }
}

} // namespace

double testing::ecpl_identity_gain() {
    SpectralFrame frame;
    DecodeState state;
    EcplStrategy strategy;
    strategy.begin = 4U;
    strategy.end = 22U;
    strategy.band_structure = default_ecpl_bands;
    unsigned bands = strategy.end - strategy.begin;
    for (unsigned subband = strategy.begin; subband < strategy.end; ++subband) {
        if (strategy.band_structure[subband]) --bands;
    }
    strategy.band_count = static_cast<std::uint8_t>(bands);
    const auto& window = ecpl_window();
    for (unsigned block = 0U; block < 3U; ++block) {
        auto& snapshot = state.ecpl_blocks[block];
        snapshot.enabled = true;
        snapshot.strategy = strategy;
        snapshot.channel_in_coupling[1] = true;
        std::array<double, 512> input{};
        for (unsigned sample = 0U; sample < 512U; ++sample) {
            const double time = static_cast<double>(block * 256U + sample);
            input[sample] = 1000000.0 * (0.4 * std::sin(2.0 *
                std::numbers::pi * 0.08 * time) + 0.3 * std::sin(2.0 *
                std::numbers::pi * 0.24 * time) + 0.2 * std::sin(2.0 *
                std::numbers::pi * 0.31 * time + 0.4));
            input[sample] *= window[sample < 256U ? sample : 511U - sample];
        }
        for (unsigned bin = strategy.begin == 0U ? 0U :
            static_cast<unsigned>(ecpl_subband_start[strategy.begin]);
            bin < ecpl_subband_start[strategy.end]; ++bin) {
            double value = 0.0;
            const double two_bin = static_cast<double>(2U * bin + 1U);
            for (unsigned sample = 0U; sample < 512U; ++sample) {
                const double phase = std::numbers::pi / 1024.0 *
                    static_cast<double>(2U * sample + 1U) * two_bin +
                    std::numbers::pi / 4.0 * two_bin;
                value += input[sample] * std::cos(phase);
            }
            frame.blocks[block].coefficients[0][bin] = static_cast<std::int32_t>(
                std::round(-value / 256.0));
        }
    }
    synthesize_ecpl(frame, state, 3U, 1U, nullptr);
    double product = 0.0;
    double energy = 0.0;
    for (unsigned bin = static_cast<unsigned>(
        ecpl_subband_start[strategy.begin] + 12U);
        bin + 12U < ecpl_subband_start[strategy.end]; ++bin) {
        const double expected = frame.blocks[1].coefficients[0][bin];
        product += expected * frame.blocks[1].coefficients[1][bin];
        energy += expected * expected;
    }
    return energy == 0.0 ? 0.0 : product / energy;
}

std::optional<SpectralFrame> decode_spectral_frame(
    std::span<const std::uint8_t> frame, const BitstreamInfo& info,
    const AudioFrameSyntax& syntax, DitherGenerator& dither, std::string* error,
    EcplDecodeState* ecpl_state) {
    if (frame.size() < info.frame_size) {
        if (error) *error = "truncated frame";
        return std::nullopt;
    }
    try {
        BitReader bits(frame.first(info.frame_size));
        bits.skip(syntax.audio_blocks_start_bit);
        DecodeState state;
        state.dither_generator = &dither;
        state.snr = syntax.snr_offset;
        state.first_spx_coordinates.fill(true);
        state.first_coupling_coordinates.fill(true);
        state.dither.fill(true);
        state.end.fill(coefficient_count);
        if (info.lfe) {
            state.end[syntax.lfe_channel] = 7U;
            state.exponent_groups[syntax.lfe_channel] = 2U;
            state.dither[syntax.lfe_channel] = false;
        }
        if (info.sample_rate == 48000U) state.allocation.sample_rate_code = 0U;
        else if (info.sample_rate == 44100U) state.allocation.sample_rate_code = 1U;
        else if (info.sample_rate == 32000U) state.allocation.sample_rate_code = 2U;
        else if (info.sample_rate == 24000U || info.sample_rate == 22050U ||
            info.sample_rate == 16000U) {
            // Annex E defines the reduced-rate header but no independent
            // public HTH table. Use the 32 kHz masking row, matching the
            // interoperable OxideAV interpretation and DolbyNet precedent.
            state.allocation.sample_rate_code = 2U;
        } else {
            if (error) *error = "unsupported sample rate";
            return std::nullopt;
        }

        SpectralFrame result;
        for (unsigned block = 0; block < info.blocks; ++block) {
            auto& output = result.blocks[block];
            if (syntax.block_switch_syntax) {
                for (unsigned channel = 1; channel <= syntax.full_bandwidth_channels; ++channel) {
                    output.block_switch[channel] = bits.read_bit();
                }
            }
            if (syntax.dither_flag_syntax) {
                for (unsigned channel = 1; channel <= syntax.full_bandwidth_channels; ++channel) {
                    state.dither[channel] = bits.read_bit();
                }
            }
            const unsigned programs = info.audio_coding_mode == 0U ? 2U : 1U;
            for (unsigned program = 0; program < programs; ++program) {
                if (bits.read_bit()) bits.skip(8U);
            }
            parse_spx_strategy(bits, block, info, syntax, state);
            result.spectral_extension_used = result.spectral_extension_used || state.spx_in_use;
            parse_coupling(bits, block, info, syntax, state);
            result.enhanced_coupling_used = result.enhanced_coupling_used ||
                state.ecpl_in_use;
            if (info.audio_coding_mode == 2U) parse_rematrix_strategy(
                bits, block, syntax, state);
            parse_exponents(bits, block, info, syntax, state);
            parse_allocation_data(bits, block, info, syntax, state);
            allocate_and_decode(bits, info, block, syntax, state, result,
                result.auxiliary_data);
            for (unsigned channel = 1; channel <=
                syntax.full_bandwidth_channels + (info.lfe ? 1U : 0U); ++channel) {
                result.nonzero_coefficients += static_cast<std::size_t>(std::count_if(
                    output.coefficients[channel].begin(),
                    output.coefficients[channel].end(),
                    [](std::int32_t value) { return value != 0; }));
            }
            output.end_bit = bits.position();
            ++result.decoded_blocks;
        }
        if (result.enhanced_coupling_used) synthesize_ecpl(result, state,
            info.blocks, syntax.full_bandwidth_channels, ecpl_state);
        else if (ecpl_state != nullptr) ecpl_state->previous_frame_mantissa.reset();
        // Annex E frame-end auxiliary data, separate from per-block skip fields.
        const auto frame_bits = info.frame_size * 8U;
        BitReader trailer(frame.first(info.frame_size));
        trailer.skip(frame_bits - 32U);
        const auto aux_bits = trailer.read(14);
        if (trailer.read_bit()) {
            if ((aux_bits & 7U) != 0 || aux_bits > frame_bits - 32U ||
                bits.position() > frame_bits - 32U - aux_bits)
                throw ParseError("invalid auxiliary data length");
            BitReader aux(frame.first(info.frame_size));
            aux.skip(frame_bits - 32U - aux_bits);
            for (unsigned index = 0; index < aux_bits / 8U; ++index)
                result.auxiliary_data.push_back(static_cast<std::uint8_t>(aux.read(8)));
        }
        return result;
    } catch (const ParseError& exception) {
        if (error) *error = exception.what();
        return std::nullopt;
    }
}

} // namespace atmos::eac3
