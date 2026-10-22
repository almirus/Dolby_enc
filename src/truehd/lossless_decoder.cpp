#include "decoder.hpp"

#include "access_unit.hpp"
#include "common/bit_reader.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <utility>

namespace atmos::truehd {
namespace {

constexpr unsigned max_channels = 16U;
constexpr unsigned max_samples = 160U;
constexpr std::uint8_t param_blocksize = 1U << 7U;
constexpr std::uint8_t param_matrix = 1U << 6U;
constexpr std::uint8_t param_outshift = 1U << 5U;
constexpr std::uint8_t param_quantstep = 1U << 4U;
constexpr std::uint8_t param_fir = 1U << 3U;
constexpr std::uint8_t param_iir = 1U << 2U;
constexpr std::uint8_t param_huffoffset = 1U << 1U;
constexpr std::uint8_t param_presence = 1U;

struct HuffCode final { std::uint16_t code; std::uint8_t bits; };
constexpr HuffCode huffman[3][18] = {
    {{1,9},{1,8},{1,7},{1,6},{1,5},{1,4},{1,3},{4,3},{5,3},{6,3},{7,3},
     {3,3},{5,4},{9,5},{17,6},{33,7},{65,8},{129,9}},
    {{1,9},{1,8},{1,7},{1,6},{1,5},{1,4},{1,3},{2,2},{3,2},
     {3,3},{5,4},{9,5},{17,6},{33,7},{65,8},{129,9},{0,0},{0,0}},
    {{1,9},{1,8},{1,7},{1,6},{1,5},{1,4},{1,3},{1,1},
     {3,3},{5,4},{9,5},{17,6},{33,7},{65,8},{129,9},{0,0},{0,0},{0,0}}
};
constexpr unsigned huffman_counts[3] = {18U, 16U, 15U};

constexpr std::int8_t noise_table[256] = {
 30,51,22,54,3,7,-4,38,14,55,46,81,22,58,-3,2,52,31,-7,51,15,44,74,30,85,-17,10,33,18,80,28,62,
 10,32,23,69,72,26,35,17,73,60,8,56,2,6,-2,-5,51,4,11,50,66,76,21,44,33,47,1,26,64,48,57,40,
 38,16,-10,-28,92,22,-18,29,-10,5,-13,49,19,24,70,34,61,48,30,14,-6,25,58,33,42,60,67,17,54,17,22,30,
 67,44,-9,50,-11,43,40,32,59,82,13,49,-14,55,60,36,48,49,31,47,15,12,4,65,1,23,29,39,45,-2,84,69,
 0,72,37,57,27,41,-15,-16,35,31,14,61,24,0,27,24,16,41,55,34,53,9,56,12,25,29,53,5,20,-20,-8,20,
 13,28,-3,78,38,16,11,62,46,29,21,24,46,65,43,-23,89,18,74,21,38,-12,19,12,-19,8,15,33,4,57,9,-8,
 36,35,26,28,7,83,63,79,75,11,3,87,37,47,34,40,39,19,20,42,27,34,39,77,13,42,59,64,45,-1,32,37,
 45,-5,53,-6,7,36,50,23,6,32,9,-21,18,71,27,52,-25,31,35,42,-1,68,63,52,26,43,66,37,41,25,40,70
};

std::int32_t shifted(std::int32_t value, unsigned amount) {
    return static_cast<std::int32_t>(static_cast<std::uint32_t>(value) << amount);
}

int read_huffman(BitReader& bits, unsigned codebook) {
    std::uint16_t code = 0;
    for (unsigned length = 1; length <= 9U; ++length) {
        code = static_cast<std::uint16_t>((code << 1U) | bits.read(1U));
        for (unsigned symbol = 0; symbol < huffman_counts[codebook]; ++symbol) {
            if (huffman[codebook][symbol].bits == length &&
                huffman[codebook][symbol].code == code) return static_cast<int>(symbol);
        }
    }
    throw ParseError("invalid TrueHD Huffman code");
}

unsigned wav_channel_rank(ChannelId channel) {
    switch (channel) {
    case ChannelId::FrontLeft: return 0U;
    case ChannelId::FrontRight: return 1U;
    case ChannelId::FrontCenter: return 2U;
    case ChannelId::Lfe: return 3U;
    case ChannelId::RearLeft: return 4U;
    case ChannelId::RearRight: return 5U;
    case ChannelId::SideLeft: return 6U;
    case ChannelId::SideRight: return 7U;
    case ChannelId::FrontLeftCenter: return 8U;
    case ChannelId::FrontRightCenter: return 9U;
    case ChannelId::RearCenter: return 10U;
    case ChannelId::TopCenter: return 11U;
    case ChannelId::TopFrontLeft: return 12U;
    case ChannelId::TopFrontCenter: return 13U;
    case ChannelId::TopFrontRight: return 14U;
    case ChannelId::WideLeft: return 15U;
    case ChannelId::WideRight: return 16U;
    }
    return std::numeric_limits<unsigned>::max();
}

} // namespace

struct BedDecoder::State final {
    struct Filter final {
        std::uint8_t order = 0;
        std::uint8_t shift = 0;
        std::array<std::int32_t, 8> coefficient{};
        std::array<std::int32_t, 8> history{};
    };
    struct Channel final {
        std::array<Filter, 2> filter{};
        std::int16_t huffman_offset = 0;
        std::int32_t signed_huffman_offset = -(1 << 23);
        std::uint8_t codebook = 0;
        std::uint8_t huffman_lsbs = 24;
    };
    struct Matrix final {
        std::uint8_t output_channel = 0;
        std::uint8_t noise_shift = 0;
        std::int8_t coefficient_shift = 0;
        std::uint8_t fractional_bits = 0;
        std::uint8_t lsb_bypass_bits = 0;
        std::uint8_t delta_bits = 0;
        std::uint8_t delta_precision = 0;
        std::uint16_t coefficient_mask = 0;
        bool lsb_bypass = false;
        std::array<std::int32_t, 18> coefficient{};
        std::array<std::int32_t, 18> delta{};
    };

    bool restart_seen = false;
    bool noise_type = false;
    bool object_substream = false;
    bool data_check_present = false;
    std::uint8_t min_channel = 0;
    std::uint8_t max_channel = 0;
    std::uint8_t max_matrix_channel = 0;
    std::uint8_t noise_shift = 0;
    std::uint8_t max_shift = 0;
    std::uint8_t max_lsbs = 24;
    std::uint32_t noise_seed = 0;
    std::uint8_t presence = 0xFFU;
    std::uint16_t block_size = 8;
    std::array<std::uint8_t, max_channels> assignment{};
    std::array<std::int8_t, max_channels> output_shift{};
    std::array<std::uint8_t, max_channels> quant_step{};
    std::array<Channel, max_channels> channel{};
    std::array<Matrix, 16> matrix{};
    std::array<std::int8_t, 256> dither_table{};
    std::uint8_t matrix_count = 0;
};

namespace {

void initialize_restart(BedDecoder::State& state, const RestartHeader& header) {
    state.restart_seen = true;
    state.noise_type = header.noise_type;
    state.object_substream = header.object_substream;
    state.data_check_present = header.data_check_present;
    state.min_channel = header.min_channel;
    state.max_channel = header.max_channel;
    state.max_matrix_channel = header.max_matrix_channel;
    state.noise_shift = header.noise_shift;
    state.max_shift = header.max_shift;
    state.max_lsbs = header.max_lsbs;
    state.noise_seed = header.noise_seed;
    state.presence = 0xFFU;
    state.block_size = 8U;
    state.matrix_count = 0U;
    state.output_shift.fill(0);
    state.quant_step.fill(0U);
    state.assignment.fill(0U);
    for (unsigned index = 0; index < header.channel_assignment.size(); ++index) {
        const unsigned assigned = header.channel_assignment[index];
        if (assigned <= state.max_matrix_channel) state.assignment[assigned] =
            static_cast<std::uint8_t>(index);
    }
    for (unsigned channel = state.min_channel; channel <= state.max_channel; ++channel) {
        state.channel[channel] = {};
    }
}

void read_filter(BitReader& bits, BedDecoder::State::Filter& filter, bool iir) {
    const std::size_t start_bit = bits.position();
    filter.order = static_cast<std::uint8_t>(bits.read(4U));
    if (filter.order > (iir ? 4U : 8U)) throw ParseError("invalid TrueHD filter order");
    if (filter.order == 0U) return;
    filter.shift = static_cast<std::uint8_t>(bits.read(4U));
    const unsigned coefficient_bits = bits.read(5U);
    const unsigned coefficient_shift = bits.read(3U);
    if (coefficient_bits == 0U || coefficient_bits > 16U ||
        coefficient_bits + coefficient_shift > 16U) {
        throw ParseError("invalid TrueHD filter coefficient precision at bit " +
            std::to_string(start_bit) + " (order " + std::to_string(filter.order) +
            ", bits " + std::to_string(coefficient_bits) + ", shift " +
            std::to_string(coefficient_shift) + ")");
    }
    for (unsigned index = 0; index < filter.order; ++index) {
        filter.coefficient[index] = shifted(bits.read_signed(coefficient_bits),
            coefficient_shift);
    }
    if (bits.read_bit()) {
        if (!iir) throw ParseError("FIR filter has state");
        const unsigned state_bits = bits.read(4U);
        const unsigned state_shift = bits.read(4U);
        for (unsigned index = 0; index < filter.order; ++index) {
            filter.history[index] = state_bits == 0U ? 0 :
                shifted(bits.read_signed(state_bits), state_shift);
        }
    }
}

void read_matrix(BitReader& bits, BedDecoder::State& state) {
    if (state.object_substream) {
        const bool new_matrix = bits.read_bit();
        if (new_matrix) {
            const bool new_configuration = bits.read_bit();
            if (new_configuration) {
                state.matrix_count = static_cast<std::uint8_t>(bits.read(4U) + 1U);
                if (state.matrix_count > state.matrix.size()) {
                    throw ParseError("too many TrueHD object matrices");
                }
                for (unsigned index = 0; index < state.matrix_count; ++index) {
                    auto& matrix = state.matrix[index];
                    const auto delta_bits = matrix.delta_bits;
                    const auto delta_precision = matrix.delta_precision;
                    const auto delta = matrix.delta;
                    matrix = {};
                    matrix.delta_bits = delta_bits;
                    matrix.delta_precision = delta_precision;
                    matrix.delta = delta;
                    matrix.output_channel = static_cast<std::uint8_t>(bits.read(4U));
                    matrix.fractional_bits = static_cast<std::uint8_t>(bits.read(4U));
                    matrix.coefficient_shift = static_cast<std::int8_t>(bits.read(3U)) - 1;
                    matrix.lsb_bypass_bits = static_cast<std::uint8_t>(bits.read(2U));
                    matrix.noise_shift = static_cast<std::uint8_t>(bits.read(4U));
                    matrix.coefficient_mask = static_cast<std::uint16_t>(
                        bits.read(state.max_matrix_channel + 1U));
                }
            }
            for (unsigned index = 0; index < state.matrix_count; ++index) {
                auto& matrix = state.matrix[index];
                for (unsigned channel = 0; channel <= state.max_matrix_channel; ++channel) {
                    if ((matrix.coefficient_mask & (1U << channel)) == 0U) {
                        matrix.coefficient[channel] = 0;
                        continue;
                    }
                    const int scale = 18 + matrix.coefficient_shift - matrix.fractional_bits;
                    const auto value = bits.read_signed(matrix.fractional_bits + 2U);
                    matrix.coefficient[channel] = scale >= 0 ? shifted(value,
                        static_cast<unsigned>(scale)) : value >> -scale;
                }
            }
        }
        const bool interpolation = bits.read_bit();
        if (interpolation) {
            const bool new_delta = bits.read_bit();
            if (new_delta) {
                if (bits.read_bit()) {
                    for (unsigned index = 0; index < state.matrix_count; ++index) {
                        state.matrix[index].delta_bits = static_cast<std::uint8_t>(bits.read(4U));
                        state.matrix[index].delta_precision = static_cast<std::uint8_t>(bits.read(2U));
                    }
                }
                for (unsigned index = 0; index < state.matrix_count; ++index) {
                    auto& matrix = state.matrix[index];
                    for (unsigned channel = 0; channel <= state.max_matrix_channel; ++channel) {
                        if (matrix.delta_bits == 0U ||
                            (matrix.coefficient_mask & (1U << channel)) == 0U) {
                            matrix.delta[channel] = 0;
                            continue;
                        }
                        const int scale = 18 + matrix.coefficient_shift -
                            matrix.fractional_bits - matrix.delta_precision;
                        const auto value = bits.read_signed(matrix.delta_bits + 1U);
                        matrix.delta[channel] = scale >= 0 ? shifted(value,
                            static_cast<unsigned>(scale)) : value >> -scale;
                    }
                }
            }
        } else {
            for (unsigned index = 0; index < state.matrix_count; ++index) {
                state.matrix[index].delta.fill(0);
            }
        }
        return;
    }
    state.matrix_count = static_cast<std::uint8_t>(bits.read(4U));
    if (state.matrix_count > state.matrix.size()) throw ParseError("too many TrueHD matrices");
    for (unsigned index = 0; index < state.matrix_count; ++index) {
        auto& matrix = state.matrix[index];
        matrix = {};
        matrix.output_channel = static_cast<std::uint8_t>(bits.read(4U));
        const unsigned fractional_bits = bits.read(4U);
        matrix.lsb_bypass = bits.read_bit();
        if (matrix.output_channel > state.max_matrix_channel || fractional_bits > 14U) {
            throw ParseError("invalid TrueHD matrix");
        }
        const unsigned maximum = state.max_matrix_channel + (state.noise_type ? 0U : 2U);
        for (unsigned channel = 0; channel <= maximum; ++channel) {
            if (bits.read_bit()) {
                matrix.coefficient[channel] = shifted(
                    bits.read_signed(fractional_bits + 2U), 14U - fractional_bits);
            }
        }
        if (state.noise_type) matrix.noise_shift = static_cast<std::uint8_t>(bits.read(4U));
    }
}

void recompute_offset(BedDecoder::State& state, unsigned channel) {
    auto& parameters = state.channel[channel];
    const int lsb_bits = static_cast<int>(parameters.huffman_lsbs) -
        static_cast<int>(state.quant_step[channel]);
    const int sign_shift = lsb_bits + (parameters.codebook != 0U ?
        2 - static_cast<int>(parameters.codebook) : -1);
    std::int32_t offset = parameters.huffman_offset;
    if (parameters.codebook != 0U) offset -= shifted(7, static_cast<unsigned>(lsb_bits));
    if (sign_shift >= 0) offset -= shifted(1, static_cast<unsigned>(sign_shift));
    parameters.signed_huffman_offset = offset;
}

void read_parameters(BitReader& bits, BedDecoder::State& state, unsigned access_unit_size) {
    if ((state.presence & param_presence) != 0U && bits.read_bit()) {
        state.presence = static_cast<std::uint8_t>(bits.read(8U));
    }
    if ((state.presence & param_blocksize) != 0U && bits.read_bit()) {
        state.block_size = static_cast<std::uint16_t>(bits.read(9U));
        if (state.block_size < 8U || state.block_size > access_unit_size) {
            throw ParseError("invalid TrueHD block size");
        }
    }
    if ((state.presence & param_matrix) != 0U && bits.read_bit()) read_matrix(bits, state);
    if ((state.presence & param_outshift) != 0U && bits.read_bit()) {
        for (unsigned channel = 0; channel <= state.max_matrix_channel; ++channel) {
            const int value = bits.read_signed(4U);
            if (!state.object_substream && value < 0) {
                throw ParseError("negative TrueHD output shift");
            }
            if (value > state.max_shift) {
                throw ParseError("TrueHD output shift exceeds restart-header maximum");
            }
            state.output_shift[channel] = static_cast<std::int8_t>(value);
        }
    }
    std::uint32_t recompute = 0;
    if ((state.presence & param_quantstep) != 0U && bits.read_bit()) {
        for (unsigned channel = 0; channel <= state.max_channel; ++channel) {
            state.quant_step[channel] = static_cast<std::uint8_t>(bits.read(4U));
            recompute |= 1U << channel;
        }
    }
    for (unsigned channel = state.min_channel; channel <= state.max_channel; ++channel) {
        if (!bits.read_bit()) continue;
        recompute |= 1U << channel;
        auto& parameters = state.channel[channel];
        if ((state.presence & param_fir) != 0U && bits.read_bit()) {
            read_filter(bits, parameters.filter[0], false);
        }
        if ((state.presence & param_iir) != 0U && bits.read_bit()) {
            read_filter(bits, parameters.filter[1], true);
        }
        if (parameters.filter[0].order + parameters.filter[1].order > 8U) {
            throw ParseError("TrueHD filter order sum too large");
        }
        if (parameters.filter[0].order == 0U && parameters.filter[1].order != 0U) {
            parameters.filter[0].shift = parameters.filter[1].shift;
        }
        if ((state.presence & param_huffoffset) != 0U && bits.read_bit()) {
            parameters.huffman_offset = static_cast<std::int16_t>(bits.read_signed(15U));
        }
        parameters.codebook = static_cast<std::uint8_t>(bits.read(2U));
        parameters.huffman_lsbs = static_cast<std::uint8_t>(bits.read(5U));
        const unsigned maximum_lsbs = state.object_substream ? 31U : 24U;
        if (parameters.codebook > 3U || parameters.huffman_lsbs > maximum_lsbs) {
            throw ParseError("invalid TrueHD channel coding parameters");
        }
    }
    for (unsigned channel = 0; channel <= state.max_channel; ++channel) {
        if ((recompute & (1U << channel)) == 0U) continue;
        if (state.channel[channel].codebook != 0U &&
            state.channel[channel].huffman_lsbs < state.quant_step[channel]) {
            throw ParseError("TrueHD quant step exceeds Huffman LSB count");
        }
        recompute_offset(state, channel);
    }
}

void filter_channel(BedDecoder::State& state, unsigned channel,
    std::vector<std::array<std::int32_t, max_channels>>& samples,
    std::size_t start) {
    auto& parameters = state.channel[channel];
    auto& fir = parameters.filter[0];
    auto& iir = parameters.filter[1];
    const std::uint32_t mask = 0U - (1U << state.quant_step[channel]);
    for (unsigned position = 0; position < state.block_size; ++position) {
        std::int64_t accumulator = 0;
        for (unsigned order = 0; order < fir.order; ++order) {
            accumulator += static_cast<std::int64_t>(fir.history[order]) * fir.coefficient[order];
        }
        for (unsigned order = 0; order < iir.order; ++order) {
            accumulator += static_cast<std::int64_t>(iir.history[order]) * iir.coefficient[order];
        }
        accumulator >>= fir.shift;
        const auto prediction = static_cast<std::int32_t>(accumulator);
        const std::uint32_t combined = static_cast<std::uint32_t>(prediction) +
            static_cast<std::uint32_t>(samples[start + position][channel]);
        const auto result = static_cast<std::int32_t>(combined & mask);
        for (unsigned order = 7U; order > 0U; --order) {
            fir.history[order] = fir.history[order - 1U];
            iir.history[order] = iir.history[order - 1U];
        }
        fir.history[0] = result;
        iir.history[0] = static_cast<std::int32_t>(
            static_cast<std::uint32_t>(result) - static_cast<std::uint32_t>(prediction));
        samples[start + position][channel] = result;
    }
}

void read_residual_block(BitReader& bits, BedDecoder::State& state,
    std::vector<std::array<std::int32_t, max_channels>>& samples,
    std::vector<std::array<std::uint8_t, 16>>& bypass, std::size_t start) {
    std::size_t expected = 0;
    if (state.data_check_present) expected = bits.position() + bits.read(16U);
    for (unsigned position = 0; position < state.block_size; ++position) {
        for (unsigned matrix = 0; matrix < state.matrix_count; ++matrix) {
            const unsigned count = state.object_substream ?
                state.matrix[matrix].lsb_bypass_bits :
                (state.matrix[matrix].lsb_bypass ? 1U : 0U);
            if (count != 0U) bypass[start + position][matrix] =
                static_cast<std::uint8_t>(bits.read(count));
        }
        for (unsigned channel = state.min_channel; channel <= state.max_channel; ++channel) {
            const auto& parameters = state.channel[channel];
            const int lsb_bits = static_cast<int>(parameters.huffman_lsbs) -
                static_cast<int>(state.quant_step[channel]);
            std::int32_t value = 0;
            if (parameters.codebook != 0U) value = read_huffman(bits, parameters.codebook - 1U);
            if (lsb_bits > 0) {
                value = shifted(value, static_cast<unsigned>(lsb_bits));
                value = static_cast<std::int32_t>(static_cast<std::uint32_t>(value) +
                    bits.read(static_cast<unsigned>(lsb_bits)));
            }
            value = static_cast<std::int32_t>(static_cast<std::uint32_t>(value) +
                static_cast<std::uint32_t>(parameters.signed_huffman_offset));
            samples[start + position][channel] = shifted(value, state.quant_step[channel]);
        }
    }
    for (unsigned channel = state.min_channel; channel <= state.max_channel; ++channel) {
        filter_channel(state, channel, samples, start);
    }
    if (state.data_check_present) {
        if (bits.position() != expected) throw ParseError("TrueHD block length mismatch");
        bits.skip(8U);
    }
}

void apply_matrices(BedDecoder::State& state,
    std::vector<std::array<std::int32_t, max_channels>>& samples,
    const std::vector<std::array<std::uint8_t, 16>>& bypass,
    std::size_t sample_count, unsigned access_unit_size_pow2) {
    std::vector<std::int8_t> noise(access_unit_size_pow2);
    std::uint32_t seed = state.noise_seed;
    unsigned maximum_source = state.max_matrix_channel;
    if (!state.noise_type && !state.object_substream) {
        maximum_source += 2U;
        for (std::size_t sample = 0; sample < sample_count; ++sample) {
            const std::uint16_t seed_high = static_cast<std::uint16_t>(seed >> 7U);
            samples[sample][state.max_matrix_channel + 1U] = shifted(
                static_cast<std::int8_t>(seed >> 15U), state.noise_shift);
            samples[sample][state.max_matrix_channel + 2U] = shifted(
                static_cast<std::int8_t>(seed_high), state.noise_shift);
            seed = (seed << 16U) ^ seed_high ^ (static_cast<std::uint32_t>(seed_high) << 5U);
        }
    } else {
        for (unsigned index = 0; index < access_unit_size_pow2; ++index) {
            const std::uint8_t seed_high = static_cast<std::uint8_t>(seed >> 15U);
            noise[index] = noise_table[seed_high];
            seed = (seed << 8U) ^ seed_high ^ (static_cast<std::uint32_t>(seed_high) << 5U);
        }
    }
    state.noise_seed = seed;

    for (unsigned matrix_index = 0; matrix_index < state.matrix_count; ++matrix_index) {
        const auto& matrix = state.matrix[matrix_index];
        const std::uint32_t mask = 0U - (1U << state.quant_step[matrix.output_channel]);
        unsigned noise_index = state.matrix_count - matrix_index;
        const unsigned noise_step = 2U * noise_index + 1U;
        for (std::size_t sample = 0; sample < sample_count; ++sample) {
            std::int64_t accumulator = 0;
            std::int64_t delta_accumulator = 0;
            for (unsigned source = 0; source <= maximum_source; ++source) {
                accumulator += static_cast<std::int64_t>(samples[sample][source]) *
                    matrix.coefficient[source];
                if (state.object_substream) {
                    delta_accumulator += static_cast<std::int64_t>(samples[sample][source]) *
                        matrix.delta[source];
                }
            }
            if (matrix.noise_shift != 0U) {
                noise_index &= access_unit_size_pow2 - 1U;
                const unsigned base_shift = state.object_substream ? 11U : 7U;
                accumulator += static_cast<std::int64_t>(noise[noise_index]) *
                    (static_cast<std::int64_t>(1) << (matrix.noise_shift + base_shift));
                noise_index += noise_step;
            }
            if (state.object_substream) {
                const std::int64_t reciprocal = (static_cast<std::int64_t>(1) << 16U) /
                    static_cast<std::int64_t>(sample_count);
                accumulator += (delta_accumulator >> 18U) *
                    static_cast<std::int64_t>(sample) * (reciprocal << 2U);
            }
            const unsigned matrix_shift = state.object_substream ? 18U : 14U;
            const auto reconstructed = static_cast<std::int32_t>(accumulator >> matrix_shift);
            samples[sample][matrix.output_channel] = static_cast<std::int32_t>(
                (static_cast<std::uint32_t>(reconstructed) & mask) +
                bypass[sample][matrix_index]);
        }
    }
    if (state.object_substream) {
        for (unsigned matrix_index = 0; matrix_index < state.matrix_count; ++matrix_index) {
            for (unsigned channel = 0; channel <= state.max_matrix_channel; ++channel) {
                state.matrix[matrix_index].coefficient[channel] = static_cast<std::int32_t>(
                    static_cast<std::uint32_t>(state.matrix[matrix_index].coefficient[channel]) +
                    static_cast<std::uint32_t>(state.matrix[matrix_index].delta[channel]));
            }
        }
    }
}

void apply_object_matrix_block(BedDecoder::State& state,
    std::vector<std::array<std::int32_t, max_channels>>& samples,
    const std::vector<std::array<std::uint8_t, 16>>& bypass,
    std::size_t start, std::size_t block_size, unsigned access_unit_size) {
    unsigned table_size = 1U;
    while (table_size < access_unit_size) table_size <<= 1U;
    if (start == 0U) {
        std::uint32_t seed = state.noise_seed;
        for (unsigned index = 0; index < table_size; ++index) {
            const std::uint8_t seed_high = static_cast<std::uint8_t>(seed >> 15U);
            state.dither_table[index] = noise_table[seed_high];
            seed = (seed << 8U) ^ seed_high ^
                (static_cast<std::uint32_t>(seed_high) << 5U);
        }
        state.noise_seed = seed;
    }

    const unsigned table_mask = table_size - 1U;
    const std::int64_t reciprocal = (static_cast<std::int64_t>(1) << 16U) /
        static_cast<std::int64_t>(access_unit_size);
    for (unsigned matrix_index = 0; matrix_index < state.matrix_count; ++matrix_index) {
        const auto& matrix = state.matrix[matrix_index];
        const std::uint32_t quantizer_mask = 0U -
            (1U << state.quant_step[matrix.output_channel]);
        for (std::size_t position = 0; position < block_size; ++position) {
            const std::size_t absolute = start + position;
            std::int64_t accumulator = 0;
            std::int64_t delta_accumulator = 0;
            for (unsigned source = 0; source <= state.max_matrix_channel; ++source) {
                accumulator += static_cast<std::int64_t>(samples[absolute][source]) *
                    matrix.coefficient[source];
                delta_accumulator += static_cast<std::int64_t>(samples[absolute][source]) *
                    matrix.delta[source];
            }
            if (matrix.noise_shift != 0U) {
                const std::size_t dither_index =
                    (state.matrix_count - matrix_index) * (2U * absolute + 1U) + absolute;
                accumulator += static_cast<std::int64_t>(
                    state.dither_table[dither_index & table_mask]) << (11U + matrix.noise_shift);
            }
            accumulator += (delta_accumulator >> 18U) *
                static_cast<std::int64_t>(absolute) * (reciprocal << 2U);
            const auto reconstructed = static_cast<std::int32_t>(accumulator >> 18U);
            samples[absolute][matrix.output_channel] = static_cast<std::int32_t>(
                (static_cast<std::uint32_t>(reconstructed) & quantizer_mask) +
                bypass[absolute][matrix_index]);
        }
    }
    if (start + block_size == access_unit_size) {
        for (unsigned matrix_index = 0; matrix_index < state.matrix_count; ++matrix_index) {
            for (unsigned channel = 0; channel <= state.max_matrix_channel; ++channel) {
                state.matrix[matrix_index].coefficient[channel] = static_cast<std::int32_t>(
                    static_cast<std::uint32_t>(state.matrix[matrix_index].coefficient[channel]) +
                    static_cast<std::uint32_t>(state.matrix[matrix_index].delta[channel]));
            }
        }
    }
}

void make_pcm_frame(const MajorSync& stream_info, unsigned presentation,
    const BedDecoder::State& state,
    const std::vector<std::array<std::int32_t, max_channels>>& samples,
    std::size_t sample_count, PcmFrame& output) {
    output.sample_rate = stream_info.sample_rate;
    output.channels = static_cast<std::uint8_t>(state.max_matrix_channel + 1U);
    output.samples_per_channel = static_cast<std::uint16_t>(sample_count);
    output.interleaved.resize(sample_count * output.channels);
    std::array<unsigned, max_channels> destination{};
    if (presentation == 3U || stream_info.channels.size() < output.channels) {
        for (unsigned channel = 0; channel < output.channels; ++channel) {
            destination[channel] = channel;
        }
    } else {
        std::vector<std::pair<unsigned, unsigned>> channel_order;
        for (unsigned channel = 0; channel < output.channels; ++channel) {
            channel_order.emplace_back(
                wav_channel_rank(stream_info.channels[channel]), channel);
        }
        std::sort(channel_order.begin(), channel_order.end());
        for (std::size_t index = 0; index < channel_order.size(); ++index) {
            destination[channel_order[index].second] = static_cast<unsigned>(index);
        }
    }
    constexpr float scale = 1.0F / 8388608.0F;
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
        for (unsigned channel = 0; channel < output.channels; ++channel) {
            const unsigned source = state.assignment[channel];
            const int output_shift = state.output_shift[source];
            const auto shifted_sample = output_shift >= 0 ?
                shifted(samples[sample][source], static_cast<unsigned>(output_shift)) :
                samples[sample][source] >> -output_shift;
            output.interleaved[sample * output.channels + destination[channel]] =
                static_cast<float>(shifted_sample) * scale;
        }
    }
}

} // namespace

BedDecoder::BedDecoder(unsigned presentation) : presentation_(presentation) {
    if (presentation_ > 3U) presentation_ = 3U;
    for (auto& state : states_) state = new State();
}

BedDecoder::~BedDecoder() {
    for (auto* state : states_) delete state;
}

void BedDecoder::reset() noexcept {
    stream_info_.reset();
    for (auto* state : states_) *state = {};
}

bool BedDecoder::decode(std::span<const std::uint8_t> bytes,
    PcmFrame& output, std::string& error) {
    return decode_internal(bytes, output, nullptr, error);
}

bool BedDecoder::decode_with_presentation2(std::span<const std::uint8_t> bytes,
    PcmFrame& output, PcmFrame& presentation2, std::string& error) {
    return decode_internal(bytes, output, &presentation2, error);
}

bool BedDecoder::decode_internal(std::span<const std::uint8_t> bytes,
    PcmFrame& output, PcmFrame* presentation2, std::string& error) {
    try {
        if (presentation2 != nullptr) *presentation2 = {};
        const auto unit = parse_access_unit(bytes, 0U, stream_info_ ? &*stream_info_ : nullptr);
        if (!unit) throw ParseError("invalid TrueHD access unit");
        if (unit->major_sync) stream_info_ = unit->major_sync;
        if (!stream_info_ || unit->substreams.empty()) throw ParseError("TrueHD major sync missing");
        const unsigned target = std::min<unsigned>(presentation_,
            static_cast<unsigned>(unit->substreams.size() - 1U));
        std::vector<std::array<std::int32_t, max_channels>> samples(max_samples);
        std::vector<std::array<std::uint8_t, 16>> bypass(max_samples);
        std::size_t sample_count = 0;
        for (unsigned substream = 0; substream <= target; ++substream) {
            auto& current_state = *states_[substream];
            const auto& entry = unit->substreams[substream];
            BitReader bits(bytes.subspan(entry.data_offset, entry.data_size));
            std::vector<std::array<std::uint8_t, 16>> local_bypass(max_samples);
            auto& current_bypass = substream == target ? bypass : local_bypass;
            std::size_t current_sample_count = 0;
            unsigned block_index = 0;
            bool end = false;
            while (!end) {
                if (bits.read_bit()) {
                    if (bits.read_bit()) {
                        const auto restart = parse_restart_header(
                            bytes.subspan(entry.data_offset, entry.data_size));
                        if (!restart) throw ParseError("invalid TrueHD restart header");
                        initialize_restart(current_state, *restart);
                        bits = BitReader(bytes.subspan(entry.data_offset, entry.data_size));
                        bits.skip(restart->end_bit);
                    }
                    if (!current_state.restart_seen) {
                        throw ParseError("TrueHD restart header not seen");
                    }
                    try {
                        read_parameters(bits, current_state,
                            stream_info_->samples_per_access_unit);
                    } catch (const ParseError& exception) {
                        throw ParseError("substream " + std::to_string(substream) +
                            " block " + std::to_string(block_index) + " parameter bit " +
                            std::to_string(bits.position()) + ": " + exception.what());
                    }
                }
                if (!current_state.restart_seen) {
                    throw ParseError("TrueHD restart state unavailable");
                }
                if (current_sample_count + current_state.block_size > samples.size()) {
                    throw ParseError("too many TrueHD samples in access unit");
                }
                read_residual_block(bits, current_state, samples, current_bypass,
                    current_sample_count);
                if (substream == target && current_state.object_substream) {
                    apply_object_matrix_block(current_state, samples, current_bypass,
                        current_sample_count, current_state.block_size,
                        stream_info_->samples_per_access_unit);
                }
                current_sample_count += current_state.block_size;
                end = bits.read_bit();
                ++block_index;
            }
            const std::size_t alignment = (16U - (bits.position() & 15U)) & 15U;
            bits.skip(alignment);
            const std::size_t checksum_bits = entry.check_data_present ? 16U : 0U;
            unsigned end_padding = 0;
            if (bits.remaining() >= checksum_bits + 32U) {
                if (bits.read(18U) != 0x348D3U) {
                    throw ParseError("invalid TrueHD substream terminator");
                }
                const bool padding_present = bits.read_bit();
                const auto count = bits.read(13U);
                if (padding_present) {
                    if (count >= stream_info_->samples_per_access_unit || count > current_sample_count)
                        throw ParseError("invalid TrueHD end padding");
                    end_padding = count;
                } else if (count != 0x1234U) {
                    throw ParseError("invalid TrueHD termination word");
                }
            }
            if (bits.remaining() != checksum_bits) {
                throw ParseError("TrueHD substream length mismatch in substream " +
                    std::to_string(substream) + " (bit " +
                    std::to_string(bits.position()) + " of " +
                    std::to_string(entry.data_size * 8U) + ")");
            }
            bits.skip(checksum_bits);
            if (presentation2 != nullptr && presentation_ == 3U &&
                target == 3U && substream == 2U) {
                auto bed_samples = samples;
                if (!current_state.object_substream) {
                    unsigned rate_shift = 0U;
                    for (unsigned size = stream_info_->samples_per_access_unit;
                        size > 40U; size >>= 1U) {
                        ++rate_shift;
                    }
                    apply_matrices(current_state, bed_samples, current_bypass,
                        current_sample_count, 64U << rate_shift);
                }
                make_pcm_frame(*stream_info_, 2U, current_state, bed_samples,
                    current_sample_count-end_padding, *presentation2);
            }
            if (substream == target) {
                sample_count = current_sample_count-end_padding;
            } else if (current_sample_count != stream_info_->samples_per_access_unit) {
                throw ParseError("TrueHD substream sample count mismatch");
            }
        }

        auto& state = *states_[target];
        unsigned rate_shift = 0;
        for (unsigned size = stream_info_->samples_per_access_unit; size > 40U; size >>= 1U) {
            ++rate_shift;
        }
        if (!state.object_substream) {
            apply_matrices(state, samples, bypass, sample_count, 64U << rate_shift);
        }
        make_pcm_frame(*stream_info_, target, state, samples, sample_count, output);
        error.clear();
        return true;
    } catch (const ParseError& exception) {
        error = exception.what();
        return false;
    }
}

} // namespace atmos::truehd
