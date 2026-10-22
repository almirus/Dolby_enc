#include "encode/eac3_core.hpp"

#include "common/bit_writer.hpp"
#include "common/crc.hpp"
#include "eac3/bit_allocation.hpp"
#include "eac3/exponents.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

namespace atmos::encode {
namespace {

constexpr unsigned block_samples = 256U;
constexpr unsigned coefficient_count = 256U;
constexpr std::size_t allocation_coefficients = eac3::coefficient_count; // 253
// Channels are addressed 1-based (index 0 is the coupling channel, unused here),
// so every per-channel table needs one slot more than the channel count.
constexpr unsigned max_channels = 7U;
constexpr unsigned max_blocks = 6U;

// Mantissa code width per bit allocation pointer (ETSI TS 102 366 table 7.13).
constexpr std::array<unsigned, 16> mantissa_bits = {
    0U, 5U, 7U, 3U, 7U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 14U, 16U
};

constexpr std::array<std::uint32_t, 3> full_rates = {48000U, 44100U, 32000U};
constexpr std::array<std::uint8_t, 4> block_counts = {1U, 2U, 3U, 6U};

double bessel_i0(double value) {
    const double half = value * 0.5;
    double sum = 1.0;
    double term = 1.0;
    for (unsigned order = 1; order < 40U; ++order) {
        term *= (half * half) /
            (static_cast<double>(order) * static_cast<double>(order));
        sum += term;
        if (term < sum * 1.0e-16) break;
    }
    return sum;
}

// Same KBD profile the decoder in this tree uses. The forward transform applies
// it squared, which turns the decoder's overlap-add into an identity.
const std::array<double, 256>& kbd_window() {
    static const auto table = [] {
        constexpr std::size_t size = 256U;
        constexpr double alpha = 5.0;
        std::array<double, size> cumulative{};
        double total = 0.0;
        for (std::size_t index = 0; index < size; ++index) {
            const double position = (2.0 * static_cast<double>(index)) /
                static_cast<double>(size) - 1.0;
            total += bessel_i0(std::numbers::pi * alpha *
                std::sqrt(std::max(0.0, 1.0 - position * position)));
            cumulative[index] = total;
        }
        total += 1.0;
        for (double& value : cumulative) value = std::sqrt(value / total);
        return cumulative;
    }();
    return table;
}

// Forward MDCT as the exact adjoint of atmos::eac3::Imdct256: the decoder
// reconstructs block t from the current block's first half and the previous
// block's second half, so its adjoint analyses [current(256), next(256)].
//
// The basis never changes and evaluating cos() inside the loop dominated the
// encoder: 256 bins x 256 terms x 36 transforms per frame. Hoisting it into a
// table is bit-exact and removes about a billion cos() calls per ten seconds.
const std::array<std::array<double, coefficient_count>, coefficient_count>& mdct_cosines() {
    static const auto table = [] {
        std::array<std::array<double, coefficient_count>, coefficient_count> result{};
        for (unsigned bin = 0; bin < coefficient_count; ++bin) {
            const double scale = std::numbers::pi / 256.0 *
                (static_cast<double>(bin) + 0.5);
            for (unsigned n = 0; n < coefficient_count; ++n)
                result[bin][n] = std::cos(scale * (static_cast<double>(n) + 256.5));
        }
        return result;
    }();
    return table;
}

void forward_mdct(std::span<const double, 512> time,
    std::span<float, coefficient_count> output) {
    const auto& window = kbd_window();
    const auto& cosines = mdct_cosines();
    std::array<double, coefficient_count> windowed{};
    for (unsigned r = 0; r < 128U; ++r) {
        windowed[r] = -window[127U - r] * time[127U - r] +
            window[128U + r] * time[128U + r];
        windowed[128U + r] = window[255U - r] * time[256U + r] +
            window[r] * time[256U + 255U - r];
    }
    for (unsigned bin = 0; bin < coefficient_count; ++bin) {
        const auto& basis = cosines[bin];
        double sum = 0.0;
        for (unsigned n = 0; n < coefficient_count; ++n)
            sum += basis[n] * windowed[n];
        output[bin] = static_cast<float>(sum);
    }
}

std::uint8_t sample_rate_code(std::uint32_t rate) {
    for (std::uint8_t code = 0; code < full_rates.size(); ++code)
        if (full_rates[code] == rate) return code;
    throw std::runtime_error("unsupported E-AC-3 sample rate");
}

std::uint8_t blocks_code(unsigned blocks) {
    for (std::uint8_t code = 0; code < block_counts.size(); ++code)
        if (block_counts[code] == blocks) return code;
    throw std::runtime_error("unsupported E-AC-3 block count");
}

// Coded bandwidth from the bitrate. The differential exponents are the fixed
// cost of a wide spectrum: D45 spends about 17.5 bits per coded bin per frame
// for six blocks and five channels, while a frame only holds
// bitrate * 1536 / sample_rate bits. Coding every one of the 253 bins at
// 384 kbps therefore spends over a third of the frame on exponents alone;
// keeping them near a quarter of it buys ~2 dB of mantissa precision at the
// low DD+ rates. The 25% target and the 17.5-bit estimate are calibrated on
// this encoder, not taken from the specification.
unsigned bandwidth_for(std::uint32_t bitrate, std::uint32_t sample_rate) {
    const double frame_bits = static_cast<double>(bitrate) * 1536.0 /
        static_cast<double>(sample_rate);
    const double bins = frame_bits * 0.25 / 17.5;
    const int code = static_cast<int>((bins - 73.0) / 3.0);
    return static_cast<unsigned>(std::clamp(code, 12, 60));
}

// Left shifts needed to bring a coefficient just under 2^23 (the decoder
// divides the mantissa back by 2^exponent).
int normalization_exponent(std::int64_t magnitude) {
    if (magnitude <= 0) return 24;
    int bits = 0;
    while (magnitude > 0) {
        magnitude >>= 1;
        ++bits;
    }
    return std::clamp(23 - bits, 0, 24);
}

// Bins each transmitted exponent delta is repeated over: D15 -> 1, D25 -> 2,
// D45 -> 4 (ETSI TS 102 366 Annex E). The decoder derives the group count from
// the same value, so encoder and decoder must agree.
unsigned exponent_group_size(eac3::ExponentStrategy strategy) {
    switch (strategy) {
    case eac3::ExponentStrategy::D25: return 2U;
    case eac3::ExponentStrategy::D45: return 4U;
    default: return 1U;
    }
}

// The 7-bit exponent code carries three deltas in [-2, 2].
unsigned exponent_code(int first, int second, int third) {
    const int a = std::clamp(first, -2, 2) + 2;
    const int b = std::clamp(second, -2, 2) + 2;
    const int c = std::clamp(third, -2, 2) + 2;
    return static_cast<unsigned>(a * 25 + b * 5 + c);
}

/// Inverse of the decoder's dequantiser. Grouped pointers 1..5 use the small odd
/// level counts (3/5/7/11/15); 6..15 use the linear quantiser widths.
std::int32_t mantissa_code(std::int32_t value, unsigned bap) {
    if (bap == 0U || bap > 15U) return 0;
    if (bap <= 5U) {
        static constexpr std::array<std::int64_t, 6> odd_levels = {0,3,5,7,11,15};
        const std::int64_t levels = odd_levels[bap];
        const std::int64_t code = std::llround(
            static_cast<double>(value) * static_cast<double>(levels) / 16777216.0) +
            levels / 2;
        return static_cast<std::int32_t>(std::clamp<std::int64_t>(code, 0, levels - 1));
    }
    const unsigned bits = mantissa_bits[bap];
    const std::int64_t levels = 1LL << bits;
    const std::int64_t code = std::llround(
        static_cast<double>(value) / static_cast<double>(1LL << (24U - bits)));
    return static_cast<std::int32_t>(std::clamp(code, -levels / 2, levels / 2 - 1));
}

} // namespace

struct CoreEncoder::Impl final {
    CoreConfig config{};
    std::uint8_t channels = 0;
    std::uint8_t full_bandwidth = 0;
    std::uint8_t lfe_channel = 0;
    std::size_t frame_size = 0;
    std::size_t frame_bits = 0;

    std::vector<float> pending;
    std::array<double, 512> time{};
    std::vector<std::uint8_t> aux_payload;
    bool skip_field = false;
    std::size_t payload_start_bit = 0;

    std::array<std::array<std::array<std::int32_t, coefficient_count>, max_blocks>,
        max_channels> coef{};
    std::array<std::array<std::size_t, max_blocks>, max_channels> end_bin{};
    std::array<std::array<eac3::ExponentStrategy, max_blocks>, max_channels> strategy{};
    std::array<std::array<std::array<std::int8_t, allocation_coefficients>, max_blocks>,
        max_channels> exponents{};
    std::array<std::array<std::array<std::uint8_t, allocation_coefficients>, max_blocks>,
        max_channels> bap{};
    std::array<std::array<std::int16_t, max_blocks>, max_channels> snr_offset{};
    std::array<std::array<std::array<std::int32_t, coefficient_count>, max_blocks>,
        max_channels> mantissa{};

    // Mantissa cost of the whole frame. Bit-allocation pointers 1, 2 and 4 are
    // grouped codes (5 bits per three values, 7 per three, 7 per two) and their
    // group counters run across all channels of a block, so the cost has to be
    // rounded up per block over the combined counts. Summing per-coefficient
    // widths instead overestimates the frame by ~30% and makes the SNR search
    // pick an offset that leaves the same amount of the frame unused.
    [[nodiscard]] std::size_t allocation_bits(unsigned blocks) const {
        std::size_t bits = 0;
        for (unsigned block = 0; block < blocks; ++block) {
            std::array<std::size_t, 16> count{};
            for (unsigned channel = 1; channel <= channels; ++channel)
                for (std::size_t bin = 0; bin < end_bin[channel][block]; ++bin)
                    ++count[bap[channel][block][bin]];
            bits += 5U * ((count[1] + 2U) / 3U);
            bits += 7U * ((count[2] + 2U) / 3U);
            bits += 7U * ((count[4] + 1U) / 2U);
            for (std::size_t pointer = 3U; pointer < count.size(); ++pointer)
                if (pointer != 4U) bits += count[pointer] * mantissa_bits[pointer];
        }
        return bits;
    }
};

CoreEncoder::CoreEncoder() : impl_(std::make_unique<Impl>()) {}
CoreEncoder::~CoreEncoder() = default;
CoreEncoder::CoreEncoder(CoreEncoder&&) noexcept = default;
CoreEncoder& CoreEncoder::operator=(CoreEncoder&&) noexcept = default;

void CoreEncoder::configure(const CoreConfig& config) {
    if (config.audio_coding_mode == 0U || config.audio_coding_mode > 7U ||
        config.stream_type > 1U || config.substream_id > 7U ||
        config.bitstream_id < 11U || config.bitstream_id > 16U ||
        config.dialnorm == 0U || config.dialnorm > 31U)
        throw std::runtime_error("unsupported E-AC-3 configuration");
    static_cast<void>(sample_rate_code(config.sample_rate));
    if (config.bitrate < 32000U || config.bitrate > 1024000U)
        throw std::runtime_error("E-AC-3 bitrate must be 32000..1024000 bps");
    auto& self = *impl_;
    self.config = config;
    if (self.config.blocks != 1U && self.config.blocks != 2U &&
        self.config.blocks != 3U && self.config.blocks != 6U) {
        throw std::runtime_error("unsupported E-AC-3 block count");
    }
    static constexpr std::array<std::uint8_t, 8> channel_counts = {
        2U, 1U, 2U, 3U, 3U, 4U, 4U, 5U
    };
    self.full_bandwidth = channel_counts[self.config.audio_coding_mode];
    if (self.full_bandwidth == 0U) throw std::runtime_error("invalid audio coding mode");
    self.channels = static_cast<std::uint8_t>(
        self.full_bandwidth + (self.config.lfe ? 1U : 0U));
    self.lfe_channel = self.config.lfe ?
        static_cast<std::uint8_t>(self.full_bandwidth + 1U) : 0U;

    const std::uint32_t samples =
        static_cast<std::uint32_t>(self.config.blocks) * block_samples;
    std::uint32_t words = static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(self.config.bitrate) / 16U) * samples /
        self.config.sample_rate);
    if (words < 64U || words > 2048U)
        throw std::runtime_error("E-AC-3 bitrate does not fit the frame size");
    self.frame_bits = static_cast<std::size_t>(words) * 16U;
    self.frame_size = static_cast<std::size_t>(words) * 2U;
    self.pending.assign(static_cast<std::size_t>(self.channels) * block_samples, 0.0F);
}

std::uint32_t CoreEncoder::sample_rate() const noexcept { return impl_->config.sample_rate; }
std::uint8_t CoreEncoder::channels() const noexcept { return impl_->channels; }
std::uint8_t CoreEncoder::full_bandwidth_channels() const noexcept {
    return impl_->full_bandwidth;
}
std::uint16_t CoreEncoder::samples_per_frame() const noexcept {
    return static_cast<std::uint16_t>(impl_->config.blocks * block_samples);
}
std::size_t CoreEncoder::frame_size() const noexcept { return impl_->frame_size; }

void CoreEncoder::set_aux_payload(std::span<const std::uint8_t> payload) {
    if (payload.size() > 2047U)
        throw std::runtime_error("EMDF exceeds the 14-bit auxiliary-data capacity");
    impl_->aux_payload.assign(payload.begin(), payload.end());
    impl_->skip_field = false;
}

void CoreEncoder::set_skip_payload(std::span<const std::uint8_t> payload) {
    const auto capacity = std::min<std::size_t>(2047U, 511U * impl_->config.blocks);
    if (payload.size() > capacity)
        throw std::runtime_error("EMDF exceeds the configured skip-field capacity");
    impl_->aux_payload.assign(payload.begin(), payload.end());
    impl_->skip_field = true;
}

std::size_t CoreEncoder::payload_start_bit() const noexcept { return impl_->payload_start_bit; }

void CoreEncoder::encode(std::span<const float> interleaved,
    std::vector<std::uint8_t>& frame) {
    auto& self = *impl_;
    if (self.channels == 0U) throw std::runtime_error("E-AC-3 encoder is not configured");
    if (!std::all_of(interleaved.begin(), interleaved.end(),
            [](float value) { return std::isfinite(value) && std::abs(value) <= 1.0F; }))
        throw std::runtime_error("PCM samples must be finite and in [-1, 1]");
    const unsigned blocks = self.config.blocks;
    const unsigned channels = self.channels;
    const bool skip_field = self.skip_field && !self.aux_payload.empty();
    if (interleaved.size() !=
        static_cast<std::size_t>(samples_per_frame()) * channels) {
        throw std::runtime_error("E-AC-3 encoder input has the wrong sample count");
    }
    const std::uint8_t sr_code = sample_rate_code(self.config.sample_rate);
    const std::uint8_t blks_code = blocks_code(blocks);
    const unsigned bandwidth_code = bandwidth_for(self.config.bitrate, self.config.sample_rate);

    // --- time to frequency -------------------------------------------------
    // Each output block's coefficients come from the analysis window
    // [pending(256), current(256)]; the current block becomes the pending
    // lookahead for the next call, giving the decoder a one-block latency.
    for (unsigned block = 0; block < blocks; ++block) {
        for (unsigned channel = 1; channel <= channels; ++channel) {
            const bool is_lfe = channel == self.lfe_channel;
            const std::size_t base = static_cast<std::size_t>(block) * block_samples *
                channels + (channel - 1U);
            const std::size_t pending = (channel - 1U) * block_samples;
            for (unsigned sample = 0; sample < block_samples; ++sample) {
                self.time[sample] = self.pending[pending + sample];
                self.time[block_samples + sample] =
                    interleaved[base + static_cast<std::size_t>(sample) * channels];
            }
            std::array<float, coefficient_count> transformed{};
            forward_mdct(self.time, transformed);
            // Adjoint normalization for the decoder's -1/2^22 IMDCT scale.
            constexpr double scale = -32768.0;
            for (unsigned bin = 0; bin < coefficient_count; ++bin) {
                const double value = static_cast<double>(transformed[bin]) * scale;
                self.coef[channel][block][bin] = static_cast<std::int32_t>(std::clamp(
                    std::llround(value), -8388608LL, 8388607LL));
            }
            self.end_bin[channel][block] = is_lfe ? 7U :
                static_cast<std::size_t>(bandwidth_code) * 3U + 73U;
            // The LFE channel only transports seven coefficients; zero the rest
            // so the exponent search never sees energy the frame cannot carry.
            if (is_lfe) {
                for (unsigned bin = 7U; bin < coefficient_count; ++bin)
                    self.coef[channel][block][bin] = 0;
            }
        }
        for (unsigned channel = 1; channel <= channels; ++channel) {
            const std::size_t base = static_cast<std::size_t>(block) * block_samples *
                channels + (channel - 1U);
            const std::size_t pending = (channel - 1U) * block_samples;
            for (unsigned sample = 0; sample < block_samples; ++sample)
                self.pending[pending + sample] =
                    interleaved[base + static_cast<std::size_t>(sample) * channels];
        }
    }

    // --- exponent strategies ----------------------------------------------
    // LFE always uses D15 (the decoder fixes its exponent group count). The full
    // channels use D45, whose four-bin groups make the exponent words cheap.
    for (unsigned channel = 1; channel <= channels; ++channel) {
        for (unsigned block = 0; block < blocks; ++block)
            self.strategy[channel][block] = channel == self.lfe_channel ?
                eac3::ExponentStrategy::D15 : eac3::ExponentStrategy::D45;
    }

    // --- exponents ---------------------------------------------------------
    // Per-subgroup exponent from the coefficients, lowered to a 2-Lipschitz
    // envelope over the subgroups so the transmitted deltas (each clipped to
    // [-2, 2]) reconstruct the same values the encoder allocates against.
    for (unsigned channel = 1; channel <= channels; ++channel) {
        for (unsigned block = 0; block < blocks; ++block) {
            const std::size_t end = self.end_bin[channel][block];
            auto& exp = self.exponents[channel][block];
            exp.fill(0);
            if (end == 0U) continue;
            const unsigned group_size = exponent_group_size(self.strategy[channel][block]);
            std::vector<int> ideal(end, 24);
            for (std::size_t bin = 0; bin < end; ++bin) {
                ideal[bin] = normalization_exponent(std::abs(
                    static_cast<std::int64_t>(self.coef[channel][block][bin])));
            }
            // The absolute exponent is transmitted in four bits.
            ideal[0] = std::min(ideal[0], 15);
            // Subgroup sequence: index 0 is the absolute exponent (bin 0), then
            // ceil((end-1)/group_size) subgroups of group_size bins each.
            std::vector<int> subgroup;
            subgroup.push_back(ideal[0]);
            for (std::size_t base = 1U; base < end; base += group_size) {
                int value = 24;
                for (unsigned index = 0; index < group_size && base + index < end; ++index)
                    value = std::min(value, ideal[base + index]);
                subgroup.push_back(value);
            }
            for (std::size_t index = 1U; index < subgroup.size(); ++index)
                subgroup[index] = std::min(subgroup[index], subgroup[index - 1U] + 2);
            for (std::size_t index = subgroup.size() - 1U; index-- > 0U;)
                subgroup[index] = std::min(subgroup[index], subgroup[index + 1U] + 2);
            for (int& value : subgroup) value = std::clamp(value, 0, 24);
            // The LFE's seven coefficients do not warrant a varying exponent;
            // a constant one keeps every delta zero and always in range.
            if (channel == self.lfe_channel) {
                const int value = *std::min_element(subgroup.begin(), subgroup.end());
                std::fill(subgroup.begin(), subgroup.end(), value);
            }
            exp[0] = static_cast<std::int8_t>(subgroup[0]);
            for (std::size_t index = 1U; index < subgroup.size(); ++index) {
                for (unsigned repeat = 0; repeat < group_size; ++repeat) {
                    const std::size_t bin = 1U + (index - 1U) * group_size + repeat;
                    if (bin < end) exp[bin] = static_cast<std::int8_t>(subgroup[index]);
                }
            }
            for (std::size_t bin = end; bin < allocation_coefficients; ++bin)
                exp[bin] = exp[end - 1U];
            // Reusing the previous block's exponents costs two strategy bits
            // instead of a whole exponent word set. A previous exponent may be
            // equal or one step smaller: larger ones can clip the scaled
            // mantissa, while much smaller ones throw away precision.
            if (block > 0U && channel != self.lfe_channel) {
                const auto& previous = self.exponents[channel][block - 1U];
                bool close_enough = true;
                for (std::size_t bin = 0; bin < end && close_enough; ++bin) {
                    const int delta = static_cast<int>(previous[bin]) -
                        static_cast<int>(exp[bin]);
                    close_enough = delta >= -1 && delta <= 0;
                }
                if (close_enough) {
                    std::copy_n(previous.begin(), end, exp.begin());
                    self.strategy[channel][block] = eac3::ExponentStrategy::Reuse;
                }
            }
        }
    }

    // --- bit allocation ----------------------------------------------------
    // Overhead is every bit that is not a mantissa: the headers, the exponent
    // words and the CRC. The SNR search below has to leave room for it, and for
    // six D45 blocks that is several thousand bits, so it is counted exactly.
    const auto exponent_code_count = [&](unsigned channel, unsigned block) {
        const std::size_t end = self.end_bin[channel][block];
        if (end == 0U) return std::size_t{0};
        const unsigned group_size = exponent_group_size(self.strategy[channel][block]);
        const std::size_t span = 3U * group_size;
        std::size_t count = 0U;
        for (std::size_t cursor = 1U; cursor < end; cursor += span) ++count;
        return count;
    };
    std::size_t overhead_bits = 54U;      // bitstream id .. additional bsi
    if (self.config.stream_type == 0U) {
        if (blocks != 6U) overhead_bits += 1U; // converter sync flag
    } else {
        overhead_bits += 1U;               // dependent channel map flag
    }
    if (blocks == 6U) overhead_bits += 2U;
    overhead_bits += 2U;                  // snr offset strategy
    overhead_bits += 6U;                  // transient .. delta bit allocation
    overhead_bits += 2U;                  // skip and spx flags
    if (self.config.audio_coding_mode > 1U)
        overhead_bits += blocks;
    overhead_bits += static_cast<std::size_t>(blocks) * self.full_bandwidth * 2U;
    if (self.config.lfe) overhead_bits += blocks;
    if (self.config.stream_type == 0U) {
        overhead_bits += blocks != 6U ? 1U :
            static_cast<std::size_t>(5U) * self.full_bandwidth;
    }
    overhead_bits += 10U;                 // coarse and fine snr offset
    if (self.config.audio_coding_mode == 2U) overhead_bits += 4U + blocks - 1U;
    if (blocks > 1U) overhead_bits += 1U; // block start info
    // program/spx flags per block, plus the stream-type-0 allocation flag
    overhead_bits += static_cast<std::size_t>(blocks) *
        (self.config.stream_type == 0U ? 3U : 2U);
    for (unsigned channel = 1; channel <= channels; ++channel) {
        for (unsigned block = 0; block < blocks; ++block) {
            if (self.strategy[channel][block] == eac3::ExponentStrategy::Reuse) continue;
            if (channel != self.lfe_channel) overhead_bits += 6U; // bandwidth
            overhead_bits += 4U + 7U * exponent_code_count(channel, block);
            if (channel != self.lfe_channel) overhead_bits += 2U; // gain range
        }
    }
    const auto skip_chunks = (self.aux_payload.size() + 510U) / 511U;
    overhead_bits += self.aux_payload.size() * 8U + 39U + (skip_field ? 9U * skip_chunks + blocks : 0U); // payload, trailer and padding

    eac3::BitAllocationParameters parameters;
    parameters.sample_rate_code = sr_code;
    parameters.sample_rate_shift = 0;
    parameters.slow_decay = 0x13;
    parameters.fast_decay = 0x53;
    parameters.slow_gain = 0x4D8;
    parameters.db_per_bit = 0x900;
    parameters.floor = -2048;

    // The PSD and the masking curve depend only on the exponents, so they are
    // built once per channel and block; the SNR-offset search then only re-runs
    // the (cheap) BAP mapping. Recomputing both for every candidate offset made
    // the encoder several times slower than real time.
    std::array<std::array<eac3::BitAllocationWorkspace, max_blocks>, max_channels> workspaces{};
    for (unsigned channel = 1; channel <= channels; ++channel) {
        for (unsigned block = 0; block < blocks; ++block) {
            const std::size_t end = self.end_bin[channel][block];
            if (end == 0U) continue;
            auto& workspace = workspaces[channel][block];
            eac3::calculate_psd(
                std::span<const std::int8_t>(self.exponents[channel][block]).subspan(0U, end),
                0U, end, workspace);
            eac3::calculate_mask(parameters, 0U, end, 0x280,
                channel == self.lfe_channel, eac3::DeltaBitAllocation{}, workspace);
        }
    }

    std::size_t total_bits = 0;
    // BAP grows monotonically with the SNR offset, so the frame cost does too.
    const auto allocate = [&](int offset) {
        for (unsigned channel = 1; channel <= channels; ++channel) {
            for (unsigned block = 0; block < blocks; ++block) {
                auto& allocation = self.bap[channel][block];
                allocation.fill(0);
                const std::size_t end = self.end_bin[channel][block];
                if (end == 0U) continue;
                eac3::calculate_bap(parameters, 0U, end, static_cast<std::int16_t>(offset),
                    workspaces[channel][block],
                    std::span<std::uint8_t>(allocation).subspan(0U, end));
                self.snr_offset[channel][block] = static_cast<std::int16_t>(offset);
            }
        }
        total_bits = self.allocation_bits(blocks);
        // 32 bits of frame CRC sit after the audio and are not part of the
        // overhead estimate; the writer refuses to write over them.
        return total_bits + overhead_bits + 32U <= self.frame_bits;
    };

    // The coarse/fine SNR offset pair can only express (coarse - 15) * 16 + fine
    // in units of 4, so the search is bounded to the lattice of that field. More
    // bits is better, so take the highest offset that still fits: the top of the
    // range first, then a binary search on the monotone cost.
    constexpr int minimum_snr_offset = -960;
    constexpr int maximum_snr_offset = 63 * 64 + 15 * 4 - 960;
    static_assert(minimum_snr_offset % 4 == 0 && maximum_snr_offset % 4 == 0);
    if (!allocate(maximum_snr_offset)) {
        int low = minimum_snr_offset / 4;
        int high = maximum_snr_offset / 4;
        if (!allocate(low * 4)) {
            throw std::runtime_error(
                "E-AC-3 bit allocation does not fit the frame; raise the bitrate");
        }
        int best = low;
        while (low < high) {
            const int middle = low + (high - low + 1) / 2;
            if (allocate(middle * 4)) {
                best = middle;
                low = middle;
            } else {
                high = middle - 1;
            }
        }
        static_cast<void>(allocate(best * 4));
    }

    // --- mantissas ---------------------------------------------------------
    for (unsigned channel = 1; channel <= channels; ++channel) {
        for (unsigned block = 0; block < blocks; ++block) {
            const std::size_t end = self.end_bin[channel][block];
            for (std::size_t bin = 0; bin < end; ++bin) {
                const std::int64_t scaled =
                    static_cast<std::int64_t>(self.coef[channel][block][bin]) *
                    (1LL << self.exponents[channel][block][bin]);
                self.mantissa[channel][block][bin] = mantissa_code(
                    static_cast<std::int32_t>(std::clamp<std::int64_t>(scaled, -8388608, 8388607)),
                    self.bap[channel][block][bin]);
            }
        }
    }

    // --- serialise ---------------------------------------------------------
    BitWriter writer;
    writer.write(0x0B77U, 16U);
    writer.write(self.config.stream_type, 2U);             // 0 independent, 1 dependent
    writer.write(self.config.substream_id, 3U);
    writer.write(static_cast<std::uint32_t>(self.frame_size / 2U - 1U), 11U);
    writer.write(sr_code, 2U);
    writer.write(blks_code, 2U);
    writer.write(self.config.audio_coding_mode, 3U);
    writer.write(self.config.lfe ? 1U : 0U, 1U);
    writer.write(self.config.bitstream_id, 5U);
    writer.write(self.config.dialnorm, 5U);
    writer.write(self.config.compression_present ? 1U : 0U, 1U);
    if (self.config.compression_present) writer.write(self.config.compression_word, 8U);
    if (self.config.stream_type == 1U) writer.write(0U, 1U); // no channel map
    if (self.config.audio_coding_mode == 0U) writer.write(0U, 2U); // dsurmod
    writer.write(0U, 1U);                                  // no mixing metadata
    writer.write(0U, 1U);                                  // no informational metadata
    if (self.config.stream_type == 0U && blocks != 6U)
        writer.write(0U, 1U);                              // converter sync flag
    writer.write(self.config.with_additional_bsi ? 1U : 0U, 1U);
    if (self.config.with_additional_bsi) {
        writer.write(1U, 6U);                              // two additional-bsi bytes
        writer.write(0U, 7U);                              // first byte: 7 reserved bits
        writer.write(1U, 1U);                              //   extension type A
        writer.write(self.config.complexity_index_type_a, 8U);
    }

    if (blocks == 6U) {
        writer.write(1U, 1U);                              // exponent strategy syntax
        writer.write(0U, 1U);                              // AHT disabled
    }
    writer.write(0U, 2U);                                  // snr offset strategy
    writer.write(0U, 1U);                                  // transient pre-noise off
    writer.write(0U, 1U);                                  // block switch syntax off
    writer.write(0U, 1U);                                  // dither flag syntax off
    writer.write(0U, 1U);                                  // bit allocation syntax off
    writer.write(0U, 1U);                                  // fast gain syntax off
    writer.write(0U, 1U);                                  // delta bit allocation off
    writer.write_bit(skip_field);                         // bounded skip-field carrier
    writer.write(0U, 1U);                                  // no spectral extension / SPX data
    if (self.config.audio_coding_mode > 1U) {
        // Block 0 states whether coupling is used; every later block only says
        // whether the strategy changes. All zero: coupling is never in use.
        for (unsigned block = 0; block < blocks; ++block) writer.write(0U, 1U);
    }
    for (unsigned block = 0; block < blocks; ++block) {
        for (unsigned channel = 1; channel <= self.full_bandwidth; ++channel)
            writer.write(static_cast<std::uint32_t>(self.strategy[channel][block]), 2U);
    }
    if (self.config.lfe) {
        for (unsigned block = 0; block < blocks; ++block) {
            writer.write(self.strategy[self.lfe_channel][block] ==
                eac3::ExponentStrategy::Reuse ? 0U : 1U, 1U);
        }
    }
    if (self.config.stream_type == 0U) {
        if (blocks != 6U) {
            writer.write(0U, 1U);
        } else {
            for (unsigned channel = 1; channel <= self.full_bandwidth; ++channel)
                writer.write(0U, 5U);
        }
    }
    // Decoder reconstructs snr = 4 * ((coarse - 15) * 16 + fine); the encoder's
    // search offset therefore maps to (offset + 960) split into 64/4 units.
    const int snr_value = static_cast<int>(self.snr_offset[1][0]) + 960;
    writer.write(static_cast<std::uint32_t>(snr_value / 64), 6U); // coarse snr offset
    writer.write(static_cast<std::uint32_t>((snr_value % 64) / 4), 4U); // fine snr offset
    if (blocks > 1U) writer.write(0U, 1U);                 // no block start info

    for (unsigned block = 0; block < blocks; ++block) {
        writer.write(0U, 1U);                              // program/aux flag (block 0)
        writer.write(0U, 1U);                              // no spectral extension strategy change
        if (self.config.audio_coding_mode == 2U)
            writer.write(0U, block == 0U ? 4U : 1U); // rematrixing disabled
        // The channel bandwidth of every coded full-bandwidth channel is
        // transmitted first (parse_exponents reads them all before any exponent).
        for (unsigned channel = 1; channel <= self.full_bandwidth; ++channel) {
            if (self.strategy[channel][block] == eac3::ExponentStrategy::Reuse) continue;
            writer.write(bandwidth_code, 6U);
        }
        for (unsigned channel = 1; channel <= channels; ++channel) {
            if (self.strategy[channel][block] == eac3::ExponentStrategy::Reuse) continue;
            const std::size_t end = self.end_bin[channel][block];
            const auto& exp = self.exponents[channel][block];
            writer.write(static_cast<std::uint32_t>(exp[0]), 4U);
            const unsigned group_size = exponent_group_size(self.strategy[channel][block]);
            const std::size_t span = 3U * group_size;
            int previous = exp[0];
            std::size_t cursor = 1U;
            while (cursor < end) {
                const int a = exp[cursor];
                const int b = cursor + group_size < end ? exp[cursor + group_size] : a;
                const int c = cursor + 2U * group_size < end ?
                    exp[cursor + 2U * group_size] : b;
                writer.write(exponent_code(a - previous, b - a, c - b), 7U);
                previous = c;
                cursor += span;
            }
            if (channel != self.lfe_channel) writer.write(0U, 2U); // no gain range
        }

        if (self.config.stream_type == 0U)
            writer.write(0U, 1U);                          // stream-type-0 dynamic range flag
        if (skip_field) {
            const auto begin = std::min<std::size_t>(block * 511U, self.aux_payload.size());
            const auto count = std::min<std::size_t>(511U, self.aux_payload.size() - begin);
            writer.write_bit(count != 0);
            if (count != 0) {
                writer.write(static_cast<std::uint32_t>(count), 9);
                if (block == 0) self.payload_start_bit = writer.position();
                for (std::size_t i = begin; i < begin + count; ++i)
                    writer.write(self.aux_payload[i], 8);
            }
        }
        // The decoder's grouped-mantissa state persists across the channels of a
        // block, so the group values (and their counters) live at block scope.
        std::array<std::vector<std::int32_t>, 5> pending{};
        for (unsigned channel = 1; channel <= channels; ++channel) {
            const std::size_t end = self.end_bin[channel][block];
            const auto& allocation = self.bap[channel][block];
            const auto& quantised = self.mantissa[channel][block];
            for (std::size_t bin = 0; bin < end; ++bin) {
                const unsigned pointer = allocation[bin];
                if (pointer == 1U || pointer == 2U || pointer == 4U)
                    pending[pointer].push_back(quantised[bin]);
            }
        }
        std::array<std::size_t, 5> cursor{};
        int left1 = 0;
        int left2 = 0;
        int left4 = 0;
        const auto at_or = [](const std::vector<std::int32_t>& values,
                              std::size_t index, std::int32_t fallback) {
            return index < values.size() ? values[index] : fallback;
        };
        for (unsigned channel = 1; channel <= channels; ++channel) {
            const std::size_t end = self.end_bin[channel][block];
            const auto& allocation = self.bap[channel][block];
            const auto& quantised = self.mantissa[channel][block];
            for (std::size_t bin = 0; bin < end; ++bin) {
                const unsigned pointer = allocation[bin];
                if (pointer == 1U) {
                    if (left1 == 0) {
                        const auto& values = pending[1];
                        const std::size_t at = cursor[1];
                        writer.write(static_cast<std::uint32_t>(
                            at_or(values, at, 1) * 9 + at_or(values, at + 1U, 1) * 3 +
                            at_or(values, at + 2U, 1)), 5U);
                        cursor[1] += 3U;
                        left1 = 2;
                    } else {
                        --left1;
                    }
                } else if (pointer == 2U) {
                    if (left2 == 0) {
                        const auto& values = pending[2];
                        const std::size_t at = cursor[2];
                        writer.write(static_cast<std::uint32_t>(
                            at_or(values, at, 2) * 25 + at_or(values, at + 1U, 2) * 5 +
                            at_or(values, at + 2U, 2)), 7U);
                        cursor[2] += 3U;
                        left2 = 2;
                    } else {
                        --left2;
                    }
                } else if (pointer == 4U) {
                    if (left4 == 0) {
                        const auto& values = pending[4];
                        const std::size_t at = cursor[4];
                        writer.write(static_cast<std::uint32_t>(
                            at_or(values, at, 5) * 11 + at_or(values, at + 1U, 5)), 7U);
                        cursor[4] += 2U;
                        left4 = 1;
                    } else {
                        --left4;
                    }
                } else if (pointer == 0U) {
                    continue;
                } else if (pointer == 3U) {
                    writer.write(static_cast<std::uint32_t>(quantised[bin]), 3U);
                } else if (pointer == 5U) {
                    writer.write(static_cast<std::uint32_t>(quantised[bin]), 4U);
                } else {
                    const unsigned bits = mantissa_bits[pointer];
                    writer.write(static_cast<std::uint32_t>(quantised[bin]) &
                        ((1U << bits) - 1U), bits);
                }
            }
        }
    }

    // Annex E auxdata is one complete frame-end carrier, never split across blocks.
    const auto payload_bits = skip_field ? 0U : self.aux_payload.size() * 8U;
    const auto payload_start = self.frame_bits - 32U - payload_bits;
    if (writer.position() > payload_start)
        throw std::runtime_error("E-AC-3 frame overflow");
    while (writer.position() < payload_start) writer.write_bit(false);
    if (!skip_field) {
        self.payload_start_bit = writer.position();
        for (const auto byte : self.aux_payload) writer.write(byte, 8);
    }
    writer.write(static_cast<std::uint32_t>(payload_bits), 14);
    writer.write_bit(payload_bits != 0);
    writer.write_bit(false); // crcrsv
    writer.write(0, 16); // CRC filled below
    frame = writer.bytes();
    const std::uint16_t crc = atmos::crc16_ansi(
        std::span<const std::uint8_t>(frame).subspan(2U, self.frame_size - 4U));
    frame[self.frame_size - 2U] = static_cast<std::uint8_t>(crc >> 8U);
    frame[self.frame_size - 1U] = static_cast<std::uint8_t>(crc & 0xFFU);
    self.aux_payload.clear();
}

} // namespace atmos::encode

