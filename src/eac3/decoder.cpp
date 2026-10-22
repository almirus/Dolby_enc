#include "decoder.hpp"

#include "audio_block.hpp"
#include "audio_frame.hpp"
#include "bitstream_info.hpp"

#include <algorithm>
#include <array>

namespace atmos::eac3 {
namespace {

constexpr std::uint8_t output_channel_map[8][2][6] = {
    {{0,1,0,0,0,0}, {0,1,2,0,0,0}},
    {{0,0,0,0,0,0}, {0,1,0,0,0,0}},
    {{0,1,0,0,0,0}, {0,1,2,0,0,0}},
    {{0,2,1,0,0,0}, {0,2,1,3,0,0}},
    {{0,1,2,0,0,0}, {0,1,3,2,0,0}},
    {{0,2,1,3,0,0}, {0,2,1,4,3,0}},
    {{0,1,2,3,0,0}, {0,1,3,4,2,0}},
    {{0,2,1,3,4,0}, {0,2,1,4,5,3}}
};

} // namespace

void BedDecoder::reset() noexcept {
    for (auto& transform : transforms_) transform.reset();
    dither_.reset();
    ecpl_state_.reset();
}

bool BedDecoder::decode(std::span<const std::uint8_t> frame,
    PcmFrame& output, std::string& error) {
    const auto info = parse_bitstream_info(frame, 0U);
    if (!info || info->stream_type != 0U) {
        error = "not an independent E-AC-3 frame";
        return false;
    }
    const auto syntax = parse_audio_frame_syntax(frame, *info);
    if (!syntax) {
        error = "invalid E-AC-3 audio frame syntax";
        return false;
    }
    const auto spectral = decode_spectral_frame(frame, *info, *syntax, dither_, &error,
        &ecpl_state_);
    if (!spectral) return false;
    output.metadata = parse_emdf(spectral->auxiliary_data);

    const unsigned channels = syntax->full_bandwidth_channels + (info->lfe ? 1U : 0U);
    output.sample_rate = info->sample_rate;
    output.channels = static_cast<std::uint8_t>(channels);
    output.samples_per_channel = static_cast<std::uint16_t>(info->blocks * 256U);
    output.interleaved.assign(
        static_cast<std::size_t>(output.samples_per_channel) * channels, 0.0F);
    const auto& channel_map = output_channel_map[info->audio_coding_mode][info->lfe ? 1U : 0U];
    for (unsigned block = 0; block < info->blocks; ++block) {
        std::array<std::array<float, 256>, 7> block_pcm{};
        for (unsigned channel = 1; channel <= channels; ++channel) {
            if (spectral->blocks[block].block_switch[channel]) ++output.short_transforms;
            transforms_[channel].synthesize(
                spectral->blocks[block].coefficients[channel], block_pcm[channel],
                spectral->blocks[block].block_switch[channel]);
        }
        for (unsigned sample = 0; sample < 256U; ++sample) {
            for (unsigned channel = 1; channel <= channels; ++channel) {
                const unsigned destination = channel_map[channel - 1U];
                output.interleaved[(block * 256U + sample) * channels + destination] =
                    block_pcm[channel][sample];
            }
        }
    }
    error.clear();
    return true;
}

} // namespace atmos::eac3
