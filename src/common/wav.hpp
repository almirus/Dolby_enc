#pragma once

#include <cstdint>
#include <filesystem>
#include <span>

namespace atmos {

void write_float_wav(const std::filesystem::path& path,
    std::uint32_t sample_rate, std::uint16_t channels,
    std::span<const float> interleaved);
void write_pcm24_wav(const std::filesystem::path& path,
    std::uint32_t sample_rate, std::uint16_t channels,
    std::span<const float> interleaved);
void write_pcm24_wav_channel(const std::filesystem::path& path,
    std::uint32_t sample_rate, std::span<const float> interleaved,
    unsigned source_channels, unsigned source_channel);

} // namespace atmos
