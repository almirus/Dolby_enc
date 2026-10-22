#include "wav.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace atmos {
namespace {

void write_u16(std::ofstream& output, std::uint16_t value) {
    const char bytes[2] = {
        static_cast<char>(value & 0xFFU), static_cast<char>(value >> 8U)
    };
    output.write(bytes, sizeof(bytes));
}

void write_u32(std::ofstream& output, std::uint32_t value) {
    const char bytes[4] = {
        static_cast<char>(value & 0xFFU), static_cast<char>((value >> 8U) & 0xFFU),
        static_cast<char>((value >> 16U) & 0xFFU), static_cast<char>(value >> 24U)
    };
    output.write(bytes, sizeof(bytes));
}

} // namespace

void write_float_wav(const std::filesystem::path& path,
    std::uint32_t sample_rate, std::uint16_t channels,
    std::span<const float> interleaved) {
    if (channels == 0U || interleaved.size() % channels != 0U) {
        throw std::runtime_error("invalid PCM shape");
    }
    const std::uint64_t byte_count64 = interleaved.size() * sizeof(float);
    if (byte_count64 > 0xFFFFFFFFULL - 36ULL) {
        throw std::runtime_error("WAV output exceeds RIFF size limit");
    }
    const auto byte_count = static_cast<std::uint32_t>(byte_count64);
    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("cannot create WAV output");
    output.write("RIFF", 4);
    write_u32(output, 36U + byte_count);
    output.write("WAVEfmt ", 8);
    write_u32(output, 16U);
    write_u16(output, 3U); // IEEE float
    write_u16(output, channels);
    write_u32(output, sample_rate);
    write_u32(output, sample_rate * channels * sizeof(float));
    write_u16(output, static_cast<std::uint16_t>(channels * sizeof(float)));
    write_u16(output, 32U);
    output.write("data", 4);
    write_u32(output, byte_count);
    output.write(reinterpret_cast<const char*>(interleaved.data()),
        static_cast<std::streamsize>(byte_count));
    if (!output) throw std::runtime_error("failed while writing WAV output");
}

void write_pcm24_wav(const std::filesystem::path& path,
    std::uint32_t sample_rate, std::uint16_t channels,
    std::span<const float> interleaved) {
    if (channels == 0U || interleaved.size() % channels != 0U) {
        throw std::runtime_error("invalid PCM shape");
    }
    const std::uint64_t byte_count64 = interleaved.size() * 3U;
    if (byte_count64 > 0xFFFFFFFFULL - 36ULL) {
        throw std::runtime_error("WAV output exceeds RIFF size limit");
    }
    const auto byte_count = static_cast<std::uint32_t>(byte_count64);
    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("cannot create WAV output");
    output.write("RIFF", 4);
    write_u32(output, 36U + byte_count);
    output.write("WAVEfmt ", 8);
    write_u32(output, 16U);
    write_u16(output, 1U);
    write_u16(output, channels);
    write_u32(output, sample_rate);
    write_u32(output, sample_rate * channels * 3U);
    write_u16(output, static_cast<std::uint16_t>(channels * 3U));
    write_u16(output, 24U);
    output.write("data", 4);
    write_u32(output, byte_count);
    for (float sample : interleaved) {
        const double scaled = std::clamp(static_cast<double>(sample), -1.0, 1.0) *
            (sample < 0.0F ? 8388608.0 : 8388607.0);
        const auto value = static_cast<std::int32_t>(std::llround(scaled));
        const char bytes[3] = {
            static_cast<char>(value & 0xFF),
            static_cast<char>((value >> 8) & 0xFF),
            static_cast<char>((value >> 16) & 0xFF)
        };
        output.write(bytes, sizeof(bytes));
    }
    if (!output) throw std::runtime_error("failed while writing WAV output");
}

void write_pcm24_wav_channel(const std::filesystem::path& path,
    std::uint32_t sample_rate, std::span<const float> interleaved,
    unsigned source_channels, unsigned source_channel) {
    if (source_channels == 0U || source_channel >= source_channels ||
        interleaved.size() % source_channels != 0U) {
        throw std::runtime_error("invalid PCM source channel");
    }
    const std::uint64_t frame_count = interleaved.size() / source_channels;
    const std::uint64_t byte_count64 = frame_count * 3U;
    if (byte_count64 > 0xFFFFFFFFULL - 36ULL) {
        throw std::runtime_error("WAV output exceeds RIFF size limit");
    }
    const auto byte_count = static_cast<std::uint32_t>(byte_count64);
    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("cannot create WAV output");
    output.write("RIFF", 4);
    write_u32(output, 36U + byte_count);
    output.write("WAVEfmt ", 8);
    write_u32(output, 16U);
    write_u16(output, 1U);
    write_u16(output, 1U);
    write_u32(output, sample_rate);
    write_u32(output, sample_rate * 3U);
    write_u16(output, 3U);
    write_u16(output, 24U);
    output.write("data", 4);
    write_u32(output, byte_count);
    for (std::uint64_t frame = 0U; frame < frame_count; ++frame) {
        const float sample = interleaved[static_cast<std::size_t>(frame) *
            source_channels + source_channel];
        const double scaled = std::clamp(static_cast<double>(sample), -1.0, 1.0) *
            (sample < 0.0F ? 8388608.0 : 8388607.0);
        const auto value = static_cast<std::int32_t>(std::llround(scaled));
        const char bytes[3] = {
            static_cast<char>(value & 0xFF),
            static_cast<char>((value >> 8) & 0xFF),
            static_cast<char>((value >> 16) & 0xFF)
        };
        output.write(bytes, sizeof(bytes));
    }
    if (!output) throw std::runtime_error("failed while writing WAV output");
}

} // namespace atmos
