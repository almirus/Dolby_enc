#pragma once

#include "audio_frame.hpp"
#include "bit_allocation.hpp"
#include "mantissa.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>
#include <utility>

namespace atmos::eac3 {

struct SpectralBlock final {
    std::array<std::array<std::int32_t, 256>, AudioFrameSyntax::max_channels>
        coefficients{};
    std::array<bool, AudioFrameSyntax::max_channels> block_switch{};
    std::size_t end_bit = 0;
};

struct SpectralFrame final {
    std::array<SpectralBlock, AudioFrameSyntax::max_blocks> blocks{};
    std::uint8_t decoded_blocks = 0;
    bool spectral_extension_used = false;
    bool enhanced_coupling_used = false;
    std::size_t nonzero_coefficients = 0;
    std::vector<std::uint8_t> auxiliary_data;
    std::vector<std::pair<std::size_t, std::size_t>> skip_ranges;
};

struct EcplDecodeState final {
    std::optional<std::array<std::int32_t, 256>> previous_frame_mantissa;
    std::uint32_t transient_generator = 0x2545F491U;

    void reset() noexcept {
        previous_frame_mantissa.reset();
        transient_generator = 0x2545F491U;
    }
};

// Decodes the channel-coded spectral core, including AHT and SPX.
[[nodiscard]] std::optional<SpectralFrame> decode_spectral_frame(
    std::span<const std::uint8_t> frame, const BitstreamInfo& info,
    const AudioFrameSyntax& syntax, DitherGenerator& dither,
    std::string* error = nullptr, EcplDecodeState* ecpl_state = nullptr);

namespace testing {
[[nodiscard]] double ecpl_identity_gain();
}

} // namespace atmos::eac3
