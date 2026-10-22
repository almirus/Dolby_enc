#pragma once

#include "eac3/emdf.hpp"
#include "joc_qmf.hpp"
#include "joc_analysis.hpp"
#include <array>
#include <optional>
#include <span>
#include <vector>

namespace atmos::encode {

// Native-compatible profile: up to 15 full-band signals in OAMD order; LFE stays in the core.
[[nodiscard]] std::vector<unsigned> joc_target_channels(const eac3::OamdFrame& metadata);

// 48kHz five-carrier JOC analysis; two frames of delayed spectra support each fit.
class JocEncoder final {
public:
    static constexpr unsigned carrier_delay = 1217;
    // Carrier: 1536 samples in WAV 5.1 order L R C LFE Ls Rs.
    // Full-scale PCM is quarter-scaled internally for the native mode2 analysis.
    // Elements: interleaved full-bandwidth objects, excluding the LFE bed.
    // Starts at first_band_index (5 = 12 bands). To fit the payload budget,
    // lowers precision then uses sparse coding before reducing bands.
    // Band reduction remaps the cached coefficients without refitting.
    // Low bitrates pass a coarser start because the gains cost core bits.
    // Optional 24x16x8 renderer history, before alignment; signals exclude LFE.
    [[nodiscard]] eac3::JocFrame encode(std::span<const float> carrier,
        std::span<const float> elements, unsigned object_count, std::size_t payload_budget,
        int first_band_index = 5, std::span<const float> carrier_history = {});
    // Small carrier adjustment minimizing reconstruction error of the packed matrix.
    [[nodiscard]] const std::array<float, 5>& carrier_correction() const noexcept { return carrier_correction_; }
private:
    std::array<float, 5> carrier_correction_{1,1,1,1,1};
    std::uint16_t sequence_count_ = 0;
    std::vector<joc::Analysis> carrier_filters_{5};
    std::vector<joc::Analysis> object_filters_{16};
    std::optional<joc::TemporalDetector> detector_;
    std::array<std::array<joc::Spectrum, 10>, 21> delay_{};
    unsigned delay_index_ = 0;
    std::array<std::vector<joc::Spectrum>, 5> previous_carriers_;
    std::vector<std::vector<joc::Spectrum>> previous_targets_;
    joc::History previous_history_{};
    joc::HistoryDelay history_delay_;
    bool history_enabled_ = false;
    std::array<bool, 16> previous_nonzero_{};
};

} // namespace atmos::encode
