#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace atmos::encode {

// E-AC-3 core encoder (ETSI TS 102 366 Annex E).
//
// Scope: 48/44.1/32 kHz, 1-6 audio blocks per syncframe, 1-5 full-bandwidth
// channels plus optional LFE. Bit allocation reuses the normative AC-3 model
// (atmos::eac3::calculate_psd/calculate_mask/calculate_bap), so the encoder and
// the decoder in this tree share one definition of the mask.
//
// shortcut: no coupling, rematrixing, spectral extension, block switching,
// adaptive hybrid transform or enhanced coupling. Add them when 5.1 below
// ~256 kbps is needed.
struct CoreConfig final {
    std::uint32_t sample_rate = 48000;
    std::uint8_t audio_coding_mode = 7; // 3/2 -> L C R Ls Rs (+ LFE)
    bool lfe = true;
    std::uint8_t blocks = 6;
    std::uint32_t bitrate = 640000;
    std::uint8_t dialnorm = 31; // stored value; 31 means -31 dB
    std::uint8_t bitstream_id = 16;
    // 0 = independent substream; 1 = dependent substream.
    std::uint8_t stream_type = 0;
    std::uint8_t substream_id = 0;
    bool compression_present = false;
    std::uint8_t compression_word = 0;
    // Advertise JOC in additional bitstream info when carrying a JOC payload.
    bool with_additional_bsi = false;
    // Extension type A byte carried after the additional-bsi length. Dolby
    // streams use this to signal the Atmos/JOC complexity index.
    std::uint8_t complexity_index_type_a = 0;
};

class CoreEncoder final {
public:
    CoreEncoder();
    ~CoreEncoder();
    CoreEncoder(CoreEncoder&&) noexcept;
    CoreEncoder& operator=(CoreEncoder&&) noexcept;
    CoreEncoder(const CoreEncoder&) = delete;
    CoreEncoder& operator=(const CoreEncoder&) = delete;

    void configure(const CoreConfig& config);

    [[nodiscard]] std::uint32_t sample_rate() const noexcept;
    [[nodiscard]] std::uint8_t channels() const noexcept;
    [[nodiscard]] std::uint8_t full_bandwidth_channels() const noexcept;
    [[nodiscard]] std::uint16_t samples_per_frame() const noexcept;
    [[nodiscard]] std::size_t frame_size() const noexcept;

    // `interleaved` carries samples_per_frame() * channels() values in the
    // classic AC-3 channel order L C R Ls Rs ... LFE.
    void encode(std::span<const float> interleaved, std::vector<std::uint8_t>& frame);

    // Complete EMDF container in frame-end auxdata (Annex E).
    // Cleared after a successful encode.
    void set_aux_payload(std::span<const std::uint8_t> payload);
    // EMDF in successive audio-block skip fields, at most 511 bytes per block
    // and 2047 bytes per frame; bit allocation must still leave room for audio.
    void set_skip_payload(std::span<const std::uint8_t> payload);
    // First fragment only; larger skip payloads are not contiguous in the frame.
    [[nodiscard]] std::size_t payload_start_bit() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace atmos::encode
