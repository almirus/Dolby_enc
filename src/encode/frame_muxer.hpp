#pragma once

#include "eac3/emdf.hpp"
#include "encode/eac3_core.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace atmos::encode {

// Canonical audio authenticated by DD+ EVO: omit non-audio BSI, skip fields,
// sync/size and the frame trailer; byte-pad, then keep complete 16-bit words.
[[nodiscard]] std::vector<std::uint8_t> protection_audio(std::span<const std::uint8_t> frame);

// Six-block/five-carrier transcoder: keep the historical 511-byte envelope unless
// mandatory OAMD needs room for one coarse sparse band per object. Audio must fit.
[[nodiscard]] std::size_t joc_payload_budget(std::size_t oamd_emdf_bytes, unsigned objects,
    std::uint32_t bitrate);

// Wrap one complete E-AC-3 syncframe in an IEC 61937 data burst: four
// little-endian 16-bit header words (Pa, Pb, data type 0x15, payload bytes)
// followed by the frame with every 16-bit word byte-swapped, because S/PDIF
// carries the words LSB first.
[[nodiscard]] std::vector<std::uint8_t> iec61937_burst(std::span<const std::uint8_t> frame);

// Encodes one frame of the core, carrying the OAMD/JOC EMDF container in the
// audio-block skip fields, with primary authentication and frame CRC.
void encode_atmos_frame(CoreEncoder& encoder, std::span<const float> interleaved,
    const eac3::OamdFrame& oamd, const eac3::JocFrame& joc, bool with_oamd,
    bool with_joc, std::vector<std::uint8_t>& frame,
    std::span<const std::uint8_t> raw_oamd = {});

} // namespace atmos::encode
