#pragma once

#include "eac3/emdf.hpp"

#include <cstdint>
#include <vector>

namespace atmos::encode {

// Fill primary protection with the leading HMAC-SHA256 bytes. The audio is
// canonicalized by protection_audio(); EMDF protection bytes are zeroed for hashing.
void sign_emdf(std::vector<std::uint8_t>& emdf, std::span<const std::uint8_t> audio);
[[nodiscard]] bool verify_emdf_primary(std::span<const std::uint8_t> emdf,
    std::span<const std::uint8_t> audio);

// Serializes the OAMD payload (EMDF payload type 11) on its own.
[[nodiscard]] std::vector<std::uint8_t> write_oamd_payload(const eac3::OamdFrame& oamd);

// Serializes the JOC reconstruction payload (EMDF payload type 14).
[[nodiscard]] std::vector<std::uint8_t> write_joc_payload(const eac3::JocFrame& joc);
// Same serializer and validation, excluding final byte padding.
[[nodiscard]] std::size_t joc_payload_bit_count(const eac3::JocFrame& joc);

// Wraps the payloads in an EMDF container: the byte image carried in the
// E-AC-3 frame's metadata carrier. `oamd`/`joc` are emitted only when their flag is
// set.
[[nodiscard]] std::vector<std::uint8_t> build_emdf(const eac3::OamdFrame& oamd,
    const eac3::JocFrame& joc, bool with_oamd, bool with_joc,
    std::span<const std::uint8_t> raw_oamd = {});

} // namespace atmos::encode
