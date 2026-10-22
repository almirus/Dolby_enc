#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace atmos::crypto {

struct MatDecHmacSha256Context final {
    std::array<std::uint32_t, 8> inner{};
    std::array<std::uint32_t, 8> outer{};
    std::array<std::uint8_t, 64> block{};
    std::size_t block_size = 0;
    std::uint64_t total_size = 0;
};

// Returns 0 on success and 1 for a null context.
int MAT_DEC_hmac_obf_sha256_init(MatDecHmacSha256Context* context) noexcept;

// DDP/JOC uses the same HMAC-SHA256 primitive with a different
// device-family obfuscated key.
int DDP_udc_hmac_obf_sha256_init(MatDecHmacSha256Context* context) noexcept;

void mat_dec_hmac_sha256_update(MatDecHmacSha256Context* context,
    const void* data, std::size_t size) noexcept;

void mat_dec_hmac_sha256_final(MatDecHmacSha256Context* context,
    std::array<std::uint8_t, 32>& digest) noexcept;

[[nodiscard]] std::array<std::uint8_t, 32> mat_dec_obfuscated_key() noexcept;

[[nodiscard]] std::array<std::uint8_t, 32> ddp_udc_obfuscated_key() noexcept;

} // namespace atmos::crypto

// ABI-compatible entry point matching the native/decompiled symbol name.
extern "C" int MAT_DEC_hmac_obf_sha256_init(void* context) noexcept;
