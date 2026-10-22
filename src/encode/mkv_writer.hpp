#pragma once

#include <filesystem>
#include <cstdint>
#include <string>

namespace atmos::encode {

// Wraps a raw E-AC-3 elementary stream in a minimal Matroska file: one
// "A_EAC3" audio track, 32 ms SimpleBlocks (one syncframe each), clusters of
// about a second. Reads the EC3 back frame by frame, so the caller can stream
// the elementary stream to disk first and mux afterwards.
[[nodiscard]] bool write_matroska(const std::filesystem::path& ec3_path,
    const std::filesystem::path& mkv_path, std::string& error);

// One TrueHD access unit per block; timestamps use a 1 microsecond scale.
[[nodiscard]] bool write_truehd_matroska(const std::filesystem::path& source,
    const std::filesystem::path& target, std::uint64_t samples, unsigned channels,
    std::string& error);

} // namespace atmos::encode
