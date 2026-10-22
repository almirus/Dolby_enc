#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace atmos::encode {

// Wraps a raw E-AC-3 elementary stream in a minimal MP4: one 'ec-3' audio
// sample entry (dec3 config for a 5.1 48 kHz independent substream), one sample
// per syncframe, all samples in a single chunk. The elementary stream is read
// twice: once to index the frame sizes, once to copy it into mdat.
[[nodiscard]] bool write_mp4(const std::filesystem::path& ec3_path,
    const std::filesystem::path& mp4_path, std::uint32_t bitrate, std::string& error);

} // namespace atmos::encode
