#pragma once

#include <cstdint>
#include <array>
#include <span>
#include <string>
#include <vector>

namespace atmos::mkv {

struct Track final {
    std::uint64_t number = 0;
    std::uint64_t type = 0;   // 1 video, 2 audio, 17 subtitle
    std::string codec_id;     // e.g. A_TRUEHD, A_EAC3, A_AC3
    std::uint64_t channels = 0;
    double sample_rate = 0.0;
};

// Returned packets borrow the input mapping. False means EOF when error is empty.
class PacketReader final {
public:
    PacketReader(std::span<const std::uint8_t> data, std::uint64_t track_number);
    [[nodiscard]] bool next(std::span<const std::uint8_t>& packet, std::string& error);
private:
    struct Level { std::size_t cursor, end; std::uint64_t id; bool unknown; };
    std::span<const std::uint8_t> data_;
    std::uint64_t track_;
    std::array<Level,3> levels_{};
    unsigned depth_ = 0;
    std::vector<std::span<const std::uint8_t>> frames_;
    std::size_t frame_ = 0;
    std::string failure_;
};

// Matroska/EBML reader that extracts the elementary stream of one track. The
// track payloads are concatenated in file order (SimpleBlock/BlockGroup frames,
// with lacing expanded), which is what a raw .thd/.ec3/.ac3 needs.
class Demuxer final {
public:
    explicit Demuxer(std::span<const std::uint8_t> data);

    [[nodiscard]] const std::vector<Track>& tracks() const noexcept { return tracks_; }

    // Codes for the common audio formats this project cares about.
    [[nodiscard]] static bool is_truehd(const Track& track);
    [[nodiscard]] static bool is_eac3(const Track& track);
    [[nodiscard]] static bool is_ac3(const Track& track);

    // Appends every frame of `track_number` to `output`. Returns false and sets
    // `error` on a malformed stream.
    bool extract(std::uint64_t track_number, std::vector<std::uint8_t>& output,
        std::string& error);

private:
    std::span<const std::uint8_t> data_;
    std::vector<Track> tracks_;
};

} // namespace atmos::mkv
