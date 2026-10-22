#pragma once

#include "eac3/emdf.hpp"

#include <cstdint>
#include <functional>
#include <array>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace atmos::render {

struct MetadataFrame final {
    std::uint64_t sample = 0;
    eac3::OamdFrame oamd;
};

struct MetadataPacket final {
    std::uint64_t source_sample = 0; // Nondecreasing lower bound for this and all later frames.
    MetadataFrame frame; // frame.sample must be >= source_sample; packet order breaks event ties.
};
using MetadataReader = std::function<bool(MetadataPacket&)>;
// Replay identical packets from the beginning. False is EOF; read failures throw.
using MetadataReaderFactory = std::function<MetadataReader()>;
// One-pass source: read returns actual samples/channel (0 = EOF), up to 1536.
// Before returning PCM, it must make all metadata through that PCM available.
using LivePcmReader = std::function<std::size_t(std::span<float>)>;

struct Layout final {
    std::string name;
    std::vector<std::string> channel_names;
};

[[nodiscard]] bool parse_layout(std::string_view name, Layout& layout);
[[nodiscard]] std::string supported_layouts();

// 24 time slots, up to 16 full-band signals in source order, 8 carrier lanes.
using CarrierHistory = std::array<float, 24 * 16 * 8>;

// Callbacks consume consecutive blocks of at most 1536 samples/channel; spans expire on return.
[[nodiscard]] bool render_objects_stream(unsigned element_count, std::uint64_t sample_count,
    std::span<const MetadataFrame> metadata, const Layout& layout,
    const std::function<void(std::span<float>)>& read,
    const std::function<void(std::span<const float>, const CarrierHistory*)>& write,
    std::string& error, unsigned first_element = 0U, bool collect_history = false);

// Validates a complete replay before PCM callbacks; retains only pending event/trim state.
[[nodiscard]] bool render_objects_stream(unsigned element_count, std::uint64_t sample_count,
    const MetadataReaderFactory& metadata, const Layout& layout,
    const std::function<void(std::span<float>)>& read,
    const std::function<void(std::span<const float>, const CarrierHistory*)>& write,
    std::string& error, unsigned first_element = 0U, bool collect_history = false);

[[nodiscard]] bool render_objects(std::span<const float> elements,
    unsigned element_count, std::uint64_t sample_count,
    std::span<const MetadataFrame> metadata, const Layout& layout,
    std::vector<float>& output, std::string& error,
    unsigned first_element = 0U,
    std::vector<CarrierHistory>* carrier_history = nullptr);

// Validates metadata as it arrives; no replay, full-source scan or known duration.
// After the initial OAMD packet, object_count=0 denotes a source_sample watermark;
// it promises that earlier metadata is complete without reading ahead for updates.
[[nodiscard]] bool render_objects_live(unsigned element_count, MetadataReader metadata,
    const Layout& layout, const LivePcmReader& read,
    const std::function<void(std::span<const float>, const CarrierHistory*)>& write,
    std::string& error);

} // namespace atmos::render
