#include "mkv/demuxer.hpp"

#include <cstring>
#include <functional>

namespace atmos::mkv {
namespace {

constexpr std::uint64_t id_ebml = 0x1A45DFA3ULL;
constexpr std::uint64_t id_segment = 0x18538067ULL;
constexpr std::uint64_t id_info = 0x1549A966ULL;
constexpr std::uint64_t id_tracks = 0x1654AE6BULL;
constexpr std::uint64_t id_track_entry = 0xAEULL;
constexpr std::uint64_t id_track_number = 0xD7ULL;
constexpr std::uint64_t id_track_type = 0x83ULL;
constexpr std::uint64_t id_codec_id = 0x86ULL;
constexpr std::uint64_t id_audio = 0xE1ULL;
constexpr std::uint64_t id_channels = 0x9FULL;
constexpr std::uint64_t id_sampling_frequency = 0xB5ULL;
constexpr std::uint64_t id_cluster = 0x1F43B675ULL;
constexpr std::uint64_t id_simple_block = 0xA3ULL;
constexpr std::uint64_t id_block_group = 0xA0ULL;
constexpr std::uint64_t id_block = 0xA1ULL;

struct Vint final {
    std::uint64_t value = 0;
    unsigned length = 0;
    bool unknown = false;
};

bool read_vint(std::span<const std::uint8_t> data, std::size_t offset, bool is_id,
    Vint& out) {
    if (offset >= data.size()) return false;
    const std::uint8_t first = data[offset];
    if (first == 0U) return false;
    unsigned length = 1U;
    std::uint8_t mask = 0x80U;
    while (length <= 8U && (first & mask) == 0U) { mask >>= 1U; ++length; }
    if (length > 8U || length > data.size() - offset) return false;
    out.length = length;
    if (is_id) {
        out.value = first;
        for (unsigned index = 1; index < length; ++index)
            out.value = (out.value << 8U) | data[offset + index];
    } else {
        out.value = first & static_cast<std::uint8_t>(mask - 1U);
        for (unsigned index = 1; index < length; ++index)
            out.value = (out.value << 8U) | data[offset + index];
        const std::uint64_t maximum = (1ULL << (7U * length)) - 1ULL;
        out.unknown = out.value == maximum;
    }
    return true;
}

std::uint64_t read_unsigned(std::span<const std::uint8_t> data, std::size_t start,
    std::size_t end) {
    std::uint64_t value = 0;
    for (std::size_t index = start; index < end && index < data.size(); ++index)
        value = (value << 8U) | data[index];
    return value;
}

std::string read_string(std::span<const std::uint8_t> data, std::size_t start,
    std::size_t end) {
    std::string value;
    for (std::size_t index = start; index < end && index < data.size(); ++index) {
        if (data[index] == 0U) break;
        value.push_back(static_cast<char>(data[index]));
    }
    return value;
}

struct Element final {
    std::uint64_t id = 0;
    std::size_t body = 0, end = 0;
    bool unknown = false;
};

bool read_element(std::span<const std::uint8_t> data, std::size_t offset,
    std::size_t end, Element& element) {
    if (end > data.size() || offset >= end) return false;
    data = data.first(end);
    Vint id, size;
    if (!read_vint(data,offset,true,id) || !read_vint(data,offset+id.length,false,size)) return false;
    const auto body = offset+id.length+size.length;
    if (!size.unknown && size.value > end-body) return false;
    element = {id.value,body,size.unknown ? end : body+static_cast<std::size_t>(size.value),size.unknown};
    return true;
}

// Visits each direct child element of [start, end).
bool for_each_child(std::span<const std::uint8_t> data, std::size_t start,
    std::size_t end, const std::function<bool(std::uint64_t, std::size_t, std::size_t)>& visit) {
    while (start < end) {
        Element element;
        if (!read_element(data,start,end,element) || !visit(element.id,element.body,element.end)) return false;
        start = element.end;
    }
    return true;
}

bool segment_bounds(std::span<const std::uint8_t> data, std::size_t& start, std::size_t& end) {
    bool found = false;
    return for_each_child(data,0,data.size(),[&](auto id,auto body,auto body_end) {
        if (id == id_segment) { start=body;end=body_end;found=true; }
        return true;
    }) && found;
}

bool segment_child(std::uint64_t id) {
    switch (id) {
    case 0x114d9b74: case id_info: case id_tracks: case id_cluster:
    case 0x1c53bb6b: case 0x1941a469: case 0x1043a770: case 0x1254c367: return true;
    default: return false;
    }
}

double read_float(std::span<const std::uint8_t> data, std::size_t start, std::size_t end) {
    const std::size_t length = end - start;
    if (length == 4U) {
        std::uint32_t raw = static_cast<std::uint32_t>(read_unsigned(data, start, end));
        float value = 0.0F;
        std::memcpy(&value, &raw, sizeof(value));
        return value;
    }
    if (length == 8U) {
        std::uint64_t raw = read_unsigned(data, start, end);
        double value = 0.0;
        std::memcpy(&value, &raw, sizeof(value));
        return value;
    }
    return 0.0;
}

void parse_track_entry(std::span<const std::uint8_t> data, std::size_t start,
    std::size_t end, Track& track) {
    static_cast<void>(for_each_child(data, start, end,
        [&](std::uint64_t id, std::size_t body, std::size_t body_end) {
            if (id == id_track_number) track.number = read_unsigned(data, body, body_end);
            else if (id == id_track_type) track.type = read_unsigned(data, body, body_end);
            else if (id == id_codec_id) track.codec_id = read_string(data, body, body_end);
            else if (id == id_audio) {
                static_cast<void>(for_each_child(data, body, body_end,
                    [&](std::uint64_t aid, std::size_t abody, std::size_t aend) {
                        if (aid == id_channels) track.channels = read_unsigned(data, abody, aend);
                        else if (aid == id_sampling_frequency)
                            track.sample_rate = read_float(data, abody, aend);
                        return true;
                    }));
            }
            return true;
        }));
}

// Validate every lace before exposing borrowed packets from the selected track.
bool parse_block(std::span<const std::uint8_t> data, std::size_t start, std::size_t end,
    std::uint64_t track_number, std::vector<std::span<const std::uint8_t>>& frames) {
    data = data.first(end);
    Vint track;
    if (!read_vint(data,start,false,track) || track.length+3U > end-start) return false;
    std::size_t offset = start+track.length+2;
    const unsigned lacing = (data[offset++] & 6U) >> 1U;
    if (!lacing) {
        if (track.value == track_number) frames.push_back(data.subspan(offset,end-offset));
        return true;
    }
    if (offset >= end) return false;
    const unsigned count = static_cast<unsigned>(data[offset++])+1;
    std::array<std::size_t,256> sizes{};
    if (lacing == 1) {
        for (unsigned index=0;index+1<count;++index) {
            unsigned byte;
            do {
                if (offset >= end) return false;
                byte=data[offset++];
                if (byte > end-start-sizes[index]) return false;
                sizes[index]+=byte;
            } while (byte == 255);
        }
    } else if (lacing == 2) {
        if ((end-offset)%count) return false;
        sizes.fill((end-offset)/count);
    } else if (count > 1) {
        Vint first;
        if (!read_vint(data,offset,false,first)) return false;
        offset+=first.length;
        if (first.value > end-offset) return false;
        sizes[0]=static_cast<std::size_t>(first.value);
        for (unsigned index=1;index+1<count;++index) {
            Vint delta;
            if (!read_vint(data,offset,false,delta)) return false;
            offset+=delta.length;
            const auto value=static_cast<std::int64_t>(delta.value)-
                static_cast<std::int64_t>((1ULL << (7U*delta.length-1U))-1);
            if (value < 0) {
                const auto magnitude=static_cast<std::uint64_t>(-value);
                if (magnitude > sizes[index-1]) return false;
                sizes[index]=sizes[index-1]-static_cast<std::size_t>(magnitude);
            } else {
                if (static_cast<std::uint64_t>(value) > end-sizes[index-1]) return false;
                sizes[index]=sizes[index-1]+static_cast<std::size_t>(value);
            }
        }
    }
    std::size_t cursor=offset;
    for (unsigned index=0;index<count;++index) {
        const auto size=index+1==count ? end-cursor : sizes[index];
        if (size > end-cursor) return false;
        if (track.value == track_number) frames.push_back(data.subspan(cursor,size));
        cursor+=size;
    }
    return true;
}

} // namespace

Demuxer::Demuxer(std::span<const std::uint8_t> data) : data_(data) {
    std::size_t segment_start=0,segment_end=0;
    if (!segment_bounds(data,segment_start,segment_end)) return;
    static_cast<void>(for_each_child(data, segment_start, segment_end,
        [&](std::uint64_t id, std::size_t body, std::size_t body_end) {
            if (id != id_tracks) return true;
            static_cast<void>(for_each_child(data, body, body_end,
                [&](std::uint64_t entry_id, std::size_t entry, std::size_t entry_end) {
                    if (entry_id != id_track_entry) return true;
                    Track track;
                    parse_track_entry(data, entry, entry_end, track);
                    tracks_.push_back(track);
                    return true;
                }));
            return false; // Tracks occurs once per Segment; media clusters need no header scan.
        }));
}

bool Demuxer::is_truehd(const Track& track) {
    return track.codec_id == "A_TRUEHD" || track.codec_id == "A_MLP";
}
bool Demuxer::is_eac3(const Track& track) { return track.codec_id == "A_EAC3"; }
bool Demuxer::is_ac3(const Track& track) { return track.codec_id == "A_AC3"; }

PacketReader::PacketReader(std::span<const std::uint8_t> data, std::uint64_t track_number)
    : data_(data),track_(track_number) {
    std::size_t start=0,end=0;
    if (!segment_bounds(data,start,end)) failure_="malformed Matroska segment";
    else { levels_[0]={start,end,id_segment,false};depth_=1; }
}

bool PacketReader::next(std::span<const std::uint8_t>& packet, std::string& error) {
    packet={};
    const auto fail=[&](const char* message) {
        failure_=message;error=failure_;return false;
    };
    if (!failure_.empty()) {error=failure_;return false;}
    for (;;) {
        if (frame_ < frames_.size()) {packet=frames_[frame_++];error.clear();return true;}
        frames_.clear();frame_=0;
        if (!depth_) {error.clear();return false;}
        auto& level=levels_[depth_-1];
        if (level.cursor == level.end) {--depth_;continue;}
        const auto begin=level.cursor;
        Element element;
        if (!read_element(data_,begin,level.end,element)) return fail("malformed Matroska element");
        if (level.id==id_cluster && level.unknown && segment_child(element.id)) {
            --depth_;levels_[depth_-1].cursor=begin;continue;
        }
        level.cursor=element.end;
        if ((level.id==id_segment && element.id==id_cluster) ||
            (level.id==id_cluster && element.id==id_block_group)) {
            if (element.unknown && element.id!=id_cluster) return fail("unknown-size Matroska block group");
            levels_[depth_++]={element.body,element.end,element.id,element.unknown};
        } else if ((level.id==id_cluster && element.id==id_simple_block) ||
            (level.id==id_block_group && element.id==id_block)) {
            if (element.unknown || !parse_block(data_,element.body,element.end,track_,frames_))
                return fail("malformed Matroska block");
        } else if (element.unknown) return fail("unsupported unknown-size Matroska element");
    }
}

bool Demuxer::extract(std::uint64_t track_number, std::vector<std::uint8_t>& output,
    std::string& error) {
    output.clear();
    PacketReader reader(data_,track_number);
    std::span<const std::uint8_t> packet;
    while (reader.next(packet,error)) output.insert(output.end(),packet.begin(),packet.end());
    if (!error.empty()) return false;
    if (output.empty()) {
        error="track "+std::to_string(track_number)+" produced no data";
        return false;
    }
    return true;
}

} // namespace atmos::mkv
