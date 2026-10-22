#include "encode/mkv_writer.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>

namespace atmos::encode {
namespace {

constexpr std::uint32_t samples_per_frame = 1536U;
constexpr std::uint32_t sample_rate = 48000U;
constexpr std::uint32_t frame_milliseconds = samples_per_frame * 1000U / sample_rate; // 32

void put_id(std::vector<std::uint8_t>& out, std::uint32_t id) {
    // EBML IDs keep their marker bits and use their minimal big-endian form.
    const unsigned length = id <= 0xFFU ? 1U : id <= 0xFFFFU ? 2U : id <= 0xFFFFFFU ? 3U : 4U;
    for (int shift = static_cast<int>(length) - 1; shift >= 0; --shift)
        out.push_back(static_cast<std::uint8_t>((id >> (8U * static_cast<unsigned>(shift))) & 0xFFU));
}

void put_size(std::vector<std::uint8_t>& out, std::uint64_t size) {
    unsigned length = 1;
    while (length < 8U && size >= ((std::uint64_t{1} << (7U * length)) - 1U)) ++length;
    for (int shift = static_cast<int>(length) - 1; shift >= 0; --shift) {
        auto byte = static_cast<std::uint8_t>((size >> (8U * static_cast<unsigned>(shift))) & 0xFFU);
        if (shift == static_cast<int>(length) - 1)
            byte = static_cast<std::uint8_t>(byte | (1U << (8U - length)));
        out.push_back(byte);
    }
}

void put_element(std::vector<std::uint8_t>& out, std::uint32_t id,
    const std::vector<std::uint8_t>& payload) {
    put_id(out, id);
    put_size(out, payload.size());
    out.insert(out.end(), payload.begin(), payload.end());
}

void put_uint(std::vector<std::uint8_t>& out, std::uint32_t id, std::uint64_t value) {
    std::vector<std::uint8_t> payload;
    for (int shift = 56; shift >= 0; shift -= 8) {
        const auto byte = static_cast<std::uint8_t>((value >> shift) & 0xFFU);
        if (payload.empty() && byte == 0U && shift != 0) continue;
        payload.push_back(byte);
    }
    put_element(out, id, payload);
}

void put_float(std::vector<std::uint8_t>& out, std::uint32_t id, double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    std::vector<std::uint8_t> payload(8);
    for (unsigned index = 0; index < 8U; ++index)
        payload[index] = static_cast<std::uint8_t>((bits >> (56U - 8U * index)) & 0xFFU);
    put_element(out, id, payload);
}

void put_string(std::vector<std::uint8_t>& out, std::uint32_t id, const char* text) {
    const std::vector<std::uint8_t> payload(text, text + std::char_traits<char>::length(text));
    put_element(out, id, payload);
}

void append(std::vector<std::uint8_t>& out, const std::vector<std::uint8_t>& more) {
    out.insert(out.end(), more.begin(), more.end());
}

} // namespace

bool write_matroska(const std::filesystem::path& ec3_path,
    const std::filesystem::path& mkv_path, std::string& error) {
    std::ifstream input(ec3_path, std::ios::binary);
    if (!input) {
        error = "cannot read " + ec3_path.string();
        return false;
    }
    std::ofstream output(mkv_path, std::ios::binary);
    if (!output) {
        error = "cannot write " + mkv_path.string();
        return false;
    }

    std::vector<std::uint8_t> header;
    {
        std::vector<std::uint8_t> ebml;
        put_uint(ebml, 0x4286U, 1);              // EBMLVersion
        put_uint(ebml, 0x42F7U, 1);              // EBMLReadVersion
        put_uint(ebml, 0x42F2U, 4);              // EBMLMaxIDLength
        put_uint(ebml, 0x42F3U, 8);              // EBMLMaxSizeLength
        put_string(ebml, 0x4282U, "matroska");   // DocType
        put_uint(ebml, 0x4287U, 4);              // DocTypeVersion
        put_uint(ebml, 0x4285U, 2);              // DocTypeReadVersion
        put_element(header, 0x1A45DFA3U, ebml);
    }
    output.write(reinterpret_cast<const char*>(header.data()),
        static_cast<std::streamsize>(header.size()));

    // The Segment size is patched once the frame count is known.
    const auto segment_size_offset = static_cast<std::streamsize>(output.tellp());
    const std::vector<std::uint8_t> segment_id = {0x18U, 0x53U, 0x80U, 0x67U};
    output.write(reinterpret_cast<const char*>(segment_id.data()),
        static_cast<std::streamsize>(segment_id.size()));
    const std::vector<std::uint8_t> unknown_size = {0x01U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    output.write(reinterpret_cast<const char*>(unknown_size.data()),
        static_cast<std::streamsize>(unknown_size.size()));

    // First pass only indexes the frames so Info can carry the real duration;
    // the second pass streams them into clusters one at a time, so memory stays
    // at one cluster regardless of the input length.
    std::size_t frame_count = 0;
    std::vector<std::uint8_t> bytes(4);
    while (input.read(reinterpret_cast<char*>(bytes.data()), 4)) {
        if (bytes[0] != 0x0BU || bytes[1] != 0x77U) {
            error = "not an E-AC-3 frame in " + ec3_path.string();
            return false;
        }
        const auto size = (static_cast<std::size_t>(bytes[2] & 0x07U) << 8U) | bytes[3];
        const std::size_t frame_size = size * 2U + 2U;
        input.seekg(static_cast<std::streamoff>(frame_size - 4U), std::ios::cur);
        if (!input) {
            error = "truncated E-AC-3 frame in " + ec3_path.string();
            return false;
        }
        ++frame_count;
    }
    if (frame_count == 0U) {
        error = "no E-AC-3 frames in " + ec3_path.string();
        return false;
    }
    input.clear();
    input.seekg(0);

    std::vector<std::uint8_t> info;
    put_uint(info, 0x2AD7B1U, 1000000U);  // TimecodeScale: 1 ms
    put_float(info, 0x4489U,
        static_cast<double>(frame_count) * frame_milliseconds);
    put_string(info, 0x4D80U, "ddp-atmos");
    put_string(info, 0x5741U, "ddp-atmos");
    std::vector<std::uint8_t> info_element;
    put_element(info_element, 0x1549A966U, info);
    output.write(reinterpret_cast<const char*>(info_element.data()),
        static_cast<std::streamsize>(info_element.size()));

    std::vector<std::uint8_t> track;
    put_uint(track, 0xD7U, 1U);            // TrackNumber
    put_uint(track, 0x73C5U, 1U);          // TrackUID
    put_uint(track, 0x83U, 2U);            // TrackType: audio
    put_uint(track, 0x9CU, 0U);            // FlagLacing
    put_string(track, 0x86U, "A_EAC3");    // CodecID
    std::vector<std::uint8_t> audio;
    put_float(audio, 0xB5U, static_cast<double>(sample_rate));
    put_uint(audio, 0x9FU, 6U);            // Channels
    std::vector<std::uint8_t> audio_element;
    put_element(audio_element, 0xE1U, audio);
    append(track, audio_element);
    std::vector<std::uint8_t> track_entry;
    put_element(track_entry, 0xAEU, track);
    std::vector<std::uint8_t> tracks;
    put_element(tracks, 0x1654AE6BU, track_entry);
    output.write(reinterpret_cast<const char*>(tracks.data()),
        static_cast<std::streamsize>(tracks.size()));

    constexpr std::size_t cluster_frames = 31U; // about one second
    std::size_t index = 0;
    while (index < frame_count) {
        const std::size_t first = index;
        std::vector<std::uint8_t> cluster;
        put_uint(cluster, 0xE7U, static_cast<std::uint64_t>(first) * frame_milliseconds);
        for (std::size_t in_cluster = 0;
             in_cluster < cluster_frames && index < frame_count; ++in_cluster, ++index) {
            if (!input.read(reinterpret_cast<char*>(bytes.data()), 4)) {
                error = "truncated E-AC-3 frame in " + ec3_path.string();
                return false;
            }
            const auto size = (static_cast<std::size_t>(bytes[2] & 0x07U) << 8U) | bytes[3];
            const std::size_t frame_size = size * 2U + 2U;
            std::vector<std::uint8_t> block(frame_size + 4U);
            block[0] = 0x81U; // track number 1 as a one-byte VINT
            const auto relative = static_cast<std::uint16_t>((index - first) * frame_milliseconds);
            block[1] = static_cast<std::uint8_t>(relative >> 8U);
            block[2] = static_cast<std::uint8_t>(relative & 0xFFU);
            block[3] = 0x80U; // keyframe
            block[4] = bytes[0]; block[5] = bytes[1]; block[6] = bytes[2]; block[7] = bytes[3];
            if (!input.read(reinterpret_cast<char*>(block.data() + 8),
                    static_cast<std::streamsize>(frame_size - 4U))) {
                error = "truncated E-AC-3 frame in " + ec3_path.string();
                return false;
            }
            put_element(cluster, 0xA3U, block);
        }
        std::vector<std::uint8_t> cluster_element;
        put_element(cluster_element, 0x1F43B675U, cluster);
        output.write(reinterpret_cast<const char*>(cluster_element.data()),
            static_cast<std::streamsize>(cluster_element.size()));
    }

    const auto segment_end = static_cast<std::streamsize>(output.tellp());
    const auto segment_size = static_cast<std::uint64_t>(
        segment_end - segment_size_offset - 12); // segment ID + size field
    output.seekp(segment_size_offset + 4);
    std::vector<std::uint8_t> size_bytes = {0x01U};
    for (int shift = 48; shift >= 0; shift -= 8)
        size_bytes.push_back(static_cast<std::uint8_t>((segment_size >> shift) & 0xFFU));
    output.write(reinterpret_cast<const char*>(size_bytes.data()),
        static_cast<std::streamsize>(size_bytes.size()));
    output.close();
    if (!output) {
        error = "failed while writing " + mkv_path.string();
        return false;
    }
    return true;
}

bool write_truehd_matroska(const std::filesystem::path& source,
    const std::filesystem::path& target, std::uint64_t samples, unsigned channels,
    std::string& error) {
    std::ifstream input(source,std::ios::binary);
    std::ofstream output(target,std::ios::binary);
    if(!input || !output || !samples || !channels || channels>16) {
        error="cannot open TrueHD Matroska input/output or invalid audio geometry";return false;
    }
    std::vector<std::uint8_t> ebml,header;
    put_uint(ebml,0x4286,1);put_uint(ebml,0x42f7,1);
    put_uint(ebml,0x42f2,4);put_uint(ebml,0x42f3,8);
    put_string(ebml,0x4282,"matroska");put_uint(ebml,0x4287,4);put_uint(ebml,0x4285,2);
    put_element(header,0x1a45dfa3,ebml);
    const auto write=[&](const auto& bytes) {
        output.write(reinterpret_cast<const char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
    };
    write(header);
    const auto segment_offset=output.tellp();
    write(std::vector<std::uint8_t>{0x18,0x53,0x80,0x67,1,255,255,255,255,255,255,255});
    std::vector<std::uint8_t> info,element;
    put_uint(info,0x2ad7b1,1000); // 1 microsecond, cumulative timestamps avoid drift.
    put_float(info,0x4489,static_cast<double>(samples)*1000000.0/48000.0);
    put_string(info,0x4d80,"ddp-atmos");put_string(info,0x5741,"ddp-atmos");
    put_element(element,0x1549a966,info);write(element);
    std::vector<std::uint8_t> track,audio,entry,tracks;
    put_uint(track,0xd7,1);put_uint(track,0x73c5,1);put_uint(track,0x83,2);
    put_uint(track,0x9c,0);put_string(track,0x86,"A_TRUEHD");
    put_float(audio,0xb5,48000);put_uint(audio,0x9f,channels);put_uint(audio,0x6264,24);
    put_element(track,0xe1,audio);put_element(entry,0xae,track);
    put_element(tracks,0x1654ae6b,entry);write(tracks);
    const auto units=(samples+39)/40;
    for(std::uint64_t index=0;index<units;) {
        const auto first=index;
        const auto start_us=first*40*1000000/48000;
        std::vector<std::uint8_t> cluster;
        put_uint(cluster,0xe7,start_us);
        for(unsigned n=0;n<32 && index<units;++n,++index) {
            std::array<std::uint8_t,4> prefix{};
            input.read(reinterpret_cast<char*>(prefix.data()),4);
            const auto size=static_cast<unsigned>(((prefix[0]&15U)<<8)|prefix[1])*2;
            if(!input || size<8) {error="truncated/invalid TrueHD Matroska packet";return false;}
            std::vector<std::uint8_t> block(size+4);
            block[0]=0x81;
            const auto relative=index*40*1000000/48000-start_us;
            block[1]=static_cast<std::uint8_t>(relative>>8);
            block[2]=static_cast<std::uint8_t>(relative);
            std::copy(prefix.begin(),prefix.end(),block.begin()+4);
            input.read(reinterpret_cast<char*>(block.data()+8),size-4);
            if(!input) {error="truncated TrueHD access unit in Matroska input";return false;}
            const bool major=block[8]==0xf8 && block[9]==0x72 && block[10]==0x6f && block[11]==0xba;
            if(index==0 && !major) {error="TrueHD Matroska input must start at major sync";return false;}
            block[3]=major ? 0x80 : 0;
            put_element(cluster,0xa3,block);
        }
        element.clear();put_element(element,0x1f43b675,cluster);write(element);
        if(!output) {error="failed writing TrueHD Matroska cluster";return false;}
    }
    if(input.peek()!=std::char_traits<char>::eof()) {error="TrueHD duration/packet count mismatch";return false;}
    const auto end=output.tellp();
    const auto segment_bytes=static_cast<std::uint64_t>(end-segment_offset-std::streamoff(12));
    output.seekp(segment_offset+std::streamoff(4));
    std::vector<std::uint8_t> size_bytes{1};
    for(int shift=48;shift>=0;shift-=8)size_bytes.push_back(static_cast<std::uint8_t>(segment_bytes>>shift));
    write(size_bytes);output.close();
    if(!output) {error="failed finishing TrueHD Matroska output";return false;}
    error.clear();return true;
}

} // namespace atmos::encode
