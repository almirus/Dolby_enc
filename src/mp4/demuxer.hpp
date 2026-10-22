#pragma once

#include <cstdint>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace atmos::mp4 {
namespace detail {
using Bytes = std::span<const std::uint8_t>;

inline std::uint64_t number(Bytes bytes, std::size_t offset, unsigned width=4) {
    if (offset>bytes.size() || width>bytes.size()-offset)
        throw std::runtime_error("truncated MP4 table");
    std::uint64_t value=0;
    for (unsigned i=0;i<width;++i) value=(value<<8)|bytes[offset+i];
    return value;
}

struct Box { std::string_view type; Bytes body; };

inline std::vector<Box> boxes(Bytes bytes) {
    std::vector<Box> result;
    while (!bytes.empty()) {
        auto size=number(bytes,0);
        unsigned header=8;
        if (size==1) { size=number(bytes,8,8); header=16; }
        else if (!size) size=bytes.size();
        if (size<header || size>bytes.size()) throw std::runtime_error("invalid MP4 box size");
        result.push_back({{reinterpret_cast<const char*>(bytes.data()+4),4},
            bytes.subspan(header,static_cast<std::size_t>(size)-header)});
        bytes=bytes.subspan(static_cast<std::size_t>(size));
    }
    return result;
}

inline Bytes child(Bytes parent, std::string_view type) {
    for (const auto& box:boxes(parent)) if (box.type==type) return box.body;
    return {};
}

inline std::size_t entries(Bytes table, unsigned header, unsigned width) {
    const auto count=number(table,header-4);
    if (count>(table.size()-header)/width) throw std::runtime_error("truncated MP4 sample table");
    return static_cast<std::size_t>(count);
}
} // namespace detail

// Classic MP4/M4A sample tables; fragmented/encrypted tracks require a different reader.
inline std::vector<std::uint8_t> extract_eac3(std::span<const std::uint8_t> file, unsigned audio_track=1) {
    using namespace detail;
    const auto moov=child(file,"moov");
    if (moov.empty()) throw std::runtime_error("MP4 has no moov box");
    if (!child(moov,"mvex").empty()) throw std::runtime_error("fragmented MP4 object export is unsupported");
    unsigned audio=0;
    for (const auto& track:boxes(moov)) {
        if (track.type!="trak") continue;
        const auto media=child(track.body,"mdia");
        const auto handler=child(media,"hdlr");
        if (handler.size()<12 || std::string_view(reinterpret_cast<const char*>(handler.data()+8),4)!="soun") continue;
        if (++audio!=audio_track) continue;
        const auto table=child(child(media,"minf"),"stbl");
        const auto descriptions=child(table,"stsd");
        if (descriptions.empty()) throw std::runtime_error("selected MP4 audio track has no sample descriptions");
        const auto description_count=number(descriptions,4);
        const auto formats=boxes(descriptions.subspan(8));
        if (formats.size()!=description_count) throw std::runtime_error("invalid MP4 sample descriptions");
        std::size_t description=0;
        for (std::size_t i=0;i<formats.size();++i) if (formats[i].type=="ec-3") {description=i+1;break;}
        if (!description) throw std::runtime_error("selected MP4 audio track is not unencrypted E-AC-3 (ec-3)");
        const auto sizes=child(table,"stsz");
        const auto fixed=number(sizes,4);
        const auto sample_count=number(sizes,8);
        if (!sample_count || sample_count>file.size()) throw std::runtime_error("invalid MP4 sample count");
        if (!fixed && entries(sizes,12,4)!=sample_count) throw std::runtime_error("invalid MP4 sample sizes");
        const auto mapping=child(table,"stsc");
        const auto mappings=entries(mapping,8,12);
        auto offsets=child(table,"stco");
        unsigned width=4;
        if (offsets.empty()) {offsets=child(table,"co64");width=8;}
        const auto chunks=entries(offsets,8,width);
        if (!mappings || !chunks) throw std::runtime_error("empty MP4 chunk table");
        std::uint64_t previous=0;
        for (std::size_t i=0;i<mappings;++i) {
            const auto first=number(mapping,8+12*i);
            if ((!i && first!=1) || first<=previous || first>chunks ||
                !number(mapping,12+12*i) || number(mapping,16+12*i)!=description)
                throw std::runtime_error("unsupported MP4 sample-to-chunk mapping");
            previous=first;
        }
        std::vector<std::uint8_t> output;
        std::size_t sample=0, group=0;
        for (std::size_t chunk=1;chunk<=chunks;++chunk) {
            if (group+1<mappings && number(mapping,8+12*(group+1))==chunk) ++group;
            auto offset=number(offsets,8+width*(chunk-1),width);
            const auto count=number(mapping,12+12*group);
            if (count>sample_count-sample) throw std::runtime_error("invalid MP4 chunk sample count");
            for (std::size_t i=0;i<count;++i,++sample) {
                const auto size=fixed ? fixed : number(sizes,12+4*sample);
                if (!size || offset>file.size() || size>file.size()-offset || size>file.size()-output.size())
                    throw std::runtime_error("invalid MP4 sample offset/size");
                const auto data=file.subspan(static_cast<std::size_t>(offset),static_cast<std::size_t>(size));
                output.insert(output.end(),data.begin(),data.end());
                offset+=size;
            }
        }
        if (sample!=sample_count) throw std::runtime_error("incomplete MP4 sample table");
        return output;
    }
    throw std::runtime_error("MP4/M4A audio track number is out of range");
}
} // namespace atmos::mp4
