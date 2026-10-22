#include "pcm_wav_reader.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

namespace atmos {
namespace {

std::uint64_t le(const std::uint8_t* p, unsigned bytes) {
    std::uint64_t result = 0;
    for (unsigned i = 0; i < bytes; ++i) result |= std::uint64_t(p[i]) << (8*i);
    return result;
}

void exact(std::ifstream& file, std::span<std::uint8_t> bytes) {
    if (!file.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size())))
        throw std::runtime_error("truncated PCM WAV");
}

std::vector<std::uint8_t> layout_ids(std::string_view layout) {
    if (layout == "5.1") return {0,1,2,3,4,5};
    if (layout == "7.1") return {0,1,2,3,4,5,6,7};
    if (layout == "5.1.2") return {0,1,2,3,4,5,10,11};
    if (layout == "5.1.4") return {0,1,2,3,4,5,8,9,12,13};
    if (layout == "7.1.4") return {0,1,2,3,4,5,6,7,8,9,12,13};
    throw std::runtime_error("unsupported PCM WAV input layout");
}

} // namespace

PcmWavReader::PcmWavReader(const std::filesystem::path& path, std::string_view layout)
    : file_(path, std::ios::binary) {
    if (!file_) throw std::runtime_error("cannot open PCM WAV");
    file_.seekg(0, std::ios::end);
    const auto length = file_.tellg();
    if (length < 12) throw std::runtime_error("truncated PCM WAV header");
    const auto physical_size = static_cast<std::uint64_t>(length);
    file_.seekg(0);
    std::array<std::uint8_t, 12> header{};
    exact(file_, header);
    const std::string type(header.begin(), header.begin()+4);
    const bool rf64 = type == "RF64";
    if ((!rf64 && type != "RIFF") || std::string(header.begin()+8, header.end()) != "WAVE")
        throw std::runtime_error("expected RIFF/WAVE or RF64 PCM input");
    auto end = rf64 ? physical_size : le(header.data()+4, 4)+8;
    if (end < 12 || end > physical_size || (rf64 && le(header.data()+4,4) != 0xffffffffU))
        throw std::runtime_error("invalid PCM WAV RIFF size");
    bool have_fmt = false, have_data = false, have_ds64 = false;
    std::uint64_t data_start = 0, data_bytes = 0, rf64_bytes = 0, rf64_samples = 0;
    std::uint32_t mask = 0;
    unsigned align = 0;
    for (std::uint64_t offset = 12; offset < end;) {
        if (end-offset < 8) throw std::runtime_error("truncated PCM WAV chunk header");
        file_.seekg(static_cast<std::streamoff>(offset));
        std::array<std::uint8_t, 8> chunk{};
        exact(file_, chunk);
        const std::string name(chunk.begin(), chunk.begin()+4);
        auto size = le(chunk.data()+4,4);
        if (rf64 && offset == 12 && name != "ds64")
            throw std::runtime_error("RF64 must begin with ds64");
        if (size == 0xffffffffU) {
            if (!rf64 || !have_ds64 || name != "data")
                throw std::runtime_error("unsupported RF64 chunk size table reference");
            size = rf64_bytes;
        }
        const auto body = offset+8;
        if (size > end-body || (size & 1U) > end-body-size)
            throw std::runtime_error("PCM WAV chunk exceeds RIFF bounds");
        if (name == "ds64" && rf64) {
            if (have_ds64 || size < 28) throw std::runtime_error("invalid RF64 ds64");
            std::array<std::uint8_t, 28> ds{};
            exact(file_, ds);
            const auto riff_bytes = le(ds.data(),8);
            if (riff_bytes > physical_size-8 || riff_bytes < body+size-8 ||
                le(ds.data()+24,4) > (size-28)/12)
                throw std::runtime_error("invalid RF64 sizes");
            end = riff_bytes+8;
            if (size > end-body || (size & 1U) > end-body-size)
                throw std::runtime_error("ds64 exceeds RF64 bounds");
            rf64_bytes = le(ds.data()+8,8);
            rf64_samples = le(ds.data()+16,8);
            have_ds64 = true;
        } else if (name == "fmt ") {
            if (have_fmt || size < 16) throw std::runtime_error("invalid PCM WAV fmt chunk");
            std::array<std::uint8_t, 40> fmt{};
            exact(file_, std::span(fmt).first(static_cast<std::size_t>(std::min<std::uint64_t>(size,40))));
            auto format = le(fmt.data(),2);
            channels_ = static_cast<unsigned>(le(fmt.data()+2,2));
            sample_rate_ = static_cast<unsigned>(le(fmt.data()+4,4));
            align = static_cast<unsigned>(le(fmt.data()+12,2));
            container_bits_ = valid_bits_ = static_cast<unsigned>(le(fmt.data()+14,2));
            if (format == 0xfffe) {
                constexpr std::array<std::uint8_t,16> pcm_guid{1,0,0,0,0,0,16,0,128,0,0,170,0,56,155,113};
                if (size < 40 || le(fmt.data()+16,2) < 22 || le(fmt.data()+16,2) > size-18 ||
                    !std::equal(pcm_guid.begin(),pcm_guid.end(),fmt.begin()+24))
                    throw std::runtime_error("expected extensible integer PCM GUID");
                valid_bits_ = static_cast<unsigned>(le(fmt.data()+18,2));
                mask = static_cast<std::uint32_t>(le(fmt.data()+20,4));
                format = 1;
            }
            if (format != 1 || (valid_bits_ != 16 && valid_bits_ != 24) ||
                (container_bits_ != 16 && container_bits_ != 24 && container_bits_ != 32) ||
                valid_bits_ > container_bits_)
                throw std::runtime_error("TrueHD input requires PCM16/PCM24; float/PCM32 conversion must be explicit");
            if (sample_rate_ != 48000 || channels_ == 0 || channels_ > 16 ||
                align != channels_*(container_bits_/8) || le(fmt.data()+8,4) != 48000ULL*align)
                throw std::runtime_error("invalid PCM WAV geometry or unsupported sample rate (requires 48 kHz)");
            have_fmt = true;
        } else if (name == "data") {
            if (have_data) throw std::runtime_error("multiple PCM WAV data chunks are unsupported");
            have_data = true;
            data_start = body;
            data_bytes = size;
        }
        offset = body+size+(size & 1U);
    }
    if (!have_fmt || !have_data || (rf64 && (!have_ds64 || rf64_bytes != data_bytes)) || data_bytes % align)
        throw std::runtime_error("missing or incomplete PCM WAV data");
    frames_ = remaining_ = data_bytes/align;
    if (!frames_ || (rf64_samples && rf64_samples != frames_))
        throw std::runtime_error("empty PCM WAV or inconsistent RF64 sample count");
    if (mask) {
        constexpr int dolby_ids[]{0,1,2,3,6,7,-1,-1,-1,4,5,-1,8,-1,9,12,-1,13};
        for (unsigned bit = 0; bit < 32; ++bit) {
            if (!(mask & (1U << bit))) continue;
            if (bit >= std::size(dolby_ids) || dolby_ids[bit] < 0)
                throw std::runtime_error("unsupported PCM WAV speaker mask");
            int id = dolby_ids[bit];
            if (channels_ == 6 && !(mask & 0x600U) && (id == 6 || id == 7)) id -= 2;
            ids_.push_back(static_cast<std::uint8_t>(id));
        }
        if (ids_.size() != channels_) throw std::runtime_error("PCM WAV mask/count mismatch");
    } else {
        if (layout.empty()) throw std::runtime_error("PCM WAV without speaker mask requires --input-layout");
        ids_ = layout_ids(layout);
        if (ids_.size() != channels_) throw std::runtime_error("PCM WAV layout/count mismatch");
    }
    order_.resize(channels_);
    std::iota(order_.begin(), order_.end(),0U);
    std::sort(order_.begin(),order_.end(),[&](unsigned a,unsigned b) { return ids_[a] < ids_[b]; });
    const auto source_ids = ids_;
    for (unsigned c = 0; c < channels_; ++c) ids_[c] = source_ids[order_[c]];
    if (!layout.empty() && ids_ != layout_ids(layout))
        throw std::runtime_error("PCM WAV mask conflicts with --input-layout");
    file_.seekg(static_cast<std::streamoff>(data_start));
}

std::span<const std::int32_t> PcmWavReader::read(std::size_t max_frames) {
    if (!max_frames || max_frames > std::numeric_limits<std::size_t>::max()/channels_/4 ||
        max_frames > static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max())/channels_/4)
        throw std::runtime_error("invalid PCM WAV read size");
    const auto frames = static_cast<std::size_t>(std::min<std::uint64_t>(remaining_,max_frames));
    const unsigned bytes = container_bits_/8, padding = container_bits_-valid_bits_;
    raw_.resize(frames*channels_*bytes);
    pcm_.resize(frames*channels_);
    exact(file_,raw_);
    for (std::size_t f = 0; f < frames; ++f) {
        for (unsigned c = 0; c < channels_; ++c) {
            const auto raw = static_cast<std::uint32_t>(le(raw_.data()+(f*channels_+order_[c])*bytes,bytes));
            if (padding && (raw & ((1U << padding)-1U)))
                throw std::runtime_error("nonzero padding in extensible PCM sample");
            const auto signed32 = std::bit_cast<std::int32_t>(raw << (32-container_bits_));
            pcm_[f*channels_+c] = signed32 >> 8;
        }
    }
    remaining_ -= frames;
    return pcm_;
}

} // namespace atmos
