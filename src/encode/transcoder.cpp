#include "encode/transcoder.hpp"
#include "encode/truehd_file.hpp"
#include "joc_compact.hpp"
#include "object_export.hpp"

#include "encode/eac3_core.hpp"
#include "encode/joc_encoder.hpp"
#include "encode/emdf_writer.hpp"
#include "encode/mkv_writer.hpp"
#include "encode/mp4_writer.hpp"
#include "encode/oamd_reframer.hpp"
#include "eac3/decoder.hpp"
#include "eac3/joc_decoder.hpp"
#include "mkv/demuxer.hpp"
#include "mp4/demuxer.hpp"
#include "render/object_renderer.hpp"
#include "render/carrier_limiter.hpp"
#include "encode/joc_gain.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>
#include "encode/frame_muxer.hpp"
#include "common/wav.hpp"
#include "common/mapped_file.hpp"
#include "model/layout.hpp"
#include "truehd/access_unit.hpp"
#include "truehd/decoder.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <memory>
#include <optional>
#include <utility>
#include <span>
#include <string>
#include <vector>

namespace atmos::encode {
namespace {

void progress(const Options& options, const char* stage, int percent) {
    if (options.progress) options.progress(stage,percent);
}

std::uint32_t read_u32(const std::uint8_t* bytes, bool big_endian) {
    if (big_endian)
        return (std::uint32_t(bytes[0]) << 24) | (std::uint32_t(bytes[1]) << 16) |
            (std::uint32_t(bytes[2]) << 8) | bytes[3];
    return (std::uint32_t(bytes[3]) << 24) | (std::uint32_t(bytes[2]) << 16) |
        (std::uint32_t(bytes[1]) << 8) | bytes[0];
}

std::uint16_t read_u16(const std::uint8_t* bytes, bool big_endian) {
    if (big_endian) return static_cast<std::uint16_t>((bytes[0] << 8) | bytes[1]);
    return static_cast<std::uint16_t>((bytes[1] << 8) | bytes[0]);
}

// Minimal RIFF/WAVE reader for PCM 16/24/32 and IEEE float 32.
bool read_wav(const std::filesystem::path& path, std::vector<float>& samples,
    std::uint32_t& sample_rate, std::uint16_t& channels, std::uint32_t& channel_mask, std::string& error, double duration=0) {
    const MappedFile file(path);
    const auto data = file.bytes();
    channel_mask = 0;
    if (data.size() < 44U || std::string(data.begin(), data.begin() + 4) != "RIFF" ||
        std::string(data.begin() + 8, data.begin() + 12) != "WAVE") {
        error = "not a RIFF/WAVE file";
        return false;
    }
    std::uint16_t format = 0;
    std::uint16_t bits = 0;
    std::size_t offset = 12U;
    std::span<const std::uint8_t> payload;
    while (offset + 8U <= data.size()) {
        const std::string id(data.begin() + static_cast<std::ptrdiff_t>(offset),
            data.begin() + static_cast<std::ptrdiff_t>(offset) + 4);
        const std::uint32_t size = read_u32(&data[offset + 4U], false);
        const std::size_t body = offset + 8U;
        if (body + size > data.size()) break;
        if (id == "fmt ") {
            if (size < 16) { error = "truncated WAV fmt chunk"; return false; }
            format = read_u16(&data[body], false);
            channels = read_u16(&data[body + 2U], false);
            sample_rate = read_u32(&data[body + 4U], false);
            bits = read_u16(&data[body + 14U], false);
            // Multichannel WAVs are almost always WAVE_FORMAT_EXTENSIBLE; the
            // real sample format lives in the first word of the sub-format GUID.
            // `bits` stays the container width, which is what the byte layout
            // below needs (a 24-in-32 file reports 32 here).
            if (format == 0xFFFEU) {
                if (size < 40) { error = "truncated WAVE_FORMAT_EXTENSIBLE chunk"; return false; }
                channel_mask = read_u32(&data[body + 20U], false);
                format = read_u16(&data[body + 24U], false);
            }
        } else if (id == "data") {
            payload = std::span<const std::uint8_t>(data).subspan(body, size);
        }
        offset = body + size + (size & 1U);
    }
    if (payload.empty() || channels == 0U || bits == 0U) {
        error = "WAV is missing fmt/data chunks";
        return false;
    }
    if (!((format == 1 && (bits == 16 || bits == 24 || bits == 32)) || (format == 3 && bits == 32))) {
        error = "unsupported WAV format"; return false;
    }
    const unsigned bytes_per_sample = bits / 8U;
    if (payload.size() % (bytes_per_sample * channels) != 0) {
        error = "incomplete WAV sample frame"; return false;
    }
    const std::size_t count = duration ? std::min(payload.size()/bytes_per_sample,
        static_cast<std::size_t>(duration*sample_rate)*channels) : payload.size()/bytes_per_sample;
    samples.resize(count);
    for (std::size_t index = 0; index < count; ++index) {
        const std::uint8_t* sample = payload.data() + index * bytes_per_sample;
        if (format == 3U && bits == 32U) {
            std::uint32_t raw = read_u32(sample, false);
            float value = 0.0F;
            std::memcpy(&value, &raw, sizeof(value));
            samples[index] = value;
        } else if (format == 1U && bits == 16U) {
            samples[index] = static_cast<float>(
                static_cast<std::int16_t>(read_u16(sample, false))) / 32768.0F;
        } else if (format == 1U && bits == 24U) {
            std::int32_t value = (static_cast<std::int32_t>(sample[2]) << 16) |
                (static_cast<std::int32_t>(sample[1]) << 8) | sample[0];
            if ((value & 0x800000) != 0) value |= ~0xFFFFFF;
            samples[index] = static_cast<float>(value) / 8388608.0F;
        } else if (format == 1U && bits == 32U) {
            samples[index] = static_cast<float>(
                static_cast<std::int32_t>(read_u32(sample, false))) / 2147483648.0F;
        } else {
            error = "unsupported WAV format";
            return false;
        }
    }
    return true;
}

// A 1 kHz tone on every bed channel, used when no input file is given.
void synthesise_bed(std::vector<float>& samples, std::uint32_t sample_rate,
    std::uint16_t channels, double seconds) {
    const std::size_t frames = static_cast<std::size_t>(
        static_cast<double>(sample_rate) * seconds);
    samples.resize(frames * channels);
    for (std::size_t frame = 0; frame < frames; ++frame) {
        const double t = static_cast<double>(frame) / static_cast<double>(sample_rate);
        const double value = 0.5 * std::sin(2.0 * std::numbers::pi * 1000.0 * t);
        for (std::uint16_t channel = 0; channel < channels; ++channel)
            samples[frame * channels + channel] = static_cast<float>(value);
    }
}

} // namespace

namespace {

mkv::Track selected_mkv_audio(std::span<const std::uint8_t> bytes, unsigned audio_track) {
    mkv::Demuxer demuxer(bytes);
    unsigned audio=0;
    for (const auto& track:demuxer.tracks())
        if (track.type==2 && ++audio==audio_track) return track;
    throw std::runtime_error("MKV audio track number is out of range");
}

class TruehdInput final {
public:
    explicit TruehdInput(std::shared_ptr<const MappedFile> file, unsigned audio_track=1) : file_(std::move(file)),raw_(file_->bytes()) {
        if (raw_.size()>=4 && raw_[0]==0x1a && raw_[1]==0x45 && raw_[2]==0xdf && raw_[3]==0xa3) {
            const auto track=selected_mkv_audio(raw_,audio_track);
            if (!mkv::Demuxer::is_truehd(track)) throw std::runtime_error("selected MKV audio track is not TrueHD");
            track_=track.number;
            packets_.emplace(raw_,track_);raw_={};
        }
    }
    std::optional<truehd::AccessUnit> next(const truehd::MajorSync* info) {
        consume(pending_);pending_=0;
        for (;;) {
            const auto header=peek(8);
            if (header.size()<8) {
                if (info && !header.empty()) throw std::runtime_error("truncated TrueHD access unit");
                return std::nullopt;
            }
            const std::size_t size=static_cast<std::size_t>(((header[0]&15U)<<8U)|header[1])*2;
            const auto data=peek(std::max<std::size_t>(8,size));
            auto unit=truehd::parse_access_unit(data,0,info);
            if (unit) {
                current_=data.first(unit->size);pending_=unit->size;return unit;
            }
            if (info) throw std::runtime_error("invalid TrueHD access unit at byte "+std::to_string(position_));
            consume(2);
        }
    }
    std::span<const std::uint8_t> bytes() const {return current_;}
    std::uint64_t position() const {return position_;}
    int percent() const {
        const auto size=file_->bytes().size();
        const auto consumed=packets_ && packet_.data() ?
            static_cast<std::uint64_t>(packet_.data()-file_->bytes().data()) : position_+current_.size();
        return size ? static_cast<int>(std::min(99.0,100.0*static_cast<double>(consumed)/size)) : 0;
    }
private:
    std::span<const std::uint8_t> peek(std::size_t count) {
        if (!packets_) return raw_.first(std::min(count,raw_.size()));
        if (count>buffer_.size()) throw std::runtime_error("TrueHD read exceeds maximum access unit");
        if (end_-begin_<count && begin_) {
            std::memmove(buffer_.data(),buffer_.data()+begin_,end_-begin_);
            end_-=begin_;begin_=0;
        }
        while (end_-begin_<count && !eof_) {
            if (packet_.empty()) {
                std::string error;
                if (!packets_->next(packet_,error)) {
                    if (!error.empty()) throw std::runtime_error("cannot demux TrueHD track: "+error);
                    eof_=true;break;
                }
                if (packet_.empty()) continue;
            }
            const auto length=std::min(count-(end_-begin_),packet_.size());
            std::copy_n(packet_.begin(),length,buffer_.begin()+end_);
            packet_=packet_.subspan(length);end_+=length;
        }
        if (eof_ && !position_ && end_==begin_)
            throw std::runtime_error("cannot demux TrueHD track: track "+std::to_string(track_)+" produced no data");
        return std::span<const std::uint8_t>(buffer_).subspan(begin_,std::min(count,end_-begin_));
    }
    void consume(std::size_t count) {
        if (packets_) {
            if (count>end_-begin_) throw std::runtime_error("invalid TrueHD reader advance");
            begin_+=count;
        } else raw_=raw_.subspan(count);
        position_+=count;
    }
    std::shared_ptr<const MappedFile> file_;
    std::span<const std::uint8_t> raw_,packet_,current_;
    std::optional<mkv::PacketReader> packets_;
    std::array<std::uint8_t,8192> buffer_{};
    std::size_t begin_=0,end_=0,pending_=0;
    std::uint64_t position_=0,track_=0;
    bool eof_=false;
};

class TruehdOamdReader final {
public:
    TruehdOamdReader(const Options& options, std::shared_ptr<const MappedFile> source) : input_(std::move(source),options.audio_track),
        limit_(options.truehd_seconds == 0 ? std::numeric_limits<std::size_t>::max() :
            static_cast<std::size_t>(options.truehd_seconds*48000)) {}
    bool next(TimedOamdPayload& output) {
        if (auto* metadata=next_metadata()) {
            output={origin_,std::move(metadata->oamd_payload)};return true;
        }
        return false;
    }
    bool next(render::MetadataPacket& output) {
        if (auto* metadata=next_metadata()) {
            output={origin_,{origin_+metadata->oamd_payload.sample_offset,std::move(metadata->oamd)}};return true;
        }
        return false;
    }
private:
    eac3::EmdfMetadata* next_metadata() {
        for (;;) {
            while (next_ < metadata_.size()) {
                auto& metadata=metadata_[next_++];
                if (!metadata.valid) throw std::runtime_error("invalid TrueHD Atmos metadata: "+metadata.error);
                if (metadata.has_oamd) return &metadata;
            }
            if (produced_ >= limit_) return nullptr;
            auto unit=input_.next(synchronized_ ? &info_ : nullptr);
            if (!unit) return nullptr;
            if (unit->major_sync) {info_=*unit->major_sync;synchronized_=true;}
            truehd::PcmFrame pcm;std::string error;
            if (!decoder_.decode(input_.bytes(),pcm,error))
                throw std::runtime_error("TrueHD decode at byte "+std::to_string(input_.position())+": "+error);
            origin_=produced_;
            produced_+=std::min<std::size_t>(pcm.samples_per_channel,limit_-produced_);
            metadata_=std::move(unit->atmos_metadata);next_=0;
        }
    }
private:
    TruehdInput input_;
    truehd::BedDecoder decoder_{3};
    truehd::MajorSync info_{};
    std::vector<eac3::EmdfMetadata> metadata_;
    std::size_t limit_,produced_=0,origin_=0,next_=0;
    bool synchronized_=false;
};

OamdReader truehd_oamd_source(const Options& options, std::shared_ptr<const MappedFile> source) {
    const auto reader=std::make_shared<TruehdOamdReader>(options,std::move(source));
    return [reader](TimedOamdPayload& output) { return reader->next(output); };
}

render::MetadataReader truehd_renderer_source(const Options& options,
    std::shared_ptr<const MappedFile> source, unsigned padding) {
    const auto reader=std::make_shared<TruehdOamdReader>(options,std::move(source));
    return [reader,padding](render::MetadataPacket& output) {
        if (!reader->next(output)) return false;
        output.source_sample+=padding;output.frame.sample+=padding;return true;
    };
}

struct AudioInput {
    std::shared_ptr<const MappedFile> source;
    std::vector<float> bed; // WAV channel order
    std::vector<float> elements;
    std::vector<render::CarrierHistory> carrier_gains;
    std::uint64_t sample_count = 0;
    unsigned render_delay = 0;
    unsigned object_count = 0;
    std::uint32_t rate = 48000;
    std::uint8_t dialnorm = 31;
    std::vector<render::MetadataFrame> timeline;
    std::vector<TimedOamdPayload> oamd_payloads;
    bool stream_oamd = false;
};

class TruehdLiveSource final {
public:
    explicit TruehdLiveSource(const Options& options, AudioInput& audio)
        : source_(std::make_shared<MappedFile>(options.truehd)), input_(source_,options.audio_track),
          limit_(options.truehd_seconds==0 ? std::numeric_limits<std::uint64_t>::max() :
              static_cast<std::uint64_t>(options.truehd_seconds*48000)) {
        if (!next()) throw std::runtime_error("no TrueHD audio decoded");
        audio.source=source_;audio.object_count=channels_;audio.dialnorm=dialnorm_;
    }
    std::size_t read(std::span<float> output) {
        const auto requested=output.size();
        if (requested%channels_) throw std::runtime_error("invalid live TrueHD PCM shape");
        while (!output.empty()) {
            if (cursor_==pcm_.interleaved.size() && !next()) break;
            const auto count=std::min(output.size(),pcm_.interleaved.size()-cursor_);
            std::copy_n(pcm_.interleaved.begin()+cursor_,count,output.begin());
            cursor_+=count;output=output.subspan(count);
        }
        return (requested-output.size())/channels_;
    }
    bool metadata(render::MetadataPacket& packet) {
        if (!metadata_.empty()) {packet=std::move(metadata_.front());metadata_.pop_front();return true;}
        if (eof_) return false;
        // A watermark prevents the renderer from reading ahead to the next OAMD
        // packet when the source has a long interval without metadata updates.
        packet={samples_,{samples_,{}}};return true;
    }
    std::uint64_t samples() const {return samples_;}
    int percent() const {return input_.percent();}
private:
    bool next() {
        if (samples_>=limit_) {eof_=true;return false;}
        auto unit=input_.next(synchronized_ ? &info_ : nullptr);
        if (!unit) {eof_=true;return false;}
        if (unit->major_sync) {
            const auto& info=*unit->major_sync;
            if (info.sample_rate!=48000 || info.substream_count<4)
                throw std::runtime_error("fast Atmos requires a 48 kHz TrueHD object presentation");
            const auto norm=info.dialogue_normalization[3].value_or(31U);
            if (!norm || norm>31) throw std::runtime_error("invalid TrueHD dialogue normalization");
            if (synchronized_ && (norm!=dialnorm_ || info.channel_based_bed_ids!=info_.channel_based_bed_ids))
                throw std::runtime_error("changing live TrueHD dialnorm or spatial bed is unsupported");
            dialnorm_=norm;info_=info;synchronized_=true;
        }
        std::string error;
        if (!decoder_.decode(input_.bytes(),pcm_,error)) throw std::runtime_error("TrueHD decode: "+error);
        if (pcm_.sample_rate!=48000 || !pcm_.channels || (channels_ && channels_!=pcm_.channels))
            throw std::runtime_error("changing live TrueHD PCM format is unsupported");
        channels_=pcm_.channels;
        const auto count=std::min<std::uint64_t>(pcm_.samples_per_channel,limit_-samples_);
        if (!count) throw std::runtime_error("empty live TrueHD PCM frame");
        bool has_oamd=false;
        for (auto& item:unit->atmos_metadata) {
            if (!item.valid) throw std::runtime_error("invalid TrueHD Atmos metadata: "+item.error);
            if (!item.has_oamd) continue;
            if (!source_oamd_ && samples_) throw std::runtime_error("initial TrueHD OAMD metadata is missing");
            if (item.oamd.object_count!=channels_) throw std::runtime_error("OAMD signal count does not match TrueHD PCM");
            source_oamd_=has_oamd=true;
            metadata_.push_back({samples_,{samples_+item.oamd_payload.sample_offset,std::move(item.oamd)}});
        }
        if (!samples_ && !has_oamd) {
            eac3::OamdFrame bed;
            bed.object_count=channels_;bed.bed_channels=static_cast<std::uint8_t>(info_.channel_based_bed_ids.size());
            bed.bed_channel_ids=info_.channel_based_bed_ids;
            if (bed.bed_channels!=channels_) throw std::runtime_error("initial TrueHD OAMD metadata is missing");
            const auto parsed=eac3::parse_oamd_payload(write_oamd_payload(bed));
            if (!parsed.valid) throw std::runtime_error("cannot describe TrueHD spatial bed");
            metadata_.push_back({0,{0,parsed.oamd}});
        }
        pcm_.interleaved.resize(static_cast<std::size_t>(count)*channels_);
        cursor_=0;samples_+=count;return true;
    }
    std::shared_ptr<const MappedFile> source_;
    TruehdInput input_;
    truehd::BedDecoder decoder_{3};
    truehd::MajorSync info_{};
    truehd::PcmFrame pcm_;
    std::deque<render::MetadataPacket> metadata_;
    std::uint64_t limit_,samples_=0;
    std::size_t cursor_=0;
    unsigned channels_=0;
    std::uint8_t dialnorm_=31;
    bool synchronized_=false,source_oamd_=false,eof_=false;
};

eac3::OamdFrame fixed_atmos_bed(std::string_view layout) {
    eac3::OamdFrame bed;
    if (layout=="5.1.2") bed.bed_channel_ids = {0,1,2,3,4,5,10,11};
    else if (layout=="5.1.4") bed.bed_channel_ids = {0,1,2,3,4,5,8,9,12,13};
    else if (layout=="7.1.4") bed.bed_channel_ids = {0,1,2,3,4,5,6,7,8,9,12,13};
    else throw std::runtime_error("--fast-atmos supports 5.1.2, 5.1.4 or 7.1.4");
    bed.object_count = bed.bed_channels = static_cast<std::uint8_t>(bed.bed_channel_ids.size());
    const auto parsed = eac3::parse_oamd_payload(write_oamd_payload(bed));
    if (!parsed.valid) throw std::runtime_error("cannot describe fixed Atmos bed");
    return parsed.oamd;
}

void reduce_atmos_scene(AudioInput& audio, std::size_t samples, std::string_view output_layout) {
    render::Layout layout;
    static_cast<void>(render::parse_layout(output_layout, layout));
    std::vector<float> bed;
    std::string error;
    if (!render::render_objects(audio.elements, audio.object_count, samples,
            audio.timeline, layout, bed, error)) throw std::runtime_error(error);
    audio.elements = std::move(bed);
    audio.object_count = static_cast<unsigned>(layout.channel_names.size());
    audio.timeline = {{0,fixed_atmos_bed(output_layout)}};
    audio.oamd_payloads.clear();
    audio.stream_oamd = false;
}

void render_spatial_audio(AudioInput& result, std::size_t produced, bool with_joc) {
    render::Layout layout;
    static_cast<void>(render::parse_layout("5.1", layout));
    std::string error;
    if (!render::render_objects(result.elements, result.object_count, produced,
            result.timeline, layout, result.bed, error, 0,
            with_joc ? &result.carrier_gains : nullptr)) throw std::runtime_error(error);
    if (with_joc) {
        render::CarrierLimiter limiter;
        constexpr auto delay = render::CarrierLimiter::delay;
        result.bed.resize(((produced + delay - 1) / delay + 1) * delay * 6, 0);
        // DEE limits quarter-scale carriers; AudioInput retains its full-scale PCM contract.
        for (auto& sample : result.bed) sample *= .25F;
        limiter.process(result.bed);
        for (auto& sample : result.bed) sample *= 4;
        result.bed.resize((produced + delay) * 6);
        // Targets and OAMD follow the limited PCM; native history is collected before the limiter.
        result.elements.insert(result.elements.begin(), delay * result.object_count, 0);
        result.render_delay += delay;
    }
}

// WAV masks use Windows speaker order; OAMD bed signals use ascending Dolby IDs.
void prepare_spatial_wav(AudioInput& audio, unsigned channels, std::uint32_t mask, bool with_joc) {
    std::vector<std::uint8_t> ids;
    if (mask) {
        constexpr int dolby_ids[] = {0,1,2,3,6,7,-1,-1,-1,4,5,-1,8,-1,9,12,-1,13};
        for (unsigned bit = 0; bit < 32; ++bit) {
            if (!(mask & (1U << bit))) continue;
            if (bit >= std::size(dolby_ids) || dolby_ids[bit] < 0)
                throw std::runtime_error("unsupported WAV speaker in channel mask");
            auto id = dolby_ids[bit];
            // A 5.1 WAV may label its sole surround pair as back speakers.
            if (channels == 6 && !(mask & 0x600U) && (id == 6 || id == 7)) id -= 2;
            ids.push_back(static_cast<std::uint8_t>(id));
        }
        if (ids.size() != channels) throw std::runtime_error("WAV channel mask does not match channel count");
    } else {
        switch (channels) {
        case 6: ids = {0,1,2,3,4,5}; break;
        case 8: ids = {0,1,2,3,4,5,6,7}; break;
        case 10: ids = {0,1,2,3,4,5,8,9,12,13}; break;
        case 12: ids = {0,1,2,3,4,5,6,7,8,9,12,13}; break;
        case 16: ids = {0,1,2,3,4,5,6,7,14,15,8,9,10,11,12,13}; break;
        default: throw std::runtime_error("WAV without a channel mask requires 6, 8, 10, 12 or 16 channels");
        }
    }
    std::vector<unsigned> order(channels);
    for (unsigned c = 0; c < channels; ++c) order[c] = c;
    std::sort(order.begin(), order.end(), [&](unsigned a, unsigned b) { return ids[a] < ids[b]; });
    audio.elements = std::move(audio.bed);
    std::vector<float> frame(channels);
    for (std::size_t offset = 0; offset < audio.elements.size(); offset += channels) {
        std::copy_n(audio.elements.begin() + offset, channels, frame.begin());
        for (unsigned c = 0; c < channels; ++c) audio.elements[offset + c] = frame[order[c]];
    }
    eac3::OamdFrame metadata;
    metadata.object_count = static_cast<std::uint16_t>(channels);
    metadata.bed_channels = static_cast<std::uint8_t>(channels);
    for (const auto c : order) metadata.bed_channel_ids.push_back(ids[c]);
    const auto parsed = eac3::parse_oamd_payload(write_oamd_payload(metadata));
    if (!parsed.valid) throw std::runtime_error("cannot describe WAV spatial bed");
    audio.object_count = channels;
    audio.timeline.push_back({0, parsed.oamd});
    render_spatial_audio(audio, audio.elements.size() / channels, with_joc);
}

AudioInput decode_truehd(const Options& options, bool buffered = true) {
    // Scan metadata first; the encoder can decode PCM again one block at a time.
    progress(options,"metadata",0);
    AudioInput result;
    result.source=std::make_shared<MappedFile>(options.truehd);
    std::optional<TruehdInput> input(std::in_place,result.source,options.audio_track);
    truehd::BedDecoder decoder(options.atmos ? 3U : 1U);
    truehd::MajorSync info{};
    std::vector<std::uint8_t> channel_based_bed;
    bool synchronized = false;
    bool source_oamd = false;
    std::size_t produced = 0;
    const auto limit = options.truehd_seconds == 0 ? std::numeric_limits<std::size_t>::max() :
        static_cast<std::size_t>(options.truehd_seconds * 48000);
    while (produced<limit) {
        const auto unit=input->next(synchronized ? &info : nullptr);
        if (!unit) break;
        const auto offset=input->position();
        if (unit->major_sync) {
            info = *unit->major_sync;
            if (options.atmos && produced && channel_based_bed != info.channel_based_bed_ids)
                throw std::runtime_error("changing TrueHD spatial bed layout is unsupported");
            channel_based_bed = info.channel_based_bed_ids;
            synchronized = true;
            if (info.sample_rate != 48000) throw std::runtime_error("TrueHD currently requires 48 kHz");
            if (options.atmos && info.substream_count < 4)
                throw std::runtime_error("TrueHD has no supported object presentation");
            const auto dialnorm = info.dialogue_normalization[options.atmos ? 3U : 1U].value_or(31U);
            if (dialnorm == 0 || dialnorm > 31)
                throw std::runtime_error("TrueHD dialogue normalization cannot be represented in DD+");
            // shortcut: fixed program dialnorm; add timed BSI/gain compensation for changing values.
            if (produced && result.dialnorm != dialnorm)
                throw std::runtime_error("changing TrueHD dialogue normalization is unsupported");
            result.dialnorm = dialnorm;
        }
        truehd::PcmFrame pcm;
        std::string error;
        if (!decoder.decode(input->bytes(), pcm, error))
            throw std::runtime_error("TrueHD decode at byte " + std::to_string(offset) + ": " + error);
        for (const auto& metadata : unit->atmos_metadata) {
            if (options.atmos && !metadata.valid)
                throw std::runtime_error("invalid TrueHD Atmos metadata: " + metadata.error);
            if (!metadata.has_oamd) continue;
            if (options.atmos && !source_oamd && produced != 0)
                throw std::runtime_error("initial TrueHD OAMD metadata is missing");
            source_oamd = true;
            if (buffered || result.timeline.empty())
                result.timeline.push_back({produced + metadata.oamd_payload.sample_offset, metadata.oamd});
            if (buffered) result.oamd_payloads.push_back({produced, metadata.oamd_payload});
        }
        const auto count = std::min<std::size_t>(pcm.samples_per_channel, limit - produced);
        if (options.atmos) {
            if (result.object_count && result.object_count != pcm.channels)
                throw std::runtime_error("changing TrueHD object count is unsupported");
            result.object_count = pcm.channels;
            if (buffered) result.elements.insert(result.elements.end(), pcm.interleaved.begin(),
                pcm.interleaved.begin() + count * pcm.channels);
        } else {
            if (pcm.channels != 6) throw std::runtime_error("TrueHD 5.1 presentation is unavailable");
            result.bed.insert(result.bed.end(), pcm.interleaved.begin(), pcm.interleaved.begin() + count * 6);
        }
        produced += count;
        progress(options,"metadata",options.truehd_seconds>0 ?
            static_cast<int>(std::min(99.0,100.0*static_cast<double>(produced)/limit)) : input->percent());
    }
    progress(options,"metadata",100);
    input.reset();
    if (buffered) result.source.reset();
    if (!produced) throw std::runtime_error("no TrueHD audio decoded");
    if (options.atmos) {
        if (!source_oamd && !channel_based_bed.empty()) {
            if (channel_based_bed.size() != result.object_count)
                throw std::runtime_error("TrueHD spatial bed does not match decoded PCM");
            eac3::OamdFrame bed;
            bed.object_count = static_cast<std::uint16_t>(result.object_count);
            bed.bed_channels = static_cast<std::uint8_t>(channel_based_bed.size());
            bed.bed_channel_ids = channel_based_bed;
            const auto parsed = eac3::parse_oamd_payload(write_oamd_payload(bed));
            if (!parsed.valid) throw std::runtime_error("cannot describe TrueHD spatial bed");
            result.timeline.push_back({0, parsed.oamd});
            result.oamd_payloads.push_back({0, parsed.oamd_payload});
        }
        if (!source_oamd && result.oamd_payloads.empty())
            throw std::runtime_error("initial TrueHD OAMD metadata is missing");
        result.stream_oamd = !buffered && source_oamd;
        if (!options.fast_atmos.empty() && buffered) reduce_atmos_scene(result, produced, options.fast_atmos);
        if (options.with_joc && options.fast_atmos.empty()) {
            progress(options,"alignment",-1);
            const unsigned delay = JocEncoder::carrier_delay + render::CarrierLimiter::delay;
            const unsigned padding = result.stream_oamd ? oamd_alignment_padding(
                [&options,&result] { return truehd_oamd_source(options,result.source); },produced+delay+1536,delay) :
                oamd_alignment_padding(result.oamd_payloads,produced+delay+1536,delay);
            if (padding) {
                if (buffered) result.elements.insert(result.elements.begin(),static_cast<std::size_t>(padding)*result.object_count,0);
                for (auto& frame : result.timeline) frame.sample += padding;
                produced += padding; result.render_delay = padding;
            }
            progress(options,"alignment",100);
        }
        if (!buffered) {
            result.sample_count = produced;
            if (options.with_joc) {
                result.sample_count += render::CarrierLimiter::delay;
                result.render_delay += render::CarrierLimiter::delay;
            }
            return result;
        }
        progress(options,"render",-1);
        render_spatial_audio(result, produced, options.with_joc);
        progress(options,"render",100);
    }
    result.sample_count = result.bed.size() / 6;
    return result;
}

class TruehdPcmReader final {
public:
    TruehdPcmReader(std::shared_ptr<const MappedFile> source, unsigned channels, unsigned padding, unsigned audio_track)
        : input_(std::move(source),audio_track),decoder_(3),channels_(channels),prefix_(static_cast<std::size_t>(padding)*channels) {}
    void read(std::span<float> output) {
        if (output.size()%channels_) throw std::runtime_error("invalid TrueHD PCM block shape");
        while (!output.empty()) {
            if (prefix_) {
                const auto count=std::min(prefix_,output.size());
                std::fill_n(output.begin(),count,0.0F);prefix_-=count;output=output.subspan(count);continue;
            }
            if (cursor_==pcm_.interleaved.size()) {
                next();cursor_=0;
                if (pcm_.channels!=channels_ || pcm_.sample_rate!=48000)
                    throw std::runtime_error("TrueHD PCM format changed after metadata scan");
                if (pcm_.interleaved.empty()) continue;
            }
            const auto count=std::min(output.size(),pcm_.interleaved.size()-cursor_);
            std::copy_n(pcm_.interleaved.begin()+cursor_,count,output.begin());
            cursor_+=count;output=output.subspan(count);
        }
    }
private:
    void next() {
        const auto unit=input_.next(synchronized_ ? &info_ : nullptr);
        if (!unit) throw std::runtime_error("TrueHD ended before the scanned PCM count");
        if (unit->major_sync) {info_=*unit->major_sync;synchronized_=true;}
        std::string error;
        if (!decoder_.decode(input_.bytes(),pcm_,error))
            throw std::runtime_error("TrueHD decode at byte "+std::to_string(input_.position())+": "+error);
    }
    TruehdInput input_;
    truehd::BedDecoder decoder_;
    truehd::MajorSync info_{};
    truehd::PcmFrame pcm_;
    unsigned channels_;
    std::size_t prefix_=0,cursor_=0;
    bool synchronized_=false;
};

void export_truehd_objects(const Options& options, const AudioInput& audio) {
    progress(options,"objects",0);
    auto source=audio.source ? audio.source : std::make_shared<MappedFile>(options.truehd);
    TruehdOamdReader metadata(options,source);
    render::MetadataPacket packet;
    bool pending=metadata.next(packet);
    const auto& layout=pending ? packet.frame.oamd : audio.timeline.front().oamd;
    ObjectExport output(options.objects_output_dir,audio.rate,layout);
    if (!pending) output.metadata(0,layout);
    TruehdPcmReader pcm(source,audio.object_count,0,options.audio_track);
    const auto samples=audio.sample_count-audio.render_delay;
    std::vector<float> block(1536*audio.object_count);
    for (std::uint64_t sample=0;sample<samples;) {
        const auto count=std::min<std::uint64_t>(1536,samples-sample);
        while (pending && packet.source_sample<sample+count) {
            output.metadata(packet.frame.sample,packet.frame.oamd);
            pending=metadata.next(packet);
        }
        auto chunk=std::span<float>(block).first(static_cast<std::size_t>(count)*audio.object_count);
        pcm.read(chunk); output.write(chunk);
        sample+=count;
        progress(options,"objects",static_cast<int>(std::min<std::uint64_t>(99,sample*100/samples)));
    }
    output.finish();
    progress(options,"objects",100);
}

void export_eac3_objects(const Options& options) {
    const MappedFile source(options.input);
    auto bytes=source.bytes();
    std::vector<std::uint8_t> demuxed;
    if (bytes.size()>=8 && std::string_view(reinterpret_cast<const char*>(bytes.data()+4),4)=="ftyp") {
        progress(options,"demux",0);
        demuxed=mp4::extract_eac3(bytes,options.audio_track);
        bytes=demuxed;
        progress(options,"demux",100);
    } else if (bytes.size()>=4 && read_u32(bytes.data(),true)==0x1a45dfa3) {
        progress(options,"demux",0);
        const auto track=selected_mkv_audio(bytes,options.audio_track);
        if (!mkv::Demuxer::is_eac3(track)) throw std::runtime_error("selected MKV audio track is not E-AC-3");
        mkv::Demuxer demuxer(bytes);
        std::string error;
        if (!demuxer.extract(track.number,demuxed,error)) throw std::runtime_error("cannot demux E-AC-3: "+error);
        bytes=demuxed;
        progress(options,"demux",100);
    }
    eac3::BedDecoder core;
    eac3::JocDecoder decoder;
    eac3::PcmFrame frame;
    std::unique_ptr<ObjectExport> output;
    std::vector<unsigned> channels;
    std::optional<unsigned> lfe;
    std::deque<float> lfe_pcm;
    std::vector<float> objects;
    eac3::JocFrame last_joc;
    unsigned rate=0, count=0, skip=1217;
    std::uint64_t samples=0, written=0;
    std::string error;
    const auto write = [&]() {
        const auto available=objects.size()/channels.size();
        const auto prefix=std::min<std::size_t>(skip,available);
        skip-=static_cast<unsigned>(prefix);
        const auto length=std::min<std::uint64_t>(available-prefix,samples-written);
        std::vector<float> pcm(length*count);
        for (std::size_t t=0;t<length;++t) {
            for (unsigned c=0;c<channels.size();++c)
                pcm[t*count+channels[c]]=objects[(t+prefix)*channels.size()+c];
            if (lfe) { pcm[t*count+*lfe]=lfe_pcm.front(); lfe_pcm.pop_front(); }
        }
        output->write(pcm);
        written+=length;
    };
    progress(options,"objects",0);
    for (std::size_t offset=0;offset<bytes.size();) {
        const auto info=eac3::parse_bitstream_info(bytes,offset);
        if (!info || info->frame_size>bytes.size()-offset)
            throw std::runtime_error("invalid or truncated E-AC-3 frame at byte "+std::to_string(offset));
        if (!core.decode(bytes.subspan(offset,info->frame_size),frame,error))
            throw std::runtime_error("E-AC-3 decode at byte "+std::to_string(offset)+": "+error);
        const auto& metadata=frame.metadata;
        if (!metadata.valid || !metadata.has_joc || !metadata.has_oamd)
            throw std::runtime_error("object export requires E-AC-3 Atmos with JOC and OAMD at byte "+std::to_string(offset));
        if (frame.channels!=6 || frame.samples_per_channel!=1536 || metadata.joc.channels!=5)
            throw std::runtime_error("object export requires independent 5.1 E-AC-3 Atmos frames with 1536 samples");
        if (!output) {
            rate=frame.sample_rate;
            count=metadata.oamd.object_count;
            for (unsigned c=0;c<count;++c) {
                if (c<metadata.oamd.bed_channel_ids.size() && metadata.oamd.bed_channel_ids[c]==3) lfe=c;
                else channels.push_back(c);
            }
            if (channels.size()!=metadata.joc.object_count)
                throw std::runtime_error("JOC object count does not match OAMD signals");
            output=std::make_unique<ObjectExport>(options.objects_output_dir,rate,metadata.oamd);
        }
        if (frame.sample_rate!=rate || metadata.joc.object_count!=channels.size())
            throw std::runtime_error("E-AC-3 Atmos format changed during object export");
        output->metadata(samples,metadata.oamd);
        const auto limit=options.truehd_seconds ? static_cast<std::uint64_t>(options.truehd_seconds*rate) :
            std::numeric_limits<std::uint64_t>::max();
        const auto length=std::min<std::uint64_t>(1536,limit-samples);
        if (lfe) for (unsigned t=0;t<length;++t) lfe_pcm.push_back(frame.interleaved[t*6+3]);
        samples+=length;
        if (!decoder.render(metadata.joc,frame.interleaved,frame.channels,1536,objects,error))
            throw std::runtime_error(error);
        write();
        last_joc=metadata.joc;
        offset+=info->frame_size;
        progress(options,"objects",static_cast<int>(offset*99/bytes.size()));
        if (samples==limit) break;
    }
    if (!output) throw std::runtime_error("E-AC-3 input is empty");
    // Drain the QMF delay so objects and the undelayed LFE share the metadata clock.
    for (unsigned i=0;i<last_joc.object_count;++i) {
        auto& object=last_joc.objects[i];
        if (object.data_points==2) {
            object.matrix[0]=object.matrix[1];
            object.sparse_channel[0]=object.sparse_channel[1];
            object.sparse_vector[0]=object.sparse_vector[1];
            object.data_points=1;
        }
    }
    if (!decoder.render(last_joc,std::vector<float>(1536*6),6,1536,objects,error))
        throw std::runtime_error(error);
    write();
    output->finish();
    progress(options,"objects",100);
}

} // namespace

int run(const Options& requested, std::string& error) {
    try {
        if(requested.codec=="truehd") return encode_truehd_file(requested,error);
        if(requested.codec!="eac3") throw std::runtime_error("--codec supports eac3 or truehd");
        if(!requested.input_layout.empty())
            throw std::runtime_error("--input-layout currently requires --codec truehd");
        auto options=requested;
        const auto fixed_bed = options.fast_atmos.empty() ? eac3::OamdFrame{} : fixed_atmos_bed(options.fast_atmos);
        bool eac3_input=false;
        if (!options.input.empty() && options.truehd.empty()) {
            const MappedFile source(options.input);
            const auto bytes=source.bytes();
            if (bytes.size()>=4 && read_u32(bytes.data(),true)==0x1a45dfa3) {
                const auto track=selected_mkv_audio(bytes,options.audio_track);
                if (mkv::Demuxer::is_truehd(track)) { options.truehd=options.input;options.input.clear(); }
                else if (mkv::Demuxer::is_eac3(track)) eac3_input=true;
                else throw std::runtime_error("unsupported selected MKV audio codec: "+track.codec_id);
            } else if (bytes.size()>=8 && read_u32(bytes.data()+4,true)==0x66747970) {
                eac3_input=true;
            } else {
                if (options.audio_track!=1) throw std::runtime_error("raw input has only one audio track");
                if (bytes.size()>=2 && bytes[0]==0x0b && bytes[1]==0x77) eac3_input=true;
                else if (!(bytes.size()>=12 && read_u32(bytes.data(),true)==0x52494646 &&
                    read_u32(bytes.data()+8,true)==0x57415645)) {
                    options.truehd=options.input;options.input.clear();
                }
            }
        }
        if (!options.fast_atmos.empty()) {
            if (options.truehd.empty() || !options.input.empty() || options.output.empty())
                throw std::runtime_error("--fast-atmos requires a TrueHD input (-i) and --output");
            if (!options.atmos || !options.with_joc)
                throw std::runtime_error("--fast-atmos requires Atmos with JOC");
        }
        if (options.output.empty() && options.objects_output_dir.empty())
            throw std::runtime_error("output path or object export directory is required");
        if (options.spdif && (options.mkv || options.mp4))
            throw std::runtime_error("--spdif cannot be combined with --mkv/--mp4");
        if (options.mkv && options.mp4)
            throw std::runtime_error("choose --mkv or --mp4, not both");
        if (!std::isfinite(options.seconds) || options.seconds <= 0 || options.seconds > 86400 ||
            !std::isfinite(options.truehd_seconds) || options.truehd_seconds < 0 || options.truehd_seconds > 86400)
            throw std::runtime_error("duration must be finite and within 0..86400 seconds");
        if (!options.input.empty() && !options.truehd.empty())
            throw std::runtime_error("choose one input file");
        if (!options.audio_track) throw std::runtime_error("audio track numbers start at 1");
        if (eac3_input && options.objects_output_dir.empty())
            throw std::runtime_error("E-AC-3 input currently requires --objects-output-dir without --output");
        for (const auto& input : {options.input, options.truehd}) {
            if (!input.empty() && std::filesystem::exists(options.output) &&
                std::filesystem::equivalent(input, options.output))
                throw std::runtime_error("input and output must be different files");
            if (!input.empty() && !options.dump_bed.empty() && std::filesystem::exists(options.dump_bed) &&
                std::filesystem::equivalent(input, options.dump_bed))
                throw std::runtime_error("input and bed dump must be different files");
        }
        if (std::filesystem::exists(options.output))
            throw std::runtime_error("output already exists; choose a new path");
        if (!options.objects_output_dir.empty()) {
            if (options.truehd.empty() && options.input.empty())
                throw std::runtime_error("--objects-output-dir requires -i with TrueHD or E-AC-3 Atmos");
            if (std::filesystem::exists(options.objects_output_dir))
                throw std::runtime_error("object export directory already exists; choose a new path");
            if (!options.input.empty()) {
                if (!eac3_input) throw std::runtime_error("object export requires TrueHD or E-AC-3 Atmos input");
                if (!options.output.empty())
                    throw std::runtime_error("E-AC-3 object export via -i cannot be combined with --output");
                export_eac3_objects(options);
                error.clear(); return 0;
            }
        }
        const bool separate_export=!options.objects_output_dir.empty() &&
            (options.output.empty() || !options.atmos || !options.fast_atmos.empty());
        if (separate_export) {
            auto source_options=options;
            source_options.fast_atmos.clear();
            source_options.atmos=true;
            source_options.with_joc=false;
            const auto source=decode_truehd(source_options,false);
            export_truehd_objects(source_options,source);
            if (options.output.empty()) { error.clear(); return 0; }
            if (std::filesystem::exists(options.output))
                throw std::runtime_error("output path conflicts with an exported object-viewer file");
        }
        model::BedLayout layout;
        if (!model::parse_bed_layout(options.bed, layout))
            throw std::runtime_error("unsupported bed layout: " + options.bed);
        const bool stream_pcm = !options.truehd.empty() && options.atmos && options.dump_bed.empty();
        AudioInput audio;
        std::unique_ptr<TruehdLiveSource> live_source;
        if (!options.truehd.empty()) {
            if (!options.objects.empty()) throw std::runtime_error("--object cannot replace TrueHD metadata");
            if (stream_pcm && !options.fast_atmos.empty()) {
                progress(options,"input",-1);
                live_source=std::make_unique<TruehdLiveSource>(options,audio);
                progress(options,"input",100);
            } else audio = decode_truehd(options, !stream_pcm);
        } else {
            progress(options,"input",-1);
            std::uint16_t channels = 6;
            std::uint32_t channel_mask = 0;
            if (options.input.empty()) synthesise_bed(audio.bed, audio.rate, channels, options.seconds);
            else if (!read_wav(options.input, audio.bed, audio.rate, channels, channel_mask, error,options.truehd_seconds)) return -1;
            if (channels != 6 || (channel_mask && channel_mask != 0x3FU && channel_mask != 0x60FU)) {
                if (options.atmos && options.with_joc && audio.rate != 48000)
                    throw std::runtime_error("recovered JOC profile requires 48 kHz");
                prepare_spatial_wav(audio, channels, channel_mask, options.atmos && options.with_joc);
            }
            if (!options.objects.empty()) throw std::runtime_error("--object requires object audio; use a TrueHD Atmos input");
            progress(options,"input",100);
        }
        if (audio.bed.empty() && !stream_pcm) throw std::runtime_error("input audio is empty");
        if (options.atmos && options.with_joc && audio.rate != 48000)
            throw std::runtime_error("recovered JOC profile requires 48 kHz");
        if (!options.objects_output_dir.empty() && !separate_export) {
            export_truehd_objects(options,audio);
            if (std::filesystem::exists(options.output))
                throw std::runtime_error("output path conflicts with an exported object-viewer file");
        }
        if (!options.dump_bed.empty()) {
            progress(options,"bed",-1);
            atmos::write_pcm24_wav(options.dump_bed, audio.rate, 6, audio.bed);
            progress(options,"bed",100);
        }
        CoreEncoder encoder;
        CoreConfig config;
        config.sample_rate = audio.rate;
        config.bitrate = options.bitrate;
        config.dialnorm = audio.dialnorm;
        config.with_additional_bsi = options.atmos && options.with_joc;
        config.complexity_index_type_a = config.with_additional_bsi ?
            static_cast<std::uint8_t>(!options.fast_atmos.empty() ? fixed_bed.object_count : audio.object_count ? audio.object_count : 6) : 0;
        encoder.configure(config);
        JocEncoder joc_encoder;
        eac3::OamdFrame metadata;
        if (!options.fast_atmos.empty()) {
            metadata = fixed_bed;
        } else if (audio.timeline.empty()) {
            metadata.object_count = 6;
            metadata.bed_channels = 1;
            metadata.bed_channel_ids = {3};
            constexpr float positions[5][2] = {{0,0},{1,0},{0.5F,0},{0,0.5F},{1,0.5F}};
            for (unsigned object = 0; object < 5; ++object) {
                eac3::OamdPositionUpdate position;
                position.object = static_cast<std::uint16_t>(object + 1);
                position.x = positions[object][0]; position.y = positions[object][1];
                metadata.position_updates.push_back(position);
            }
        } else {
            metadata.object_count = audio.timeline.front().oamd.object_count;
            metadata.bed_channels = audio.timeline.front().oamd.bed_channels;
            metadata.isf_channels = audio.timeline.front().oamd.isf_channels;
            metadata.bed_channel_ids = audio.timeline.front().oamd.bed_channel_ids;
        }
        std::vector<unsigned> target_channels;
        if (options.atmos && options.with_joc && audio.object_count) {
            if (options.fast_atmos.empty() && metadata.object_count != audio.object_count)
                throw std::runtime_error("OAMD signal count does not match TrueHD PCM");
            target_channels = joc_target_channels(metadata);
        }
        const unsigned joc_delay = options.atmos && options.with_joc ? JocEncoder::carrier_delay : 0;
        // JOC synthesis needs one more silent frame beyond the core filter flush.
        const unsigned joc_drain = joc_delay ? 1536 : 0;
        // JOC gains share the frame with audio bits, so the band profile follows
        // the bitrate: at 192 kbps a 12-band payload would be a quarter of the
        // frame. 3 bands at <=256k, 5 at <=448k, 12 above; measured +0.6 dB at
        // 192k and +0.65/+1.66 dB at 384k against the finer profiles, while at
        // 768k the 12-band profile stays ahead.
        const int joc_first_band = options.bitrate <= 256000U ? 1 :
            options.bitrate <= 448000U ? 2 : 5;
        auto samples = stream_pcm ? audio.sample_count : audio.bed.size() / 6;
        progress(options,"prepare",-1);
        std::optional<OamdReframer> reframer;
        if (audio.stream_oamd && options.fast_atmos.empty())
            reframer.emplace(truehd_oamd_source(options,audio.source),samples + joc_delay + joc_drain,joc_delay + audio.render_delay);
        else if (options.fast_atmos.empty() && options.atmos && !audio.oamd_payloads.empty())
            reframer.emplace(audio.oamd_payloads, samples + joc_delay + joc_drain, joc_delay + audio.render_delay);
        std::vector<std::uint8_t> raw_oamd;
        const auto next_oamd = [&] {
            if (reframer && !reframer->next(raw_oamd)) throw std::runtime_error("OAMD ended before output audio");
        };
        static_assert(JocEncoder::carrier_delay >= joc::CarrierGain::delay);
        // Keep total timing fixed while the normalizer supplies the last 256 samples.
        const unsigned queue_delay = joc_delay ? joc_delay - joc::CarrierGain::delay : 0;
        std::vector<float> carrier_history(queue_delay * 6);
        joc::CarrierGain carrier_gain;
        joc::CarrierCorrection carrier_correction;
        unsigned carrier_cursor = 0, joc_objects = 0;
        int frames = 0;
        eac3::JocFrame last_joc;
        const auto elementary_path = (options.mkv || options.mp4) ?
            std::filesystem::path(options.output + ".ec3.part") : std::filesystem::path(options.output);
        if ((options.mkv || options.mp4) && std::filesystem::exists(elementary_path))
            throw std::runtime_error("temporary file already exists; choose a new output path");
        std::ofstream output(elementary_path, std::ios::binary);
        if (!output) throw std::runtime_error("cannot write " + elementary_path.string());
        progress(options,"prepare",100);
        progress(options,"encode",0);
        const auto total_frames=(samples+joc_delay+joc_drain+1535)/1536+1;
        const auto write_frame = [&](std::span<const std::uint8_t> frame) {
            if (options.spdif) {
                const auto burst = iec61937_burst(frame);
                output.write(reinterpret_cast<const char*>(burst.data()),
                    static_cast<std::streamsize>(burst.size()));
            } else {
                output.write(reinterpret_cast<const char*>(frame.data()),
                    static_cast<std::streamsize>(frame.size()));
            }
            if (!output) throw std::runtime_error("failed while writing output");
        };
        try {
            const render::CarrierHistory zero_gains{};
            const auto encode_block = [&](std::vector<float> carrier, std::span<const float> targets,
                std::span<const float> gains) {
                next_oamd();
                eac3::JocFrame joc;
                if (options.atmos && options.with_joc) {
                    const unsigned object_count = audio.object_count ? static_cast<unsigned>(target_channels.size()) : 5;
                    const auto metadata_size = build_emdf(metadata, {}, true, false, raw_oamd).size();
                    joc = joc_encoder.encode(carrier, targets, object_count,
                        joc_payload_budget(metadata_size, object_count, options.bitrate), joc_first_band, gains);
                    joc_objects = object_count;
                    for (auto& sample : carrier) sample *= .25F;
                }
                for (std::size_t t = 0; t < 1536 && queue_delay; ++t) {
                    for (unsigned c = 0; c < 6; ++c)
                        std::swap(carrier[t * 6 + c], carrier_history[carrier_cursor * 6 + c]);
                    carrier_cursor = (carrier_cursor + 1) % queue_delay;
                }
                if (joc_delay) {
                    carrier_correction.process(carrier, joc_encoder.carrier_correction());
                    const unsigned gain = carrier_gain.process(carrier);
                    joc.gain = std::ldexp(1.0F + float(gain & 31) / 32, int(gain >> 5) - 4);
                    joc::compact(joc);
                    last_joc = joc;
                }
                constexpr unsigned source[] = {0,2,1,4,5,3};
                std::vector<float> coded(carrier.size());
                for (std::size_t t = 0; t < 1536; ++t)
                    for (unsigned c = 0; c < 6; ++c) coded[t * 6 + c] = carrier[t * 6 + source[c]];
                std::vector<std::uint8_t> frame;
                encode_atmos_frame(encoder, coded, metadata, joc, options.atmos,
                    options.atmos && options.with_joc, frame, raw_oamd);
                write_frame(frame);
                ++frames;
                progress(options,"encode",live_source ? live_source->percent() :
                    static_cast<int>(std::min<std::uint64_t>(99,static_cast<std::uint64_t>(frames)*100/total_frames)));
            };
            if (stream_pcm) {
                const unsigned limiter_delay = joc_delay ? render::CarrierLimiter::delay : 0;
                const auto render_samples = live_source ? 0 : samples - limiter_delay;
                std::optional<TruehdPcmReader> reader;
                if (!live_source) reader.emplace(audio.source,audio.object_count,audio.render_delay-limiter_delay,options.audio_track);
                const bool fixed_scene = !options.fast_atmos.empty();
                const unsigned fixed_channels = fixed_bed.object_count;
                render::Layout render_layout;
                static_cast<void>(render::parse_layout(fixed_scene ? options.fast_atmos : "5.1",render_layout));
                // A fixed bed has constant carrier gains: compute them once, then
                // downmix each rendered block without a second streaming renderer.
                std::vector<float> fixed_matrix;
                render::CarrierHistory fixed_history{};
                if (fixed_scene) {
                    std::vector<float> impulses(1536*fixed_channels);
                    for (unsigned c=0;c<fixed_channels;++c) impulses[c*fixed_channels+c]=1;
                    const std::array<render::MetadataFrame,1> timeline{{{0,metadata}}};
                    render::Layout carrier_layout;
                    static_cast<void>(render::parse_layout("5.1",carrier_layout));
                    std::vector<render::CarrierHistory> history;
                    if (!render::render_objects(impulses,fixed_channels,1536,timeline,carrier_layout,
                            fixed_matrix,error,0,&history)) throw std::runtime_error(error);
                    if (history.size()!=1) throw std::runtime_error("invalid fixed bed carrier history");
                    fixed_history=history.front();
                }
                render::CarrierLimiter limiter;
                const auto target_count=target_channels.size();
                std::vector<float> target_delay(limiter_delay*target_count),targets;
                unsigned target_cursor=0;
                const auto delay_targets = [&](std::span<const float> elements, std::span<float> target) {
                    for (std::size_t t=0;t<target.size()/target_count;++t) {
                        for (std::size_t c=0;c<target_count;++c) {
                            float value=elements.empty() ? 0 : elements[t*(fixed_scene ? fixed_channels : audio.object_count)+target_channels[c]];
                            std::swap(value,target_delay[target_cursor*target_count+c]);
                            target[t*target_count+c]=value;
                        }
                        target_cursor=(target_cursor+1)%limiter_delay;
                    }
                };
                std::uint64_t rendered=0;
                const auto render_input = [&](const auto& timeline) {
                    const auto read_input = [&](std::span<float> elements) {
                        reader->read(elements);
                        if (joc_delay && !fixed_scene) {
                            targets.assign((1536+limiter_delay)*target_count,0);
                            delay_targets(elements,std::span(targets).first(elements.size()/audio.object_count*target_count));
                        }
                    };
                    const auto write_output = [&](std::span<const float> block,const render::CarrierHistory* history) {
                        const auto count=block.size()/(fixed_scene ? fixed_channels : 6);
                        const bool final=live_source ? count<1536 : rendered+count==render_samples;
                        std::vector<float> limited;
                        if (fixed_scene) {
                            targets.assign((1536+limiter_delay)*target_count,0);
                            delay_targets(block,std::span(targets).first(count*target_count));
                            limited.resize(count*6);
                            for (std::size_t t=0;t<count;++t) for (unsigned c=0;c<6;++c)
                                for (unsigned element=0;element<fixed_channels;++element)
                                    limited[t*6+c]+=block[t*fixed_channels+element]*fixed_matrix[element*6+c];
                            history=count ? &fixed_history : nullptr;
                        } else limited.assign(block.begin(),block.end());
                        if (joc_delay) {
                            if (final) {
                                limited.resize(((count+limiter_delay-1)/limiter_delay+1)*limiter_delay*6,0);
                                delay_targets({},std::span(targets).subspan(count*target_count,limiter_delay*target_count));
                            }
                            for (auto& value:limited)value*=.25F;
                            limiter.process(limited);
                            for (auto& value:limited)value*=4;
                            if (final)limited.resize((count+limiter_delay)*6);
                        }
                        for (std::size_t start=0;start<limited.size()/6;start+=1536) {
                            const auto length=std::min<std::size_t>(1536,limited.size()/6-start);
                            std::vector<float> carrier(1536*6),target(1536*target_count);
                            std::copy_n(limited.begin()+start*6,length*6,carrier.begin());
                            if (joc_delay)std::copy_n(targets.begin()+start*target_count,length*target_count,target.begin());
                            encode_block(std::move(carrier),target,history && !start ?
                                std::span<const float>(*history) : joc_delay ? std::span<const float>(zero_gains) : std::span<const float>{});
                        }
                        rendered+=count;
                    };
                    if (live_source) {
                        const auto ok=render::render_objects_live(audio.object_count,
                            [&](render::MetadataPacket& packet) {return live_source->metadata(packet);},render_layout,
                            [&](std::span<float> elements) {return live_source->read(elements);},write_output,error);
                        if (ok) {
                            // At an exact frame boundary EOF is discovered on the next
                            // read; flush the limiter's final 32 samples separately.
                            if (rendered%1536==0) write_output({},nullptr);
                            samples=live_source->samples()+limiter_delay;
                        }
                        return ok;
                    }
                    return render::render_objects_stream(audio.object_count,render_samples,timeline,render_layout,
                        read_input,write_output,error,0,options.with_joc && !fixed_scene);
                };
                const render::MetadataReaderFactory source=[&options,&audio,limiter_delay] {
                    return truehd_renderer_source(options,audio.source,audio.render_delay-limiter_delay);
                };
                if (!(audio.stream_oamd ? render_input(source) : render_input(std::span<const render::MetadataFrame>(audio.timeline))))
                    throw std::runtime_error(error);
                while (static_cast<std::uint64_t>(frames)*1536<samples+joc_delay+joc_drain)
                    encode_block(std::vector<float>(1536*6),std::vector<float>(1536*target_count),
                        joc_delay ? std::span<const float>(zero_gains) : std::span<const float>{});
            } else {
                for (std::size_t offset = 0; offset < samples + joc_delay + joc_drain; offset += 1536) {
                    const auto count = offset < samples ? std::min<std::size_t>(1536, samples - offset) : 0;
                    std::vector<float> carrier(1536 * 6, 0), targets;
                    if (count) std::copy_n(audio.bed.begin() + offset * 6, count * 6, carrier.begin());
                    if (options.atmos && options.with_joc) {
                        const unsigned object_count=audio.object_count ? static_cast<unsigned>(target_channels.size()) : 5;
                        targets.resize(1536*object_count,0);
                        if (audio.object_count) {
                            for (std::size_t t=0;t<count;++t) for (unsigned object=0;object<object_count;++object)
                                targets[t*object_count+object]=audio.elements[(offset+t)*audio.object_count+target_channels[object]];
                        } else {
                            constexpr unsigned source[]{0,1,2,4,5};
                            for (std::size_t t=0;t<1536;++t) for (unsigned c=0;c<5;++c) targets[t*5+c]=carrier[t*6+source[c]];
                        }
                    }
                    const std::span<const float> gains=audio.carrier_gains.empty() ? std::span<const float>{} :
                        offset/1536<audio.carrier_gains.size() ? audio.carrier_gains[offset/1536] : zero_gains;
                    encode_block(std::move(carrier),targets,gains);
                }
            }
            // Flush the analysis filter's final 256 delayed samples.
            std::vector<float> silence(1536 * 6, 0);
            std::vector<std::uint8_t> tail;
            next_oamd();
            if (options.atmos && options.with_joc) {
                const auto metadata_size = build_emdf(metadata, {}, true, false, raw_oamd).size();
                last_joc = joc_encoder.encode(silence, std::vector<float>(1536 * joc_objects), joc_objects,
                    joc_payload_budget(metadata_size, joc_objects, options.bitrate), joc_first_band, !stream_pcm && audio.carrier_gains.empty() ?
                        std::span<const float>{} : std::span<const float>(zero_gains));
                const unsigned gain = carrier_gain.process(silence);
                last_joc.gain = std::ldexp(1.0F + float(gain & 31) / 32, int(gain >> 5) - 4);
                joc::compact(last_joc);
            }
            encode_atmos_frame(encoder, silence, metadata, last_joc, options.atmos, options.atmos && options.with_joc, tail,
                raw_oamd);
            write_frame(tail);
            output.close();
            if (!output) throw std::runtime_error("failed while closing output");
            ++frames;
            progress(options,"encode",100);
            if (options.mkv || options.mp4) {
                progress(options,"mux",-1);
                std::string mux_error;
                const bool muxed = options.mkv ?
                    write_matroska(elementary_path, options.output, mux_error) :
                    write_mp4(elementary_path, options.output, options.bitrate, mux_error);
                if (!muxed) {
                    std::filesystem::remove(elementary_path);
                    throw std::runtime_error(mux_error);
                }
                std::filesystem::remove(elementary_path);
                progress(options,"mux",100);
            }
        } catch (...) {
            output.close();
            std::filesystem::remove(elementary_path);
            std::filesystem::remove(options.output);
            throw;
        }
        error.clear();
        return frames;
    } catch (const std::exception& exception) {
        error = exception.what();
        return -1;
    }
}

} // namespace atmos::encode
