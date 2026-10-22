#pragma once

#include "eac3/emdf.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <map>
#include <memory>
#include <numbers>
#include <span>
#include <stdexcept>
#include <vector>

namespace atmos::encode {

// Reserve ds64 space so long exports can become RF64 without moving PCM.
class ObjectExportWav final {
public:
    ObjectExportWav(const std::filesystem::path& path, unsigned channels, unsigned rate)
        : channels_(channels) {
        output_.exceptions(std::ios::failbit | std::ios::badbit);
        output_.open(path, std::ios::binary);
        output_.write("RIFF",4); number(0,4); output_.write("WAVEJUNK",8); number(28,4);
        for (unsigned i=0;i<28;++i) output_.put(0);
        output_.write("fmt ",4); number(16,4); number(3,2); number(channels,2);
        number(rate,4); number(rate*channels*4,4); number(channels*4,2); number(32,2);
        output_.write("data",4); number(0,4);
    }
    void write(std::span<const float> samples) {
        if (samples.size()%channels_) throw std::runtime_error("invalid object export PCM shape");
        output_.write(reinterpret_cast<const char*>(samples.data()),samples.size_bytes());
        bytes_+=samples.size_bytes();
    }
    void finish() {
        if (bytes_>0xffffffffULL-72) {
            output_.seekp(0); output_.write("RF64",4); number(0xffffffff,4);
            output_.seekp(12); output_.write("ds64",4); number(28,4);
            number(bytes_+72,8); number(bytes_,8); number(bytes_/(channels_*4),8); number(0,4);
        } else { output_.seekp(4); number(bytes_+72,4); }
        output_.seekp(76); number(bytes_>0xffffffffULL-72 ? 0xffffffff : bytes_,4);
        output_.close();
    }
private:
    void number(std::uint64_t value, unsigned count) {
        for (unsigned i=0;i<count;++i) { output_.put(static_cast<char>(value&255)); value>>=8; }
    }
    std::ofstream output_;
    unsigned channels_;
    std::uint64_t bytes_=0;
};

class ObjectExport final {
    struct Motion {
        float from=0, to=0;
        std::uint64_t start=0,end=0;
        float at(std::uint64_t sample) const {
            if (sample>=end || end<=start) return to;
            if (sample<=start) return from;
            return from+(to-from)*static_cast<float>(sample-start)/static_cast<float>(end-start);
        }
        void set(float value, std::uint64_t sample, unsigned ramp) {
            *this={at(sample),value,sample,sample+ramp};
        }
    };
    struct State {
        Motion x{.5F,.5F}, y, z, gain{1,1};
        float distance=1;
        bool positioned=false;
    };
    struct Event {
        unsigned object;
        bool position;
        eac3::OamdPositionUpdate point;
        eac3::OamdGainUpdate gain;
    };
public:
    ObjectExport(const std::filesystem::path& directory, unsigned rate, const eac3::OamdFrame& layout)
        : rate_(rate), beds_(layout.bed_channel_ids), states_(layout.object_count) {
        validate(layout);
        if (!std::filesystem::create_directories(directory))
            throw std::runtime_error("object export directory already exists; choose a new path");
        constexpr const char* names[]{"FL","FR","FC","LFE","SL","SR","BL","BR",
            "TFL","TFR","TML","TMR","TBL","TBR","WL","WR","LFE2"};
        if (!beds_.empty()) {
            bed_=std::make_unique<ObjectExportWav>(directory/"bed.wav",static_cast<unsigned>(beds_.size()),rate);
            std::ofstream json;
            json.exceptions(std::ios::failbit | std::ios::badbit);
            json.open(directory/"bed.json");
            json << "{\"type\":\"dtsx-object-viewer-bed\",\"kind\":\"multichannel_bed\","
                "\"audioFile\":\"bed.wav\",\"sampleRate\":" << rate << ",\"channels\":[";
            for (unsigned i=0;i<beds_.size();++i) {
                if (i) json << ',';
                json << '"' << names[beds_[i]] << '"';
            }
            json << "]}\n"; json.close();
        }
        for (unsigned i=static_cast<unsigned>(beds_.size());i<states_.size();++i) {
            const auto name="object_"+std::to_string(i);
            objects_.push_back(std::make_unique<ObjectExportWav>(directory/(name+".wav"),1,rate));
            auto sidecar=std::make_unique<std::ofstream>();
            sidecar->exceptions(std::ios::failbit | std::ios::badbit);
            sidecar->open(directory/(name+".coordinates.jsonl"));
            sidecar->imbue(std::locale::classic());
            *sidecar << std::setprecision(9);
            coordinates_.push_back(std::move(sidecar));
        }
    }
    void metadata(std::uint64_t sample, const eac3::OamdFrame& frame) {
        validate(frame);
        for (const auto& p:frame.position_updates)
            events_.emplace(sample+p.sample_offset,Event{p.object,true,p,{}});
        for (const auto& g:frame.gain_updates)
            events_.emplace(sample+g.sample_offset,Event{g.object,false,{},g});
    }
    void write(std::span<const float> pcm) {
        const auto channels=states_.size();
        if (pcm.size()%channels) throw std::runtime_error("object export channel count changed");
        const auto count=pcm.size()/channels;
        std::vector<float> bed(count*beds_.size()), mono(count);
        // Sample Cartesian motion at <=128 samples and every metadata boundary.
        for (std::size_t offset=0;offset<count;) {
            const auto sample=sample_+offset;
            while (!events_.empty() && events_.begin()->first<=sample) {
                const auto [time,event]=*events_.begin(); events_.erase(events_.begin());
                if (event.object>=channels) throw std::runtime_error("invalid export object index");
                auto& state=states_[event.object];
                if (event.position) {
                    const auto& p=event.point;
                    state.x.set(p.differential ? std::clamp(state.x.at(time)+p.x,0.0F,1.0F) : p.x,time,p.ramp_duration);
                    state.y.set(p.differential ? std::clamp(state.y.at(time)+p.y,0.0F,1.0F) : p.y,time,p.ramp_duration);
                    state.z.set(p.differential ? std::clamp(state.z.at(time)+p.z,-1.0F,1.0F) : p.z,time,p.ramp_duration);
                    if (p.distance_specified) state.distance=p.distance;
                    state.positioned=true;
                } else state.gain.set(event.gain.gain,time,event.gain.ramp_duration);
            }
            auto length=std::min<std::size_t>(128,count-offset);
            if (!events_.empty()) length=std::min<std::uint64_t>(length,events_.begin()->first-sample);
            for (unsigned i=0;i<beds_.size();++i)
                for (std::size_t j=0;j<length;++j)
                    bed[(offset+j)*beds_.size()+i]=pcm[(offset+j)*channels+i]*states_[i].gain.at(sample+j);
            for (unsigned i=static_cast<unsigned>(beds_.size());i<channels;++i) {
                const auto& s=states_[i];
                float peak=0;
                for (std::size_t j=0;j<length;++j) peak=std::max(peak,std::abs(pcm[(offset+j)*channels+i]));
                const float x=2*s.x.at(sample)-1, y=1-2*s.y.at(sample), z=s.z.at(sample);
                constexpr float degrees=180/std::numbers::pi_v<float>;
                // Viewer azimuth is positive to the right, +Z/front is zero.
                *coordinates_[i-beds_.size()] << "{\"ptsSamples\":" << sample
                    << ",\"durationSamples\":" << length << ",\"sampleRate\":" << rate_
                    << ",\"objectId\":" << i << ",\"waveformId\":" << i
                    << ",\"coordinateStatus\":\"" << (s.positioned ? "decoded" : "unavailable")
                    << "\",\"azimuthDeg\":" << std::atan2(x,y)*degrees
                    << ",\"elevationDeg\":" << std::atan2(z,std::hypot(x,y))*degrees
                    << ",\"distance\":" << std::sqrt(x*x+y*y+z*z)*s.distance
                    << ",\"metadataGainLinear\":" << s.gain.at(sample)
                    << ",\"audioActive\":" << (peak>0 ? "true" : "false")
                    << ",\"interpolation\":\"step\"}\n";
            }
            offset+=length;
        }
        if (bed_) bed_->write(bed);
        for (unsigned i=static_cast<unsigned>(beds_.size());i<channels;++i) {
            for (std::size_t j=0;j<count;++j) mono[j]=pcm[j*channels+i];
            objects_[i-beds_.size()]->write(mono);
        }
        sample_+=count;
    }
    void finish() {
        if (bed_) bed_->finish();
        for (auto& wav:objects_) wav->finish();
        for (auto& sidecar:coordinates_) sidecar->close();
    }
private:
    void validate(const eac3::OamdFrame& frame) const {
        if (!frame.object_count || frame.object_count!=states_.size() || frame.isf_channels ||
            frame.bed_channel_ids!=beds_ || frame.bed_channels!=beds_.size() || beds_.size()>states_.size())
            throw std::runtime_error("object export requires a fixed bed/object layout without ISF");
        for (auto id:beds_)
            if (id>16 || id==10 || id==11)
                throw std::runtime_error("bed speaker is unsupported by object-viewer");
    }
    unsigned rate_;
    std::vector<std::uint8_t> beds_;
    std::vector<State> states_;
    std::multimap<std::uint64_t,Event> events_;
    std::unique_ptr<ObjectExportWav> bed_;
    std::vector<std::unique_ptr<ObjectExportWav>> objects_;
    std::vector<std::unique_ptr<std::ofstream>> coordinates_;
    std::uint64_t sample_=0;
};
} // namespace atmos::encode
