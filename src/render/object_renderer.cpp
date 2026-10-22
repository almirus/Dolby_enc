#include "object_renderer.hpp"
#include "carrier_panner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <tuple>

namespace atmos::render {
namespace {

struct Vector final { float x; float y; float z; };

struct Speaker final {
    const char* name;
    std::uint8_t id;
    Vector position;
    bool lfe;
};

constexpr std::array<Speaker, 17> speakers = {{
    {"L",0,{-.707F,.707F,0},false}, {"R",1,{.707F,.707F,0},false},
    {"C",2,{0,1,0},false}, {"LFE",3,{0,0,0},true},
    {"Ls",4,{-1,0,0},false}, {"Rs",5,{1,0,0},false},
    {"Lrs",6,{-.707F,-.707F,0},false}, {"Rrs",7,{.707F,-.707F,0},false},
    {"Ltf",8,{-.5F,.7F,.51F},false}, {"Rtf",9,{.5F,.7F,.51F},false},
    {"Ltm",10,{-.7F,0,.714F},false}, {"Rtm",11,{.7F,0,.714F},false},
    {"Ltr",12,{-.5F,-.7F,.51F},false}, {"Rtr",13,{.5F,-.7F,.51F},false},
    {"Lw",14,{-.94F,.342F,0},false}, {"Rw",15,{.94F,.342F,0},false},
    {"LFE2",16,{0,0,0},true}
}};

const Speaker* speaker_by_name(std::string_view name) {
    for (const auto& speaker : speakers) if (name == speaker.name) return &speaker;
    return nullptr;
}

const Speaker* speaker_by_id(unsigned id) {
    for (const auto& speaker : speakers) if (id == speaker.id) return &speaker;
    return nullptr;
}

Vector normalized(Vector value) {
    const float length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    if (length > 0.00001F) {
        value.x /= length; value.y /= length; value.z /= length;
    }
    return value;
}

Vector object_vector(float x, float y, float z) {
    return normalized({2.0F * x - 1.0F, 1.0F - 2.0F * y, z});
}

void gains_for_position(Vector position, const std::vector<const Speaker*>& output,
    std::vector<float>& gains) {
    gains.assign(output.size(), 0.0F);
    std::array<std::pair<float, std::size_t>, 3> nearest = {{
        {-1.0F,0U},{-1.0F,0U},{-1.0F,0U}
    }};
    for (std::size_t index = 0; index < output.size(); ++index) {
        if (output[index]->lfe) continue;
        const Vector speaker = normalized(output[index]->position);
        const float dot = position.x * speaker.x + position.y * speaker.y + position.z * speaker.z;
        if (dot <= nearest[2].first) continue;
        nearest[2] = {dot,index};
        std::sort(nearest.begin(), nearest.end(), [](const auto& left, const auto& right) {
            return left.first > right.first;
        });
    }
    float power = 0.0F;
    for (const auto& candidate : nearest) {
        if (candidate.first <= 0.0F) continue;
        const float gain = std::pow(candidate.first, 8.0F);
        gains[candidate.second] = gain;
        power += gain * gain;
    }
    if (power == 0.0F) {
        const std::size_t count = static_cast<std::size_t>(std::count_if(output.begin(),
            output.end(), [](const Speaker* speaker) { return !speaker->lfe; }));
        if (count == 0U) return;
        const float gain = 1.0F / std::sqrt(static_cast<float>(count));
        for (std::size_t index = 0; index < output.size(); ++index)
            if (!output[index]->lfe) gains[index] = gain;
        return;
    }
    const float scale = 1.0F / std::sqrt(power);
    for (float& gain : gains) gain *= scale;
}

void apply_spread(float spread, const std::vector<const Speaker*>& output,
    std::vector<float>& gains) {
    spread = std::clamp(spread, 0.0F, 1.0F);
    if (spread <= 0.0F) return;
    const std::size_t count = static_cast<std::size_t>(std::count_if(output.begin(),
        output.end(), [](const Speaker* speaker) { return !speaker->lfe; }));
    if (count == 0U) return;
    const float diffuse = 1.0F / std::sqrt(static_cast<float>(count));
    float power = 0.0F;
    for (std::size_t channel = 0; channel < gains.size(); ++channel) {
        if (output[channel]->lfe) continue;
        gains[channel] = gains[channel] * (1.0F - spread) + diffuse * spread;
        power += gains[channel] * gains[channel];
    }
    if (power <= 0.0F) return;
    const float scale = 1.0F / std::sqrt(power);
    for (std::size_t channel = 0; channel < gains.size(); ++channel) {
        if (!output[channel]->lfe) gains[channel] *= scale;
    }
}

struct Event final {
    std::uint64_t sample;
    eac3::OamdPositionUpdate update;
    bool position_valid = true;
    bool gain_valid = false;
    float gain = 1.0F;
    bool spread_valid = false;
    float size = -1.0F;
    float divergence = -1.0F;
    bool zone_valid = false;
    std::uint8_t zone = 0;
    bool elevation = true;
    std::shared_ptr<const eac3::OamdFrame> trim;
};

struct QueuedEvent final {
    Event event;
    std::size_t frame, ordinal;
};

struct Motion final {
    Vector start{.5F,0,0};
    Vector target{.5F,0,0};
    std::uint64_t start_sample = 0;
    std::uint64_t end_sample = 0;
};

struct ScalarMotion final {
    float start = 0.0F;
    float target = 0.0F;
    std::uint64_t start_sample = 0;
    std::uint64_t end_sample = 0;
};

Vector position_at(const Motion& motion, std::uint64_t sample) {
    if (sample >= motion.end_sample || motion.end_sample <= motion.start_sample) return motion.target;
    if (sample <= motion.start_sample) return motion.start;
    const float mix = static_cast<float>(sample - motion.start_sample) /
        static_cast<float>(motion.end_sample - motion.start_sample);
    return {motion.start.x + (motion.target.x - motion.start.x) * mix,
        motion.start.y + (motion.target.y - motion.start.y) * mix,
        motion.start.z + (motion.target.z - motion.start.z) * mix};
}

float scalar_at(const ScalarMotion& motion, std::uint64_t sample) {
    if (sample >= motion.end_sample || motion.end_sample <= motion.start_sample) {
        return motion.target;
    }
    if (sample <= motion.start_sample) return motion.start;
    const float mix = static_cast<float>(sample - motion.start_sample) /
        static_cast<float>(motion.end_sample - motion.start_sample);
    return motion.start + (motion.target - motion.start) * mix;
}

} // namespace

bool parse_layout(std::string_view name, Layout& layout) {
    struct Definition final { const char* name; std::initializer_list<const char*> channels; };
    static const std::array<Definition, 5> definitions = {{
        {"5.1", {"L","R","C","LFE","Ls","Rs"}},
        {"5.1.2", {"L","R","C","LFE","Ls","Rs","Ltm","Rtm"}},
        {"5.1.4", {"L","R","C","LFE","Ls","Rs","Ltf","Rtf","Ltr","Rtr"}},
        {"7.1.4", {"L","R","C","LFE","Ls","Rs","Lrs","Rrs","Ltf","Rtf","Ltr","Rtr"}},
        {"9.1.6", {"L","R","C","LFE","Ls","Rs","Lrs","Rrs","Lw","Rw","Ltf","Rtf","Ltm","Rtm","Ltr","Rtr"}}
    }};
    for (const auto& definition : definitions) {
        if (name != definition.name) continue;
        layout.name = definition.name;
        layout.channel_names.assign(definition.channels.begin(), definition.channels.end());
        return true;
    }
    return false;
}

std::string supported_layouts() { return "5.1, 5.1.2, 5.1.4, 7.1.4, 9.1.6"; }

bool render_objects_stream(unsigned element_count, std::uint64_t sample_count,
    std::span<const MetadataFrame> metadata, const Layout& layout,
    const std::function<void(std::span<float>)>& read,
    const std::function<void(std::span<const float>, const CarrierHistory*)>& write,
    std::string& error, unsigned first_element, bool collect_history) {
    const auto lower_bounds=std::make_shared<std::vector<std::uint64_t>>();
    if (!std::is_sorted(metadata.begin(),metadata.end(),[](const auto& a,const auto& b) {return a.sample<b.sample;})) {
        lower_bounds->resize(metadata.size());
        auto bound=std::numeric_limits<std::uint64_t>::max();
        for (std::size_t index=metadata.size();index--;) (*lower_bounds)[index]=bound=std::min(bound,metadata[index].sample);
    }
    return render_objects_stream(element_count,sample_count,MetadataReaderFactory{[metadata,lower_bounds] {
        return MetadataReader{[metadata,lower_bounds,index=std::size_t{0}](MetadataPacket& packet) mutable {
            if (index==metadata.size()) return false;
            packet={lower_bounds->empty() ? metadata[index].sample : (*lower_bounds)[index],metadata[index]};
            ++index;return true;
        }};
    }},layout,read,write,error,first_element,collect_history);
}

static bool render_objects_impl(unsigned element_count, std::uint64_t sample_count,
    const MetadataReaderFactory& metadata, const Layout& layout,
    const std::function<void(std::span<float>)>& read,
    const std::function<void(std::span<const float>, const CarrierHistory*)>& write,
    std::string& error, unsigned first_element, bool collect_history,
    const LivePcmReader& live_read) {
    if (!element_count || !read || !write) {
        error = "invalid renderer block callbacks or element count"; return false;
    }
    if (collect_history && sample_count > std::numeric_limits<std::uint64_t>::max() - 1535) {
        error = "renderer history sample count overflow"; return false;
    }
    if (!metadata) { error = "missing renderer metadata factory"; return false; }
    std::vector<const Speaker*> output_speakers;
    for (const std::string& name : layout.channel_names) {
        const Speaker* speaker = speaker_by_name(name);
        if (speaker == nullptr) { error = "invalid output speaker: " + name; return false; }
        output_speakers.push_back(speaker);
    }
    std::vector<std::uint8_t> bed_ids;
    bool carrier_layout = output_speakers.size() == 6;
    for (unsigned i = 0; i < output_speakers.size() && carrier_layout; ++i)
        carrier_layout = output_speakers[i]->id == i;
    const auto visit_events = [&](const MetadataFrame& frame, const auto& emit,
        const std::shared_ptr<const eac3::OamdFrame>& policy = {}) {
        unsigned first_update = std::numeric_limits<unsigned>::max();
        bool timing_valid = true;
        const auto first = [&](const auto& updates) {
            for (const auto& update : updates) {
                first_update = std::min(first_update, unsigned(update.sample_offset));
                if (frame.sample > std::numeric_limits<std::uint64_t>::max() -
                    update.sample_offset - update.ramp_duration) timing_valid = false;
            }
        };
        first(frame.oamd.position_updates); first(frame.oamd.gain_updates);
        first(frame.oamd.spread_updates); first(frame.oamd.zone_updates);
        if (!timing_valid) { error = "OAMD renderer timestamp overflow"; return false; }
        if (first_update != std::numeric_limits<unsigned>::max()) {
            Event trim_event{frame.sample + first_update,{}};
            trim_event.position_valid = false; trim_event.trim = policy;
            emit(trim_event);
        }
        for (const auto& update : frame.oamd.position_updates) {
            if (!std::isfinite(update.x) || !std::isfinite(update.y) || !std::isfinite(update.z)) {
                error = "non-finite object position"; return false;
            }
            emit(Event{frame.sample + update.sample_offset, update, true, false, 1.0F});
        }
        for (const auto& update : frame.oamd.gain_updates) {
            if (!std::isfinite(update.gain) || update.gain < 0 || update.gain > 5.623414F) {
                error = "invalid object gain"; return false;
            }
            eac3::OamdPositionUpdate position;
            position.object = update.object;
            position.sample_offset = update.sample_offset;
            position.block = update.block;
            position.ramp_duration = update.ramp_duration;
            emit(Event{frame.sample + update.sample_offset, position,
                false, true, update.gain});
        }
        for (const auto& update : frame.oamd.spread_updates) {
            for (const float value : {update.size, update.divergence})
                if (!std::isfinite(value) || value > 1 || (value < 0 && value != -1)) {
                    error = "invalid object spread"; return false;
                }

            eac3::OamdPositionUpdate position;
            position.object = update.object;
            position.sample_offset = update.sample_offset;
            position.ramp_duration = update.ramp_duration;
            position.block = update.block;
            emit(Event{frame.sample + update.sample_offset, position,
                false, false, 1.0F, true, update.size, update.divergence});
        }
        for (const auto& update : frame.oamd.zone_updates) {
            if (update.horizontal > 7) { error = "invalid object zone"; return false; }
            eac3::OamdPositionUpdate position;
            position.object=update.object; position.ramp_duration=update.ramp_duration;
            Event event{frame.sample+update.sample_offset,position};
            event.position_valid=false; event.zone_valid=true;
            event.zone=update.horizontal; event.elevation=update.elevation;
            emit(event);
        }
        return true;
    };
    const auto later = [](const QueuedEvent& left, const QueuedEvent& right) {
        return std::tie(left.event.sample,left.frame,left.ordinal) > std::tie(right.event.sample,right.frame,right.ordinal);
    };
    std::priority_queue<QueuedEvent,std::vector<QueuedEvent>,decltype(later)> events(later);
    std::vector<Motion> motions(element_count);
    std::vector<ScalarMotion> sizes(element_count);
    std::vector<ScalarMotion> divergences(element_count);
    std::vector<unsigned> zones(element_count,0);
    std::vector<bool> elevations(element_count,true), snaps(element_count,false);
    std::vector<float> element_gains(element_count, 1.0F);
    std::vector<carrier_panner::GainRamp> point_ramps(element_count);
    std::vector<std::array<std::array<float, 6>, 32>> point_blocks(element_count);
    std::vector<bool> point_changed(element_count, false), point_received(element_count, false);
    std::vector<bool> point_level_included(element_count, false);
    std::vector<std::uint64_t> point_end(element_count, 0);
    const auto validate_trim = [&](const eac3::OamdFrame& trim) {
        if (!trim.trim_present) return true;
        if (!carrier_layout || trim.warp_mode > 1) {
            error = "OAMD trims require canonical 5.1 and warp mode0/1"; return false;
        }
        if (trim.global_trim_mode > 2 || trim.trim_reserved > 3 ||
            (!trim.trim_bypass.empty() && trim.trim_bypass.size() != element_count)) {
            error = "invalid OAMD trim configuration or bypass count"; return false;
        }
        try {
            for (const auto& config : trim.trims) {
                carrier_panner::validate_trim(config);
                if (trim.global_trim_mode != 2 && config.mode != trim.global_trim_mode)
                    throw std::runtime_error("inconsistent OAMD global trim mode");
            }
        } catch (const std::exception& invalid) { error = invalid.what(); return false; }
        return true;
    };
    const auto valid_packet = [&](const MetadataPacket& packet, std::uint64_t previous) {
        if (packet.source_sample < previous || packet.frame.sample < packet.source_sample) {
            error = "invalid renderer metadata source timestamp";return false;
        }
        return true;
    };
    auto validator=metadata();
    if (!validator) {error="missing renderer metadata reader";return false;}
    MetadataPacket packet;
    std::uint64_t previous=0;
    bool seen=false;
    // Replay sources validate completely; live sources validate packets as they arrive.
    while (validator(packet)) {
        if (!valid_packet(packet,previous)) return false;
        if (packet.frame.oamd.isf_channels) {error="rendering OAMD ISF signals is unsupported";return false;}
        previous=packet.source_sample;seen=true;
        if (bed_ids.empty() && !packet.frame.oamd.bed_channel_ids.empty()) {
            bed_ids=packet.frame.oamd.bed_channel_ids;
            if (bed_ids.size()>element_count) {error="OAMD bed exceeds decoded elements";return false;}
        }
        if (!visit_events(packet.frame,[](const Event&) {}) || !validate_trim(packet.frame.oamd)) return false;
        if (live_read) break;
    }
    if (!seen) {error="Atmos rendering requires OAMD metadata";return false;}
    if (!live_read) validator={};
    auto metadata_reader=live_read ? std::move(validator) : metadata();
    if (!metadata_reader) {error="missing renderer metadata reader";return false;}
    bool have_packet=live_read ? seen : metadata_reader(packet);
    if (!have_packet) {error="Atmos rendering requires OAMD metadata";return false;}
    previous=0;
    std::shared_ptr<const eac3::OamdFrame> active_trim;
    const eac3::OamdTrimConfig default_trim;
    const auto bypass = [](const eac3::OamdFrame* trim, unsigned object) {
        return trim && !trim->trim_bypass.empty() && trim->trim_bypass[object];
    };
    std::vector<unsigned> history_elements;
    if (collect_history) {
        if (!carrier_layout || first_element) {
            error = "carrier history requires the complete canonical 5.1 render"; return false;
        }
        for (unsigned element = 0; element < element_count; ++element)
            if (element >= bed_ids.size() || (bed_ids[element] != 3 && bed_ids[element] != 16))
                history_elements.push_back(element);
        if (history_elements.size() > 16) {
            error = "carrier history exceeds 16 full-band signals"; return false;
        }
    }
    std::vector<std::vector<float>> gains(element_count,
        std::vector<float>(output_speakers.size(), 0.0F));
    for (unsigned element = 0; element < bed_ids.size(); ++element) {
        const Speaker* source = speaker_by_id(bed_ids[element]);
        if (source == nullptr) continue;
        bool exact = false;
        for (std::size_t channel = 0; channel < output_speakers.size(); ++channel) {
            if (output_speakers[channel]->id == source->id ||
                (source->lfe && output_speakers[channel]->lfe)) {
                gains[element][channel] = 1.0F; exact = true; break;
            }
        }
        if (!exact && !source->lfe) gains_for_position(normalized(source->position),
            output_speakers, gains[element]);
    }
    for (unsigned element = static_cast<unsigned>(bed_ids.size()); element < element_count;
        ++element) gains_for_position(object_vector(motions[element].target.x,
            motions[element].target.y, motions[element].target.z), output_speakers, gains[element]);

    std::vector<float> elements, output;
    CarrierHistory history{};
    std::size_t next_frame = 0;
    auto render_samples = collect_history ? (sample_count / 1536 + (sample_count % 1536 != 0)) * 1536 : sample_count;
    for (std::uint64_t sample = 0; sample < render_samples; ++sample) {
        if (sample % 1536 == 0) {
            auto count = static_cast<std::size_t>(std::min<std::uint64_t>(1536, sample_count - sample));
            elements.resize(count * element_count);
            if (live_read) {
                count=live_read(elements);
                if (count>1536) {error="invalid live PCM block size";return false;}
                if (!count) break;
                elements.resize(count*element_count);
                if (count<1536) sample_count=render_samples=sample+count;
            } else read(elements);
            output.assign(count * output_speakers.size(), 0);
            history = {};
        }
        while (have_packet) {
            if (!valid_packet(packet,previous)) return false;
            if (packet.source_sample > sample) break;
            const auto& frame=packet.frame;
            if (live_read && !frame.oamd.object_count) {
                previous=packet.source_sample;
                have_packet=metadata_reader(packet);continue; // Source watermark, no OAMD update.
            }
            if (frame.oamd.isf_channels) {error="rendering OAMD ISF signals is unsupported";return false;}
            if (live_read && !frame.oamd.bed_channel_ids.empty() && frame.oamd.bed_channel_ids!=bed_ids) {
                error="changing live OAMD bed layout is unsupported";return false;
            }
            if (!validate_trim(frame.oamd)) return false;
            auto policy=std::make_shared<eac3::OamdFrame>();
            policy->trim_present=frame.oamd.trim_present;
            policy->warp_mode=frame.oamd.warp_mode;
            policy->trims=frame.oamd.trims;
            policy->trim_bypass=frame.oamd.trim_bypass;
            std::size_t ordinal = 0;
            if (!visit_events(frame,[&](const Event& event) { events.push({event,next_frame,ordinal++}); },policy)) return false;
            ++next_frame;
            previous=packet.source_sample;
            have_packet=metadata_reader(packet);
        }
        while (!events.empty() && events.top().event.sample <= sample) {
            const auto event = events.top().event;
            events.pop();
            if (event.trim) {
                const auto next = event.trim->trim_present ? event.trim : std::shared_ptr<const eac3::OamdFrame>{};
                const bool policy_changed = (active_trim ? active_trim->warp_mode : 1) != (next ? next->warp_mode : 1) ||
                    (active_trim ? active_trim->trims[1] : default_trim) != (next ? next->trims[1] : default_trim);
                for (unsigned object = 0; object < element_count; ++object)
                    if (policy_changed || bypass(active_trim.get(),object) != bypass(next.get(),object)) {
                        point_changed[object] = true; point_end[object] = sample;
                    }
                active_trim = next;
                continue;
            }
            if (event.update.object >= element_count) continue;
            if (event.gain_valid) {
                element_gains[event.update.object] = event.gain;
                point_changed[event.update.object] = true;
                point_end[event.update.object] = sample + event.update.ramp_duration;
                if (event.update.object < bed_ids.size()) point_received[event.update.object] = true;
            }
            if (event.spread_valid) {
                const std::uint64_t end = sample + event.update.ramp_duration;
                if ((event.size >= 0 && sizes[event.update.object].target != event.size) ||
                    (event.divergence >= 0 && divergences[event.update.object].target != event.divergence)) {
                    point_changed[event.update.object] = true;
                    point_end[event.update.object] = end;
                }
                if (event.size >= 0.0F) {
                    const float current = scalar_at(sizes[event.update.object], sample);
                    sizes[event.update.object] = {current,event.size,sample,end};
                }
                if (event.divergence >= 0.0F) {
                    const float current = scalar_at(divergences[event.update.object], sample);
                    divergences[event.update.object] = {
                        current,event.divergence,sample,end};
                }
            }
            if (event.zone_valid) {
                constexpr unsigned native_zone[]{0,3,2,4,1,5,6,0};
                const auto zone=native_zone[event.zone];
                if (zones[event.update.object]!=zone || elevations[event.update.object]!=event.elevation) {
                    point_changed[event.update.object]=true;
                    point_end[event.update.object]=sample+event.update.ramp_duration;
                }
                zones[event.update.object]=zone; elevations[event.update.object]=event.elevation;
            }
            if (!event.position_valid || event.update.object < bed_ids.size()) continue;
            snaps[event.update.object]=event.update.snap;
            Motion& motion = motions[event.update.object];
            const Vector current = position_at(motion, sample);
            Vector target;
            if (event.update.differential) {
                target = {std::clamp(current.x + event.update.x, 0.0F, 1.0F),
                    std::clamp(current.y + event.update.y, 0.0F, 1.0F),
                    std::clamp(current.z + event.update.z, -1.0F, 1.0F)};
            } else {
                target = {event.update.x,event.update.y,event.update.z};
            }
            motion = {current,target,sample,sample + event.update.ramp_duration};
            point_end[event.update.object] = motion.end_sample;
            point_changed[event.update.object] = point_received[event.update.object] = true;
        }
        for (unsigned element = 0; element < element_count; ++element) {
            const bool bed = element < bed_ids.size();
            const float size = scalar_at(sizes[element], sample);
            const float divergence = scalar_at(divergences[element], sample);
            // Trim changes are effective at the packet's first object block; trim-only packets do not retarget.
            if (carrier_layout && (!bed || bed_ids[element] < 17)) {
                point_level_included[element] = true;
                if (sample % 32 == 0) {
                    const auto quantize = [](float x, int minimum, int maximum) {
                        return static_cast<int>(std::lrint(std::clamp(x * 32768.0F, float(minimum), float(maximum))));
                    };
                    const auto target = motions[element].target;
                    // Native gain participates in the panner arithmetic before channel rounding.
                    const float level = element_gains[element] * 4096.0F;
                    std::array<float, 6> point{};
                    if (point_received[element]) {
                        if (bed) point = carrier_panner::trim_bed_gains(bed_ids[element],
                            active_trim ? active_trim->trims[1] : default_trim,
                            !active_trim || active_trim->warp_mode != 0,level,bypass(active_trim.get(),element));
                        else if (active_trim) point = carrier_panner::trim_gains({
                            quantize(target.x,0,32767),quantize(target.y,0,32767),
                            quantize(elevations[element] ? target.z : 0.0F,-32768,32767)},
                            static_cast<int>(std::lrint(divergences[element].target * 16384.0F)),
                            active_trim->trims[1],active_trim->warp_mode != 0,level,false,bypass(active_trim.get(),element),
                            zones[element],snaps[element]);
                        else point = carrier_panner::width_gains({
                            quantize(target.x, 0, 32767), quantize(target.y, 0, 32767),
                            quantize(elevations[element] ? target.z : 0.0F, -32768, 32767)},
                            static_cast<int>(std::lrint(divergences[element].target * 16384.0F)),
                            level,true,false,true,zones[element],snaps[element]);
                    }
                    const auto end = point_end[element];
                    point_blocks[element] = point_ramps[element].block(point, point_changed[element],
                        static_cast<unsigned>((end > sample ? end - sample : 0) / 32));
                    point_changed[element] = false;
                }
                const auto& point = point_blocks[element][sample % 32];
                std::copy(point.begin(), point.end(), gains[element].begin());
            } else {
                point_level_included[element] = false;
                if (bed) continue;
                // shortcut: other layouts use the earlier approximation until their native panner is recovered.
                const Vector position = position_at(motions[element], sample);
                gains_for_position(object_vector(position.x, position.y, position.z), output_speakers, gains[element]);
                apply_spread(1.0F - (1.0F - size) * (1.0F - divergence), output_speakers, gains[element]);
            }
        }
        if (collect_history && sample % 32 == 31) {
            for (unsigned object = 0; object < history_elements.size(); ++object) {
                const unsigned element = history_elements[object];
                for (unsigned channel = 0; channel < 6; ++channel) {
                    const float level = point_level_included[element] ? point_ramps[element].current[channel] :
                        gains[element][channel] * element_gains[element];
                    const float half = 0.5F * level;
                    auto& value = history[(sample % 1536 / 64) * 128 + object * 8 + channel];
                    if (sample % 64 == 31) value = half;
                    else value += half;
                }
            }
        }
        if (sample < sample_count) {
            for (unsigned element = (std::min)(first_element, element_count);
                element < element_count; ++element) {
                const float value = elements[static_cast<std::size_t>(sample % 1536) * element_count + element];
                for (std::size_t channel = 0; channel < output_speakers.size(); ++channel) {
                    output[static_cast<std::size_t>(sample % 1536) * output_speakers.size() + channel] +=
                        value * (point_level_included[element] ? 1.0F : element_gains[element]) * gains[element][channel];
                }
            }
        }
        if (sample % 1536 == 1535 || sample + 1 == render_samples)
            write(output, collect_history ? &history : nullptr);
    }
    return true;
}

bool render_objects_stream(unsigned element_count, std::uint64_t sample_count,
    const MetadataReaderFactory& metadata, const Layout& layout,
    const std::function<void(std::span<float>)>& read,
    const std::function<void(std::span<const float>, const CarrierHistory*)>& write,
    std::string& error, unsigned first_element, bool collect_history) {
    return render_objects_impl(element_count,sample_count,metadata,layout,read,write,
        error,first_element,collect_history,{});
}

bool render_objects_live(unsigned element_count, MetadataReader metadata,
    const Layout& layout, const LivePcmReader& read,
    const std::function<void(std::span<const float>, const CarrierHistory*)>& write,
    std::string& error) {
    if (!read) {error="missing live PCM reader";return false;}
    return render_objects_impl(element_count,std::numeric_limits<std::uint64_t>::max(),
        [metadata=std::move(metadata)] {return metadata;},layout,[](std::span<float>) {},
        write,error,0,false,read);
}

bool render_objects(std::span<const float> elements, unsigned element_count,
    std::uint64_t sample_count, std::span<const MetadataFrame> metadata,
    const Layout& layout, std::vector<float>& output, std::string& error,
    unsigned first_element, std::vector<CarrierHistory>* carrier_history) {
    if (element_count == 0 || sample_count > elements.size() / element_count ||
        elements.size() != sample_count * element_count) {
        error = "invalid object element buffer"; return false;
    }
    output.clear();
    if (carrier_history) carrier_history->clear();
    std::size_t cursor = 0;
    return render_objects_stream(element_count, sample_count, metadata, layout,
        [&](std::span<float> block) {
            std::copy_n(elements.begin() + cursor, block.size(), block.begin());
            cursor += block.size();
        },
        [&](std::span<const float> block, const CarrierHistory* history) {
            if (output.empty()) output.reserve(static_cast<std::size_t>(sample_count) * layout.channel_names.size());
            output.insert(output.end(), block.begin(), block.end());
            if (history) carrier_history->push_back(*history);
        }, error, first_element, carrier_history != nullptr);
}

} // namespace atmos::render
