#include "encode/oamd_reframer.hpp"

#include "common/bit_reader.hpp"
#include "common/bit_writer.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <limits>
#include <queue>
#include <stdexcept>

namespace atmos::encode {
namespace {

class AlignmentError final : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

OamdReader span_reader(std::span<const TimedOamdPayload> input) {
    return [input,index=std::size_t{0}](TimedOamdPayload& output) mutable {
        if (index == input.size()) return false;
        output = input[index++];return true;
    };
}

void copy_bits(BitReader& reader, BitWriter& writer, std::size_t count) {
    for (std::size_t bit = 0; bit < count; ++bit) writer.write_bit(reader.read_bit());
}

void write_length(BitWriter& writer, std::size_t value) {
    std::vector<unsigned> groups;
    for (;;) {
        groups.push_back(static_cast<unsigned>(value & 15));
        value >>= 4;
        if (!value) break;
        --value;
    }
    for (std::size_t index = groups.size(); index-- > 0;) {
        writer.write(groups[index], 4);
        writer.write_bit(index != 0);
    }
}

void copy_range(const eac3::OamdElement& element, BitWriter& writer,
    std::size_t begin, std::size_t end) {
    if (begin > end || end > element.body.size() * 8)
        throw std::runtime_error("invalid OAMD object bit range");
    BitReader reader(element.body);
    reader.skip(begin);
    copy_bits(reader, writer, end - begin);
}

struct RelocatedObject {
    std::size_t frame;
    std::vector<std::uint8_t> body;
    unsigned first, last;
};

void write_basic(BitWriter& writer, const eac3::OamdBasicInfo& info) {
    if (info.gain_index > 2 || info.gain_bits > 63 || info.priority > 32)
        throw std::runtime_error("invalid resolved OAMD basic state");
    writer.write(info.gain_index, 2);
    if (info.gain_index == 2) writer.write(info.gain_bits, 6);
    writer.write_bit(info.priority == 32);
    if (info.priority != 32) writer.write(info.priority, 5);
}

void write_render(BitWriter& writer, const eac3::OamdRenderInfo& info) {
    if (!info.absolute_position || info.x > 63 || info.y > 63 || info.z > 15 ||
        info.distance > 15 || info.zone > 7 || info.size_index > 2 ||
        std::any_of(info.size.begin(), info.size.end(), [](auto value) { return value > 31; }) ||
        info.screen_factor > 7 || info.depth_factor > 3)
        throw std::runtime_error("unsupported resolved OAMD render state");
    writer.write(info.x, 6); writer.write(info.y, 6);
    writer.write_bit(info.positive_z); writer.write(info.z, 4);
    writer.write_bit(info.distance_specified);
    if (info.distance_specified) {
        writer.write_bit(info.infinity);
        if (!info.infinity) writer.write(info.distance, 4);
    }
    writer.write(info.zone, 3); writer.write_bit(info.elevation);
    writer.write(info.size_index, 2);
    const unsigned dimensions = info.size_index == 2 ? 3U : info.size_index;
    for (unsigned dimension = 0; dimension < dimensions; ++dimension) writer.write(info.size[dimension], 5);
    writer.write_bit(info.screen_reference);
    if (info.screen_reference) {
        writer.write(info.screen_factor, 3); writer.write(info.depth_factor, 2);
    }
    writer.write_bit(info.snap);
}

struct ExtensionData {
    using Range = std::pair<std::size_t, std::size_t>;
    bool has_divergence = false, has_precision = false;
    std::vector<std::vector<Range>> divergence, precision;
};

ExtensionData read_extension(const eac3::OamdElement& element,
    const eac3::OamdElement& parent, bool alternate) {
    using Range = ExtensionData::Range;
    ExtensionData data;
    auto& divergence = data.divergence; auto& precision = data.precision;
    BitReader reader(element.body);
    const unsigned prefix = alternate ? 5 : 1;
    reader.skip(prefix);
    const bool has_divergence = data.has_divergence = reader.read_bit();
    for (const auto& object : parent.object_records) {
        auto& ranges = divergence.emplace_back(object.size());
        Range previous{};
        for (std::size_t block = 0; block < object.size(); ++block) {
            const auto& record = object[block];
            Range current{};
            if (has_divergence && record.dynamic && !record.inactive) {
                const auto begin = reader.position();
                bool inherited = false;
                if (reader.read_bit()) {
                    const auto mode = reader.read(2);
                    inherited = mode == 1;
                    if (!inherited) reader.skip(mode == 0 ? 2 : 6);
                }
                current = inherited ? previous : Range{begin, reader.position()};
            }
            ranges[block] = current;
            // Inheritance refers to the immediately preceding block, including inactivity.
            previous = current;
        }
    }
    const bool has_precision = data.has_precision = reader.read_bit();
    for (const auto& object : parent.object_records) {
        auto& ranges = precision.emplace_back(object.size());
        for (std::size_t block = 0; block < object.size(); ++block) {
            if (!has_precision || !object[block].dynamic || object[block].inactive) continue;
            const auto begin = reader.position();
            if (reader.read_bit()) {
                const auto fields = reader.read(3);
                for (unsigned mask : {1U, 2U, 4U})
                    if (fields & mask) reader.skip(2);
            }
            ranges[block] = {begin, reader.position()};
        }
    }
    if (reader.remaining() > 7 || reader.read(static_cast<unsigned>(reader.remaining())) != 0)
        throw std::runtime_error("unsupported OAMD extension trailing data");
    return data;
}

std::vector<std::vector<std::uint8_t>> split_extension(const eac3::OamdElement& element,
    const eac3::OamdElement& parent, const std::vector<RelocatedObject>& fragments, bool alternate) {
    const auto data = read_extension(element,parent,alternate);
    const unsigned prefix = alternate ? 5 : 1;
    std::vector<std::vector<std::uint8_t>> result;
    for (const auto& fragment : fragments) {
        BitWriter writer;
        copy_range(element, writer, 0, prefix);
        const auto write_records = [&](const auto& ranges, bool present) {
            writer.write_bit(present);
            if (!present) return;
            for (std::size_t object = 0; object < parent.object_records.size(); ++object) {
                for (unsigned block = fragment.first; block < fragment.last; ++block) {
                    const auto& record = parent.object_records[object][block];
                    if (!record.dynamic || record.inactive) continue;
                    const auto [begin, end] = ranges[object][block];
                    if (begin == end) writer.write_bit(false);
                    else copy_range(element, writer, begin, end);
                }
            }
        };
        write_records(data.divergence, data.has_divergence);
        write_records(data.precision, data.has_precision);
        writer.align();
        result.push_back(writer.bytes());
    }
    return result;
}

struct BlockTiming {
    std::uint64_t absolute;
    std::size_t ramp_begin, ramp_end;
};

struct ObjectTiming {
    unsigned offset;
    std::vector<BlockTiming> blocks;
    std::size_t header_begin;
};

ObjectTiming read_object_timing(
    const eac3::OamdElement& element, bool alternate, std::uint64_t origin) {
    BitReader reader(element.body);
    const unsigned prefix = alternate ? 5 : 1;
    reader.skip(prefix);
    const auto mode = reader.read(2);
    unsigned offset = 0;
    if (mode == 1) {
        constexpr unsigned offsets[] = {8,16,18,24};
        offset = offsets[reader.read(2)];
    } else if (mode == 2) offset = reader.read(5);
    else if (mode == 3) throw std::runtime_error("unsupported OAMD offset mode");
    const auto blocks = reader.read(3) + 1;
    std::vector<BlockTiming> timing;
    for (unsigned block = 0; block < blocks; ++block) {
        const auto absolute = origin + offset + 32U * reader.read(6);
        const auto ramp_begin = reader.position();
        if (reader.read(2) == 3) {
            const bool extended = reader.read_bit();
            reader.skip(extended ? 4 : 11);
        }
        timing.push_back({absolute, ramp_begin, reader.position()});
    }
    return {offset,std::move(timing),reader.position()};
}

// Necessary timing conditions only; every surviving candidate still needs full reframing.
std::bitset<1536> alignment_candidates(OamdReader read, unsigned delay) {
    if (!read) throw std::runtime_error("missing OAMD input callback");
    struct State {
        std::uint64_t frame=std::numeric_limits<std::uint64_t>::max();
        unsigned residue=0, blocks=0;
    };
    std::array<State,1536> states{};
    std::bitset<1536> possible;possible.set();possible.reset(0);
    std::priority_queue<std::uint64_t,std::vector<std::uint64_t>,std::greater<>> pending;
    const auto drain=[&](std::uint64_t bound,bool finish=false) {
        while (!pending.empty() && (finish || pending.top()<bound)) {
            const auto time=pending.top();pending.pop();
            for (unsigned padding=1;padding<1536;++padding) if (possible[padding]) {
                // Overflow is left to the full validator rather than used to prune.
                if (time>std::numeric_limits<std::uint64_t>::max()-padding) continue;
                const auto shifted=time+padding;
                const auto frame=shifted/1536;
                const auto residue=static_cast<unsigned>(shifted%32);
                auto& state=states[padding];
                if (frame!=state.frame) state={frame,residue,1};
                else if (residue!=state.residue || state.blocks==8) possible.reset(padding);
                else ++state.blocks;
            }
            if (possible.none()) return;
        }
    };
    TimedOamdPayload packet;
    std::uint64_t previous=0;bool seen=false;
    while (read(packet)) {
        if (!seen && packet.sample!=0) throw std::runtime_error("initial OAMD payload is missing");
        seen=true;
        if (packet.sample<previous || packet.sample>std::numeric_limits<std::uint64_t>::max()-
            packet.payload.sample_offset-delay-2047) throw std::runtime_error("invalid OAMD timestamp");
        previous=packet.sample;
        // Future packet origins and all inner offsets are nonnegative.
        drain(packet.sample+delay);
        if (possible.none()) return possible;
        for (const auto& element:packet.payload.elements) if (element.id==1) {
            const auto timing=read_object_timing(element,packet.payload.alternate_object,
                packet.sample+packet.payload.sample_offset+delay);
            for (const auto& block:timing.blocks) pending.push(block.absolute);
        }
    }
    if (!seen) throw std::runtime_error("initial OAMD payload is missing");
    drain(0,true);
    return possible;
}

std::vector<RelocatedObject> retime_object(
    const eac3::OamdElement& element, bool alternate, std::uint64_t origin) {
    const auto parsed=read_object_timing(element,alternate,origin);
    const auto& timing=parsed.blocks;
    const auto offset=parsed.offset;
    const auto header_begin=parsed.header_begin;
    const unsigned prefix=alternate ? 5 : 1;
    const auto blocks=static_cast<unsigned>(timing.size());
    const bool split = std::any_of(timing.begin(), timing.end(), [&](const BlockTiming& block) {
        return block.absolute / 1536 != timing.front().absolute / 1536;
    });
    if (split) {
        if (!std::is_sorted(timing.begin(), timing.end(), [](const BlockTiming& a, const BlockTiming& b) {
                return a.absolute < b.absolute;
            }) || element.object_records.empty() || element.object_header_begin != header_begin ||
            element.object_data_begin < header_begin ||
            element.object_data_end > element.body.size() * 8 ||
            element.body.size() * 8 - element.object_data_end > 7)
            throw std::runtime_error("unsupported OAMD object layout for splitting");
        auto end = element.object_data_begin;
        for (const auto& object : element.object_records) {
            if (object.size() != blocks) throw std::runtime_error("invalid OAMD object block count");
            for (const auto& record : object) {
                if (record.begin != end || record.render_begin <= record.begin ||
                    record.render_begin > record.render_end || record.render_end > record.end ||
                    record.end > element.object_data_end)
                    throw std::runtime_error("invalid OAMD object record range");
                end = record.end;
            }
        }
        if (end != element.object_data_end) throw std::runtime_error("invalid OAMD object data length");
    }
    std::vector<RelocatedObject> result;
    for (unsigned first = 0; first < blocks;) {
        unsigned last = first + 1;
        const auto frame = timing[first].absolute / 1536;
        while (last < blocks && timing[last].absolute / 1536 == frame) ++last;
        BitWriter writer;
        copy_range(element, writer, 0, prefix);
        const auto residue = static_cast<unsigned>((origin + offset) % 32);
        writer.write(residue ? 2U : 0U, 2);
        if (residue) writer.write(residue, 5);
        writer.write(last - first - 1, 3);
        for (unsigned block = first; block < last; ++block) {
            writer.write(static_cast<unsigned>((timing[block].absolute % 1536) / 32), 6);
            copy_range(element, writer, timing[block].ramp_begin, timing[block].ramp_end);
        }
        if (!split) {
            copy_range(element, writer, header_begin, element.body.size() * 8);
        } else {
            copy_range(element, writer, header_begin, element.object_data_begin);
            for (const auto& object : element.object_records) {
                for (unsigned block = first; block < last; ++block) {
                    const auto& record = object[block];
                    if (block != first || first == 0 || record.inactive) {
                        copy_range(element, writer, record.begin, record.end);
                        continue;
                    }
                    if (record.dynamic && !record.render.absolute_position)
                        throw std::runtime_error("cross-frame OAMD needs a full absolute update at the split");
                    writer.write_bit(false);
                    if (record.full_basic) copy_range(element, writer, record.begin + 3, record.render_begin);
                    else {
                        write_basic(writer, record.basic);
                    }
                    if (record.dynamic && (!record.full_render || record.differential)) {
                        write_render(writer, record.render);
                        copy_range(element, writer, record.render_end, record.end);
                    } else {
                        // First blocks imply full basic/render fields and absolute positions.
                        copy_range(element, writer, record.render_begin + (record.dynamic ? 3 : 0), record.end);
                    }
                }
            }
            copy_range(element, writer, element.object_data_end, element.body.size() * 8);
        }
        writer.align();
        result.push_back({static_cast<std::size_t>(frame), writer.bytes(), first, last});
        first = last;
    }
    return result;
}

std::vector<std::uint8_t> serialize_payload(const eac3::OamdPayload& header,
    const std::vector<eac3::OamdElement>& elements) {
    if (elements.size() > 46) throw std::runtime_error("too many OAMD elements in DD+ frame");
    BitWriter writer; BitReader reader(header.header);
    copy_bits(reader,writer,header.header_bits);
    writer.write(static_cast<unsigned>(std::min<std::size_t>(elements.size(),15)),4);
    if (elements.size() >= 15) writer.write(static_cast<unsigned>(elements.size()-15),5);
    for (const auto& element : elements) {
        if (element.body.empty()) throw std::runtime_error("empty OAMD element");
        writer.write(element.id,4); write_length(writer,element.body.size()-1);
        for (auto byte : element.body) writer.write(byte,8);
    }
    writer.align(); return writer.bytes();
}

void coalesce_objects(const eac3::OamdPayload& header, std::vector<eac3::OamdElement>& elements) {
    if (std::count_if(elements.begin(),elements.end(),[](const auto& e){return e.id == 1;}) <= 1) return;
    const auto parsed = eac3::parse_oamd_payload(serialize_payload(header,elements));
    if (!parsed.valid) throw std::runtime_error("invalid OAMD fragments before merging");
    struct Block { unsigned time; std::size_t ramp_begin, ramp_end; };
    struct Part {
        const eac3::OamdElement* object;
        const eac3::OamdElement* extension = nullptr;
        std::vector<Block> blocks;
        ExtensionData extra;
    };
    std::vector<Part> parts;
    for (const auto& element : parsed.oamd_payload.elements) {
        if (element.id == 1) parts.push_back({&element});
        else if (element.id == 5) {
            if (parts.empty() || parts.back().extension) throw std::runtime_error("unsupported duplicate OAMD extension");
            parts.back().extension = &element;
        } else if (element.id != 2) throw std::runtime_error("unsupported OAMD merge element");
    }
    const unsigned prefix = header.alternate_object ? 5 : 1;
    unsigned count = 0, residue = 0;
    bool have_time = false, divergence = false, precision = false, have_extension = false;
    const auto range = [](const eac3::OamdElement& e,std::size_t first,std::size_t last) {
        BitWriter bits; copy_range(e,bits,first,last); return std::pair{last-first,bits.bytes()};
    };
    const auto& first = *parts.front().object;
    const eac3::OamdElement* first_extension = nullptr;
    for (auto& part : parts) {
        const auto& object = *part.object;
        if (range(object,0,prefix) != range(first,0,prefix) ||
            range(object,object.object_header_begin,object.object_data_begin) !=
                range(first,first.object_header_begin,first.object_data_begin) ||
            object.object_records.size() != first.object_records.size())
            throw std::runtime_error("incompatible OAMD object headers during merging");
        BitReader reader(object.body); reader.skip(prefix);
        const auto mode = reader.read(2); unsigned offset = 0;
        if (mode == 1) { constexpr unsigned offsets[] = {8,16,18,24}; offset = offsets[reader.read(2)]; }
        else if (mode == 2) offset = reader.read(5);
        else if (mode == 3) throw std::runtime_error("unsupported OAMD merge offset");
        const unsigned blocks = reader.read(3)+1;
        for (unsigned block = 0; block < blocks; ++block) {
            const unsigned time = offset+32*reader.read(6);
            const auto begin = reader.position();
            if (reader.read(2) == 3) { const bool extended = reader.read_bit(); reader.skip(extended ? 4 : 11); }
            if (time >= 1536 || (!part.blocks.empty() && time < part.blocks.back().time))
                throw std::runtime_error("unsorted OAMD blocks inside a source fragment");
            if (have_time && time%32 != residue)
                throw AlignmentError("OAMD updates require different sub-32-sample offsets in one DD+ frame");
            residue = time%32; have_time = true;
            part.blocks.push_back({time,begin,reader.position()});
        }
        if (reader.position() != object.object_header_begin ||
            std::any_of(object.object_records.begin(),object.object_records.end(),
                [&](const auto& records){return records.size() != blocks;}))
            throw std::runtime_error("invalid OAMD block layout during merging");
        count += blocks;
        if (count > 8) throw AlignmentError("more than eight OAMD updates in one DD+ frame");
        if (part.extension) {
            if (have_extension && range(*part.extension,0,prefix) != range(*first_extension,0,prefix))
                throw std::runtime_error("incompatible OAMD extension headers during merging");
            part.extra = read_extension(*part.extension,object,header.alternate_object);
            divergence = divergence || part.extra.has_divergence;
            precision = precision || part.extra.has_precision;
            have_extension = true; first_extension = part.extension;
        }
    }
    struct ScheduledBlock { const Part* part; unsigned block; bool interrupted = false; };
    std::vector<ScheduledBlock> schedule;
    for (const auto& part : parts) for (unsigned block = 0; block < part.blocks.size(); ++block)
        schedule.push_back({&part,block});
    const auto earlier = [](const auto& a,const auto& b) {
        return a.part->blocks[a.block].time < b.part->blocks[b.block].time;
    };
    if (!std::is_sorted(schedule.begin(),schedule.end(),earlier)) {
        std::stable_sort(schedule.begin(),schedule.end(),earlier);
        for (std::size_t i = 0; i < schedule.size(); ++i) {
            auto& entry = schedule[i];
            if (i && entry.block && schedule[i-1].part == entry.part && schedule[i-1].block+1 == entry.block) continue;
            entry.interrupted = entry.block != 0;
        }
    }
    BitWriter object_writer; copy_range(first,object_writer,0,prefix);
    object_writer.write(residue ? 2 : 0,2); if (residue) object_writer.write(residue,5);
    object_writer.write(count-1,3);
    for (const auto& entry : schedule) {
        const auto& block = entry.part->blocks[entry.block];
        object_writer.write(block.time/32,6);
        copy_range(*entry.part->object,object_writer,block.ramp_begin,block.ramp_end);
    }
    copy_range(first,object_writer,first.object_header_begin,first.object_data_begin);
    for (unsigned object = 0; object < first.object_records.size(); ++object) {
        bool initial = true;
        for (const auto& entry : schedule) {
            const auto& source = *entry.part->object; const auto block = entry.block;
            const auto& record = source.object_records[object][block];
            if (entry.interrupted && !record.inactive &&
                (!record.full_basic || (record.dynamic && (!record.full_render || record.differential)))) {
                if (record.full_basic) copy_range(source,object_writer,record.begin,record.render_begin);
                else {
                    object_writer.write_bit(false); object_writer.write(1,2);
                    write_basic(object_writer,record.basic);
                }
                if (record.dynamic) {
                    if (record.full_render && !record.differential)
                        copy_range(source,object_writer,record.render_begin,record.render_end);
                    else {
                        object_writer.write(1,2); object_writer.write_bit(false);
                        write_render(object_writer,record.render);
                    }
                }
                copy_range(source,object_writer,record.render_end,record.end);
            } else if (initial || block != 0 || record.inactive) copy_range(source,object_writer,record.begin,record.end);
            else {
                // A later fragment's first block gains explicit full/basic/render mode bits.
                object_writer.write_bit(false); object_writer.write(1,2);
                copy_range(source,object_writer,record.begin+1,record.render_begin);
                if (record.dynamic) { object_writer.write(1,2); object_writer.write_bit(false); }
                copy_range(source,object_writer,record.render_begin,record.end);
            }
            initial = false;
        }
    }
    object_writer.align();
    std::vector<eac3::OamdElement> merged{{1,object_writer.bytes()}};
    if (have_extension) {
        const auto extension = std::find_if(parts.begin(),parts.end(),[](const auto& part){return part.extension != nullptr;});
        BitWriter writer; copy_range(*extension->extension,writer,0,prefix);
        const auto fields = [&](bool present,bool div) {
            writer.write_bit(present); if (!present) return;
            for (unsigned object = 0; object < first.object_records.size(); ++object)
                for (const auto& entry : schedule) {
                    const auto& part = *entry.part; const auto block = entry.block;
                    const auto& record = part.object->object_records[object][block];
                    if (!record.dynamic || record.inactive) continue;
                    if (!part.extension || !(div ? part.extra.has_divergence : part.extra.has_precision)) writer.write_bit(false);
                    else {
                        const auto [begin,end] = (div ? part.extra.divergence : part.extra.precision)[object][block];
                        if (begin == end) writer.write_bit(false);
                        else copy_range(*part.extension,writer,begin,end);
                    }
                }
        };
        fields(divergence,true); fields(precision,false); writer.align();
        merged.push_back({5,writer.bytes()});
    }
    for (const auto& element : elements) if (element.id == 2) merged.push_back(element);
    elements = std::move(merged);
}

} // namespace

OamdReframer::OamdReframer(std::span<const TimedOamdPayload> input, std::size_t samples, unsigned delay)
    : OamdReframer(span_reader(input),samples,delay) {}

OamdReframer::OamdReframer(OamdReader read, std::size_t samples, unsigned delay)
    : read_(std::move(read)),frames_(samples/1536+(samples%1536!=0)+1),delay_(delay) {
    if (!read_) throw std::runtime_error("missing OAMD input callback");
    have_packet_ = read_(packet_);
    if (!have_packet_ || !samples || packet_.sample != 0)
        throw std::runtime_error("initial OAMD payload is missing");
    header_.header = packet_.payload.header;
    header_.header_bits = packet_.payload.header_bits;
    header_.alternate_object = packet_.payload.alternate_object;
    if (!header_.header_bits || header_.header_bits > header_.header.size() * 8)
        throw std::runtime_error("invalid OAMD header");
}

bool OamdReframer::next(std::vector<std::uint8_t>& output) {
    output.clear();
    if (frame_ == frames_) return false;
    const auto& header = header_;
    while (have_packet_) {
        const auto& packet = packet_;
        if (packet.sample < previous_ || packet.sample >
            std::numeric_limits<std::uint64_t>::max() - packet.payload.sample_offset - delay_ - 2047)
            throw std::runtime_error("invalid OAMD timestamp");
        // Future packets cannot contribute to this frame: all inner offsets are nonnegative.
        if (frame_+1 < frames_ && (packet.sample+delay_)/1536 > frame_) break;
        previous_ = packet.sample;
        if (packet.payload.header_bits != header.header_bits || packet.payload.header != header.header ||
            packet.payload.alternate_object != header.alternate_object)
            throw std::runtime_error("changing OAMD configuration is unsupported");
        const eac3::OamdElement* trim = nullptr;
        bool has_objects = false, different_trims = false;
        for (const auto& element : packet.payload.elements) {
            has_objects = has_objects || element.id == 1;
            if (element.id == 2) {
                different_trims = different_trims || (trim && trim->body != element.body);
                trim = &element;
            }
        }
        if (has_objects && different_trims) throw std::runtime_error("conflicting OAMD trims in one payload");
        if (has_objects && trim && (last_validated_trim_.empty() || trim->body != last_validated_trim_)) {
            if (trim->body.empty()) throw std::runtime_error("empty OAMD trim element");
            BitWriter probe; BitReader source(header.header); copy_bits(source,probe,header.header_bits);
            probe.write(1,4); probe.write(2,4); write_length(probe,trim->body.size()-1);
            for (auto byte : trim->body) probe.write(byte,8);
            probe.align();
            if (!eac3::parse_oamd_payload(probe.bytes()).valid) throw std::runtime_error("invalid OAMD trim element");
            last_validated_trim_ = trim->body;
        }
        const eac3::OamdElement* parent = nullptr;
        std::vector<RelocatedObject> fragments;
        for (const auto& element : packet.payload.elements) {
            if (element.id == 2) continue;
            if (element.id == 1) {
                parent = &element;
                fragments = retime_object(element, header.alternate_object,
                    packet.sample + packet.payload.sample_offset + delay_);
                for (auto& fragment : fragments) {
                    if (fragment.frame >= frames_) throw std::runtime_error("OAMD update exceeds output duration");
                    auto& target = pending_[fragment.frame];
                    const auto* previous_trim = target.trim ? &*target.trim : nullptr;
                    if (target.profile_seen && (bool(trim) != bool(previous_trim) ||
                        (trim && trim->body != previous_trim->body)))
                        throw AlignmentError("conflicting OAMD trims in one DD+ frame");
                    if (trim) target.trim = *trim;
                    else target.trim.reset();
                    target.profile_seen = true;
                    target.elements.push_back({element.id, std::move(fragment.body)});
                }
            } else {
                if (element.id != 5 || !parent)
                    throw std::runtime_error("unsupported OAMD extension during reframing");
                if (fragments.size() == 1) pending_[fragments.front().frame].elements.push_back(element);
                else {
                    const auto bodies = split_extension(element, *parent, fragments, header.alternate_object);
                    for (std::size_t index = 0; index < fragments.size(); ++index)
                        pending_[fragments[index].frame].elements.push_back({element.id, bodies[index]});
                }
            }
        }
        have_packet_ = read_(packet_);
    }
    // A payload has one global trim profile. Trim-only packets have no object fragments and do not select it.
    auto found = pending_.find(frame_);
    std::vector<eac3::OamdElement> empty;
    auto& elements = found == pending_.end() ? empty : found->second.elements;
    if (found != pending_.end() && found->second.trim) elements.push_back(*found->second.trim);
    coalesce_objects(header,elements);
    output = serialize_payload(header,elements);
    if (found != pending_.end()) pending_.erase(found);
    ++frame_;
    return true;
}

std::vector<std::vector<std::uint8_t>> reframe_oamd(
    std::span<const TimedOamdPayload> input, std::size_t samples, unsigned delay) {
    OamdReframer reader(input,samples,delay);
    std::vector<std::vector<std::uint8_t>> result;
    std::vector<std::uint8_t> frame;
    while (reader.next(frame)) {
        result.push_back(std::move(frame));
    }
    return result;
}

unsigned oamd_alignment_padding(std::span<const TimedOamdPayload> input,
    std::size_t samples, unsigned delay) {
    return oamd_alignment_padding([input] { return span_reader(input); },samples,delay);
}

unsigned oamd_alignment_padding(const OamdReaderFactory& input,
    std::size_t samples, unsigned delay) {
    if (!input) throw std::runtime_error("missing OAMD source factory");
    if (samples > std::numeric_limits<std::size_t>::max()-1535 ||
        delay > std::numeric_limits<unsigned>::max()-1535)
        throw std::runtime_error("OAMD alignment duration overflows");
    std::string reason;
    std::bitset<1536> possible;possible.set();
    for (unsigned padding = 0; padding < 1536; ++padding) {
        if (!possible[padding]) continue;
        try {
            OamdReframer reader(input(),samples+padding,delay+padding);
            std::vector<std::uint8_t> frame;
            while (reader.next(frame)) {}
            return padding;
        } catch (const AlignmentError& error) {
            if (reason.empty()) reason = error.what();
            if (padding==0) possible=alignment_candidates(input(),delay);
        }
    }
    throw std::runtime_error("no exact OAMD frame alignment within one frame: "+reason);
}

} // namespace atmos::encode
