#include "access_unit.hpp"

#include "common/bit_reader.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace atmos::truehd {
namespace {

std::uint32_t variable_bits(BitReader& bits, unsigned width, unsigned maximum_groups) {
    std::uint32_t value = 0;
    for (unsigned group = 0; group < maximum_groups; ++group) {
        const auto chunk = bits.read(width);
        if (chunk > std::numeric_limits<std::uint32_t>::max() - value)
            throw ParseError("TrueHD variable-bit value overflows");
        value += chunk;
        if (!bits.read_bit()) return value;
        if (value >= (std::numeric_limits<std::uint32_t>::max() >> width))
            throw ParseError("TrueHD variable-bit value overflows");
        value = (value + 1U) << width;
    }
    throw ParseError("unterminated TrueHD variable-bit value");
}

} // namespace

std::vector<eac3::EmdfMetadata> parse_evolution_frame(
    std::span<const std::uint8_t> data) try {
    BitReader bits(data);
    unsigned version = bits.read(2U);
    if (version == 3U) version += variable_bits(bits, 2U, 16U);
    unsigned key = bits.read(3U);
    if (key == 7U) key += variable_bits(bits, 3U, 10U);
    if (version != 0U || key != 0U) throw ParseError("unsupported TrueHD evolution version/key");
    std::vector<eac3::EmdfMetadata> result;
    for (;;) {
        const unsigned payload_id = bits.read(5U);
        if (payload_id == 0U) break;
        const bool sample_offset = bits.read_bit();
        const auto offset = sample_offset ? variable_bits(bits, 11U, 2U) : 0U;
        if (bits.read_bit()) static_cast<void>(variable_bits(bits, 11U, 2U));
        if (bits.read_bit()) static_cast<void>(variable_bits(bits, 2U, 16U));
        const unsigned presentations = bits.read_bit() ? bits.read(8U) : 0xffU;
        if (!bits.read_bit()) {
            const bool aligned = !sample_offset && bits.read_bit();
            if (aligned) bits.skip(2U);
            if (sample_offset || aligned) bits.skip(7U);
        }
        const std::size_t payload_size = variable_bits(bits, 8U, 4U);
        if (bits.remaining() < payload_size * 8U) throw ParseError("truncated TrueHD evolution payload");
        // LG dthd_evo.c selects presentation 3 with mask 8 (table 1,2,4,8).
        if (payload_id == 11U && (presentations & 8U) != 0U) {
            std::vector<std::uint8_t> payload(payload_size);
            for (auto& value : payload) value = static_cast<std::uint8_t>(bits.read(8U));
            auto metadata = eac3::parse_oamd_payload(payload);
            if (!metadata.valid) throw ParseError(metadata.error);
            metadata.oamd_payload.sample_offset = offset;
            result.push_back(std::move(metadata));
        } else {
            bits.skip(payload_size * 8U);
        }
    }
    constexpr unsigned protection_bytes[] = {0, 1, 4, 16};
    const auto primary = protection_bytes[bits.read(2U)];
    const auto secondary = protection_bytes[bits.read(2U)];
    bits.skip((primary + secondary) * 8U);
    while (bits.remaining())
        if (bits.read(static_cast<unsigned>(std::min<std::size_t>(bits.remaining(), 32U))) != 0U)
            throw ParseError("nonzero TrueHD evolution padding");
    return result;
} catch (const ParseError& error) {
    eac3::EmdfMetadata result;
    result.error = error.what();
    return {std::move(result)};
}

std::optional<AccessUnit> parse_access_unit(
    std::span<const std::uint8_t> data, std::size_t offset,
    const MajorSync* stream_info) {
    if (offset + 8U > data.size()) {
        return std::nullopt;
    }
    const std::uint16_t words = static_cast<std::uint16_t>(
        ((data[offset] & 0x0FU) << 8U) | data[offset + 1U]);
    const std::size_t bytes = static_cast<std::size_t>(words) * 2U;
    if (bytes < 8U || offset + bytes > data.size()) {
        return std::nullopt;
    }
    AccessUnit result;
    result.offset = offset;
    result.size = bytes;
    result.input_timing = static_cast<std::uint16_t>(
        (data[offset + 2U] << 8U) | data[offset + 3U]);
    result.major_sync = parse_major_sync(data, offset + 4U);
    result.has_major_sync = result.major_sync.has_value();
    const MajorSync* parameters = result.major_sync ? &*result.major_sync : stream_info;
    if (parameters == nullptr || parameters->substream_count == 0U) return result;

    const std::size_t directory_offset = offset + 4U +
        (result.major_sync ? result.major_sync->header_size : 0U);
    if (directory_offset > offset + bytes) return std::nullopt;
    try {
        BitReader bits(data.subspan(directory_offset, offset + bytes - directory_offset));
        std::size_t previous_end = 0;
        result.substreams.reserve(parameters->substream_count);
        for (unsigned index = 0; index < parameters->substream_count; ++index) {
            SubstreamEntry entry;
            entry.extra_word_present = bits.read_bit();
            entry.nonrestart = bits.read_bit();
            entry.check_data_present = bits.read_bit();
            bits.skip(1U);
            const std::size_t end = static_cast<std::size_t>(bits.read(12U)) * 2U;
            if (entry.extra_word_present) {
                entry.extra_word = static_cast<std::uint16_t>(bits.read(16U));
            }
            if (end < previous_end) return std::nullopt;
            entry.data_size = end - previous_end;
            previous_end = end;
            result.substreams.push_back(entry);
        }
        const std::size_t directory_size = bits.position() / 8U;
        const std::size_t payload_offset = directory_offset + directory_size;
        if (payload_offset + previous_end > offset + bytes) return std::nullopt;
        std::size_t relative = 0;
        for (auto& entry : result.substreams) {
            entry.data_offset = payload_offset + relative;
            if (!entry.nonrestart && entry.data_size != 0U) {
                entry.restart_header = parse_restart_header(
                    data.subspan(entry.data_offset, entry.data_size));
            }
            relative += entry.data_size;
        }
        const std::size_t payload_end = payload_offset + previous_end;
        result.extra_data_offset = payload_end;
        result.extra_data_size = offset + bytes - payload_end;
        if ((parameters->flags & 0x1000U) != 0U && result.extra_data_size != 0U) {
            const auto block = data.subspan(payload_end, result.extra_data_size);
            BitReader extra(block);
            const unsigned check = extra.read(4U);
            const std::size_t extra_words = extra.read(12U);
            if (check != 0U || extra_words != 0U) {
                if (((check ^ (extra_words >> 8U) ^ (extra_words >> 4U) ^ extra_words) & 15U) != 15U ||
                    extra_words < 2U || extra_words > (block.size() - 2U) / 2U)
                    return std::nullopt;
                const unsigned reserved = extra.read(4U);
                const std::size_t evolution_size = extra.read(12U);
                const auto body = block.subspan(2U, extra_words * 2U);
                if (evolution_size > body.size() - 3U) return std::nullopt;
                // LG dthd_evo.c folds the reserved nibble unshifted into the XOR seed.
                unsigned parity = 0xa9U ^ reserved ^ (reserved << 4U);
                for (std::size_t i = 0; i + 1U < body.size(); ++i) parity ^= body[i];
                if (parity != body.back() ||
                    !std::all_of(body.begin() + 2U + evolution_size, body.end() - 1,
                        [](std::uint8_t value) { return value == 0U; }))
                    return std::nullopt;
                if (evolution_size != 0U)
                    result.atmos_metadata = parse_evolution_frame(body.subspan(2U, evolution_size));
            } else if (!std::all_of(block.begin() + 2U, block.end(),
                [](std::uint8_t value) { return value == 0U; })) {
                return std::nullopt;
            }
        }
    } catch (const ParseError&) {
        return std::nullopt;
    }
    return result;
}

} // namespace atmos::truehd
