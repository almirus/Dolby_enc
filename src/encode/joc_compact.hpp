// Preserve expanded integer matrices while selecting a smaller JOC wire representation.
#pragma once
#include "eac3/joc_tables.hpp"
#include "encode/emdf_writer.hpp"
#include <algorithm>
#include <stdexcept>

namespace atmos::encode::joc {
using Expanded = std::array<std::array<std::array<int, 64>, 7>, 2>;
inline Expanded expand(const atmos::eac3::JocObject &o, unsigned channels) {
    Expanded result{};
    if (!o.active)
        return result;
    if (o.bands_index > 7 || o.bands != atmos::eac3::joc_tables::band_count[o.bands_index] ||
        o.data_points < 1 || o.data_points > 2)
        throw std::runtime_error("invalid compact JOC object");
    const int modulus = o.fine_quantization ? 192 : 96, center = modulus / 2,
              scale = o.fine_quantization ? 1 : 2;
    for (unsigned p = 0; p < o.data_points; ++p) {
        unsigned channel = 0;
        int value = o.fine_quantization ? 100 : 50;
        std::array<int, 7> previous{};
        for (unsigned b = 0; b < o.bands; ++b) {
            std::array<int, 7> row{};
            if (o.sparse) {
                channel = b ? (channel + o.sparse_channel[p][b]) % channels : o.sparse_channel[p][b];
                if (channel >= channels)
                    throw std::runtime_error("invalid compact sparse channel");
                value = (value + o.sparse_vector[p][b]) % modulus;
                row[channel] = (value - center) * scale;
            } else
                for (unsigned c = 0; c < channels; ++c) {
                    previous[c] = (previous[c] + o.matrix[p][c][b]) % modulus;
                    row[c] = (previous[c] >= center ? previous[c] - modulus : previous[c]) * scale;
                }
            const unsigned begin = atmos::eac3::joc_tables::band_edges[o.bands_index][b],
                           end = b + 1 < o.bands ? atmos::eac3::joc_tables::band_edges[o.bands_index][b + 1]
                                                 : 64;
            for (unsigned c = 0; c < channels; ++c)
                std::fill(result[p][c].begin() + begin, result[p][c].begin() + end, row[c]);
        }
    }
    return result;
}
inline void compact(atmos::eac3::JocFrame &frame) {
    auto best_bits = atmos::encode::joc_payload_bit_count(frame);
    for (unsigned object = 0; object < frame.object_count; ++object) {
        const auto original = frame.objects[object];
        if (!original.active)
            continue;
        const auto full = expand(original, frame.channels);
        auto best = original;
        for (unsigned grid = 0; grid <= original.bands_index; ++grid) {
            const unsigned bands = atmos::eac3::joc_tables::band_count[grid];
            bool constant = true;
            for (unsigned p = 0; p < original.data_points; ++p)
                for (unsigned c = 0; c < frame.channels; ++c)
                    for (unsigned b = 0; b < bands; ++b) {
                        const unsigned begin = atmos::eac3::joc_tables::band_edges[grid][b],
                                       end = b + 1 < bands ? atmos::eac3::joc_tables::band_edges[grid][b + 1]
                                                           : 64;
                        for (unsigned k = begin + 1; k < end; ++k)
                            constant = constant && full[p][c][k] == full[p][c][begin];
                    }
            if (!constant)
                continue;
            for (bool fine : {false, true})
                for (bool sparse : {false, true}) {
                    auto candidate = original;
                    candidate.bands_index = static_cast<std::uint8_t>(grid);
                    candidate.bands = static_cast<std::uint8_t>(bands);
                    candidate.fine_quantization = fine;
                    candidate.sparse = sparse;
                    candidate.matrix = {};
                    candidate.sparse_channel = {};
                    candidate.sparse_vector = {};
                    const int scale = fine ? 1 : 2, modulus = fine ? 192 : 96, center = modulus / 2;
                    bool exact = true;
                    for (unsigned p = 0; p < original.data_points && exact; ++p) {
                        std::array<int, 7> previous{};
                        int sparse_value = fine ? 100 : 50;
                        unsigned sparse_channel = 0;
                        for (unsigned b = 0; b < bands && exact; ++b) {
                            const unsigned begin = atmos::eac3::joc_tables::band_edges[grid][b];
                            std::array<int, 7> row{};
                            unsigned nonzero = 0, selected = sparse_channel;
                            for (unsigned c = 0; c < frame.channels; ++c) {
                                const int x = full[p][c][begin];
                                if (x % scale) {
                                    exact = false;
                                    break;
                                }
                                row[c] = x / scale;
                                if (row[c] < -center || row[c] >= center) {
                                    exact = false;
                                    break;
                                }
                                if (row[c]) {
                                    ++nonzero;
                                    selected = c;
                                }
                            }
                            if (!exact)
                                break;
                            if (sparse) {
                                if (nonzero > 1) {
                                    exact = false;
                                    break;
                                }
                                candidate.sparse_channel[p][b] = static_cast<std::uint8_t>(
                                    b ? (selected + frame.channels - sparse_channel) % frame.channels
                                      : selected);
                                const int value = row[selected] + center;
                                candidate.sparse_vector[p][b] =
                                    static_cast<std::uint8_t>((value - sparse_value + modulus) % modulus);
                                sparse_value = value;
                                sparse_channel = selected;
                            } else
                                for (unsigned c = 0; c < frame.channels; ++c) {
                                    candidate.matrix[p][c][b] =
                                        static_cast<std::uint8_t>((row[c] - previous[c] + modulus) % modulus);
                                    previous[c] = row[c];
                                }
                        }
                    }
                    if (!exact)
                        continue;
                    frame.objects[object] = candidate;
                    const auto bits = atmos::encode::joc_payload_bit_count(frame);
                    if (bits < best_bits) {
                        best_bits = bits;
                        best = candidate;
                    }
                }
        }
        frame.objects[object] = best;
        if (expand(best, frame.channels) != full)
            throw std::runtime_error("JOC compaction changed matrix");
    }
}
} // namespace atmos::encode::joc
