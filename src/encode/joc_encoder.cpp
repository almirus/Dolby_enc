#include "joc_encoder.hpp"
#include "joc_refine.hpp"
#include "emdf_writer.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace atmos::encode {

std::vector<unsigned> joc_target_channels(const eac3::OamdFrame& metadata) {
    if (metadata.isf_channels) throw std::runtime_error("JOC encoding of OAMD ISF signals is unsupported");
    if (!metadata.object_count || metadata.bed_channels != metadata.bed_channel_ids.size() ||
        metadata.bed_channels > metadata.object_count)
        throw std::runtime_error("invalid JOC OAMD signal layout");
    std::vector<unsigned> channels;
    bool lfe = false;
    for (unsigned channel = 0; channel < metadata.object_count; ++channel) {
        if (channel < metadata.bed_channels) {
            const auto id = metadata.bed_channel_ids[channel];
            if (id > 15) throw std::runtime_error("JOC supports one LFE channel; LFE2 is unsupported");
            if (id == 3) {
                if (lfe) throw std::runtime_error("JOC supports one LFE channel");
                lfe = true;
                continue;
            }
        }
        channels.push_back(channel);
    }
    if (channels.empty() || channels.size() > 15)
        throw std::runtime_error("native-compatible JOC requires 1 to 15 full-band signals");
    return channels;
}

eac3::JocFrame JocEncoder::encode(std::span<const float> carrier,
    std::span<const float> elements, unsigned object_count, std::size_t payload_budget,
    int first_band_index, std::span<const float> carrier_history) {
    constexpr unsigned samples = 1536, slots = 24, channels = 5;
    if (!object_count || object_count > 16 || carrier.size() != samples * 6 ||
        elements.size() != samples * object_count || first_band_index < 0 || first_band_index > 7)
        throw std::runtime_error("invalid JOC input shape");
    if (!carrier_history.empty() && carrier_history.size() != joc::History{}.size())
        throw std::runtime_error("invalid JOC carrier history shape");
    if (detector_ && history_enabled_ != !carrier_history.empty())
        throw std::runtime_error("JOC carrier history mode changed");
    for (const auto input : {carrier, elements, carrier_history})
        if (!std::all_of(input.begin(), input.end(), [](float x) { return std::isfinite(x); }))
            throw std::runtime_error("non-finite JOC input");
    if (detector_ && previous_targets_.size() != object_count)
        throw std::runtime_error("JOC object count changed");
    if (!detector_) {
        history_enabled_ = !carrier_history.empty();
        detector_.emplace(object_count, 1.5999999547489097e-8F);
        previous_targets_.assign(object_count, std::vector<joc::Spectrum>(slots));
        for (auto& channel : previous_carriers_) channel.resize(slots);
    }
    std::array<std::vector<joc::Spectrum>, channels> downmix;
    for (auto& channel : downmix) channel.resize(slots);
    std::vector<std::vector<joc::Spectrum>> targets(object_count, std::vector<joc::Spectrum>(slots));
    const auto delayed = [&](joc::Spectrum converted, unsigned channel) {
        std::swap(converted, delay_[channel][delay_index_]);
        return converted;
    };
    constexpr unsigned source[] = {0, 1, 2, 4, 5};
    for (unsigned slot = 0; slot < slots; ++slot) {
        std::array<float, 64> input{};
        for (unsigned channel = 0; channel < channels; ++channel) {
            for (unsigned t = 0; t < 64; ++t) input[t] = carrier[(slot * 64 + t) * 6 + source[channel]] * .25F;
            downmix[channel][slot] = delayed(carrier_filters_[channel].forward(input), channel);
        }
        for (unsigned object = 0; object < object_count; ++object) {
            for (unsigned t = 0; t < 64; ++t) input[t] = elements[(slot * 64 + t) * object_count + object] * .25F;
            targets[object][slot] = delayed(object_filters_[object].forward(input), channels + object);
        }
        delay_index_ = (delay_index_ + 1) % 10;
    }
    const auto previous_header = detector_->detect(targets);
    const auto window = joc::temporal_weights(previous_header, detector_->current, 0);
    std::vector<joc::Mask> allowed(object_count, joc::Mask{1,1,1,1,1,0,0});
    if (history_enabled_) {
        joc::History history{};
        std::copy(carrier_history.begin(), carrier_history.end(), history.begin());
        const auto current = history_delay_.advance(history);
        allowed = joc::seed_masks(previous_history_, current, window, object_count, 6);
        previous_history_ = current;
        for (auto& mask : allowed)
            if (std::none_of(mask.begin(), mask.end(), [](auto x) { return x != 0; }))
                mask = {1,1,1,1,1,0,0};
    }
    const std::vector<float> weights(window.begin(), window.end());
    std::vector<joc::Window> weighted(object_count);
    for (unsigned object = 0; object < object_count; ++object) {
        auto& w = weighted[object];
        w.channels = channels; w.begin = 0; w.end = 64;
        w.target = previous_targets_[object];
        w.target.insert(w.target.end(), targets[object].begin(), targets[object].end());
        for (unsigned channel = 0; channel < channels; ++channel) {
            w.carrier[channel] = previous_carriers_[channel];
            w.carrier[channel].insert(w.carrier[channel].end(), downmix[channel].begin(), downmix[channel].end());
        }
        joc::weight(w, weights);
    }
    previous_targets_ = std::move(targets);
    previous_carriers_ = std::move(downmix);
    eac3::JocFrame result;
    result.sequence_count = sequence_count_;
    result.channels = channels;
    result.object_count = static_cast<std::uint8_t>(object_count);
    const unsigned original_bands = eac3::joc_tables::band_count[first_band_index];
    std::vector<joc::Result> fits(object_count * original_bands);
    std::vector<joc::Matrix> grams(fits.size());
    std::vector<joc::Row> crosses(fits.size());
    std::size_t bytes = 0;
    {
        const unsigned bands = original_bands;
        for (unsigned object = 0; object < object_count; ++object) {
            auto& out = result.objects[object];
            out.active = true;
            out.bands_index = static_cast<std::uint8_t>(first_band_index);
            out.bands = static_cast<std::uint8_t>(bands);
            out.fine_quantization = true;
            out.sparse = false;
            out.data_points = 1;
            out.steep_slope = previous_header[0] != 0;
            out.timeslot_offsets[0] = static_cast<std::uint8_t>(previous_header[2] + 1);
            std::array<int, channels> previous{};
            for (unsigned band = 0; band < bands; ++band) {
                auto& w = weighted[object];
                w.begin = eac3::joc_tables::band_edges[first_band_index][band];
                w.end = band + 1 < bands ? eac3::joc_tables::band_edges[first_band_index][band + 1] : 64;
                joc::Matrix gram{}; joc::Row cross{};
                joc::correlations(w, gram, cross);
                grams[object * bands + band] = gram;
                crosses[object * bands + band] = cross;
                const auto fitted = joc::refine_quantized(w, joc::fit(gram, cross, w, allowed[object], true));
                fits[object * bands + band] = fitted;
                for (unsigned channel = 0; channel < channels; ++channel) {
                    const int value = joc::quantized_code(fitted.coefficients[channel], true) - 96;
                    out.matrix[0][channel][band] = static_cast<std::uint8_t>((value - previous[channel] + 192) % 192);
                    previous[channel] = value;
                }
            }
        }
        bytes = write_joc_payload(result).size();
        if (bytes > payload_budget) {
            // DEE lowers precision using the existing fit before discarding frequency bands.
            for (unsigned object = 0; object < object_count; ++object) {
                auto& out = result.objects[object];
                out.fine_quantization = false;
                std::array<int, channels> previous{};
                for (unsigned band = 0; band < bands; ++band) {
                    for (unsigned channel = 0; channel < channels; ++channel) {
                        const int value = joc::quantized_code(fits[object * bands + band].coefficients[channel], false) - 48;
                        out.matrix[0][channel][band] = static_cast<std::uint8_t>((value - previous[channel] + 96) % 96);
                        previous[channel] = value;
                    }
                }
            }
            bytes = write_joc_payload(result).size();
        }
        if (bytes > payload_budget) {
            for (unsigned object = 0; object < object_count; ++object) {
                auto& out = result.objects[object];
                out.sparse = true;
                for (unsigned band = 0; band < bands; ++band) {
                    auto& fitted = fits[object * bands + band];
                    fitted.coefficients[fitted.first_channel] = fitted.first_gain;
                }
            }
        }
    }
    const auto encode_sparse = [&](unsigned bands) {
        for (unsigned object = 0; object < object_count; ++object) {
            auto& out = result.objects[object];
            unsigned previous_channel = 0;
            int previous_value = 2; // Sparse wire prediction starts at50, while coarse zero is48.
            for (unsigned band = 0; band < bands; ++band) {
                const auto& fitted = fits[object * original_bands + band];
                const unsigned channel = fitted.first_channel;
                const int value = joc::quantized_code(fitted.coefficients[channel], false) - 48;
                out.sparse_channel[0][band] = static_cast<std::uint8_t>(band ? (channel + channels - previous_channel) % channels : channel);
                out.sparse_vector[0][band] = static_cast<std::uint8_t>((value - previous_value + 96) % 96);
                previous_channel = channel;
                previous_value = value;
            }
        }
        return write_joc_payload(result).size();
    };
    if (result.objects[0].sparse) bytes = encode_sparse(original_bands);
    constexpr std::array<std::array<unsigned, 15>, 8> coarser_bands{{
        {}, {{0}}, {{0,2,3}}, {{0,1,2,4,6}}, {{0,1,2,3,5,7,8}},
        {{0,1,2,3,4,5,6,8,10}}, {{0,1,2,3,4,6,8,10,11,12,13,14}},
        {{0,1,2,3,4,5,6,7,8,9,11,13,15,17,20}}
    }};
    for (int band_index = first_band_index - 1; bytes > payload_budget && band_index >= 0; --band_index) {
        const unsigned bands = eac3::joc_tables::band_count[band_index];
        for (unsigned object = 0; object < object_count; ++object) {
            auto& out = result.objects[object];
            out.bands_index = static_cast<std::uint8_t>(band_index); out.bands = static_cast<std::uint8_t>(bands);
            for (unsigned band = 0; band < bands; ++band) {
                auto& fitted = fits[object * original_bands + band];
                const unsigned channel = fitted.first_channel;
                // Native remapping copies only the selected row and retains channel indices.
                fitted.coefficients[channel] = fits[object * original_bands + coarser_bands[band_index + 1][band]].coefficients[channel];
            }
        }
        bytes = encode_sparse(bands);
    }
    if (bytes <= payload_budget) {
        std::array<bool, 16> nonzero{};
        joc::Matrix adjustment_gram{};
        joc::Row adjustment_cross{};
        for (unsigned object = 0; object < object_count; ++object) {
            const auto& out = result.objects[object];
            const int modulus = out.fine_quantization ? 192 : 96, center = modulus / 2;
            const float step = out.fine_quantization ? 205.0F / 2048 : 410.0F / 2048;
            std::array<int, channels> previous{};
            unsigned sparse_channel = 0;
            int sparse_value = out.fine_quantization ? 100 : 50;
            for (unsigned band = 0; band < out.bands; ++band) {
                joc::Row coefficients{};
                if (out.sparse) {
                    sparse_channel = band ? (sparse_channel + out.sparse_channel[0][band]) % channels :
                        out.sparse_channel[0][0];
                    sparse_value = (sparse_value + out.sparse_vector[0][band]) % modulus;
                    coefficients[sparse_channel] = float(sparse_value - center) * step;
                } else for (unsigned channel = 0; channel < channels; ++channel) {
                    previous[channel] = (previous[channel] + out.matrix[0][channel][band]) % modulus;
                    coefficients[channel] = float(previous[channel] >= center ?
                        previous[channel] - modulus : previous[channel]) * step;
                }
                auto gram = grams[object * original_bands + band];
                auto cross = crosses[object * original_bands + band];
                if (out.bands_index != first_band_index) {
                    auto& w = weighted[object];
                    w.begin = eac3::joc_tables::band_edges[out.bands_index][band];
                    w.end = band + 1 < out.bands ? eac3::joc_tables::band_edges[out.bands_index][band + 1] : 64;
                    joc::correlations(w, gram, cross);
                }
                for (unsigned i = 0; i < channels; ++i) {
                    nonzero[object] = nonzero[object] || coefficients[i] != 0;
                    adjustment_cross[i] += coefficients[i] * cross[i];
                    for (unsigned j = 0; j < channels; ++j)
                        adjustment_gram[i][j] += (coefficients[i] * coefficients[j]) * gram[i][j];
                }
            }
        }
        // Solve around unity: the ridge preserves the downmix instead of shrinking silent carriers.
        for (unsigned i = 0; i < channels; ++i)
            for (unsigned j = 0; j < channels; ++j) adjustment_cross[i] -= adjustment_gram[i][j];
        const auto delta = joc::solve(adjustment_gram, adjustment_cross, {1,1,1,1,1,0,0}, channels);
        float maximum_delta = .05F;
        for (unsigned channel = 0; channel < channels; ++channel)
            maximum_delta = std::max(maximum_delta, std::abs(delta[channel]));
        const float scale = .05F / maximum_delta;
        // A shared bound preserves the descent direction for correlated carriers.
        for (unsigned channel = 0; channel < channels; ++channel)
            carrier_correction_[channel] = 1.0F + delta[channel] * scale;
        // Keep the first zero matrix: its interpolation may still fade the preceding one.
        for (unsigned object = 0; object < object_count; ++object) {
            // Explicit initial matrices initialize native surround reconstruction, even when zero.
            result.objects[object].active = sequence_count_ == 0 || nonzero[object] || previous_nonzero_[object];
            previous_nonzero_[object] = nonzero[object];
        }
        sequence_count_ = static_cast<std::uint16_t>(sequence_count_ == 1023 ? 1 : sequence_count_ + 1);
        return result;
    }
    throw std::runtime_error("JOC cannot fit the metadata payload budget");
}
} // namespace atmos::encode
