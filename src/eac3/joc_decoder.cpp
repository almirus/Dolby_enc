#include "joc_decoder.hpp"

#include "joc_tables.hpp"
#include "joc_surround_phase.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace atmos::eac3 {
namespace {

constexpr unsigned subbands = 64U;
constexpr unsigned frame_samples = 1536U;
constexpr unsigned timeslots = frame_samples / subbands;
constexpr unsigned maximum_objects = 64U;
constexpr unsigned maximum_channels = 7U;

using qmf::ComplexBands;
using qmf::Qmf;

unsigned parameter_band(const JocObject& object, unsigned subband) {
    unsigned result = 0;
    for (unsigned band = 1; band < object.bands; ++band) {
        if (joc_tables::band_edges[object.bands_index][band] > subband) break;
        result = band;
    }
    return result;
}

} // namespace

struct JocDecoder::Impl final {
    using ChannelMatrix = std::array<std::array<float, subbands>, maximum_channels>;
    using TimeslotMatrix = std::array<ChannelMatrix, timeslots>;

    std::array<Qmf, maximum_channels> analysis{};
    std::array<std::array<ComplexBands, 10>, maximum_channels> input_delay{};
    std::array<qmf::SurroundPhase, 2> surround{};
    unsigned input_cursor = 0;
    float previous_gain = -1;
    std::array<Qmf, maximum_objects> synthesis{};
    std::array<ChannelMatrix, maximum_objects> previous{};
    std::array<bool, maximum_objects> initialized{};

    void mixing_matrices(const JocObject& object, unsigned object_index,
        unsigned channels, TimeslotMatrix& interpolated) {
        std::array<ChannelMatrix, 2> decoded{};
        if (object.active) {
            const float code_center = object.fine_quantization ? 96.0F : 48.0F;
            const float step = object.fine_quantization ? 0.1F : 0.2F;
            for (unsigned point = 0; point < object.data_points; ++point) {
                if (object.sparse) {
                    const int integer_maximum = static_cast<int>(code_center * 2.0F);
                    unsigned channel = 0;
                    int value = object.fine_quantization ? 100 : 50;
                    for (unsigned band = 0; band < object.bands; ++band) {
                        channel = band == 0U ?
                            object.sparse_channel[point][0] :
                            (channel +
                                object.sparse_channel[point][band]) % channels;
                        value = (value + object.sparse_vector[point][band]) % integer_maximum;
                        decoded[point][channel][band] = (static_cast<float>(value) - code_center) * step;
                    }
                } else {
                    const int center = static_cast<int>(code_center);
                    const int maximum = center * 2;
                    for (unsigned channel = 0; channel < channels; ++channel) {
                        int value = 0;
                        for (unsigned band = 0; band < object.bands; ++band) {
                            value = (value + object.matrix[point][channel][band]) % maximum;
                            decoded[point][channel][band] =
                                static_cast<float>(value < center ? value : value - maximum) * step;
                        }
                    }
                }
            }
        }
        if (!object.active) {
            previous[object_index] = {};
            for (auto& slot : interpolated) slot = {};
            return;
        }

        const auto expand = [&](const ChannelMatrix& source, unsigned channel,
                                unsigned band) -> float {
            return source[channel][parameter_band(object, band)];
        };
        // Native has no previous matrix until this object's first active payload.
        const bool has_previous = initialized[object_index];
        initialized[object_index] = true;
        if (object.data_points == 1U) {
            if (object.steep_slope) {
                for (unsigned slot = 0; slot < timeslots; ++slot) {
                    const ChannelMatrix& source = has_previous && slot + 1U < object.timeslot_offsets[0] ?
                        previous[object_index] : decoded[0];
                    for (unsigned channel = 0; channel < channels; ++channel) {
                        for (unsigned band = 0; band < subbands; ++band) {
                            interpolated[slot][channel][band] =
                                &source == &previous[object_index] ? source[channel][band] :
                                expand(source, channel, band);
                        }
                    }
                }
            } else {
                for (unsigned slot = 0; slot < timeslots; ++slot) {
                    const float blend = static_cast<float>(slot + 1U) /
                        static_cast<float>(timeslots);
                    for (unsigned channel = 0; channel < channels; ++channel) {
                        for (unsigned band = 0; band < subbands; ++band) {
                            const float next = expand(decoded[0], channel, band);
                            const float prior = previous[object_index][channel][band];
                            interpolated[slot][channel][band] = has_previous ? prior + (next - prior) * blend : next;
                        }
                    }
                }
            }
        } else if (object.steep_slope) {
            for (unsigned slot = 0; slot < timeslots; ++slot) {
                const ChannelMatrix& source = has_previous && slot + 1U < object.timeslot_offsets[0] ?
                    previous[object_index] :
                    slot + 1U < object.timeslot_offsets[1] ? decoded[0] : decoded[1];
                for (unsigned channel = 0; channel < channels; ++channel) {
                    for (unsigned band = 0; band < subbands; ++band) {
                        interpolated[slot][channel][band] =
                            &source == &previous[object_index] ? source[channel][band] :
                            expand(source, channel, band);
                    }
                }
            }
        } else {
            constexpr unsigned half = timeslots / 2U;
            for (unsigned slot = 0; slot < timeslots; ++slot) {
                const bool first = slot < half;
                const float blend = first ? static_cast<float>(slot + 1U) / half :
                    static_cast<float>(slot + 1U - half) / (timeslots - half);
                for (unsigned channel = 0; channel < channels; ++channel) {
                    for (unsigned band = 0; band < subbands; ++band) {
                        const float from = first ? previous[object_index][channel][band] :
                            expand(decoded[0], channel, band);
                        const float to = expand(decoded[first ? 0U : 1U], channel, band);
                        interpolated[slot][channel][band] = first && !has_previous ? to : from + (to - from) * blend;
                    }
                }
            }
        }
        for (unsigned channel = 0; channel < channels; ++channel) {
            for (unsigned band = 0; band < subbands; ++band) {
                previous[object_index][channel][band] =
                    expand(decoded[object.data_points - 1U], channel, band);
            }
        }
    }
};

JocDecoder::JocDecoder() : impl_(std::make_unique<Impl>()) {}
JocDecoder::~JocDecoder() = default;

void JocDecoder::reset() noexcept {
    *impl_ = {};
}

bool JocDecoder::render(const JocFrame& joc, std::span<const float> bed,
    std::uint8_t bed_channels, std::uint16_t samples_per_channel,
    std::vector<float>& objects, std::string& error) {
    if (samples_per_channel != frame_samples ||
        (joc.channels != 5U && joc.channels != 7U) ||
        bed_channels < joc.channels + 1U ||
        joc.object_count == 0U || joc.object_count > maximum_objects ||
        bed.size() != static_cast<std::size_t>(bed_channels) * samples_per_channel) {
        error = "unsupported JOC PCM layout";
        return false;
    }
    static constexpr std::array<unsigned, 7> source_channels = {0U,1U,2U,4U,5U,6U,7U};
    // Native reconstruction ramps gain before analysis and delays spectra by ten slots.
    std::array<float, frame_samples> gain{};
    float current_gain = impl_->previous_gain < 0 ? joc.gain : impl_->previous_gain;
    const float increment = (joc.gain - current_gain) * (1.0F / 256);
    for (unsigned t = 0; t < frame_samples; ++t) {
        gain[t] = t < 256 ? current_gain : joc.gain;
        if (t < 256) current_gain += increment;
    }
    impl_->previous_gain = joc.gain;
    std::vector<Impl::TimeslotMatrix> matrices(joc.object_count);
    for (unsigned object = 0; object < joc.object_count; ++object) {
        impl_->mixing_matrices(joc.objects[object], object, joc.channels, matrices[object]);
    }
    objects.assign(static_cast<std::size_t>(samples_per_channel) * joc.object_count, 0.0F);
    for (unsigned slot = 0; slot < timeslots; ++slot) {
        std::array<ComplexBands, maximum_channels> channel_bands{};
        for (unsigned channel = 0; channel < joc.channels; ++channel) {
            std::array<float, subbands> input{};
            for (unsigned sample = 0; sample < subbands; ++sample) {
                input[sample] = bed[(slot * subbands + sample) * bed_channels +
                    source_channels[channel]] * gain[slot * subbands + sample];
            }
            channel_bands[channel] = impl_->analysis[channel].forward(input);
            if (channel == 3 || channel == 4)
                channel_bands[channel] = impl_->surround[channel - 3].process(channel_bands[channel],
                    joc.downmix_config == 3 || joc.downmix_config == 4);
            else
                std::swap(channel_bands[channel], impl_->input_delay[channel][impl_->input_cursor]);
        }
        impl_->input_cursor = (impl_->input_cursor + 1) % 10;
        for (unsigned object = 0; object < joc.object_count; ++object) {
            ComplexBands mixed;
            for (unsigned channel = 0; channel < joc.channels; ++channel) {
                for (unsigned band = 0; band < subbands; ++band) {
                    const float coefficient = matrices[object][slot][channel][band];
                    mixed.real[band] += channel_bands[channel].real[band] * coefficient;
                    mixed.imaginary[band] += channel_bands[channel].imaginary[band] * coefficient;
                }
            }
            const auto pcm = impl_->synthesis[object].inverse(mixed);
            for (unsigned sample = 0; sample < subbands; ++sample) {
                objects[(slot * subbands + sample) * joc.object_count + object] =
                    pcm[sample];
            }
        }
    }
    error.clear();
    return true;
}

} // namespace atmos::eac3
