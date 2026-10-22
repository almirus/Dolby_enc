#pragma once

#include "joc_tables.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <span>

namespace atmos::eac3::qmf {
inline constexpr unsigned subbands = 64U;
inline constexpr unsigned transform_size = 128U;

struct ComplexBands final {
    std::array<float, subbands> real{};
    std::array<float, subbands> imaginary{};
};

// The modulation matrix never changes: evaluating cos/sin per band and sample
// on every call cost more than the transform itself (the encoder runs this for
// every carrier and object of every frame).
inline const std::array<std::array<double, transform_size>, subbands>& modulation_cos() {
    static const auto table = [] {
        std::array<std::array<double, transform_size>, subbands> result{};
        constexpr double divisor = 1.0 / subbands;
        for (unsigned band = 0; band < subbands; ++band)
            for (unsigned sample = 0; sample < transform_size; ++sample)
                result[band][sample] = std::cos(
                    std::numbers::pi * (band + 0.5) * (sample - 0.5) * divisor);
        return result;
    }();
    return table;
}

inline const std::array<std::array<double, transform_size>, subbands>& modulation_sin() {
    static const auto table = [] {
        std::array<std::array<double, transform_size>, subbands> result{};
        constexpr double divisor = 1.0 / subbands;
        for (unsigned band = 0; band < subbands; ++band)
            for (unsigned sample = 0; sample < transform_size; ++sample)
                result[band][sample] = std::sin(
                    std::numbers::pi * (band + 0.5) * (sample - 0.5) * divisor);
        return result;
    }();
    return table;
}

inline const std::array<std::array<float, subbands>, transform_size>& inverse_cos() {
    static const auto table = [] {
        std::array<std::array<float, subbands>, transform_size> result{};
        constexpr float divisor = 1.0F / static_cast<float>(subbands);
        for (std::size_t sample = 0; sample < transform_size; ++sample)
            for (unsigned band = 0; band < subbands; ++band)
                result[sample][band] = std::cos(std::numbers::pi_v<float> *
                    (static_cast<float>(band) + 0.5F) *
                    (static_cast<float>(sample) - static_cast<float>(transform_size) + 0.5F) *
                    divisor);
        return result;
    }();
    return table;
}

inline const std::array<std::array<float, subbands>, transform_size>& inverse_sin() {
    static const auto table = [] {
        std::array<std::array<float, subbands>, transform_size> result{};
        constexpr float divisor = 1.0F / static_cast<float>(subbands);
        for (std::size_t sample = 0; sample < transform_size; ++sample)
            for (unsigned band = 0; band < subbands; ++band)
                result[sample][band] = std::sin(std::numbers::pi_v<float> *
                    (static_cast<float>(band) + 0.5F) *
                    (static_cast<float>(sample) - static_cast<float>(transform_size) + 0.5F) *
                    divisor);
        return result;
    }();
    return table;
}

struct Qmf final {
    std::array<float, 640> forward_history{};
    std::array<float, 1280> inverse_history{};

    ComplexBands forward(std::span<const float, subbands> input) {
        std::move_backward(forward_history.begin(), forward_history.end() - subbands,
            forward_history.end());
        for (unsigned sample = 0; sample < subbands; ++sample) {
            forward_history[sample] = input[subbands - sample - 1U];
        }
        std::array<float, transform_size> grouped{};
        for (unsigned offset = 0; offset < forward_history.size(); offset += transform_size) {
            for (unsigned sample = 0; sample < transform_size; ++sample) {
                grouped[sample] += forward_history[offset + sample] *
                    joc_tables::qmf_coefficients[offset + sample];
            }
        }
        ComplexBands result;
        const auto& cosines = modulation_cos();
        const auto& sines = modulation_sin();
        for (unsigned band = 0; band < subbands; ++band) {
            double real = 0, imaginary = 0;
            for (unsigned sample = 0; sample < transform_size; ++sample) {
                // Float angles leak tonal energy into quiet JOC bands and change quantized gains.
                real += cosines[band][sample] * grouped[sample];
                imaginary += sines[band][sample] * grouped[sample];
            }
            result.real[band] = static_cast<float>(real);
            result.imaginary[band] = static_cast<float>(imaginary);
        }
        return result;
    }

    std::array<float, subbands> inverse(const ComplexBands& input) {
        std::move_backward(inverse_history.begin(), inverse_history.end() - transform_size,
            inverse_history.end());
        constexpr float divisor = 1.0F / static_cast<float>(subbands);
        const auto& cosines = inverse_cos();
        const auto& sines = inverse_sin();
        for (unsigned sample = 0; sample < transform_size; ++sample) {
            float value = 0.0F;
            for (unsigned band = 0; band < subbands; ++band) {
                value += (cosines[sample][band] * input.real[band] -
                    sines[sample][band] * input.imaginary[band]) * divisor;
            }
            inverse_history[sample] = value;
        }
        std::array<float, subbands> output{};
        for (unsigned group = 0; group < 5U; ++group) {
            const unsigned time = subbands * 4U * group;
            const unsigned coefficient = transform_size * group;
            for (unsigned sample = 0; sample < subbands; ++sample) {
                output[sample] += inverse_history[time + sample] *
                    joc_tables::qmf_coefficients[coefficient + sample] +
                    inverse_history[time + subbands * 3U + sample] *
                    joc_tables::qmf_coefficients[coefficient + subbands + sample];
            }
        }
        return output;
    }
};

} // namespace atmos::eac3::qmf
