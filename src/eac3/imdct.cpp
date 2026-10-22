#include "imdct.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace atmos::eac3 {
namespace {

double bessel_i0(double value) {
    const double half = value * 0.5;
    double sum = 1.0;
    double term = 1.0;
    for (unsigned order = 1; order < 40U; ++order) {
        term *= (half * half) /
            (static_cast<double>(order) * static_cast<double>(order));
        sum += term;
        if (term < sum * 1.0e-16) break;
    }
    return sum;
}

std::array<double, 256> make_kbd_half_window() {
    constexpr std::size_t size = 256U;
    constexpr double alpha = 5.0;
    std::array<double, size> cumulative{};
    double total = 0.0;
    for (std::size_t index = 0; index < size; ++index) {
        const double position = (2.0 * static_cast<double>(index)) /
            static_cast<double>(size) - 1.0;
        total += bessel_i0(std::numbers::pi * alpha *
            std::sqrt(std::max(0.0, 1.0 - position * position)));
        cumulative[index] = total;
    }
    total += 1.0; // final symmetric Kaiser endpoint
    for (double& value : cumulative) value = std::sqrt(value / total);
    return cumulative;
}

const std::vector<std::array<double, 256>>& imdct_basis() {
    static const auto table = [] {
        std::vector<std::array<double, 256>> result(512U);
        for (std::size_t sample = 0; sample < result.size(); ++sample) {
            const double phase = std::numbers::pi / 256.0 *
                (static_cast<double>(sample) + 128.5);
            for (std::size_t bin = 0; bin < result[sample].size(); ++bin) {
                result[sample][bin] = std::cos(
                    phase * (static_cast<double>(bin) + 0.5));
            }
        }
        return result;
    }();
    return table;
}

const std::vector<std::array<double, 128>>& short_imdct_basis() {
    static const auto table = [] {
        std::vector<std::array<double, 128>> result(256U);
        for (std::size_t sample = 0; sample < result.size(); ++sample) {
            const double phase = std::numbers::pi / 128.0 *
                (static_cast<double>(sample) + 64.5);
            for (std::size_t bin = 0; bin < result[sample].size(); ++bin) {
                result[sample][bin] = std::cos(
                    phase * (static_cast<double>(bin) + 0.5));
            }
        }
        return result;
    }();
    return table;
}

void overlap_add(const std::array<double, 128>& delay,
    const std::array<double, 128>& current, const std::array<double, 256>& window,
    std::span<float, 256> output) {
    for (std::size_t sample = 0; sample < 128U; ++sample) {
        const std::size_t reverse = 127U - sample;
        const double previous = delay[sample];
        const double next = current[reverse];
        output[sample] = static_cast<float>(previous * window[255U - sample] -
            next * window[sample]);
        output[255U - sample] = static_cast<float>(previous * window[sample] +
            next * window[255U - sample]);
    }
}

} // namespace

Imdct256::Imdct256() {
    window_ = make_kbd_half_window();
}

void Imdct256::reset() noexcept {
    delay_.fill(0.0);
}

void Imdct256::synthesize(std::span<const std::int32_t, 256> coefficients,
    std::span<float, 256> output, bool short_blocks) {
    constexpr double scale = -1.0 / 4194304.0;
    if (short_blocks) {
        const auto& basis = short_imdct_basis();
        std::array<double, 128> current{};
        std::array<double, 128> next_delay{};
        for (std::size_t sample = 0; sample < 128U; ++sample) {
            double even = 0.0;
            double odd = 0.0;
            for (std::size_t bin = 0; bin < 128U; ++bin) {
                even += static_cast<double>(coefficients[bin * 2U]) *
                    basis[sample + 64U][bin];
                odd += static_cast<double>(coefficients[bin * 2U + 1U]) *
                    basis[sample + 64U][bin];
            }
            current[sample] = even * scale;
            next_delay[sample] = odd * scale;
        }
        overlap_add(delay_, current, window_, output);
        delay_ = next_delay;
        return;
    }
    std::array<double, 256> half{};
    const auto& basis = imdct_basis();
    for (std::size_t sample = 0; sample < half.size(); ++sample) {
        double value = 0.0;
        for (std::size_t bin = 0; bin < coefficients.size(); ++bin) {
            value += static_cast<double>(coefficients[bin]) *
                basis[sample + 128U][bin];
        }
        half[sample] = value * scale;
    }
    std::array<double, 128> current{};
    std::copy_n(half.begin(), 128U, current.begin());
    overlap_add(delay_, current, window_, output);
    std::copy_n(half.begin() + 128U, 128U, delay_.begin());
}

} // namespace atmos::eac3
