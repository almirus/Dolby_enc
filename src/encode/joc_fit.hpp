// Recovered JOC per-band selection; checked against the DEE numerical helpers.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace atmos::encode::joc {
using Row = std::array<float, 7>;
using Matrix = std::array<Row, 7>;
using Mask = std::array<unsigned, 7>;
using Spectrum = std::array<float, 128>;
struct Window {
    unsigned channels, begin, end;
    std::vector<Spectrum> target;
    std::array<std::vector<Spectrum>, 7> carrier;
};
// The reference returns unquantized gains divided by16, rather than wire codes.
struct Result { Row coefficients{}; float first_gain = 0; unsigned first_channel = 0; };
inline constexpr float floor = 2.220446e-15F;

inline Row solve(const Matrix& gram, const Row& cross, const Mask& mask, unsigned channels) {
    std::array<unsigned, 7> indices{};
    unsigned size = 0;
    for (unsigned i = 0; i < channels; ++i) if (mask[i]) indices[size++] = i;
    Matrix a{}, lower{};
    Row rhs{}, diagonal{}, forward{}, scaled{}, backward{}, result{};
    float trace = 0;
    for (unsigned i = 0; i < size; ++i) {
        rhs[i] = cross[indices[i]];
        for (unsigned j = 0; j < size; ++j) a[i][j] = gram[indices[i]][indices[j]];
        trace += a[i][i];
    }
    const float ridge = trace * 0.01F;
    for (unsigned i = 0; i < size; ++i) a[i][i] = (a[i][i] + ridge) + floor;
    for (unsigned i = 0; i < size; ++i) {
        lower[i][i] = 1;
        float value = a[i][i];
        for (unsigned j = 0; j < i; ++j) value -= (lower[i][j] * lower[i][j]) * diagonal[j];
        diagonal[i] = std::max(floor, value);
        for (unsigned j = i + 1; j < size; ++j) {
            value = a[j][i];
            for (unsigned k = 0; k < i; ++k) value -= (lower[i][k] * lower[j][k]) * diagonal[k];
            lower[j][i] = value / diagonal[i];
        }
    }
    for (unsigned i = 0; i < size; ++i) {
        float value = rhs[i];
        for (unsigned j = 0; j < i; ++j) value -= lower[i][j] * forward[j];
        forward[i] = value;
        scaled[i] = value / diagonal[i];
    }
    for (unsigned i = size; i-- > 0;) {
        float value = scaled[i];
        for (unsigned j = i + 1; j < size; ++j) value -= lower[j][i] * backward[j];
        backward[i] = value;
    }
    for (unsigned i = 0; i < size; ++i) result[indices[i]] = backward[i];
    return result;
}

inline Row normalize(Row coefficients, const Window& w, const Mask& mask) {
    float target_energy = 0, reconstructed_energy = 0;
    for (unsigned t = 0; t < w.target.size(); ++t) {
        float te = 0, re = 0;
        for (unsigned band = w.begin; band < w.end; ++band) {
            float real = 0, imaginary = 0;
            for (unsigned i = 0; i < w.channels; ++i) if (mask[i] && coefficients[i] != 0) {
                real += coefficients[i] * w.carrier[i][t][2 * band];
                imaginary += coefficients[i] * w.carrier[i][t][2 * band + 1];
            }
            const float x = w.target[t][2 * band], y = w.target[t][2 * band + 1];
            te = (te + x * x) + y * y;
            re = (re + real * real) + imaginary * imaginary;
        }
        target_energy += te;
        reconstructed_energy += re;
    }
    const float gain = std::sqrt(target_energy / (reconstructed_energy + floor));
    for (unsigned i = 0; i < w.channels; ++i) if (mask[i]) coefficients[i] *= gain;
    return coefficients;
}

inline double score(const Row& coefficients, const Window& w, const Mask& mask) {
    double error = 0, reconstructed_energy = 0, target_energy = 0;
    for (unsigned t = 0; t < w.target.size(); ++t) for (unsigned band = w.begin; band < w.end; ++band) {
        const double x = w.target[t][2 * band], y = w.target[t][2 * band + 1];
        double real = -x, imaginary = -y;
        for (unsigned i = 0; i < w.channels; ++i) if (mask[i] && coefficients[i] != 0) {
            real += double(coefficients[i]) * w.carrier[i][t][2 * band];
            imaginary += double(coefficients[i]) * w.carrier[i][t][2 * band + 1];
        }
        error = (real * real + error) + imaginary * imaginary;
        reconstructed_energy = ((real + x) * (real + x) + reconstructed_energy) + (imaginary + y) * (imaginary + y);
        target_energy = (x * x + target_energy) + y * y;
    }
    constexpr double epsilon = double(1e-12F);
    return std::pow(10.0, std::abs(std::log10((reconstructed_energy + epsilon) / (target_energy + epsilon) + 0.05))) * (error + epsilon);
}

inline int quantized_code(float scaled, bool fine) {
    const int divisor = fine ? 205 : 410, center = fine ? 96 : 48, maximum = fine ? 191 : 95;
    if (!std::isfinite(scaled)) throw std::runtime_error("nonfinite coefficient before quantization");
    const float bounded = std::clamp(scaled, -1.0F, 1.0F);
    const int q15 = static_cast<int>(std::clamp(std::lrint(bounded * 32768.0F), -32768L, 32767L));
    const int biased = q15 + (q15 >= 0 ? divisor / 2 : -divisor / 2);
    return std::clamp(biased / divisor + center, 0, maximum);
}

inline Row quantize(Row scaled, bool fine, unsigned channels) {
    const int divisor = fine ? 205 : 410, center = fine ? 96 : 48;
    for (unsigned i = 0; i < channels; ++i) {
        const int code = quantized_code(scaled[i], fine);
        scaled[i] = float((code - center) * divisor) / 32768.0F * 16.0F;
    }
    return scaled;
}

inline Result fit(const Matrix& gram, const Row& cross, const Window& w, Mask allowed, bool fine) {
    Result result;
    double minimum = std::numeric_limits<double>::infinity();
    Mask current = allowed; // The initial mask seeds the search; later steps may add any carrier.
    for (unsigned iteration = 0; iteration < w.channels; ++iteration) {
        std::vector<Mask> candidates;
        if (iteration == 0 || (w.begin >= 18 && iteration == 1)) {
            for (unsigned channel = 0; channel < w.channels; ++channel) if (iteration || current[channel]) {
                Mask mask{}; mask[channel] = 1; candidates.push_back(mask);
            }
        } else if (w.begin >= 18) break;
        else for (unsigned channel = 0; channel < w.channels; ++channel) if (!current[channel]) {
            Mask mask = current; mask[channel] = 1; candidates.push_back(mask);
        }
        if (candidates.empty()) break;
        double best_error = std::numeric_limits<double>::infinity();
        Row best_coefficients{};
        Mask best_mask{};
        for (const auto& mask : candidates) {
            Row coefficients = normalize(solve(gram, cross, mask, w.channels), w, mask);
            for (auto& value : coefficients) value *= 0.0625F;
            const double error = score(quantize(coefficients, fine, w.channels), w, mask);
            if (!std::isfinite(error)) throw std::runtime_error("nonfinite fitting score");
            if (error < best_error) { best_error = error; best_coefficients = coefficients; best_mask = mask; }
        }
        current = best_mask;
        if (!iteration) {
            while (!current[result.first_channel]) ++result.first_channel;
            result.first_gain = best_coefficients[result.first_channel];
        }
        for (unsigned i = 0; i < iteration; ++i) {
            best_error = best_error * double(0.56F) * 2.0;
            if (best_error == 1.0 || best_error == -1.0) break;
        }
        if (best_error < minimum) { minimum = best_error; result.coefficients = best_coefficients; }
    }
    return result;
}
} // namespace atmos::encode::joc
