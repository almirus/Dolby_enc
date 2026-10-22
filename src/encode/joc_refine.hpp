#pragma once
#include "joc_fit.hpp"

namespace atmos::encode::joc {

// Refine fine gains within the recovered mask; retain the native sparse fallback.
inline Result refine_quantized(const Window& w, Result native) {
    constexpr unsigned channels = 5;
    if (w.channels != channels) throw std::runtime_error("unsupported JOC refinement channels");
    std::array<std::array<double, channels + 1>, channels> equation{};
    for (unsigned t = 0; t < w.target.size(); ++t)
        for (unsigned k = 2 * w.begin; k < 2 * w.end; ++k)
            for (unsigned i = 0; i < channels; ++i) {
                const double x = w.carrier[i][t][k];
                equation[i][channels] += x * w.target[t][k];
                for (unsigned j = 0; j < channels; ++j)
                    equation[i][j] += x * w.carrier[j][t][k];
            }
    const auto gram = equation;
    auto reduced = equation;
    double trace = 0;
    for (unsigned i = 0; i < channels; ++i) trace += equation[i][i];
    for (unsigned i = 0; i < channels; ++i) reduced[i][i] += trace * 1e-12 + 1e-30;
    for (unsigned i = 0; i < channels; ++i) if (native.coefficients[i] == 0) {
        for (unsigned j = 0; j < channels; ++j) reduced[i][j] = reduced[j][i] = 0;
        reduced[i][i] = 1;
        reduced[i][channels] = 0;
    }
    bool solvable = true;
    for (unsigned i = 0; i < channels; ++i) {
        unsigned pivot = i;
        for (unsigned j = i + 1; j < channels; ++j)
            if (std::abs(reduced[j][i]) > std::abs(reduced[pivot][i])) pivot = j;
        std::swap(reduced[i], reduced[pivot]);
        const double diagonal = reduced[i][i];
        if (std::abs(diagonal) < 1e-30) { solvable = false; break; }
        for (unsigned k = i; k <= channels; ++k) reduced[i][k] /= diagonal;
        for (unsigned j = 0; j < channels; ++j) if (j != i) {
            const double multiplier = reduced[j][i];
            for (unsigned k = i; k <= channels; ++k) reduced[j][k] -= multiplier * reduced[i][k];
        }
    }
    constexpr double step = 205.0 / 2048;
    using Codes = std::array<int, channels>;
    const auto code = [](double coefficient) {
        return static_cast<int>(std::clamp(std::nearbyint(coefficient / step), -96.0, 95.0));
    };
    const auto cost = [&](const Codes& codes) {
        double result = 0;
        for (unsigned i = 0; i < channels; ++i) {
            const double a = codes[i] * step;
            result -= 2 * a * gram[i][channels];
            for (unsigned j = 0; j < channels; ++j) result += a * gram[i][j] * codes[j] * step;
        }
        return result;
    };
    Codes best{}, least_squares{};
    for (unsigned i = 0; i < channels; ++i) {
        best[i] = quantized_code(native.coefficients[i], true) - 96;
        least_squares[i] = solvable ? code(reduced[i][channels]) : 0;
    }
    if (cost(Codes{}) < cost(best)) best = {};
    if (cost(least_squares) < cost(best)) best = least_squares;
    for (unsigned pass = 0; pass < 12; ++pass) for (unsigned i = 0; i < channels; ++i) {
        if (native.coefficients[i] == 0 || gram[i][i] < 1e-30) continue;
        double rhs = gram[i][channels];
        for (unsigned j = 0; j < channels; ++j) if (i != j) rhs -= gram[i][j] * best[j] * step;
        auto candidate = best;
        candidate[i] = code(rhs / gram[i][i]);
        if (cost(candidate) < cost(best)) best = candidate;
    }
    for (unsigned i = 0; i < channels; ++i)
        native.coefficients[i] = static_cast<float>(best[i] * 205.0 / 32768);
    return native;
}

} // namespace atmos::encode::joc
