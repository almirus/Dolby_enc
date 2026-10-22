// Recovered temporal analysis and two-frame correlation preparation.
#pragma once
#include "joc_fit.hpp"

namespace atmos::encode::joc {
using Header = std::array<unsigned, 4>;
using History = std::array<float, 3072>;

struct HistoryDelay {
    // Native processor aligns gain history by 18 slots before temporal screening.
    std::array<float, 18 * 128> tail{};
    History advance(const History& current) {
        History result{};
        std::copy(tail.begin(), tail.end(), result.begin());
        std::copy_n(current.begin(), 6 * 128, result.begin() + tail.size());
        std::copy(current.begin() + 6 * 128, current.end(), tail.begin());
        return result;
    }
};

// Fixed recovered profile: 24 slots,first energy band5,one point per frame.
struct TemporalDetector {
    Header current{0, 1, 23, 0};
    bool initial = true;
    std::vector<std::array<float, 16>> history;
    std::vector<unsigned> cooldown;
    float floor;

    explicit TemporalDetector(unsigned objects, float energy_floor = 2.5599999275982555e-7F)
        : history(objects), cooldown(objects), floor(energy_floor) {
        if (!objects || objects > 16) throw std::runtime_error("invalid detector object count");
        if (!std::isfinite(floor) || floor < 0) throw std::runtime_error("invalid detector energy floor");
    }

    Header detect(const std::vector<std::vector<Spectrum>>& objects) {
        if (objects.size() != history.size()) throw std::runtime_error("detector object count changed");
        std::vector<std::array<float, 24>> energy(objects.size());
        for (unsigned channel = 0; channel < objects.size(); ++channel) {
            if (objects[channel].size() != 24) throw std::runtime_error("invalid detector slots");
            for (unsigned t = 0; t < 24; ++t) {
                float value = 0;
                for (unsigned k = 10; k < 128; ++k) {
                    const float sample = objects[channel][t][k];
                    if (!std::isfinite(sample)) throw std::runtime_error("nonfinite detector input");
                    value += (sample * sample) * 0.0078125F;
                }
                if (!std::isfinite(value)) throw std::runtime_error("nonfinite detector energy");
                energy[channel][t] = value;
            }
        }
        const Header previous = current;
        current[0] = current[1] = 0;
        const float threshold = initial ? 0 : floor;
        initial = false;
        const auto level = [&](unsigned channel, unsigned time) {
            return time < 16 ? history[channel][time] : energy[channel][time - 16];
        };
        for (unsigned time = 16; time < 40 && !current[1]; ++time) {
            for (unsigned channel = 0; channel < objects.size(); ++channel) {
                if (time < cooldown[channel] + 8) continue;
                const float value = level(channel, time);
                if (value < threshold) continue;
                float average = 0;
                for (unsigned earlier = time - 16; earlier < time; ++earlier)
                    average += level(channel, earlier) * 0.0625F;
                if (value * 0.11111111F > average) {
                    cooldown[channel] = time;
                    current[0] = current[1] = 1;
                    current[2] = time - 16;
                    break;
                }
            }
        }
        if (!current[0]) { current[1] = 1; current[2] = 23; }
        for (unsigned channel = 0; channel < objects.size(); ++channel) {
            cooldown[channel] = cooldown[channel] > 24 ? cooldown[channel] - 24 : 0;
            const unsigned start = cooldown[channel] > 8 ? cooldown[channel] - 8 : 0;
            for (unsigned t = start; t < 16; ++t) history[channel][t] = energy[channel][t + 8];
        }
        return previous;
    }
};

inline std::vector<Mask> seed_masks(const History& previous, const History& current,
    const std::array<float, 48>& weights, unsigned objects, unsigned channels) {
    if (!objects || objects > 16 || (channels != 6 && channels != 8))
        throw std::runtime_error("invalid seed dimensions");
    const auto finite = [](const auto& values) {
        return std::all_of(values.begin(), values.end(), [](float x) { return std::isfinite(x); });
    };
    if (!finite(previous) || !finite(current) || !finite(weights) ||
        std::any_of(weights.begin(), weights.end(), [](float x) { return x < 0 || x > 1; }))
        throw std::runtime_error("invalid seed history/weights");
    std::vector<Mask> result(objects);
    for (unsigned object = 0; object < objects; ++object) {
        std::array<float, 8> levels{};
        float maximum = 0;
        for (unsigned channel = 0; channel < channels; ++channel) {
            float value = 0;
            for (unsigned t = 0; t < 24; ++t) {
                const unsigned index = 128 * t + 8 * object + channel;
                value += previous[index] * weights[t];
                value += current[index] * weights[t + 24];
            }
            if (!std::isfinite(value)) throw std::runtime_error("nonfinite seed level");
            levels[channel] = value;
            maximum = std::max(maximum, value);
        }
        const float threshold = maximum * 0.0625F;
        for (unsigned channel = 0; channel < channels; ++channel)
            if (channel != 3 && levels[channel] >= threshold)
                result[object][channel < 3 ? channel : channel - 1] = 1;
    }
    return result;
}

inline std::array<float, 48> temporal_weights(const Header& previous, const Header& current, unsigned point) {
    const auto valid = [](const Header& h) {
        return h[0] <= 1 && (h[1] == 1 || h[1] == 2) && h[2] < 24 &&
            (h[1] == 1 || (h[3] < 24 && h[2] < h[3]));
    };
    if (!valid(previous) || !valid(current) || point >= previous[1])
        throw std::runtime_error("invalid temporal header");
    std::array<float, 24> full{};
    std::array<float, 12> half{};
    float value = 0;
    const float step = float(1.0 / 24.0);
    for (unsigned i = 0; i < 23; ++i) full[i] = value += step;
    full.back() = 1;
    value = 0;
    for (unsigned i = 0; i < 11; ++i) half[i] = value += step * 2;
    half.back() = 1;
    std::array<float, 48> result{};
    for (unsigned t = 0; t < 24; ++t) {
        if (point) {
            result[t] = previous[0] ? float(t >= previous[3]) : (t < 12 ? 0 : half[t - 12]);
        } else if (previous[0]) {
            result[t] = float(t >= previous[2] && (previous[1] == 1 || t < previous[3]));
        } else {
            result[t] = previous[1] == 1 ? full[t] : (t < 12 ? half[t] : (t < 23 ? half[22 - t] : 0));
        }
        if (!point && previous[1] == 2) continue;
        if (current[0]) result[t + 24] = float(t < current[2]);
        else if (current[1] == 1) result[t + 24] = t < 23 ? full[22 - t] : 0;
        else result[t + 24] = t < 11 ? half[10 - t] : 0;
    }
    return result;
}

inline void weight(Window& w, const std::vector<float>& weights) {
    if (!w.channels || w.channels > 7 || w.begin >= w.end || w.end > 64 ||
        weights.empty() || weights.size() > 48 || w.target.size() != weights.size())
        throw std::runtime_error("invalid preparation dimensions");
    for (unsigned i = 0; i < w.channels; ++i)
        if (w.carrier[i].size() != weights.size()) throw std::runtime_error("invalid carrier slots");
    for (unsigned t = 0; t < weights.size(); ++t) {
        if (!std::isfinite(weights[t]) || weights[t] < 0 || weights[t] > 1)
            throw std::runtime_error("invalid temporal weight");
        for (unsigned k = 0; k < 128; ++k) {
            w.target[t][k] *= weights[t];
            for (unsigned i = 0; i < w.channels; ++i) w.carrier[i][t][k] *= weights[t];
        }
    }
}

inline void correlations(const Window& w, Matrix& gram, Row& cross) {
    gram = {};
    cross = {};
    for (unsigned i = 0; i < w.channels; ++i) {
        gram[i][i] = 1e-20F;
        for (unsigned k = w.begin; k < w.end; ++k) {
            float sum = 0;
            for (unsigned t = 0; t < w.target.size(); ++t) {
                sum += w.target[t][2 * k] * w.carrier[i][t][2 * k];
                sum += w.target[t][2 * k + 1] * w.carrier[i][t][2 * k + 1];
            }
            cross[i] += sum;
            for (unsigned j = i; j < w.channels; ++j) {
                sum = 0;
                for (unsigned t = 0; t < w.target.size(); ++t) {
                    sum += w.carrier[i][t][2 * k] * w.carrier[j][t][2 * k];
                    sum += w.carrier[i][t][2 * k + 1] * w.carrier[j][t][2 * k + 1];
                }
                gram[i][j] += sum;
                gram[j][i] = gram[i][j];
            }
        }
    }
}

inline void prepare(Window& w, const std::vector<float>& weights, Matrix& gram, Row& cross) {
    weight(w, weights);
    correlations(w, gram, cross);
}
} // namespace atmos::encode::joc
