// Fixed64-band DEE analysis: preserve polyphase, radix4 FFT and table rounding.
#pragma once
#include "eac3/joc_tables.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <span>
#include <stdexcept>

namespace atmos::encode::joc {
namespace qmf_detail {
struct Complex { float r, i; };

inline std::array<Complex, 4> butterfly(Complex a, Complex b, Complex c, Complex d) {
    const float ac_r = c.r + a.r, ac_i = c.i + a.i;
    const float bd_r = d.r + b.r, bd_i = d.i + b.i;
    const float amc_r = a.r - c.r, amc_i = a.i - c.i;
    const float bmd_r = b.r - d.r, bmd_i = b.i - d.i;
    return {{{bd_r + ac_r, bd_i + ac_i}, {ac_r - bd_r, ac_i - bd_i},
             {bmd_i + amc_r, amc_i - bmd_r}, {amc_r - bmd_i, bmd_r + amc_i}}};
}

inline constexpr std::array<float, 128> modulation{
    -0.F,0.5F,0.0122706145F,0.499849409F,0.024533838F,0.499397725F,0.0367822833F,0.498645216F,
    0.0490085706F,0.49759236F,0.0612053387F,0.496239781F,0.0733652338F,0.494588256F,0.0854809433F,0.492638826F,
    0.0975451618F,0.490392625F,0.109550618F,0.487851053F,0.121490091F,0.485015631F,0.133356377F,0.481888026F,
    0.145142332F,0.478470176F,0.156840876F,0.474764079F,0.168444932F,0.470772028F,0.179947525F,0.466496408F,
    0.191341713F,0.461939752F,0.202620655F,0.457104892F,0.213777542F,0.451994658F,0.224805668F,0.446612149F,
    0.235698372F,0.440960646F,0.246449098F,0.435043484F,0.257051378F,0.4288643F,0.267498821F,0.42242679F,
    0.277785122F,0.415734798F,0.287904084F,0.408792406F,0.297849655F,0.401603758F,0.307615787F,0.394173205F,
    0.317196637F,0.386505216F,0.326586425F,0.378604412F,0.335779488F,0.37047556F,0.344770283F,0.362123549F,
    0.353553385F,0.353553385F,0.362123549F,0.344770283F,0.37047556F,0.335779488F,0.378604412F,0.326586425F,
    0.386505216F,0.317196637F,0.394173205F,0.307615787F,0.401603758F,0.297849655F,0.408792406F,0.287904084F,
    0.415734798F,0.277785122F,0.42242679F,0.267498821F,0.4288643F,0.257051378F,0.435043484F,0.246449098F,
    0.440960646F,0.235698372F,0.446612149F,0.224805668F,0.451994658F,0.213777542F,0.457104892F,0.202620655F,
    0.461939752F,0.191341713F,0.466496408F,0.179947525F,0.470772028F,0.168444932F,0.474764079F,0.156840876F,
    0.478470176F,0.145142332F,0.481888026F,0.133356377F,0.485015631F,0.121490091F,0.487851053F,0.109550618F,
    0.490392625F,0.0975451618F,0.492638826F,0.0854809433F,0.494588256F,0.0733652338F,0.496239781F,0.0612053387F,
    0.49759236F,0.0490085706F,0.498645216F,0.0367822833F,0.499397725F,0.024533838F,0.499849409F,0.0122706145F,
};
inline constexpr std::array<std::array<float, 16>, 6> twiddles{{
  {{
    1.F,0.707106769F,0.923879504F,0.382683426F,0.980785251F,0.555570245F,0.831469595F,0.195090324F,
    0.99518472F,0.634393275F,0.881921291F,0.290284663F,0.956940353F,0.471396744F,0.773010433F,0.0980171412F,
  }},
  {{
    0.F,-0.707106769F,-0.382683426F,-0.923879504F,-0.195090324F,-0.831469595F,-0.555570245F,-0.980785251F,
    -0.0980171412F,-0.773010433F,-0.471396744F,-0.956940353F,-0.290284663F,-0.881921291F,-0.634393275F,-0.99518472F,
  }},
  {{
    1.F,0.F,0.707106769F,-0.707106769F,0.923879504F,-0.382683426F,0.382683426F,-0.923879504F,
    0.980785251F,-0.195090324F,0.555570245F,-0.831469595F,0.831469595F,-0.555570245F,0.195090324F,-0.980785251F,
  }},
  {{
    0.F,-1.F,-0.707106769F,-0.707106769F,-0.382683426F,-0.923879504F,-0.923879504F,-0.382683426F,
    -0.195090324F,-0.980785251F,-0.831469595F,-0.555570245F,-0.555570245F,-0.831469595F,-0.980785251F,-0.195090324F,
  }},
  {{
    1.F,-0.707106769F,0.382683426F,-0.923879504F,0.831469595F,-0.980785251F,-0.195090324F,-0.555570245F,
    0.956940353F,-0.881921291F,0.0980171412F,-0.773010433F,0.634393275F,-0.99518472F,-0.471396744F,-0.290284663F,
  }},
  {{
    0.F,-0.707106769F,-0.923879504F,0.382683426F,-0.555570245F,-0.195090324F,-0.980785251F,0.831469595F,
    -0.290284663F,-0.471396744F,-0.99518472F,0.634393275F,-0.773010433F,0.0980171412F,-0.881921291F,0.956940353F,
  }}
}};

inline Complex rotate(Complex value, unsigned table, unsigned index) {
    const float c = twiddles[table][index], s = twiddles[table + 1][index];
    return {value.r * c - value.i * s, value.i * c + value.r * s};
}

inline std::array<float, 128> fft64(std::array<float, 128> work) {
    const auto original = work;
    for (unsigned k = 0; k < 16; ++k) {
        std::array<Complex, 4> input{};
        for (unsigned g = 0; g < 4; ++g)
            input[g] = {original[32 * g + 2 * k] * .015625F, original[32 * g + 2 * k + 1] * .015625F};
        const auto output = butterfly(input[0], input[1], input[2], input[3]);
        for (unsigned g = 0; g < 4; ++g) {
            work[32 * g + (k / 4) * 8 + k % 4] = output[g].r;
            work[32 * g + (k / 4) * 8 + k % 4 + 4] = output[g].i;
        }
    }
    for (unsigned block = 0; block < 4; ++block) {
        for (unsigned lane = 0; lane < 4; ++lane) {
            const unsigned base = block * 32 + lane;
            std::array<Complex, 4> input{};
            for (unsigned g = 0; g < 4; ++g) input[g] = {work[base + 8 * g], work[base + 8 * g + 4]};
            if (block) for (unsigned g = 1; g < 4; ++g) input[g] = rotate(input[g], 2 * (g - 1), block);
            const auto output = butterfly(input[0], input[1], input[2], input[3]);
            for (unsigned g = 0; g < 4; ++g) {
                work[base + 8 * g] = output[g].r;
                work[base + 8 * g + 4] = output[g].i;
            }
        }
    }
    std::array<float, 128> result{};
    constexpr unsigned permutation[] = {0, 4, 2, 6}, groups[] = {0, 2, 1, 3};
    for (unsigned block = 0; block < 4; ++block) {
        for (unsigned lane = 0; lane < 4; ++lane) {
            const unsigned base = block * 32 + lane * 8;
            std::array<Complex, 4> input{};
            for (unsigned g = 0; g < 4; ++g) input[g] = {work[base + g], work[base + g + 4]};
            for (unsigned g = 1; g < 4; ++g) input[g] = rotate(input[g], 2 * (g - 1), block * 4 + lane);
            const auto output = butterfly(input[0], input[1], input[2], input[3]);
            const unsigned offset = permutation[block] + (lane % 2) * 16 + (lane / 2) * 8;
            for (unsigned g = 0; g < 4; ++g) {
                result[offset + 32 * groups[g]] = output[g].r;
                result[offset + 32 * groups[g] + 1] = output[g].i;
            }
        }
    }
    return result;
}
} // namespace qmf_detail

class Analysis final {
public:
    std::array<float, 128> forward(std::span<const float, 64> input) {
        if (!std::all_of(input.begin(), input.end(), [](float x) { return std::isfinite(x); }))
            throw std::runtime_error("nonfinite JOC QMF input");
        std::array<float, 128> polyphase{};
        for (unsigned t = 0; t < 64; ++t) {
            float even = history_[8 * 64 + t] * coefficient(t) + history_[6 * 64 + t] * coefficient(2 * 64 + t);
            even += history_[4 * 64 + t] * coefficient(4 * 64 + t);
            even += history_[2 * 64 + t] * coefficient(6 * 64 + t);
            even += history_[t] * coefficient(8 * 64 + t);
            float odd = history_[7 * 64 + t] * coefficient(64 + t) + history_[5 * 64 + t] * coefficient(3 * 64 + t);
            odd += history_[3 * 64 + t] * coefficient(5 * 64 + t);
            odd += history_[64 + t] * coefficient(7 * 64 + t);
            odd += input[t] * coefficient(9 * 64 + t);
            polyphase[2 * (63 - t)] = odd;
            polyphase[2 * (63 - t) + 1] = even;
        }
        std::move_backward(history_.begin(), history_.end() - 64, history_.end());
        std::copy(input.begin(), input.end(), history_.begin());
        std::array<float, 128> rotated{};
        for (unsigned k = 0; k < 64; ++k) {
            const float r = polyphase[2 * k], i = polyphase[2 * k + 1];
            const float c = qmf_detail::modulation[2 * k], s = qmf_detail::modulation[2 * k + 1];
            rotated[2 * k] = i * c - r * s;
            rotated[2 * k + 1] = i * s + r * c;
        }
        const auto transformed = qmf_detail::fft64(rotated);
        std::array<float, 128> result{};
        for (unsigned k = 0; k < 32; ++k) {
            result[4 * k] = transformed[2 * k];
            result[4 * k + 1] = -transformed[2 * k + 1];
            result[4 * k + 2] = transformed[126 - 2 * k];
            result[4 * k + 3] = transformed[127 - 2 * k];
        }
        return result;
    }
private:
    static float coefficient(unsigned index) {
        static constexpr auto table = [] {
            std::array<float, 640> values{};
            for (unsigned i = 0; i < values.size(); ++i) {
                const float value = eac3::joc_tables::qmf_coefficients[(i + 1) % 640];
                values[i] = i < 512 && i % 128 == 127 ? -value : value;
            }
            return values;
        }();
        return table[index];
    }
    std::array<float, 9 * 64> history_{};
};
} // namespace atmos::encode::joc
