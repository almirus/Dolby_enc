// Native-compatible inverse surround phase compensation in the 64-band QMF domain.
#pragma once
#include "qmf.hpp"
#include <array>
#include <utility>

namespace atmos::eac3::qmf {
class SurroundPhase final {
public:
    ComplexBands process(const ComplexBands& input, bool compensate) {
        auto output = delay_[cursor_];
        delay_[cursor_] = input;
        cursor_ = (cursor_ + 1) % delay_.size();
        low_[20] = {input.real[0], input.imaginary[0]};
        if (compensate) {
            for (unsigned band = 0; band < subbands; ++band) {
                const float real = output.real[band];
                output.real[band] = output.imaginary[band];
                output.imaginary[band] = -real;
            }
            float real = 0, imaginary = 0;
            for (unsigned tap = 0; tap < low_.size(); ++tap) {
                real = (real_coefficients_[tap] * low_[tap].first + real)
                    - imaginary_coefficients_[tap] * low_[tap].second;
                imaginary = (real_coefficients_[tap] * low_[tap].second + imaginary)
                    + imaginary_coefficients_[tap] * low_[tap].first;
            }
            output.real[0] = real * 2;
            output.imaginary[0] = imaginary * 2;
        }
        std::move(low_.begin() + 1, low_.end(), low_.begin());
        return output;
    }

private:
    std::array<ComplexBands, 10> delay_{};
    std::array<std::pair<float, float>, 21> low_{};
    unsigned cursor_ = 0;
    static constexpr std::array<float, 21> imaginary_coefficients_{
        -0.000624255044F, -0.00192346866F, -0.00426546484F, -0.00816830806F,
        -0.0143272011F, -0.0237598605F, -0.037572328F, -0.0557756945F,
        -0.0756827667F, -0.0917247236F, -0.597937405F, -0.0917247236F,
        -0.0756827667F, -0.0557756945F, -0.037572328F, -0.0237598605F,
        -0.0143272011F, -0.00816830806F, -0.00426546484F, -0.00192346866F,
        -0.000624255044F,
    };
    static constexpr std::array<float, 21> real_coefficients_{
        0.00139960402F, 0.00383915077F, 0.00751264254F, 0.0124193737F,
        0.0183674283F, 0.0249701627F, 0.0316790082F, 0.0378500037F,
        0.0428341255F, 0.046075616F, 0.047200121F, 0.046075616F,
        0.0428341255F, 0.0378500037F, 0.0316790082F, 0.0249701627F,
        0.0183674283F, 0.0124193737F, 0.00751264254F, 0.00383915077F,
        0.00139960402F,
    };
};
} // namespace atmos::eac3::qmf
