#pragma once
#include "headers.h"

namespace PhysicsEngine::MathUtils {

template <typename... Args>
inline auto NORM(Args... args)
    -> std::enable_if_t<(std::is_arithmetic_v<Args> && ...), float> {
    return std::sqrt((static_cast<float>(args * args) + ...));
}

// c-style fixed arrays
template <std::size_t N> inline float NORM(const float (&arr)[N]) {
    float sum = 0.0f;
    for (std::size_t i = 0; i < N; i++)
        sum += arr[i] * arr[i];
    return std::sqrt(sum);
}

// anything with a .data_ C-array member (Mat<R,C>, Quaternion, etc.)
template <typename T> inline auto NORM(const T &v) -> decltype(NORM(v.data_)) {
    return NORM(v.data_);
}

inline bool approx_zero(float val, float eps = 1e-7f) {
    return std::abs(val) < eps;
}

inline bool approx_equal(float a, float b, float eps = 1e-7f) {
    const float diff = std::abs(a - b);
    const float largest = std::max(std::abs(a), std::abs(b));

    // absolute check near zero, relative check for larger values
    return diff <= std::max(eps, largest * eps);
}

} // namespace PhysicsEngine::MathUtils
