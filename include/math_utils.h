#pragma once
#include "headers.h"

namespace PhysicsEngine::MathUtils {

template <typename... Args> inline float NORM(Args... args) {
    static_assert((std::is_arithmetic_v<Args> && ...),
                  "make the args numeric dumbass");
    return std::sqrt((static_cast<float>(args * args) + ...));
}
} // namespace PhysicsEngine::MathUtils
