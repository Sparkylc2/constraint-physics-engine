#pragma once
#include "vec3.h"

namespace PhysicsEngine {

struct SolverSettings {
    float dt;
    float beta;
    Vec3 gravity = {0.0f, -9.81f, 0.0f};
    float mu = 0.1f;
};

} // namespace PhysicsEngine
