#pragma once
#include "rigid_body.h"

namespace PhysicsEngine::Collisions {

struct AABB {
    Vec3 min, max;

    bool overlaps(const AABB &other) const;

    // from body's current shape + transform
    static AABB from_body(const RigidBody &body);
};

} // namespace PhysicsEngine::Collisions
