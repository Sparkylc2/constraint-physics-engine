#pragma once
#include "aabb.h"
#include "rigid_body.h"

namespace PhysicsEngine::Collisions {

struct CollisionPair {
    std::size_t body_a, body_b;
};

// computes aabbs, and tests all non-static pairs for overlap
// then returns candidate pairs for narrowphase

std::vector<CollisionPair>
broadphase_bruteforce(const std::vector<RigidBody> &bodies);

} // namespace PhysicsEngine::Collisions
