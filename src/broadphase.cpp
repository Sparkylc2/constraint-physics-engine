#include "broadphase.h"

namespace PhysicsEngine::Collisions {
std::vector<CollisionPair>
broadphase_bruteforce(const std::vector<RigidBody> &bodies) {

    std::vector<CollisionPair> pairs;

    // aabbs computed upfront
    // todo: moving this so that each body has it's own aabb
    // which just gets bounds refreshed, would be more efficient
    std::vector<AABB> aabbs(bodies.size());
    for (std::size_t i = 0; i < bodies.size(); i++) {
        aabbs[i] = AABB::from_body(bodies[i]);
    }

    // go through all unique pairs
    for (std::size_t i = 0; i < bodies.size(); i++) {
        for (std::size_t j = i + 1; j < bodies.size(); j++) {

            if (bodies[i].is_static() && bodies[j].is_static()) {
                continue;
            }

            //
            if (aabbs[i].overlaps(aabbs[j])) {

                pairs.push_back({i, j});
            }
        }
    }
    return pairs;
};
} // namespace PhysicsEngine::Collisions
