#pragma once
#include "collision.h"
#include "rigid_body.h"
#include "vec3.h"

namespace PhysicsEngine::Collisions {

// returns the number of contact points written
std::size_t clip_manifold(const RigidBody &a, std::size_t a_idx,
                          const RigidBody &b, std::size_t b_idx,
                          const Vec3 &epa_normal, float epa_depth,
                          ContactManifold &manifold);

} // namespace PhysicsEngine::Collisions
