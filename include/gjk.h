#pragma once
#include "collision.h"
#include "rigid_body.h"
#include "vec3.h"

namespace PhysicsEngine::Collisions::GJK {
// the support function for a single convex shape in world space
// retuns the point on the shape furthest from the given direction

Vec3 support(const RigidBody &body, const Vec3 &dir);

// general convex-convex collisions using gjk for the intersection test
// and epa for penetration depth.

ContactManifold convex_convex(const RigidBody &a, std::size_t a_idx,
                              const RigidBody &b, std::size_t b_idx);
} // namespace PhysicsEngine::Collisions::GJK
