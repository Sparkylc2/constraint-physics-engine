#pragma once
#include "rigid_body.h"
#include "vec3.h"

namespace PhysicsEngine::Collisions {

struct ContactPoint {
    Vec3 r1_world;     // body 1 com to contact point
    Vec3 r2_world;     // body 2 com to contact point
    Vec3 normal;       // contact normal
    float penetration; // contact penetration
    uint32_t id;       // for contact caching
};

struct ContactManifold {
    std::size_t body1, body2; // body indices
    ContactPoint points[8]; // just box for now, can have up to 8 contact points
    std::size_t num_points = 0; // num contact points
};

// todo: replace later with a dispatch function checking shapetype
ContactManifold box_plane(const RigidBody &plane, std::size_t plane_idx,
                          const RigidBody &box, std::size_t box_idx);
} // namespace PhysicsEngine::Collisions
