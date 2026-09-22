#pragma once
#include "rigid_body.h"
#include "vec3.h"
#include <vector>

namespace PhysicsEngine {

struct RayHit {
    bool hit = false;
    int body_idx = -1;
    float t = 1e30f;
    Vec3 world_point = Vec3::zeros();
    Vec3 local_point = Vec3::zeros(); // in body's local frame
    Vec3 normal = Vec3::zeros();      // surface normal at hit
};

// closest hit across all bodies
RayHit raycast(const Vec3 &origin, const Vec3 &direction,
               const std::vector<RigidBody> &bodies);

// individual shape tests, return t parameter, negative if miss
float ray_box(const Vec3 &origin, const Vec3 &dir, const RigidBody &body);
float ray_sphere(const Vec3 &origin, const Vec3 &dir, const RigidBody &body);

} // namespace PhysicsEngine
