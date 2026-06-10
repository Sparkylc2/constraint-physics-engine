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
    ContactPoint points[8]; // technically just the box for now, can have up to
                            // 8 contact points
    std::size_t num_points = 0; // num contact points

    inline void to_string() {
        std::cout << "body 1 idx: " << body1 << std::endl;
        std::cout << "body 2 idx: " << body2 << std::endl;
        std::cout << "num_points: " << num_points << std::endl;
    };
};

ContactManifold collide(const std::vector<RigidBody> bodies,
                        std::size_t body_1_idx, std::size_t body_2_idx);

ContactManifold box_plane(const RigidBody &plane, std::size_t plane_idx,
                          const RigidBody &box, std::size_t box_idx);

ContactManifold sphere_plane(const RigidBody &plane, std::size_t plane_idx,
                             const RigidBody &sphere, std::size_t sphere_idx);

ContactManifold box_sphere(const RigidBody &box, std::size_t box_idx,
                           const RigidBody &sphere, std::size_t sphere_idx);

ContactManifold sphere_sphere(const RigidBody &sphere_1,
                              std::size_t sphere_1_idx,
                              const RigidBody &sphere_2,
                              std::size_t sphere_2_idx);

ContactManifold box_box(const RigidBody &box_1, std::size_t box_1_idx,
                        const RigidBody &box_2, std::size_t box_2_idx);

ContactManifold polygon_sphere(const RigidBody &polygon,
                               std::size_t polygon_idx, const RigidBody &sphere,
                               std::size_t sphere_idx);

ContactManifold polygon_box(const RigidBody &polygon, std::size_t polygon_idx,
                            const RigidBody &box, std::size_t box_idx);

ContactManifold polygon_polygon(const RigidBody &body_1, std::size_t body_1_idx,
                                const RigidBody &body_2,
                                std::size_t body_2_idx);
} // namespace PhysicsEngine::Collisions
