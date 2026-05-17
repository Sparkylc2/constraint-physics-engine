#include "collision.h"
#include "mat.h"
#include "vec3.h"

namespace PhysicsEngine::Collisions {

ContactManifold box_plane(const RigidBody &plane, std::size_t plane_idx,
                          const RigidBody &box, std::size_t box_idx) {
    // body 1 is ground, body 2 is the box
    Vec3 b1_normal = {0.0f, 1.0f,
                      0.0f}; // simplified so normal is just straight up

    Vec3 b1_position = plane.position;

    // body 2 info
    Mat<3, 3> b2_rotation_matrix = box.rotation_matrix();
    Vec3 b2_position = box.position;

    Vec3 b2_local_vertices[8]; // todo: maybe store this in the shape object
                               // itself so we dont recompute

    box.shape.box.get_vertices(b2_local_vertices);

    Vec3 b2_world_vertices[8];
    for (std::size_t i = 0; i < 8; i++) {
        b2_world_vertices[i] =
            b2_position + b2_rotation_matrix * b2_local_vertices[i];
    }

    const float plane_y = vy(b1_position) + vy(plane.shape.box.half_extents);

    ContactManifold contact_manifold = {plane_idx, box_idx};
    // find the contact point
    for (std::size_t i = 0; i < 8; i++) {
        float pen = vy(b2_world_vertices[i]) - plane_y;
        if (pen > 0.0f)
            continue; // no penetration

        const Vec3 r2_world = b2_world_vertices[i] - b2_position;
        const Vec3 r1_world = make_vec3(vx(b2_world_vertices[i]), plane_y,
                                        vz(b2_world_vertices[i])) -
                              b1_position;
        const Vec3 normal = b1_normal;
        const float penetration = pen;
        const uint32_t id = i;

        ContactPoint point = {r1_world, r2_world, normal, penetration, id};
        contact_manifold.points[contact_manifold.num_points] = point;
        contact_manifold.num_points++;
    }
    return contact_manifold;
}
} // namespace PhysicsEngine::Collisions
