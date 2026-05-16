#pragma once
#include "quaternion.h"
#include "shape.h"
#include "vec3.h"

namespace PhysicsEngine {

struct RigidBody {
    // -- stattttee ---
    Vec3 position;
    Quaternion orientation;

    Vec3 linear_velocity;
    Vec3 angular_velocity;

    // --- shapess ---
    Shape shape;

    // --- mass shit ---
    float mass;
    float inv_mass;             // which is 0 for static/kinematic
    Mat<3, 3> inv_inertia_body; // body-space inertia tensor (constant)

    // --- derived each frame ---
    Mat<3, 3> inv_inertia_world; // R * inv_inertia_body * R_transpose

    // --- force accumulators ---
    Vec3 force;
    Vec3 torque;

    // updates inv_inertia_world from current orientation
    void update_inertia();

    void apply_force(const Vec3 &f);
    void apply_force_at(const Vec3 &f, const Vec3 &world_point);
    void clear_accumulators();

    // integrate velocity -> position (eq 36-37 from paper)
    void integrate_position(float dt);

    // the 3x3 rotation matrix from current orientation
    Mat<3, 3> rotation_matrix() const;

    // basically == 0.0f
    bool is_static() const { return inv_mass <= 1e-10; }

    // factory
    static RigidBody create_dynamic(const Shape &shape, float mass,
                                    const Vec3 &pos);
    static RigidBody create_static(const Shape &shape, const Vec3 &pos);
};
} // namespace PhysicsEngine
