#pragma once
#include "constraint.h"
#include "raylib.h"

namespace PhysicsEngine::Constraints {

enum class DistanceType { Spring, Damper, Rod };

struct DistanceConstraint : public Constraint {
    Vec3 local_anchor1; // attachment in body1 local frame
    Vec3 local_anchor2; // attachment in body2 local frame
    float rest_length;
    DistanceType type;

    float stiffness = 20.0f; // spring ks
    float damping = 5.0f;    // damper kd

    DistanceConstraint(std::size_t b1, std::size_t b2, const Vec3 &a1,
                       const Vec3 &a2, float rest, DistanceType t)
        : local_anchor1(a1), local_anchor2(a2), rest_length(rest), type(t) {
        body1 = b1;
        body2 = b2;
    }

    std::size_t num_rows() const override { return 1; }
    std::size_t prepare(ConstraintRow *out,
                        const std::vector<RigidBody> &bodies,
                        const SolverSettings &settings) override;

    // world-space endpoints for drawing
    void endpoints(const std::vector<RigidBody> &bodies, Vec3 &p1,
                   Vec3 &p2) const;

    Color colour() const;
    const char *label() const;
};

} // namespace PhysicsEngine::Constraints
