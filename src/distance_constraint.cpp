#include "distance_constraint.h"
#include "math_utils.h"
#include <cmath>

namespace PhysicsEngine::Constraints {

std::size_t DistanceConstraint::prepare(ConstraintRow *out,
                                        const std::vector<RigidBody> &bodies,
                                        const SolverSettings &settings) {
    // world-space attachment points
    const Mat<3, 3> R1 = bodies[body1].rotation_matrix();
    const Mat<3, 3> R2 = bodies[body2].rotation_matrix();

    Vec3 p1 = bodies[body1].position + R1 * local_anchor1;
    Vec3 p2 = bodies[body2].position + R2 * local_anchor2;

    Vec3 delta = p2 - p1;
    float dist = length(delta);
    Vec3 d = (dist > 1e-6f) ? delta * (1.0f / dist) : make_vec3(0, 1, 0);

    float C = dist - rest_length;

    // r vectors from body centres to attachment points
    Vec3 r1 = R1 * local_anchor1;
    Vec3 r2 = R2 * local_anchor2;

    ConstraintRow &row = out[0];
    row.J1_linear = -d;
    row.J2_linear = d;
    row.J1_angular = -cross(r1, d);
    row.J2_angular = cross(r2, d);

    row.body1 = body1;
    row.body2 = body2;

    // can push or pull
    row.lambda_min = -std::numeric_limits<float>::max();
    row.lambda_max = std::numeric_limits<float>::max();

    switch (type) {
    case DistanceType::Spring:
        // soft position correction proportional to displacement
        row.zeta = -stiffness * C;
        break;
    case DistanceType::Damper:
        // velocity damping via Jv term
        row.zeta = 0.0f;
        break;
    case DistanceType::Rod:
        // baumgarte correction
        row.zeta = -settings.beta * C;
        break;
    }

    row.precompute(bodies, settings);
    return 1;
}

void DistanceConstraint::endpoints(const std::vector<RigidBody> &bodies,
                                   Vec3 &p1, Vec3 &p2) const {
    p1 = bodies[body1].position +
         bodies[body1].rotation_matrix() * local_anchor1;
    p2 = bodies[body2].position +
         bodies[body2].rotation_matrix() * local_anchor2;
}

Color DistanceConstraint::colour() const {
    switch (type) {
    case DistanceType::Spring:
        return YELLOW;
    case DistanceType::Damper:
        return SKYBLUE;
    case DistanceType::Rod:
        return WHITE;
    }
    return WHITE;
}

const char *DistanceConstraint::label() const {
    switch (type) {
    case DistanceType::Spring:
        return "Spring";
    case DistanceType::Damper:
        return "Damper";
    case DistanceType::Rod:
        return "Rod";
    }
    return "?";
}

} // namespace PhysicsEngine::Constraints
