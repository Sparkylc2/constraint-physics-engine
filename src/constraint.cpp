#include "constraint.h"
#include "collision.h"
#include "rigid_body.h"

namespace PhysicsEngine::Constraints {

void ConstraintRow::precompute(const std::vector<RigidBody> &bodies,
                               const SolverSettings &settings) {

    // B blocks are M^-1 * J^T, so inverse mass/inertia applied to the Jacobian
    // this is the same across all constraints so we can put it here
    // for solving the JB*lambda = eta equation (eq 34)

    // body 1
    this->B1_linear = this->J1_linear * bodies[this->body1].inv_mass;
    this->B1_angular = bodies[this->body1].inv_inertia_world * this->J1_angular;

    // body 2
    this->B2_linear = this->J2_linear * bodies[this->body2].inv_mass;
    this->B2_angular = bodies[this->body2].inv_inertia_world * this->J2_angular;

    // we compute d for the gauss seidel method, as it divides by d every
    // iteration di = Jsp(i, 1) * Bsp(1, i) + Jsp(i, 2) * Bsp(2, i)
    this->d_inv = 1.0f / (dot(this->J1_linear, this->B1_linear) +
                          dot(this->J1_angular, this->B1_angular) +
                          dot(this->J2_linear, this->B2_linear) +
                          dot(this->J2_angular, this->B2_angular));

    // compute the portion of eta thats constant across all of the constraints
    // -Jv/dt, as eta = (zeta - Jv)/dt
    const float Jv =
        dot(this->J1_linear, bodies[this->body1].linear_velocity) +
        dot(this->J1_angular, bodies[this->body1].angular_velocity) +
        dot(this->J2_linear, bodies[this->body2].linear_velocity) +
        dot(this->J2_angular, bodies[this->body2].angular_velocity);

    this->eta = (this->zeta - Jv) / settings.dt;
}

void prepare_contact_rows(const Collisions::ContactManifold &manifold,
                          const std::vector<RigidBody> &bodies,
                          const SolverSettings &settings,
                          std::vector<ConstraintRow> &rows) {

    // looping over the contact points, we create the constraints for each
    // contact point
    for (std::size_t i = 0; i < manifold.num_points; i++) {
        // we compute the jacobians for the normal and friction constraints
        // (eq 18, 23)

        Collisions::ContactPoint contact = manifold.points[i];

        ConstraintRow normal_row{};
        ConstraintRow friction_row1{};
        ConstraintRow friction_row2{};

        // the constraints all
        // normal row (eq 18)
        // paper writes jacobian as [ J1_v1 J1_w1 J2_v2 J2_w2] * [v1 w1 v2 w2]^T
        // J1_linear = J1_v1, etc
        // formula is [ -n^T -(r1 x n)^T n^T (r2 x n)^T ]
        // eta is -(beta * penetration - Jv)/dt (baumgarte correction, eq 20, v
        // is tentative velocity) and lambda bounds are [0, infty)
        //
        normal_row.J1_linear = -contact.normal;
        normal_row.J2_linear = contact.normal;
        normal_row.J1_angular = -cross(contact.r1_world, contact.normal);
        normal_row.J2_angular = cross(contact.r2_world, contact.normal);

        normal_row.lambda_min = 0.0f;
        normal_row.lambda = 0.0f;
        normal_row.lambda_max = std::numeric_limits<float>::max();

        normal_row.zeta = -settings.beta * contact.penetration;

        // add the body indices
        normal_row.body1 = manifold.body1;
        normal_row.body2 = manifold.body2;

        // updates the B blocks and d_inv
        normal_row.precompute(bodies, settings);

        // friction rows (eq 23)
        // paper writes jacobians as:
        // Ju1 = [-u1^T -(r1 x u1)^T u1^T (r2 x u1)^T]
        // Ju2 = [-u2^T -(r1 x u2)^T u2^T (r2 x u2)^T]
        // bias is 0
        // and lambda bounds are [-mc * mu * g, mc * mu * g]
        // average mass model bc why not,
        // so mc = 1.0f / (1.0/m1 + 1.0/m2) / num_points
        // todo: figure out which mass model to use for this
        // u1 and u2 are tangent directions with u1 x u2 = normal
        Vec3 reference{};
        if (std::abs(dot(contact.normal, {0.0f, 1.0f, 0.0f})) < 0.9) {
            reference = {0.0f, 1.0f, 0.0f};
        } else {
            reference = {1.0f, 0.0f, 0.0f};
        }

        const Vec3 u1 = normalise(cross(contact.normal, reference));
        const Vec3 u2 = cross(contact.normal, u1);

        friction_row1.J1_linear = -u1;
        friction_row1.J2_linear = u1;
        friction_row1.J1_angular = -cross(contact.r1_world, u1);
        friction_row1.J2_angular = cross(contact.r2_world, u1);

        friction_row2.J1_linear = -u2;
        friction_row2.J2_linear = u2;
        friction_row2.J1_angular = -cross(contact.r1_world, u2);
        friction_row2.J2_angular = cross(contact.r2_world, u2);

        // updating body indices
        friction_row1.body1 = manifold.body1;
        friction_row2.body1 = manifold.body1;
        friction_row1.body2 = manifold.body2;
        friction_row2.body2 = manifold.body2;

        friction_row1.zeta = 0.0f;
        friction_row2.zeta = 0.0f;

        // updates B blocks and d_inv again
        friction_row1.precompute(bodies, settings);
        friction_row2.precompute(bodies, settings);

        // bound setting
        const float mc = 1.0f /
                         (bodies[manifold.body1].inv_mass +
                          bodies[manifold.body2].inv_mass) /
                         manifold.num_points;

        const float lambda_min = -mc * length(settings.gravity) * settings.mu;
        const float lambda_max = -lambda_min;

        friction_row1.lambda_min = lambda_min;
        friction_row2.lambda_min = lambda_min;
        friction_row1.lambda_max = lambda_max;
        friction_row2.lambda_max = lambda_max;

        // once everythings computed we can add to the constraint array
        rows.push_back(normal_row);
        rows.push_back(friction_row1);
        rows.push_back(friction_row2);
    };
}

} // namespace PhysicsEngine::Constraints
