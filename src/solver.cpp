#include "solver.h"

namespace PhysicsEngine {

void Solver::solve(std::vector<Constraints::ConstraintRow> &rows,
                   std::vector<RigidBody> &bodies,
                   const SolverSettings &settings) {

    // this implements algorithm 4 from the paper, the PSG method
    // (Projected Gauss-Seidel)
    // first we create per-body accumulators, zeroed
    std::vector<Vec3> a_linear(bodies.size(), Vec3::zeros());
    std::vector<Vec3> a_angular(bodies.size(), Vec3::zeros());

    // Algorithm 5 warm starting
    for (const Constraints::ConstraintRow &row : rows) {
        if (std::abs(row.lambda) > 1e-10f) {
            a_linear[row.body1] += row.B1_linear * row.lambda;
            a_angular[row.body1] += row.B1_angular * row.lambda;
            a_linear[row.body2] += row.B2_linear * row.lambda;
            a_angular[row.body2] += row.B2_angular * row.lambda;
        }
    }

    for (std::size_t iter = 0; iter < this->iterations; iter++) {
        // run for the allowable number of iterations

        // for each row
        for (Constraints::ConstraintRow &row : rows) {
            // compute J dot a
            // line: Jsp(i, 1) * a(b1) + Jsp(i, 2) * a(b2)
            // basically the current constraint 'correction velocity'
            const float Ja = dot(row.J1_linear, a_linear[row.body1]) +
                             dot(row.J1_angular, a_angular[row.body1]) +
                             dot(row.J2_linear, a_linear[row.body2]) +
                             dot(row.J2_angular, a_angular[row.body2]);
            // compute delta lambda
            // line:
            // delta lambda_i = (eta_i - Jsp(i, 1)*a(b1) - Jsp(i,2)*a(b2))/di
            // => delta lambda_i = (eta_i - Ja)*d_inv;
            // the force correction this row needs
            float delta_lambda = (row.eta - Ja) * row.d_inv;

            // clamping the accumulated lambda
            // lines:
            // lambda_i^0 = lambda_i
            // lambda_i = max(lambda_i^-, min(lambda_i^0 +
            //                                delta_lambda_i
            //                                ),
            //                lambda_i^+)
            // delta_lambda_i = lambda_i - lambda_i^0
            const float old_lambda = row.lambda;
            row.lambda = std::clamp(old_lambda + delta_lambda, row.lambda_min,
                                    row.lambda_max);
            // now getting our new clamped delta lambda
            delta_lambda = row.lambda - old_lambda;

            // then updating a = B*lambda for the affected bodies
            // lines:
            // a(b1) = a(b1) + delta_lambda_i * Bsp(1, i)
            // a(b2) = a(b2) + delta_lambda_i * Bsp(2, i)
            a_linear[row.body1] += row.B1_linear * delta_lambda;
            a_linear[row.body2] += row.B2_linear * delta_lambda;
            a_angular[row.body1] += row.B1_angular * delta_lambda;
            a_angular[row.body2] += row.B2_angular * delta_lambda;
        }
    }

    // finally applying the velocity update as the a vectors are equivalent to
    // our V^2 vector (or well the linear and angular components etc.
    for (std::size_t i = 0; i < bodies.size(); i++) {
        if (bodies[i].is_static())
            continue;
        bodies[i].linear_velocity += a_linear[i] * settings.dt;
        bodies[i].angular_velocity += a_angular[i] * settings.dt;
    }
}
} // namespace PhysicsEngine
