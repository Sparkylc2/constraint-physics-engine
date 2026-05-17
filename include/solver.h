#pragma once
#include "constraint.h"
#include "rigid_body.h"
#include "solver_settings.h"

namespace PhysicsEngine {

struct Solver {
    std::size_t iterations = 10;

    // runs the PGS algorithm (Algorithm 4) from the paper
    void solve(std::vector<Constraints::ConstraintRow> &rows,
               std::vector<RigidBody> &bodies, const SolverSettings &settings);
};

} // namespace PhysicsEngine
