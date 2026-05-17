#pragma once
#include "constraint.h"
#include "rigid_body.h"
#include "solver.h"
#include "solver_settings.h"

namespace PhysicsEngine {

struct World {
    std::vector<RigidBody> bodies;
    std::vector<std::unique_ptr<Constraints::Constraint>> constraints;

    // gets rebuilt each step
    std::vector<Constraints::ConstraintRow> constraint_rows;

    Solver solver;
    SolverSettings settings{};

    World(float dt, const Vec3 gravity) {
        settings.dt = dt;
        settings.gravity = gravity;
        settings.beta = 1.0f / dt; // for now
    };

    std::size_t add_body(const RigidBody &body);
    void add_constraint(std::unique_ptr<Constraints::Constraint> constraint);

    void step();
};

} // namespace PhysicsEngine
