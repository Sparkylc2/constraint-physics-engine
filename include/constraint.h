#pragma once
#include "collision.h"
#include "contact_cache.h"
#include "headers.h"
#include "rigid_body.h"
#include "solver_settings.h"
#include "vec3.h"

namespace PhysicsEngine::Constraints {

// scalar constraint between two bodies
struct ConstraintRow {
    // jacobian blocks, linear and angular parts
    Vec3 J1_linear, J1_angular; // first body
    Vec3 J2_linear, J2_angular; // second body

    // M^-1 * J_transpose (gets precomputed in the prepare step)
    Vec3 B1_linear, B1_angular;
    Vec3 B2_linear, B2_angular;

    float d_inv;       // 1/(J1*B1 + J2 * B2), effective inverse mass
    float eta = 0.0f;  // zeta - Jv/dt, Jv is tentative velocity
    float zeta = 0.0f; // position/bias terms

    float lambda = 0.0f; // accumulated impulse
    float lambda_min =
        -std::numeric_limits<float>::max(); // lower bound from eq 14
    float lambda_max =
        std::numeric_limits<float>::max(); // upper bound from eq 14

    std::size_t body1, body2; // indices into body array

    void precompute(const std::vector<RigidBody> &bodies,
                    const SolverSettings
                        &settings); // computes b1, b2, d_inv, and eta-=Jv/dt
};

// a base class for constraint rows
struct Constraint {
    std::size_t body1, body2;

    virtual ~Constraint() = default;
    virtual std::size_t num_rows() const = 0;

    // fills rows starting at 'out', given bgodies array and timestep, returning
    // rows written
    virtual std::size_t prepare(ConstraintRow *out,
                                const std::vector<RigidBody> &bodies,
                                const SolverSettings &settings) = 0;
};

// for a contact between two surfaces, transient so doesn't need its
// own type as it's not persistent
// warm_cache is null when no cache hit exists for this pair
void prepare_contact_rows(const Collisions::ContactManifold &manifold,
                          const std::vector<RigidBody> &bodies,
                          const SolverSettings &settings,
                          std::vector<ConstraintRow> &rows,
                          const PairCache *warm_cache = nullptr);

} // namespace PhysicsEngine::Constraints
