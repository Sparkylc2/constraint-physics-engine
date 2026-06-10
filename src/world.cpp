#include "world.h"
#include "broadphase.h"
#include "collision.h"
#include "constraint.h"
#include "contact_cache.h"

namespace PhysicsEngine {

std::size_t World::add_body(const RigidBody &body) {
    this->bodies.push_back(body);
    return this->bodies.size();
}

void World::step() {

    // clear from the last frame
    this->constraint_rows.clear();
    this->cache_new.clear();

    for (auto &body : bodies) {

        // zero accumulators and update inertia
        body.clear_accumulators();
        body.update_inertia();

        // apply the extrenal gravitational force
        body.force += this->settings.gravity * body.mass;

        // compute the tentative velocity (so without the constraints)
        body.linear_velocity += body.force * body.inv_mass * this->settings.dt;
        body.angular_velocity +=
            body.inv_inertia_world * body.torque * this->settings.dt;
    }

    // broadphase
    std::vector<Collisions::CollisionPair> pairs =
        Collisions::broadphase_bruteforce(this->bodies);

    // narrowphase
    std::vector<Collisions::ContactManifold> manifolds;
    for (const auto &pair : pairs) {
        Collisions::ContactManifold manifold =
            Collisions::collide(this->bodies, pair.body_a, pair.body_b);
        if (manifold.num_points > 0) {
            manifolds.push_back(manifold);
        }
    }

    // look up old cache for warm starting
    // build constraint rows, then store solved lambdas into new cache
    // Algorithm 5 from the paper
    for (const Collisions::ContactManifold &manifold : manifolds) {
        PairKey key = make_pair_key(manifold.body1, manifold.body2);

        // look up old cache for this body pair
        const PairCache *warm_cache = nullptr;
        auto it = this->cache_old.find(key);
        if (it != this->cache_old.end())
            warm_cache = &it->second;

        Constraints::prepare_contact_rows(manifold, bodies, this->settings,
                                          this->constraint_rows, warm_cache);

        PairCache new_entry;
        new_entry.num_contacts = manifold.num_points;
        for (std::size_t i = 0; i < manifold.num_points; i++) {
            new_entry.contacts[i].id = manifold.points[i].id;
            new_entry.contacts[i].lambda_normal = 0.0f;
            new_entry.contacts[i].lambda_friction1 = 0.0f;
            new_entry.contacts[i].lambda_friction2 = 0.0f;
        }
        this->cache_new[key] = new_entry;
    }

    // each constraint appends its rows
    for (auto &constraint : this->constraints) {
        std::size_t start = this->constraint_rows.size();
        this->constraint_rows.resize(start + constraint->num_rows());
        constraint->prepare(&this->constraint_rows[start], this->bodies,
                            this->settings);
    }

    // runs the PGS (with warm starting from cached lambdas)
    this->solver.solve(this->constraint_rows, this->bodies, this->settings);

    // post-solve: read back solved lambdas into the new cache
    // contact rows are at the beginning of constraint_rows, in order:
    // for each manifold, for each contact point: normal, friction1, friction2
    {
        std::size_t row_idx = 0;
        for (const Collisions::ContactManifold &manifold : manifolds) {
            PairKey key = make_pair_key(manifold.body1, manifold.body2);
            auto it = this->cache_new.find(key);
            if (it != this->cache_new.end()) {
                PairCache &entry = it->second;
                for (std::size_t i = 0; i < manifold.num_points; i++) {
                    if (row_idx + 2 < this->constraint_rows.size()) {
                        entry.contacts[i].lambda_normal =
                            this->constraint_rows[row_idx].lambda;
                        entry.contacts[i].lambda_friction1 =
                            this->constraint_rows[row_idx + 1].lambda;
                        entry.contacts[i].lambda_friction2 =
                            this->constraint_rows[row_idx + 2].lambda;
                    }
                    row_idx += 3;
                }
            } else {
                // shouldn't happen, but skip the rows
                row_idx += manifold.num_points * 3;
            }
        }
    }

    //  new cache becomes old cache for next frame
    this->cache_old = std::move(this->cache_new);

    for (auto &body : this->bodies) {
        if (body.is_static())
            continue;

        body.integrate_position(this->settings.dt);
    }
}
} // namespace PhysicsEngine
