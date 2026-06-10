#include "world.h"
#include "broadphase.h"
#include "collision.h"
#include "constraint.h"

namespace PhysicsEngine {

std::size_t World::add_body(const RigidBody &body) {
    this->bodies.push_back(body);
    return this->bodies.size();
}

void World::step() {

    // clear from the last frame
    this->constraint_rows.clear();

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

    // transient so we can add directly to avoid the need to track the contact
    // constraint (if we had added it to the constraints vector)
    for (Collisions::ContactManifold manifold : manifolds) {
        Constraints::prepare_contact_rows(manifold, bodies, this->settings,
                                          this->constraint_rows);
    }

    // each constraint appends its rows
    for (auto &constraint : this->constraints) {
        std::size_t start = this->constraint_rows.size();
        this->constraint_rows.resize(start + constraint->num_rows());
        constraint->prepare(&this->constraint_rows[start], this->bodies,
                            this->settings);
    }

    // std::cout << constraint_rows.size() << std::endl;
    // runs the PGS
    this->solver.solve(this->constraint_rows, this->bodies, this->settings);

    // and finally integrate
    for (auto &body : this->bodies) {
        if (body.is_static())
            continue;

        body.integrate_position(this->settings.dt);
        // std::cout << vx(body.position) << ", " << vy(body.position) << ", "
        //           << vz(body.position) << std::endl;
    }
}
} // namespace PhysicsEngine
