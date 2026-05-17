#include "rigid_body.h"

namespace PhysicsEngine {

void RigidBody::clear_accumulators() {
    this->force = Vec3::zeros();
    this->torque = Vec3::zeros();
}

void RigidBody::apply_force(const Vec3 &f) { this->force = this->force + f; }

void RigidBody::apply_force_at(const Vec3 &f, const Vec3 &world_point) {
    // force contributes to linear force directly
    this->force = this->force + f;

    // then torque = r cross f, r is offset from com
    const Vec3 r = world_point - this->position;
    this->torque += cross(r, f);
}

Mat<3, 3> RigidBody::rotation_matrix() const {
    return this->orientation.rotation_matrix();
}

void RigidBody::update_inertia() {
    // computes world-space inverse inertia from body-space version
    // I^-1_world = R * I^-1_body * R_transpose
    // where R = rotation_matrix()
    //
    // for static bodies this should stay zero
    Mat<3, 3> rotation_matrix = this->rotation_matrix();
    this->inv_inertia_world =
        rotation_matrix * this->inv_inertia_body * rotation_matrix.transpose();
}

void RigidBody::integrate_position(float dt) {

    // (eq 36 and 37 from paper)
    // basically just symplectic euler
    // x2 = x1 + dt * v1
    // q2 = q1 + dt/2.0f q1 * omega2
    //
    // omega2 (w) is a quaternion with 0 real part

    Quaternion w_quat = {0.0f, vx(this->angular_velocity),
                         vy(this->angular_velocity),
                         vz(this->angular_velocity)};

    this->position += this->linear_velocity * dt;
    this->orientation += (w_quat * this->orientation) * (dt / 2.0f);
}

RigidBody RigidBody::create_dynamic(const Shape &shape, float mass,
                                    const Vec3 &pos) {
    assert(mass > 0.0f && "dynamic body needs positive mass");

    RigidBody body{};
    body.shape = shape;
    body.position = pos;
    body.mass = mass;
    body.inv_mass = 1.0f / mass;

    const Mat<3, 3> inertia = shape.compute_inertia(mass);
    body.inv_inertia_body = inertia.inverse();

    body.orientation = Quaternion{}; // identity
    body.linear_velocity = Vec3::zeros();
    body.angular_velocity = Vec3::zeros();
    body.force = Vec3::zeros();
    body.torque = Vec3::zeros();
    body.inv_inertia_world =
        body.inv_inertia_body; // correct at identity rotation

    return body;
}

RigidBody RigidBody::create_static(const Shape &shape, const Vec3 &pos) {
    RigidBody body{};

    body.shape = shape;
    body.position = pos;
    body.mass = 0.0f;
    body.inv_mass = 0.0f;
    body.inv_inertia_body = Mat<3, 3>::zeros();
    body.inv_inertia_world = Mat<3, 3>::zeros();

    body.orientation = Quaternion{};
    body.linear_velocity = Vec3::zeros();
    body.angular_velocity = Vec3::zeros();
    body.force = Vec3::zeros();
    body.torque = Vec3::zeros();

    return body;
}

} // namespace PhysicsEngine
