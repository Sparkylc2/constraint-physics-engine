#include "raycast.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

namespace PhysicsEngine {

float ray_box(const Vec3 &origin, const Vec3 &dir, const RigidBody &body) {
    const Mat<3, 3> R = body.rotation_matrix();
    const Mat<3, 3> Rt = R.transpose();

    // transform ray into box local space
    Vec3 local_o = Rt * (origin - body.position);
    Vec3 local_d = Rt * dir;

    const Vec3 &he = body.shape.box.half_extents;

    float t_enter = -1e30f;
    float t_exit = 1e30f;

    for (int i = 0; i < 3; i++) {
        if (std::abs(local_d.data_[i]) < 1e-8f) {
            if (local_o.data_[i] < -he.data_[i] ||
                local_o.data_[i] > he.data_[i])
                return -1.0f;
        } else {
            float inv_d = 1.0f / local_d.data_[i];
            float t1 = (-he.data_[i] - local_o.data_[i]) * inv_d;
            float t2 = (he.data_[i] - local_o.data_[i]) * inv_d;
            if (t1 > t2)
                std::swap(t1, t2);
            t_enter = std::max(t_enter, t1);
            t_exit = std::min(t_exit, t2);
            if (t_enter > t_exit)
                return -1.0f;
        }
    }

    if (t_exit < 0.0f)
        return -1.0f;
    return (t_enter >= 0.0f) ? t_enter : t_exit;
}

float ray_sphere(const Vec3 &origin, const Vec3 &dir, const RigidBody &body) {
    Vec3 oc = origin - body.position;
    float r = body.shape.sphere.radius;

    float a = dot(dir, dir);
    float b = 2.0f * dot(oc, dir);
    float c = dot(oc, oc) - r * r;
    float disc = b * b - 4.0f * a * c;

    if (disc < 0.0f)
        return -1.0f;

    float sqrt_disc = std::sqrt(disc);
    float t1 = (-b - sqrt_disc) / (2.0f * a);
    float t2 = (-b + sqrt_disc) / (2.0f * a);

    if (t1 >= 0.0f)
        return t1;
    if (t2 >= 0.0f)
        return t2;
    return -1.0f;
}

RayHit raycast(const Vec3 &origin, const Vec3 &direction,
               const std::vector<RigidBody> &bodies) {
    RayHit best;

    for (std::size_t i = 0; i < bodies.size(); i++) {
        float t = -1.0f;
        switch (bodies[i].shape.type) {
        case ShapeType::box:
            t = ray_box(origin, direction, bodies[i]);
            break;
        case ShapeType::sphere:
            t = ray_sphere(origin, direction, bodies[i]);
            break;
        default:
            break;
        }

        if (t >= 0.0f && t < best.t) {
            best.hit = true;
            best.body_idx = i;
            best.t = t;
            best.world_point = origin + direction * t;

            // local point for constraint anchoring
            const Mat<3, 3> R = bodies[i].rotation_matrix();
            best.local_point =
                R.transpose() * (best.world_point - bodies[i].position);

            // approximate normal
            if (bodies[i].shape.type == ShapeType::sphere) {
                Vec3 d = best.world_point - bodies[i].position;
                float len = length(d);
                best.normal =
                    (len > 1e-6f) ? d * (1.0f / len) : make_vec3(0, 1, 0);
            } else {
                // normal is the face axis with largest local component
                Vec3 abs_local;
                const Vec3 &he = bodies[i].shape.box.half_extents;
                for (int a = 0; a < 3; a++)
                    abs_local.data_[a] =
                        std::abs(best.local_point.data_[a] / he.data_[a]);

                int face = 0;
                if (abs_local.data_[1] > abs_local.data_[face])
                    face = 1;
                if (abs_local.data_[2] > abs_local.data_[face])
                    face = 2;

                Vec3 local_n = Vec3::zeros();
                local_n.data_[face] =
                    (best.local_point.data_[face] >= 0.0f) ? 1.0f : -1.0f;
                best.normal = R * local_n;
            }
        }
    }

    return best;
}

} // namespace PhysicsEngine
