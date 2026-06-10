#include "shape.h"

namespace PhysicsEngine {

void BoxShape::get_vertices(Vec3 out[8]) const {
    const float hx = vx(half_extents);
    const float hy = vy(half_extents);
    const float hz = vz(half_extents);

    out[0] = {-hx, -hy, hz};
    out[1] = {hx, -hy, hz};
    out[2] = {-hx, -hy, -hz};
    out[3] = {hx, -hy, -hz};
    out[4] = {-hx, hy, hz};
    out[5] = {hx, hy, hz};
    out[6] = {-hx, hy, -hz};
    out[7] = {hx, hy, -hz};
}

Shape Shape::make_box(float hx, float hy, float hz) {
    Shape s = {ShapeType::box};
    s.type = ShapeType::box;
    s.box = {make_vec3(hx, hy, hz)};
    return s;
}

Shape Shape::make_sphere(float radius) {
    Shape s = {ShapeType::sphere};
    s.sphere = {radius};
    return s;
}

Mat<3, 3> Shape::compute_inertia(float mass) const {
    switch (this->type) {
    case ShapeType::box: {

        const float coeff = mass / 12.0f;
        const Vec3 dims = this->box.half_extents * 2;

        const float x2 = vx(dims) * vx(dims);
        const float y2 = vy(dims) * vy(dims);
        const float z2 = vz(dims) * vz(dims);

        return {coeff * (y2 + z2), 0.0f, 0.0f, 0.0f,
                coeff * (x2 + z2), 0.0f, 0.0f, 0.0f,
                coeff * (x2 + y2)};
    }
    case ShapeType::sphere: {
        const float coeff =
            mass * 2.0f / 5.0f * this->sphere.radius * this->sphere.radius;
        return {coeff, 0.0f, 0.0f, 0.0f, coeff, 0.0f, 0.0f, 0.0f, coeff};
    }
    case ShapeType::polygon: {
        // TODO
        return {1.0f, 1.0f, 1.0f};
    }
    }
    return Mat<3, 3>::identity();
}

} // namespace PhysicsEngine
