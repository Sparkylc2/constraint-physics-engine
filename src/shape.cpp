#include "shape.h"

namespace PhysicsEngine {

Shape Shape::make_box(float hx, float hy, float hz) {
    Shape s;
    s.type = ShapeType::box;
    s.box = {make_vec3(hx, hy, hz)};
    return s;
}

Shape Shape::make_sphere(float radius) {
    Shape s;
    s.type = ShapeType::sphere;
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
    }
    return Mat<3, 3>::identity();
}

} // namespace PhysicsEngine
