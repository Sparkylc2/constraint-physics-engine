#include "aabb.h"

namespace PhysicsEngine::Collisions {

bool AABB::overlaps(const AABB &other) const {
    // separating axis test on all 3 world axes
    if (this->max.data_[0] < other.min.data_[0] ||
        this->min.data_[0] > other.max.data_[0])
        return false;
    if (this->max.data_[1] < other.min.data_[1] ||
        this->min.data_[1] > other.max.data_[1])
        return false;
    if (this->max.data_[2] < other.min.data_[2] ||
        this->min.data_[2] > other.max.data_[2])
        return false;
    return true;
}

AABB AABB::from_body(const RigidBody &body) {
    switch (body.shape.type) {
    case ShapeType::box: {
        // for a rotated OBB, the AABB half-extent along world axis i is:
        //   e_i = sum_j |R(i,j)| * h_j
        // avoids transforming all 8 vertices
        const Mat<3, 3> R = body.rotation_matrix();
        const Vec3 &he = body.shape.box.half_extents;

        Vec3 extent;
        for (std::size_t i = 0; i < 3; i++) {
            extent.data_[i] = std::abs(R(i, 0)) * he.data_[0] +
                              std::abs(R(i, 1)) * he.data_[1] +
                              std::abs(R(i, 2)) * he.data_[2];
        }

        return {body.position - extent, body.position + extent};
    }
    case ShapeType::sphere: {
        const float r = body.shape.sphere.radius;
        const Vec3 r_vec = make_vec3(r, r, r);
        return {body.position - r_vec, body.position + r_vec};
    }
    case ShapeType::polygon: {
        // transforms all vertices, tracks component-wise min/max
        const Mat<3, 3> R = body.rotation_matrix();
        const auto &poly = body.shape.polygon;
        assert(poly.num_vertices > 0 && "polygon has no vertices");

        Vec3 world_v = body.position + R * poly.vertices[0];
        Vec3 min_v = world_v;
        Vec3 max_v = world_v;

        for (std::size_t i = 1; i < poly.num_vertices; i++) {
            world_v = body.position + R * poly.vertices[i];
            for (std::size_t axis = 0; axis < 3; axis++) {
                if (world_v.data_[axis] < min_v.data_[axis])
                    min_v.data_[axis] = world_v.data_[axis];
                if (world_v.data_[axis] > max_v.data_[axis])
                    max_v.data_[axis] = world_v.data_[axis];
            }
        }

        return {min_v, max_v};
    }
    }

    return {Vec3::zeros(), Vec3::zeros()};
}

} // namespace PhysicsEngine::Collisions
