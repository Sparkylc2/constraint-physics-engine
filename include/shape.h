#pragma once
#include "vec3.h"

namespace PhysicsEngine {

enum class ShapeType { box, sphere };

struct BoxShape {
    Vec3 half_extents;                    // half-widths along local xyz
    void get_vertices(Vec3 out[8]) const; // gets local box vertices
};

struct SphereShape {
    float radius; // shocker its the radius
};

struct Shape {
    ShapeType type;
    union {
        BoxShape box;
        SphereShape sphere;
    };

    // comnputes the body-space inertia tensor for a given mass
    Mat<3, 3> compute_inertia(float mass) const;

    static Shape make_box(float hx, float hy, float hz);
    static Shape make_sphere(float radius);
};

} // namespace PhysicsEngine
