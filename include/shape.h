#pragma once
#include "vec3.h"

namespace PhysicsEngine {

enum class ShapeType { box, sphere, polygon };

struct BoxShape {
    Vec3 half_extents;                    // half-widths along local xyz
    void get_vertices(Vec3 out[8]) const; // gets local box vertices
};

struct SphereShape {
    float radius; // shocker its the radius
};

struct PolygonShape {
    // for cache locality a stack array is the smarter choice
    static constexpr std::size_t MAX_VERTICES = 32;
    Vec3 vertices[MAX_VERTICES]; // local-space
    std::size_t num_vertices = 0;
};

struct Shape {
    ShapeType type;
    union {
        BoxShape box;
        SphereShape sphere;
        PolygonShape polygon;
    };

    Shape() : type(ShapeType::box) {}
    Shape(const ShapeType type) : type(type) {};

    // comnputes the body-space inertia tensor for a given mass
    Mat<3, 3> compute_inertia(float mass) const;

    static Shape make_box(float hx, float hy, float hz);
    static Shape make_sphere(float radius);
    static Shape make_polygon(const Vec3 *verts, std::size_t count);
};

} // namespace PhysicsEngine
