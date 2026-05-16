#pragma once
#include "headers.h"
#include "mat.h"

namespace PhysicsEngine {

struct Quaternion {
    float w, x, y, z;

    Quaternion() : w{1.0f}, x{0.0f}, y{0.0f}, z{0.0f} {}
    Quaternion(float w, float x, float y, float z) : w{w}, x{x}, y{y}, z{z} {}

    static Quaternion from_axis_angle(float ax, float ay, float az,
                                      float angle);
    static Quaternion from_normalised_axis_angle(float ax, float ay, float az,
                                                 float angle);

    Quaternion operator*(const Quaternion &rhs) const;

    Mat<3, 3> rotation_matrix() const;

    Quaternion conjugate() const;
    Quaternion normalised() const;
    void NORMALISE();
    float norm() const;
};
} // namespace PhysicsEngine
