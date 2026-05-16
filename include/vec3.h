#pragma once
#include "headers.h"
#include "mat.h"

namespace PhysicsEngine {

using Vec3 = Mat<3, 1>;

// convenience
inline Vec3 make_vec3(float x, float y, float z) { return {x, y, z}; }

// access helpers for clarity
inline float vx(const Vec3 &v) { return v.data_[0]; }
inline float vy(const Vec3 &v) { return v.data_[1]; }
inline float vz(const Vec3 &v) { return v.data_[2]; }
inline float &vx(Vec3 &v) { return v.data_[0]; }
inline float &vy(Vec3 &v) { return v.data_[1]; }
inline float &vz(Vec3 &v) { return v.data_[2]; }

inline float dot(const Vec3 &a, const Vec3 &b) {
    return vx(a) * vx(b) + vy(a) * vy(b) + vz(a) * vz(b);
}

inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
    return {vy(a) * vz(b) - vz(a) * vy(b), vz(a) * vx(b) - vx(a) * vz(b),
            vx(a) * vy(b) - vy(a) * vx(b)};
};

inline float length(const Vec3 &v) { return MathUtils::NORM(v); }

inline Vec3 normalised(const Vec3 &v) {
    const float len = length(v);
    assert(!MathUtils::approx_zero(len) && "normalizing zero vec");
    return v * (1.0f / len);
}

inline Vec3 operator-(const Vec3 &v) { return v * (-1.0f); }
} // namespace PhysicsEngine
