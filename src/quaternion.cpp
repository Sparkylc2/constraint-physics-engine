#include "quaternion.h"
#include "math_utils.h"

namespace PhysicsEngine {
Quaternion Quaternion::from_axis_angle(float ax, float ay, float az,
                                       float angle) {

    const float norm = MathUtils::NORM(ax, ay, az);
    assert(!MathUtils::approx_zero(norm) && "zero norm");

    ax /= norm;
    ay /= norm;
    az /= norm;
    return from_normalised_axis_angle(ax, ay, az, angle);
};

Quaternion Quaternion::from_normalised_axis_angle(float ax, float ay, float az,
                                                  float angle) {
    const float angle_h = angle / 2.0;
    const float sin_h = std::sin(angle_h);
    const float cos_h = std::cos(angle_h);

    return {
        cos_h,
        ax * sin_h,
        ay * sin_h,
        az * sin_h,
    };
}

Mat<3, 3> Quaternion::rotation_matrix() const {
    Quaternion rot_q = MathUtils::approx_equal(this->norm(), 1.0f)
                           ? *this
                           : this->normalised();

    const float x2 = rot_q.x * rot_q.x;
    const float y2 = rot_q.y * rot_q.y;
    const float z2 = rot_q.z * rot_q.z;

    const float xy = rot_q.x * rot_q.y;
    const float xz = rot_q.x * rot_q.z;
    const float yz = rot_q.y * rot_q.z;

    const float wx = rot_q.w * rot_q.x;
    const float wy = rot_q.w * rot_q.y;
    const float wz = rot_q.w * rot_q.z;

    const float R00 = 1 - 2 * (y2 + z2);
    const float R01 = 2 * (xy - wz);
    const float R02 = 2 * (xz + wy);

    const float R10 = 2 * (xy + wz);
    const float R11 = 1 - 2 * (x2 + z2);
    const float R12 = 2 * (yz - wx);

    const float R20 = 2 * (xz - wy);
    const float R21 = 2 * (yz + wx);
    const float R22 = 1 - 2 * (x2 + y2);

    return {
        R00, R01, R02, R10, R11, R12, R20, R21, R22,
    };
}

Quaternion Quaternion::operator*(const Quaternion &rhs) const {
    return {
        this->w * rhs.w - this->x * rhs.x - this->y * rhs.y - this->z * rhs.z,
        this->w * rhs.x + this->x * rhs.w + this->y * rhs.z - this->z * rhs.y,
        this->w * rhs.y - this->x * rhs.z + this->y * rhs.w + this->z * rhs.x,
        this->w * rhs.z + this->x * rhs.y - this->y * rhs.x + this->z * rhs.w,
    };
}
Quaternion Quaternion::conjugate() const {
    return {this->w, -this->x, -this->y, -this->z};
}

Quaternion Quaternion::normalised() const {
    const float norm = this->norm();
    return {this->w / norm, this->x / norm, this->y / norm, this->z / norm};
}
void Quaternion::NORMALISE() {
    const float norm = this->norm();
    assert(!MathUtils::approx_zero(norm) && "norm is zero");
    this->w /= norm;
    this->x /= norm;
    this->y /= norm;
    this->z /= norm;
}
float Quaternion::norm() const {
    return MathUtils::NORM(this->w, this->x, this->y, this->z);
}

} // namespace PhysicsEngine
