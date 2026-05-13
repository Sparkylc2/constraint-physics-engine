#include "quaternion.h"

namespace PhysicsEngine {
Quaternion Quaternion::from_axis_angle(float ax, float ay, float az,
                                       float angle) {

    const float norm = MathUtils::NORM(ax, ay, az);
    assert(norm > 1e-10 && "zero norm");

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
    this->w /= norm;
    this->x /= norm;
    this->y /= norm;
    this->z /= norm;
}
float Quaternion::norm() const {
    return MathUtils::NORM(this->w, this->x, this->y, this->z);
}

} // namespace PhysicsEngine
