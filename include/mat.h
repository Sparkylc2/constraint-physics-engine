#pragma once
#include "headers.h"

namespace PhysicsEngine {

template <std::size_t R, std::size_t C> struct Mat {
    float data[R * C];

    // access n stuff
    float &operator()(std::size_t row, std::size_t col);
    const float &operator()(std::size_t row, std::size_t col) const;

    // and then there was light
    Mat<R, C> operator+(const Mat<R, C> &rhs) const;
    Mat<R, C> operator-(const Mat<R, C> &rhs) const;
    Mat<R, C> operator*(float scalar) const;
    template <std::size_t C2> Mat<R, C2> operator*(const Mat<C, C2> &rhs) const;

    // operations
    Mat<C, R> transpose() const;
    float trace() const;       // square only
    float determinant() const; // up to 3x3
    Mat<R, C> inverse() const; // up to 3x3 again

    // block ops because we love russian dolls
    template <std::size_t R2, std::size_t C2>
    void set_block(std::size_t row, std::size_t col, const Mat<R2, C2> &block);
    template <std::size_t R2, std::size_t C2>
    Mat<R2, C2> get_block(std::size_t row, std::size_t col) const;

    // static constructors
    static Mat<R, C> zeros();
    static Mat<R, C> identity(); // square only obv
};

template <std::size_t R, std::size_t C>
float &Mat<R, C>::operator()(std::size_t row, std::size_t col) {
    return data[row * C + col];
}
} // namespace PhysicsEngine
