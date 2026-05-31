#pragma once
#include "headers.h"

namespace PhysicsEngine {

template <std::size_t R, std::size_t C> struct Mat {
    float data_[R * C]; // matrices use row-major flat storage

    // access n stuff
    float &operator()(std::size_t row, std::size_t col);
    const float &operator()(std::size_t row, std::size_t col) const;

    // and then there was light
    Mat<R, C> operator+(const Mat<R, C> &rhs) const;
    Mat<R, C> operator-(const Mat<R, C> &rhs) const;
    Mat<R, C> operator*(float scalar) const;
    template <std::size_t C2> Mat<R, C2> operator*(const Mat<C, C2> &rhs) const;

    Mat<R, C> &operator+=(const Mat<R, C> &rhs);
    Mat<R, C> &operator-=(const Mat<R, C> &rhs);
    Mat<R, C> &operator*=(float scalar);

    // operations
    Mat<C, R> transpose() const;
    float trace() const;       // square only
    float determinant() const; // up to 3x3
    Mat<R, C> inverse() const; // up to 3x3 again
    Mat<R, C> multiply_elementwise(const Mat<R, C> &rhs) const;
    Mat<R, C> square() const;
    Mat<R, C>
    apply_function(const std::function<float(const float &)> function) const;

    // block ops because we love russian dolls
    template <std::size_t R2, std::size_t C2>
    void set_block(std::size_t row, std::size_t col, const Mat<R2, C2> &block);
    template <std::size_t R2, std::size_t C2>
    Mat<R2, C2> get_block(std::size_t row, std::size_t col) const;

    // static constructors
    static Mat<R, C> zeros();
    static Mat<R, C> identity(); // square only obv

    // print
    void print_shape();
    void print();
};

// the implementations
template <std::size_t R, std::size_t C>
float &Mat<R, C>::operator()(std::size_t row, std::size_t col) {
    assert(row < R && col < C && "matrix index out of bounds");
    return data_[row * C + col];
}

template <std::size_t R, std::size_t C>
const float &Mat<R, C>::operator()(std::size_t row, std::size_t col) const {
    assert(row < R && col < C && "matrix index out of bounds");
    return data_[row * C + col];
}

template <std::size_t R, std::size_t C>
Mat<R, C> Mat<R, C>::operator+(const Mat<R, C> &rhs) const {
    Mat<R, C> res = Mat<R, C>::zeros();
    for (std::size_t i = 0; i < R * C; i++) {
        res.data_[i] = this->data_[i] + rhs.data_[i];
    }
    return res;
}

template <std::size_t R, std::size_t C>
Mat<R, C> Mat<R, C>::operator-(const Mat<R, C> &rhs) const {
    Mat<R, C> res = Mat<R, C>::zeros();
    for (std::size_t i = 0; i < R * C; i++) {
        res.data_[i] = this->data_[i] - rhs.data_[i];
    }
    return res;
}

template <std::size_t R, std::size_t C>
Mat<R, C> Mat<R, C>::operator*(float scalar) const {
    Mat<R, C> res = Mat<R, C>::zeros();
    for (std::size_t i = 0; i < R * C; i++) {
        res.data_[i] = this->data_[i] * scalar;
    }
    return res;
}

template <std::size_t R, std::size_t C>
template <std::size_t C2>
Mat<R, C2> Mat<R, C>::operator*(const Mat<C, C2> &rhs) const {
    Mat<R, C2> res = Mat<R, C2>::zeros();

    for (std::size_t i = 0; i < R; i++) {
        for (std::size_t j = 0; j < C2; j++) {

            for (std::size_t k = 0; k < C; k++) {
                res(i, j) += (*this)(i, k) * rhs(k, j);
            }
        }
    }

    return res;
}

template <std::size_t R, std::size_t C>
Mat<R, C> &Mat<R, C>::operator+=(const Mat<R, C> &rhs) {
    for (std::size_t i = 0; i < R * C; i++) {
        data_[i] += rhs.data_[i];
    }

    return *this;
}

template <std::size_t R, std::size_t C>
Mat<R, C> &Mat<R, C>::operator-=(const Mat<R, C> &rhs) {
    for (std::size_t i = 0; i < R * C; i++) {
        data_[i] -= rhs.data_[i];
    }

    return *this;
}

template <std::size_t R, std::size_t C>
Mat<R, C> &Mat<R, C>::operator*=(float scalar) {
    for (std::size_t i = 0; i < R * C; i++) {
        data_[i] *= scalar;
    }

    return *this;
}

template <std::size_t R, std::size_t C> Mat<C, R> Mat<R, C>::transpose() const {
    Mat<C, R> transpose = Mat<C, R>::zeros();
    for (std::size_t i = 0; i < R; i++) {
        for (std::size_t j = 0; j < C; j++) {
            transpose(j, i) = (*this)(i, j);
        }
    }
    return transpose;
}

template <std::size_t R, std::size_t C> float Mat<R, C>::trace() const {

    static_assert(R == C, "matrix needs to be square to use trace()");
    float trace = 0.0f;
    for (std::size_t k = 0; k < R; k++) {
        trace += (*this)(k, k);
    }
    return trace;
}

template <std::size_t R, std::size_t C> float Mat<R, C>::determinant() const {
    static_assert(R == C, "determinant requires square matrix");
    static_assert(R == 2 || R == 3,
                  "determinant only implemented for 2x2 and 3x3");

    if constexpr (R == 2) {
        return (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
    } else {
        return (*this)(0, 0) * ((*this)(1, 1) * (*this)(2, 2) -
                                (*this)(1, 2) * (*this)(2, 1)) -
               (*this)(0, 1) * ((*this)(1, 0) * (*this)(2, 2) -
                                (*this)(1, 2) * (*this)(2, 0)) +
               (*this)(0, 2) * ((*this)(1, 0) * (*this)(2, 1) -
                                (*this)(1, 1) * (*this)(2, 0));
    }
}

template <std::size_t R, std::size_t C> Mat<R, C> Mat<R, C>::inverse() const {
    static_assert(R == C, "inverse requires square matrix");
    static_assert(R == 2 || R == 3, "inverse only implemented for 2x2 and 3x3");

    const float det = this->determinant();
    assert(!MathUtils::approx_zero(det) && "singular matrix");
    const float inv_det = 1.0f / det;

    if constexpr (R == 2) {
        return {
            (*this)(1, 1) * inv_det,
            -(*this)(0, 1) * inv_det,
            -(*this)(1, 0) * inv_det,
            (*this)(0, 0) * inv_det,
        };
    } else {
        return {
            ((*this)(1, 1) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 1)) *
                inv_det,
            ((*this)(0, 2) * (*this)(2, 1) - (*this)(0, 1) * (*this)(2, 2)) *
                inv_det,
            ((*this)(0, 1) * (*this)(1, 2) - (*this)(0, 2) * (*this)(1, 1)) *
                inv_det,
            ((*this)(1, 2) * (*this)(2, 0) - (*this)(1, 0) * (*this)(2, 2)) *
                inv_det,
            ((*this)(0, 0) * (*this)(2, 2) - (*this)(0, 2) * (*this)(2, 0)) *
                inv_det,
            ((*this)(0, 2) * (*this)(1, 0) - (*this)(0, 0) * (*this)(1, 2)) *
                inv_det,
            ((*this)(1, 0) * (*this)(2, 1) - (*this)(1, 1) * (*this)(2, 0)) *
                inv_det,
            ((*this)(0, 1) * (*this)(2, 0) - (*this)(0, 0) * (*this)(2, 1)) *
                inv_det,
            ((*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0)) *
                inv_det,
        };
    }
}
template <std::size_t R, std::size_t C>
Mat<R, C> Mat<R, C>::multiply_elementwise(const Mat<R, C> &rhs) const {
    Mat<R, C> out = Mat<R, C>::zeros();
    for (std::size_t i = 0; i < R; i++) {
        for (std::size_t j = 0; j < C; j++) {
            out(i, j) = rhs(i, j) * (*this)(i, j);
        }
    }
    return out;
}

template <std::size_t R, std::size_t C> Mat<R, C> Mat<R, C>::square() const {
    return this->multiply_elementwise(*this);
}

template <std::size_t R, std::size_t C>
Mat<R, C> Mat<R, C>::apply_function(
    const std::function<float(const float &)> function) const {
    Mat<R, C> out;
    for (std::size_t i = 0; i < R; i++) {
        for (std::size_t j = 0; j < C; j++) {
            out(i, j) = function((*this)(i, j));
        }
    }
    return out;
}

template <std::size_t R, std::size_t C>
template <std::size_t R2, std::size_t C2>
void Mat<R, C>::set_block(std::size_t row, std::size_t col,
                          const Mat<R2, C2> &block) {
    for (std::size_t i = 0; i < R2; i++) {
        for (std::size_t j = 0; j < C2; j++) {
            (*this)(i + row, j + col) = block(i, j);
        }
    }
}

template <std::size_t R, std::size_t C>
template <std::size_t R2, std::size_t C2>
Mat<R2, C2> Mat<R, C>::get_block(std::size_t row, std::size_t col) const {

    Mat<R2, C2> res = Mat<R2, C2>::zeros();
    for (std::size_t i = 0; i < R2; i++) {
        for (std::size_t j = 0; j < C2; j++) {
            res(i, j) = (*this)(i + row, j + col);
        }
    }
    return res;
}

template <std::size_t R, std::size_t C> Mat<R, C> Mat<R, C>::zeros() {
    return {};
}

template <std::size_t R, std::size_t C> Mat<R, C> Mat<R, C>::identity() {
    static_assert(R == C, "ensure matrices are square");

    Mat<R, C> identity = Mat<R, C>::zeros();
    for (std::size_t k = 0; k < R; k++) {
        identity(k, k) = 1.0f;
    }
    return identity;
}

template <std::size_t R, std::size_t C> void Mat<R, C>::print_shape() {
    std::cout << "Matrix Size([" << R << ", " << C << "])" << std::endl;
}
template <std::size_t R, std::size_t C> void Mat<R, C>::print() {
    for (std::size_t i = 0; i < R; i++) {
        for (std::size_t j = 0; j < C; j++) {
            std::cout << (*this)(i, j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
} // namespace PhysicsEngine
