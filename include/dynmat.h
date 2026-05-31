#pragma once
#include "headers.h"
#include "mat.h"

namespace PhysicsEngine {

struct DynMat {
    std::size_t rows_, cols_;
    std::vector<float> data_;

    // constructor
    DynMat(std::size_t rows, std::size_t cols)
        : rows_(rows), cols_(cols), data_(rows * cols, 0.0f) {}

    // copy constructor
    DynMat(const DynMat &mat)
        : rows_(mat.rows_), cols_(mat.cols_), data_(mat.data_) {}

    float &operator()(std::size_t row, std::size_t col);
    const float &operator()(std::size_t row, std::size_t col) const;

    // getters because messing with internal state always happens accidentally
    std::size_t rows() const;
    std::size_t cols() const;

    // reinventing the wheel
    DynMat operator+(const DynMat &rhs)
        const; // NOTE: broadcasting only implemented for + op
    DynMat operator-(const DynMat &rhs) const;
    DynMat operator*(float scalar) const;
    DynMat operator*(const DynMat &rhs) const;

    DynMat &operator+=(const DynMat &rhs);
    DynMat &operator-=(const DynMat &rhs);
    DynMat &operator*=(float scalar);
    DynMat &operator*=(const DynMat &rhs);

    // oppperationsss
    DynMat transpose() const;
    void resize(std::size_t rows, std::size_t cols);
    void zero_fill();
    DynMat multiply_elementwise(const DynMat &rhs) const;
    DynMat square() const;
    DynMat
    apply_function(const std::function<float(const float &)> function) const;

    // more block ops - these are the important ones for assembly
    template <std::size_t R, std::size_t C>
    void set_block(std::size_t row, std::size_t col, const Mat<R, C> &block);
    void set_block(std::size_t row, std::size_t col, const DynMat &block);
    DynMat get_block(std::size_t row, std::size_t col, std::size_t rows,
                     std::size_t cols) const;

    // static
    static DynMat zeros(std::size_t rows, std::size_t cols);
    static DynMat identity(std::size_t n);

    // print
    void print_shape() const;
    void print() const;
};

template <std::size_t R, std::size_t C>
void DynMat::set_block(std::size_t row, std::size_t col,
                       const Mat<R, C> &block) {
    assert(row + R <= this->rows() && col + C <= this->cols() &&
           "attempting to index a block out of bounds");

    for (std::size_t i = 0; i < R; i++) {
        for (std::size_t j = 0; j < C; j++) {
            (*this)(i + row, j + col) = block(i, j);
        }
    }
}

} // namespace PhysicsEngine
