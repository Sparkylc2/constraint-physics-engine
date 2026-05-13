#include "dynmat.h"

// struct DynMat {
//     std::size_t rows, cols;
//     std::vector<float> data;
//
//     float &operator()(std::size_t row, std::size_t col);
//     const float &operator()(std::size_t row, std::size_t col) const;
//
//     // reinventing the wheel
//     DynMat operator+(const DynMat &rhs) const;
//     DynMat operator-(const DynMat &rhs) const;
//     DynMat operator*(float scalar) const;
//     DynMat operator*(const DynMat &rhs) const;
//
//     // oppperationsss
//     DynMat transpose() const;
//     void resize(std::size_t rows, std::size_t cols);
//     void zero_fill();
//
//     // more block ops - these are the important ones for assembly
//     template <std::size_t R, std::size_t C>
//     void set_block(std::size_t row, std::size_t col, const Mat<R, C> &block);
//     void set_block(std::size_t row, std::size_t col, const DynMat &block);
//     DynMat get_block(std::size_t row, std::size_t col, std::size_t rows,
//                      std::size_t cols) const;
//
//     // static
//     static DynMat zeros(std::size_t rows, std::size_t cols);
//     static DynMat identity(std::size_t n);
// };
