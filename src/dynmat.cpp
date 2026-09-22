#include "dynmat.h"

namespace PhysicsEngine {

float &DynMat::operator()(std::size_t row, std::size_t col) {
    assert(row < this->rows() && col < this->cols() &&
           "matrix index out of bounds");
    return data_[row * this->cols() + col];
}

const float &DynMat::operator()(std::size_t row, std::size_t col) const {
    assert(row < this->rows() && col < this->cols() &&
           "matrix index out of bounds");
    return data_[row * this->cols() + col];
}

std::size_t DynMat::rows() const { return this->rows_; }
std::size_t DynMat::cols() const { return this->cols_; }

DynMat DynMat::operator+(const DynMat &rhs) const {
    assert(this->rows() == rhs.rows() &&
           "row dimensions must match for addition");

    bool can_broadcast = (rhs.cols() == 1);
    assert((this->cols() == rhs.cols() || can_broadcast) &&
           "matrix column mistmatch and RHS is not a broadcastable vector");
    DynMat res = {this->rows(), this->cols()};

    for (std::size_t i = 0; i < this->rows(); i++) {
        for (std::size_t j = 0; j < this->cols(); j++) {
            std::size_t rhs_j = can_broadcast ? 0 : j;
            res(i, j) = (*this)(i, j) + rhs(i, rhs_j);
        }
    }
    return res;
}

DynMat DynMat::operator-(const DynMat &rhs) const {
    assert(this->rows() == rhs.rows() && this->cols() == rhs.cols() &&
           "matrices must be the same size");
    DynMat res = {this->rows(), this->cols()};
    for (std::size_t i = 0; i < this->rows() * this->cols(); i++) {
        res.data_[i] = this->data_[i] - rhs.data_[i];
    }
    return res;
}

DynMat DynMat::operator*(const float scalar) const {
    DynMat res = {this->rows(), this->cols()};
    for (std::size_t i = 0; i < this->rows() * this->cols(); i++) {
        res.data_[i] = this->data_[i] * scalar;
    }
    return res;
}

DynMat DynMat::operator*(const DynMat &rhs) const {
    assert(this->cols() == rhs.rows() && "dimension mismatch");
    DynMat res = {this->rows(), rhs.cols()};

    for (std::size_t i = 0; i < this->rows(); i++) {
        for (std::size_t j = 0; j < rhs.cols(); j++) {
            for (std::size_t k = 0; k < this->cols(); k++) {
                res(i, j) += (*this)(i, k) * rhs(k, j);
            }
        }
    }
    return res;
}

DynMat &DynMat::operator+=(const DynMat &rhs) {
    assert(this->rows() == rhs.rows() && this->cols() == rhs.cols() &&
           "matrices must be the same size");

    for (std::size_t i = 0; i < this->rows() * this->cols(); i++) {
        this->data_[i] += rhs.data_[i];
    }

    return *this;
}

DynMat &DynMat::operator-=(const DynMat &rhs) {
    assert(this->rows() == rhs.rows() && this->cols() == rhs.cols() &&
           "matrices must be the same size");

    for (std::size_t i = 0; i < this->rows() * this->cols(); i++) {
        this->data_[i] -= rhs.data_[i];
    }

    return *this;
}

DynMat &DynMat::operator*=(float scalar) {
    for (std::size_t i = 0; i < this->rows() * this->cols(); i++) {
        this->data_[i] *= scalar;
    }

    return *this;
}

DynMat DynMat::transpose() const {
    DynMat transpose = {this->cols(), this->rows()};
    for (std::size_t i = 0; i < this->rows(); i++) {
        for (std::size_t j = 0; j < this->cols(); j++) {
            transpose(j, i) = (*this)(i, j);
        }
    }
    return transpose;
}

void DynMat::resize(std::size_t rows, std::size_t cols) {
    this->rows_ = rows;
    this->cols_ = cols;
    this->data_.resize(rows * cols);
    this->zero_fill();
}

void DynMat::zero_fill() { std::fill(data_.begin(), data_.end(), 0.0f); }

DynMat DynMat::multiply_elementwise(const DynMat &rhs) const {
    assert(this->rows() == rhs.rows() && this->cols() == rhs.cols() &&
           "ensure matrices have the same size");
    DynMat out(this->rows(), this->cols());
    for (std::size_t i = 0; i < this->rows(); i++) {
        for (std::size_t j = 0; j < this->cols(); j++) {
            out(i, j) = rhs(i, j) * (*this)(i, j);
        }
    }
    return out;
}

DynMat DynMat::square() const { return this->multiply_elementwise(*this); }
DynMat DynMat::apply_function(
    const std::function<float(const float &)> function) const {
    DynMat out(this->rows(), this->cols());
    for (std::size_t i = 0; i < this->rows(); i++) {
        for (std::size_t j = 0; j < this->cols(); j++) {
            out(i, j) = function((*this)(i, j));
        }
    }
    return out;
}

void DynMat::set_block(std::size_t row, std::size_t col, const DynMat &block) {
    assert(row + block.rows() <= this->rows() &&
           col + block.cols() <= this->cols() &&
           "attempting to index a block out of bounds");

    for (std::size_t i = 0; i < block.rows(); i++) {
        for (std::size_t j = 0; j < block.cols(); j++) {
            (*this)(i + row, j + col) = block(i, j);
        }
    }
}

DynMat DynMat::get_block(std::size_t row, std::size_t col, std::size_t rows,
                         std::size_t cols) const {
    assert(row + rows <= this->rows() && col + cols <= this->cols() &&
           "attempting to retrieve block out of bounds");

    DynMat res = {rows, cols};
    for (std::size_t i = 0; i < rows; i++) {
        for (std::size_t j = 0; j < cols; j++) {
            res(i, j) = (*this)(i + row, j + col);
        }
    }
    return res;
}

DynMat DynMat::zeros(std::size_t rows, std::size_t cols) {
    return {rows, cols};
}

DynMat DynMat::identity(std::size_t n) {
    DynMat identity = {n, n};
    for (std::size_t k = 0; k < n; k++) {
        identity(k, k) = 1.0f;
    }
    return identity;
};

void DynMat::print_shape() const {
    std::cout << "Matrix Size([" << rows_ << ", " << cols_ << "])" << std::endl;
}
void DynMat::print() const {
    for (std::size_t row = 0; row < this->rows_; row++) {
        for (std::size_t col = 0; col < this->cols_; col++) {
            std::cout << (*this)(row, col) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

} // namespace PhysicsEngine
