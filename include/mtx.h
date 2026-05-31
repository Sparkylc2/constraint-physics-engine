#pragma once
#include "dynmat.h"
#include "mat.h"
namespace PhysicsEngine {

struct mtx {
  private:
    // so we seed only once
    static std::mt19937 &get_generator() {
        static std::random_device rd{};
        static std::mt19937 gen{rd()};
        return gen;
    }

  public:
    template <std::size_t R, std::size_t C> static Mat<R, C> generate_randn() {
        Mat<R, C> out;
        float numel = out.rows() * out.cols();
        float stdev = 1.0f / std::sqrt(numel);

        std::normal_distribution<float> d{0, stdev};

        auto &gen = get_generator();

        for (std::size_t i = 0; i < R; i++) {
            for (std::size_t j = 0; j < C; j++) {
                out(i, j) = d(gen);
            }
        }
        return out;
    }

    static DynMat generate_randn(std::size_t rows, std::size_t cols) {
        DynMat out(rows, cols);
        float numel = out.rows() * out.cols();
        float stdev = 1.0f / std::sqrt(numel);

        std::normal_distribution<float> d{0, stdev};

        auto &gen = get_generator();

        for (std::size_t i = 0; i < rows; i++) {
            for (std::size_t j = 0; j < cols; j++) {
                out(i, j) = d(gen);
            }
        }
        return out;
    }
};
} // namespace PhysicsEngine
