#pragma once
#include "dynmat.h"
#include "headers.h"
#include "mtx.h"

namespace PhysicsEngine::NN {
class MLP {
  public:
    // self explanatory
    std::vector<std::size_t> units_per_layer;
    std::vector<DynMat> bias_vectors;
    std::vector<DynMat> weight_matrices;
    std::vector<DynMat> activations;

    float lr;

    // constructorr
    explicit MLP(std::vector<std::size_t> units_per_layer, float lr = 0.001f)
        : units_per_layer(units_per_layer), bias_vectors(), weight_matrices(),
          activations(), lr(lr) {

        for (std::size_t i = 0; i < units_per_layer.size() - 1; i++) {
            size_t in_channels{units_per_layer[i]};
            size_t out_channels{units_per_layer[i + 1]};

            // we can initialize to a random gaussian
            auto W = mtx::randn(out_channels, in_channels);
            // matrix has "out_channels" rows and "in_channels" cols.

            // bias row vector for the output channels
            auto b = mtx::randn(out_channels, 1);
            bias_vectors.push_back(b);

            activations.resize(units_per_layer.size());
        }
    };

    // sigmoid function
    static inline float sigmoid(float x) {
        return 1.0f / (1.0f + std::exp(-x));
    };

    // derivative of the sigmoid function for back prop
    static inline float d_sigmoid(float x) { return (x * (1 - x)); }

    DynMat forward(DynMat x) {
        // compute the activations at a given layer and saving it, and then
        // passing it forward to use as the input to the next layer

        // check if initial input dim is the same as first network layer,
        // and that cols is non-zero
        assert(x.rows() == units_per_layer[0] && x.cols());

        // initial activation is input
        activations[0] = x;
        DynMat prev(x); // temp variable
        //
        //
        // to do a forward pass, each layer i computes the following
        // z_i+1 = W_i * x_i + b_i;
        // a_i+1 = sigmoid(z_i+1)
        // where W_i has shape (units out x units in),
        // a_i has shape (units in x 1)
        // and sigmoid is applied element wise
        for (std::size_t i = 0; i < units_per_layer.size() - 1; i++) {

            // this computes x = W * x
            // weight matrix W has a shape of (outputs x inputs), where outputs
            // is the number of neurons in the current layer and inputs is hte
            // number of neurons in the previous layer
            //
            // prev has shape of (inputs x batch_size)
            // multiplying gives a new matrix Z of shape (outputs x batch size)
            // each column in Z is the raw unactivated linear combinations for a
            // single data sample in the batch

            DynMat z = weight_matrices[i] * prev;

            // this computes Z = W*X + b
            // b has shape (outputs x 1), and Z has shape (outputs x batch size)
            // we are just adding the bias to each column of Z
            // this uses broadcasting
            z = z + bias_vectors[i];

            // this computes the final activation A = sigma(Y)
            z = z.apply_function(MLP::sigmoid);

            // we cache A into activations for later back prop calculations
            // and prev is overwritten so it can be used as the input to the
            // next layer's matrix multiplication
            activations[i + 1] = z;
            prev = z;
        }
        return prev;
    };

    void backprop(DynMat mat) {
        // ensure our matrix has the same number of rows as the output
        assert(mat.rows() == units_per_layer.back());

        // get the simple error
        // error = target - output
        DynMat y = mat;
        DynMat y_hat = activations.back();
        DynMat error = (y - y_hat); // raw residual of shape (output_size x 1)

        // the backpropagation algorithm
        for (int i = weight_matrices.size() - 1; i >= 0; i--) {
            // error propagation to the previous layer
            //
            // W_t has shape (inputs x outputs), error has shape (outputs x 1)
            // so result is (inputs x 1)
            //
            // this projects error at layer i+1 through the weights to get the
            // error contribution at layer i
            //
            // basically chain rule d = partial
            // dL/da_i = dL/dz_i+1 * dz_i+1/da_i
            // where dz_i+1/da_i = Wt_i
            DynMat W_t = weight_matrices[i].transpose();
            DynMat prev_errors = W_t * error;

            // gradient computation
            //
            // sigma'(a_i+1) is the derivative of the sigmoid evaluated at the
            // activations
            //
            // the element wise multiply gives us
            // delta_i = error.element-wise_mult(sigma'(a_i+1))
            // which is dL/dz_i+1
            // this is the error signal gated by the neuron activity
            //
            // d_outputs has shape (units out x 1)
            DynMat d_outputs =
                activations[i + 1].apply_function(MLP::d_sigmoid);
            DynMat gradients = error.multiply_elementwise(d_outputs);
            gradients *= this->lr; // multiply by learning rate

            // weight gradients
            //
            // this is an outer product
            // gradients is (units out x 1), a_i^T is (1 x units_in), so the
            // result is (units out x units in), same shape as W each entry is:
            // dL/dWjk = delta_j * a_k
            // which is just from the chain rule, error at neuron j times the
            // activation that fed into it from neuron k
            DynMat a_t = activations[i].transpose();
            DynMat weight_gradients = gradients * a_t;

            // updates
            //
            // the bias gradient is just delta_i (as dz_i+1/db_i = 1), and then
            // error is replaced with prev errors so the loop continues
            bias_vectors[i] += gradients;
            weight_matrices[i] += weight_gradients;
            error = prev_errors;
        }
    };

    static MLP make_model(std::size_t in_channels, std::size_t out_channels,
                          std::size_t hidden_units_per_layer, int hidden_layers,
                          float lr) {
        std::vector<size_t> units_per_layer;
        units_per_layer.push_back(in_channels);

        for (int i = 0; i < hidden_layers; i++)
            units_per_layer.push_back(hidden_units_per_layer);

        units_per_layer.push_back(out_channels);

        MLP model(units_per_layer, lr);
        return model;
    };
};
} // namespace PhysicsEngine::NN
