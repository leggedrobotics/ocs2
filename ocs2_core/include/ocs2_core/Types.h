//
// Created by rgrandia on 24.03.20.
//

#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

namespace ocs2 {

/** size_t trajectory type. */
using size_array_t = std::vector<size_t>;
/** Array of size_t trajectory type. */
using size_array2_t = std::vector<size_array_t>;

/** Scalar type. */
using scalar_t = double;
/** Scalar trajectory type. */
using scalar_array_t = std::vector<scalar_t>;
/** Array of scalar trajectory type. */
using scalar_array2_t = std::vector<scalar_array_t>;
/** Array of arrays of scalar trajectory type. */
using scalar_array3_t = std::vector<scalar_array2_t>;

/** Dynamic-size vector type. */
using dynamic_vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
/** Dynamic vector's trajectory type. */
using dynamic_vector_array_t = std::vector<dynamic_vector_t, Eigen::aligned_allocator<dynamic_vector_t>>;
/** Array of dynamic vector's trajectory type. */
using dynamic_vector_array2_t = std::vector<dynamic_vector_array_t, Eigen::aligned_allocator<dynamic_vector_array_t>>;

/** Dynamic-size matrix type. */
using dynamic_matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
/** Dynamic matrix's trajectory type. */
using dynamic_matrix_array_t = std::vector<dynamic_matrix_t, Eigen::aligned_allocator<dynamic_matrix_t>>;
/** Array of dynamic matrix's trajectory type. */
using dynamic_matrix_array2_t = std::vector<dynamic_matrix_array_t, Eigen::aligned_allocator<dynamic_matrix_array_t>>;

/** Eigen scalar type. */
using eigen_scalar_t = Eigen::Matrix<scalar_t, 1, 1>;
/** Eigen scalar trajectory type. */
using eigen_scalar_array_t = std::vector<eigen_scalar_t, Eigen::aligned_allocator<eigen_scalar_t>>;
/** Array of eigen scalar trajectory type. */
using eigen_scalar_array2_t = std::vector<eigen_scalar_array_t, Eigen::aligned_allocator<eigen_scalar_array_t>>;
/** Array of arrays of eigen scalar trajectory type. */
using eigen_scalar_array3_t = std::vector<eigen_scalar_array2_t, Eigen::aligned_allocator<eigen_scalar_array2_t>>;

}  // namespace ocs2
