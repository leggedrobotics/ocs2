#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <memory>
#include <vector>

#include "ocs2_core/Dimensions.h"

namespace ocs2 {

/**
 * The base class for model data.
 */
struct ModelDataBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = typename Dimensions<0, 0>::scalar_t;
  using dynamic_vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
  using dynamic_matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;

  using self_t = ModelDataBase;
  using array_t = std::vector<self_t, Eigen::aligned_allocator<self_t>>;

  /**
   * Default constructor.
   */
  ModelDataBase() = default;

  /**
   * Default destructor.
   */
  virtual ~ModelDataBase() = default;

  /**
   * Copy constructor.
   */
  ModelDataBase(const ModelDataBase& rhs) = default;

  /**
   * Creates a deep copy of the object.
   * @warning Cloning implies that the caller takes ownership and deletes the created object.
   * @return Pointer to a new instance.
   */
  virtual ModelDataBase* clone() const { return new self_t(*this); }

  /**
   * Displays all variables
   */
  virtual void display() {
    std::cerr << std::endl;
    std::cerr << "time: " << time_ << "\n";
    std::cerr << "Flow Map: " << flowMap_.transpose() << "\n";
    std::cerr << "Flow Map State Derivative: " << flowMapStateDerivative_ << "\n";
    std::cerr << "Flow Map Input Derivative: " << flowMapInputDerivative_ << "\n";

    std::cerr << "cost: " << cost << "\n";
    std::cerr << "Cost State Derivative: " << costStateDerivative_ << "\n";
    std::cerr << "Cost Input Derivative: " << costInputDerivative_ << "\n";
    std::cerr << "Cost State Second Derivative: " << costStateSecondDerivative_ << "\n";
    std::cerr << "Cost Input Second Derivative: " << costInputSecondDerivative_ << "\n";
    std::cerr << "Cost Input State Derivative:  " << costInputStateDerivative_ << "\n";
    std::cerr << "Cost Input State Derivative:  " << costInputStateDerivative_ << "\n";
    std::cerr << std::endl;
  }

  /**
   * Variables
   */
  scalar_t time_;
  int stateDim_;
  int inputDim_;

  // dynamics
  dynamic_vector_t flowMap_;
  dynamic_matrix_t flowMapStateDerivative_;
  dynamic_matrix_t flowMapInputDerivative_;

  dynamic_vector_t jumpMap_;
  dynamic_matrix_t jumpMapStateDerivative_;
  dynamic_matrix_t jumpMapInputDerivative_;

  // cost
  scalar_t cost;
  dynamic_vector_t costStateDerivative_;
  dynamic_vector_t costInputDerivative_;
  dynamic_matrix_t costStateSecondDerivative_;
  dynamic_matrix_t costInputSecondDerivative_;
  dynamic_matrix_t costInputStateDerivative_;
};

}  // namespace ocs2
