//
// Created by rgrandia on 03.04.19.
//

#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <map>
#include <memory>
#include <vector>

#include <ocs2_core/Types.h>

#include "ocs2_switched_model_interface/constraint/ConstraintTerm.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class ConstraintCollection {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;

  using constraint_term_t = ConstraintTerm<STATE_DIM, INPUT_DIM>;
  using LinearApproximation_t = LinearConstraintApproximation<STATE_DIM, INPUT_DIM>;
  using LinearApproximationAsMatrices_t = LinearConstraintApproximationAsMatrices<STATE_DIM, INPUT_DIM>;
  using QuadraticApproximation_t = QuadraticConstraintApproximation<STATE_DIM, INPUT_DIM>;

  ConstraintCollection() = default;
  ~ConstraintCollection() = default;
  ConstraintCollection& operator=(const ConstraintCollection&) = delete;

  /**
   * Move constructor
   */
  ConstraintCollection(ConstraintCollection&& rhs) noexcept;

  /**
   * Copy constructor
   */
  ConstraintCollection(const ConstraintCollection& rhs);

  /**
   * Adds a constraint to the collection, and transfer ownership to the collection
   * The provided name must be unique and is later used to access the constraint.
   * @param constraintTerm: Constraint to be added.
   * @param name: Name stored along with the constraint.
   */
  void add(std::string name, std::unique_ptr<constraint_term_t> constraintTerm);

  /**
   * Use to modify a constraint. The returned pointer is not to be stored since the ConstraintCollection contains a unique
   * pointer to the object
   * @tparam Derived: derived class of ConstraintTerm to cast to. Casts to the base class by default
   * @param name: Name of the constraint to modify
   * @return A reference to the underlying constraint
   */
  template <typename Derived = constraint_term_t>
  Derived& get(const std::string& name);

  /**
   * Returns the name of the all the collected constraints.
   */
  std::vector<std::string> constraintNameList() const;

  size_t getNumConstraints(scalar_t time) const;

  // Gets as std::vectors
  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const;
  LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const;
  QuadraticApproximation_t getQuadraticApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const;

  // Gets as Eigen
  Eigen::VectorXd getValueAsVector(scalar_t time, const state_vector_t& state, const input_vector_t& input) const;
  LinearApproximationAsMatrices_t getLinearApproximationAsMatrices(scalar_t time, const state_vector_t& state,
                                                                   const input_vector_t& input) const;

 private:
  /**
   * Appends the vector v1 by moving v2 which allows for the efficient transfer of resources.
   */
  template <typename T, typename Allocator>
  void appendVectorToVectorByMoving(std::vector<T, Allocator>& v1, std::vector<T, Allocator>& v2) const;

  std::map<std::string, std::unique_ptr<constraint_term_t>> constraintTermMap_;
};

}  // namespace ocs2

#include "implementation/ConstraintCollection.h"
