/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <map>
#include <memory>

#include <ocs2_core/Types.h>

#include "StateConstraint.h"
#include "StateInputConstraint.h"

namespace ocs2 {

/**
 * Constraint collection class
 *
 * This class collects a variable number of constraint functions and provides methods to get the
 * concatenated constraint vectors and approximations. Each constraint can be accessed through its
 * string name and can be activated or deactivated.
 *
 * @tparam CONSTRAINT the constraint type: StateInputConstraint or StateConstraint
 */
template <typename CONSTRAINT>
class ConstraintCollection final {
 public:
  using LinearApproximation_t = VectorFunctionLinearApproximation;
  using QuadraticApproximation_t = VectorFunctionQuadraticApproximation;

  ConstraintCollection() = default;
  ~ConstraintCollection() = default;

  /** Copy constructor */
  ConstraintCollection(const ConstraintCollection<CONSTRAINT>& rhs);

  /** Move constructor */
  ConstraintCollection(ConstraintCollection<CONSTRAINT>&& rhs) noexcept;

  /** Copy assignment */
  ConstraintCollection<CONSTRAINT>& operator=(const ConstraintCollection<CONSTRAINT>& rhs);

  /** Move assignment */
  ConstraintCollection<CONSTRAINT>& operator=(ConstraintCollection<CONSTRAINT>&& rhs);

  /**
   * Adds a constraint to the collection, and transfer ownership to the collection
   * The provided name must be unique and is later used to access the constraint.
   * @param constraintTerm: Constraint to be added.
   * @param name: Name stored along with the constraint.
   */
  void add(std::string name, std::unique_ptr<CONSTRAINT> constraintTerm);

  /**
   * Use to modify a constraint. The returned pointer is not to be stored since the ConstraintCollection contains a unique
   * pointer to the object
   * @tparam Derived: derived class of ConstraintTerm to cast to. Casts to the base class by default
   * @param name: Name of the constraint to modify
   * @return A reference to the underlying constraint
   */
  template <typename Derived = CONSTRAINT>
  Derived& get(const std::string& name) {
    static_assert(std::is_base_of<CONSTRAINT, Derived>::value, "Template argument must derive from CONSTRAINT");
    // if the key does not exist throws an exception
    return dynamic_cast<Derived&>(*constraintTermMap_.at(name));
  }

  /** Returns the number of active constraints at given time. */
  size_t getNumConstraints(scalar_t time) const;

  /* State-input constraints */
  template <typename T = CONSTRAINT>
  ifStateInputConstraint_t<T, vector_t> getValue(scalar_t time, const vector_t& state, const vector_t& input) const;
  template <typename T = CONSTRAINT>
  ifStateInputConstraint_t<T, LinearApproximation_t> getLinearApproximation(scalar_t time, const vector_t& state,
                                                                            const vector_t& input) const;
  template <typename T = CONSTRAINT>
  ifStateInputConstraint_t<T, QuadraticApproximation_t> getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                  const vector_t& input) const;

  /* State-only constraints */
  template <typename T = CONSTRAINT>
  ifStateConstraint_t<T, vector_t> getValue(scalar_t time, const vector_t& state) const;
  template <typename T = CONSTRAINT>
  ifStateConstraint_t<T, LinearApproximation_t> getLinearApproximation(scalar_t time, const vector_t& state) const;
  template <typename T = CONSTRAINT>
  ifStateConstraint_t<T, QuadraticApproximation_t> getQuadraticApproximation(scalar_t time, const vector_t& state) const;

 private:
  /**
   * Appends the vector v1 by moving v2 which allows for the efficient transfer of resources.
   */
  template <typename T, typename Allocator>
  void appendVectorToVectorByMoving(std::vector<T, Allocator>& v1, std::vector<T, Allocator>& v2) const;

 private:
  std::map<std::string, std::unique_ptr<CONSTRAINT>> constraintTermMap_;
};

// explicit template instantiation
extern template class ConstraintCollection<StateInputConstraint>;
extern template class ConstraintCollection<StateConstraint>;

}  // namespace ocs2
