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
#include <string>
#include <type_traits>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include "StateCost.h"
#include "StateInputCost.h"

namespace ocs2 {

template <typename COST>
class CostCollection final {
 public:
  CostCollection() = default;
  ~CostCollection() = default;

  /** Copy constructor */
  CostCollection(const CostCollection<COST>& rhs);

  /** Move constructor */
  CostCollection(CostCollection<COST>&& rhs) noexcept;

  /** Copy assignment */
  CostCollection<COST>& operator=(const CostCollection<COST>& rhs);

  /** Move assignment */
  CostCollection<COST>& operator=(CostCollection<COST>&& rhs);

  /**
   * Adds a cost term to the collection, and transfer ownership to the collection
   * The provided name must be unique and is later used to access the cost term.
   * @param name: Name stored along with the cost term.
   * @param cost: Cost to be added.
   */
  void add(std::string name, std::unique_ptr<COST> cost);

  /**
   * Use to modify a cost term. The returned pointer is not to be stored since the CostCollection
   * contains a unique pointer to the object
   * @tparam Derived: derived class of COST to cast to. Casts to the base class by default
   * @param name: Name of the cost term to modify
   * @return A reference to the underlying cost term
   */
  template <typename Derived = COST>
  Derived& get(const std::string& name) {
    static_assert(std::is_base_of<COST, Derived>::value, "Template argument must derive from COST");
    // if the key does not exist throws an exception
    return dynamic_cast<Derived&>(*costTermMap_.at(name));
  }

  /** Get state-input cost value */
  template <typename T = COST, EnableIfStateInputCost_t<T> = true>
  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories) const;

  /** Get state-input cost quadratic approximation */
  template <typename T = COST, EnableIfStateInputCost_t<T> = true>
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const TargetTrajectories& targetTrajectories) const;

  /** Get state-only cost value */
  template <typename T = COST, EnableIfStateCost_t<T> = true>
  scalar_t getValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const;

  /** Get state-only cost quadratic approximation */
  template <typename T = COST, EnableIfStateCost_t<T> = true>
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const TargetTrajectories& targetTrajectories) const;

 private:
  std::map<std::string, std::unique_ptr<COST>> costTermMap_;
};

// explicit template instantiation
extern template class CostCollection<StateInputCost>;
extern template class CostCollection<StateCost>;

}  // namespace ocs2
