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

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>

#include "StateCost.h"

namespace ocs2 {

/**
 * Cost function combining a collection of cost terms.
 *
 * This class collects a variable number of cost terms and provides methods to get the
 * summed cost values and quadratic approximations. Each cost term can be accessed through its
 * string name and can be activated or deactivated.
 */
class StateCostCollection : StateCost {
 public:
  StateCostCollection() = default;
  ~StateCostCollection() override = default;
  StateCostCollection* clone() const override;

  /**
   * Adds a cost term to the collection, and transfer ownership to the collection
   * The provided name must be unique and is later used to access the cost term.
   * @param name: Name stored along with the cost term.
   * @param cost: Cost to be added.
   */
  void add(std::string name, std::unique_ptr<StateCost> cost);

  /**
   * Use to modify a cost term. The returned pointer is not to be stored since the StateCostCollection
   * contains a unique pointer to the object
   * @tparam Derived: derived class of StateCost to cast to. Casts to the base class by default
   * @param name: Name of the cost term to modify
   * @return A reference to the underlying cost term
   */
  template <typename Derived = StateCost>
  Derived& get(const std::string& name) {
    static_assert(std::is_base_of<StateCost, Derived>::value, "Template argument must derive from StateCost");
    // if the key does not exist throws an exception
    return dynamic_cast<Derived&>(*costTermMap_.at(name));
  }

  /** Get state-only cost value */
  scalar_t getValue(scalar_t time, const vector_t& state, const CostDesiredTrajectories& desiredTrajectory,
                    const PreComputation* preCompPtr) const override;

  /** Get state-only cost quadratic approximation */
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const CostDesiredTrajectories& desiredTrajectory,
                                                                 const PreComputation* preCompPtr) const override;

 protected:
  /** Copy constructor */
  StateCostCollection(const StateCostCollection& other);

 private:
  std::map<std::string, std::unique_ptr<StateCost>> costTermMap_;
};

}  // namespace ocs2
