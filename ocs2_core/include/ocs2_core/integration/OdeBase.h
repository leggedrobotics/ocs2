/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <Eigen/Dense>
#include <list>
#include <memory>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/model_data/ModelDataBase.h"

namespace ocs2 {

/**
 * The base class for autonomous system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class OdeBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<STATE_DIM, 0>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using system_func_t = std::function<void(const state_vector_t& x, state_vector_t& dxdt, scalar_t t)>;
  using model_data_array_t = ModelDataBase::array_t;

  static constexpr size_t DEFAULT_MODEL_DATA_CACHE_SIZE = 7;

  /**
   * Constructor.
   */
  OdeBase() : numFunctionCalls_(0) {
    modelDataArray_.reserve(DEFAULT_MODEL_DATA_CACHE_SIZE);
    systemFunction_ = [this](const state_vector_t& x, state_vector_t& dxdt, scalar_t t) { computeFlowMap(t, x, dxdt); };
  }

  /**
   * Default destructor
   */
  virtual ~OdeBase() = default;

  /**
   * Default copy constructor
   */
  OdeBase(const OdeBase& rhs) = default;

  /**
   * Get a system function callback that calls computeFlowMap for integration.
   */
  system_func_t systemFunction() { return systemFunction_; }

  /**
   * Gets the number of function calls.
   *
   * @return size_t: number of function calls
   */
  int getNumFunctionCalls() const { return numFunctionCalls_; }

  /**
   * Resets the number of function calls to zero.
   *
   */
  void resetNumFunctionCalls() { numFunctionCalls_ = 0; }

  /**
   * Returns model data array begin() iterator
   * @return modelDataArray_.begin()
   */
  model_data_array_t::iterator beginModelDataIterator() { return modelDataArray_.begin(); }

  /**
   * Returns model data array end() iterator
   * @return modelDataArray_.end()
   */
  model_data_array_t::iterator endModelDataIterator() { return modelDataArray_.end(); }

  /**
   * Append model data array.
   * @return reference to new element.
   */
  ModelDataBase& modelDataEmplaceBack() {
    modelDataArray_.emplace_back();
    return modelDataArray_.back();
  }

  /**
   * Clear model data array.
   */
  void clearModelDataArray() {
    modelDataArray_.clear();  // keeps allocated storage
  }

  /**
   * Computes the autonomous system dynamics.
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @param [out] dxdt: Current state time derivative
   */
  virtual void computeFlowMap(const scalar_t& t, const state_vector_t& x, state_vector_t& dxdt) = 0;

  /**
   * State map at the transition time
   *
   * @param [in] time: transition time
   * @param [in] state: transition state
   * @param [out] mappedState: mapped state after transition
   */
  virtual void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) { mappedState = state; }

  /**
   * Interface method to the guard surfaces.
   *
   * @param [in] time: transition time
   * @param [in] state: transition state
   * @param [out] guardSurfacesValue: An array of guard surfaces values
   */
  virtual void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) {
    guardSurfacesValue = -dynamic_vector_t::Ones(1);
  }

 protected:
  int numFunctionCalls_;
  system_func_t systemFunction_;
  model_data_array_t modelDataArray_;
};

}  // namespace ocs2
