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

#include <functional>
#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/model_data/ModelDataBase.h>

namespace ocs2 {

/**
 * The base class for autonomous system dynamics.
 */
class OdeBase {
 public:
  // TODO(mspieler): change to our convention using return by value.
  // system interface for boost::odeint
  using system_func_t = std::function<void(const vector_t& x, vector_t& dxdt, scalar_t t)>;

  static constexpr size_t DEFAULT_MODEL_DATA_CACHE_SIZE = 7;

  /**
   * Constructor.
   */
  OdeBase();

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
  system_func_t systemFunction();

  /**
   * Gets the number of function calls.
   *
   * @return size_t: number of function calls
   */
  int getNumFunctionCalls() const;

  /**
   * Resets the number of function calls to zero.
   *
   */
  void resetNumFunctionCalls();

  /**
   * Returns model data array begin() iterator
   * @return modelDataArray_.begin()
   */
  std::vector<ModelDataBase>::iterator beginModelDataIterator();

  /**
   * Returns model data array end() iterator
   * @return modelDataArray_.end()
   */
  std::vector<ModelDataBase>::iterator endModelDataIterator();

  /**
   * Append model data array.
   * @return reference to new element.
   */
  ModelDataBase& modelDataEmplaceBack();

  /**
   * Clear model data array.
   */
  void clearModelDataArray();

  /**
   * Computes the autonomous system dynamics.
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @return Current state time derivative
   */
  virtual vector_t computeFlowMap(scalar_t t, const vector_t& x) = 0;

  /**
   * State map at the transition time
   *
   * @param [in] time: transition time
   * @param [in] state: transition state
   * @return mapped state after transition
   */
  virtual vector_t computeJumpMap(scalar_t time, const vector_t& state);

  /**
   * Interface method to the guard surfaces.
   *
   * @param [in] time: transition time
   * @param [in] state: transition state
   * @return An array of guard surfaces values
   */
  virtual vector_t computeGuardSurfaces(scalar_t time, const vector_t& state);

 protected:
  int numFunctionCalls_;
  system_func_t systemFunction_;
  std::vector<ModelDataBase> modelDataArray_;
};

}  // namespace ocs2
