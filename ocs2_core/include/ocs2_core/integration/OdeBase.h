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

#ifndef ODE_BASE_OCS2_H_
#define ODE_BASE_OCS2_H_

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

  /**
   * Constructor.
   * @param [in] modelData: The model data which will be used for storing outputs.
   */
  explicit OdeBase(const ModelDataBase& modelData = ModelDataBase())
      : numFunctionCalls_(0), defaultModelDataArraySize_(7), modelDataPtrArray_(defaultModelDataArraySize_) {
    for (auto& modelPtr_i : modelDataPtrArray_) {
      modelPtr_i.reset(modelData.clone());
    }
    nextModelDataPtrIterator() = beginModelDataPtrIterator();
    systemFunction_ = [this](const state_vector_t& x, state_vector_t& dxdt, scalar_t t) { computeFlowMap(t, x, dxdt); };
  }

  /**
   * Default destructor
   */
  virtual ~OdeBase() = default;

  /**
   * Default copy constructor
   */
  OdeBase(const OdeBase& rhs) : OdeBase(*rhs.modelDataPtrArray_.front()) {}

  /**
   * Get a system function callback that calls computeFlowMap for integration.
   */
  inline system_func_t systemFunction() { return systemFunction_; }

  /**
   * Gets the number of function calls.
   *
   * @return size_t: number of function calls
   */
  inline int getNumFunctionCalls() const { return numFunctionCalls_; }

  /**
   * Resets the number of function calls to zero.
   *
   */
  inline void resetNumFunctionCalls() { numFunctionCalls_ = 0; }

  /**
   * Returns the iterator pointing to the next free model data.
   * @return iterator to the next free model data.
   */
  inline std::list<std::shared_ptr<ModelDataBase>>::iterator& nextModelDataPtrIterator() { return nextModelDataPtrIterator_; }

  /**
   * Returns the iterator to begin()
   * @return modelDataPtrArray_.begin()
   */
  inline std::list<std::shared_ptr<ModelDataBase>>::iterator beginModelDataPtrIterator() { return modelDataPtrArray_.begin(); }

  /**
   * Returns the iterator to end()
   * @return modelDataPtrArray_.end()
   */
  inline std::list<std::shared_ptr<ModelDataBase>>::iterator endModelDataPtrIterator() { return modelDataPtrArray_.end(); }

  /**
   * Resizes the internal model data array.
   *
   */
  void resizeInternalModelDataPtrArray() {
    --nextModelDataPtrIterator_;  // since next will change
    for (int i = 0; i < defaultModelDataArraySize_; i++) {
      // TODO(mspieler): Better way to keep it a derived class?
      modelDataPtrArray_.emplace_back(modelDataPtrArray_.front()->clone());
    }
    ++nextModelDataPtrIterator_;  // new next
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
  const int defaultModelDataArraySize_;
  std::list<std::shared_ptr<ModelDataBase>> modelDataPtrArray_;
  std::list<std::shared_ptr<ModelDataBase>>::iterator nextModelDataPtrIterator_;
};

}  // namespace ocs2

#endif /* ODE_BASE_OCS2_H_ */
