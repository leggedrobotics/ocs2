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

#ifndef SYSTEMDYNAMICSBASEAD_OCS2_H_
#define SYSTEMDYNAMICSBASEAD_OCS2_H_

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include "ocs2_core/dynamics/SystemDynamicsBase.h"

namespace ocs2 {

/**
 * The system dynamics Base with Algorithmic Differentiation (i.e. Auto Differentiation).
 * The linearized system flow map is defined as: \n
 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * The linearized system jump map is defined as: \n
 * \f$ x^+ = G \delta x + H \delta u \f$ \n
 *
 * @tparam Derived: Derived class type.
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES = 1>
class SystemDynamicsBaseAD : public SystemDynamicsBase<STATE_DIM, INPUT_DIM, NUM_MODES> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = SystemDynamicsBase<STATE_DIM, INPUT_DIM, NUM_MODES>;

  using typename BASE::dynamic_input_matrix_t;
  using typename BASE::dynamic_state_matrix_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  using ad_interface_t = CppAdInterface<scalar_t>;
  using ad_scalar_t = typename ad_interface_t::ad_scalar_t;
  using ad_dynamic_vector_t = typename ad_interface_t::ad_dynamic_vector_t;

  using state_timeStateInput_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM, 1 + STATE_DIM + INPUT_DIM>;
  using state_timeState_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM, 1 + STATE_DIM>;
  using mode_timeState_matrix_t = Eigen::Matrix<scalar_t, NUM_MODES, 1 + STATE_DIM>;

  SystemDynamicsBaseAD();

  /**
   * Copy constructor
   */
  SystemDynamicsBaseAD(const SystemDynamicsBaseAD& rhs);

  /**
   * Default destructor
   */
  ~SystemDynamicsBaseAD() override = default;

  /**
   * Initializes model libraries
   *
   * @param modelName : name of the generate model library
   * @param modelFolder : folder to save the model library files to
   * @param recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
   * available.
   * @param verbose : print information.
   */
  void initialize(const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true,
                  bool verbose = true);

  void computeFlowMap(const scalar_t& time, const state_vector_t& state, const input_vector_t& input,
                      state_vector_t& stateDerivative) final;

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& jumpedState) final;

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) final;

  void setCurrentStateAndControl(const scalar_t& time, const state_vector_t& state, const input_vector_t& input) final;

  void getFlowMapDerivativeTime(state_vector_t& df) final { df = flowJacobian_.template leftCols<1>(); }

  void getFlowMapDerivativeState(state_matrix_t& A) final { A = flowJacobian_.template middleCols<STATE_DIM>(1); }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) final { B = flowJacobian_.template rightCols<INPUT_DIM>(); }

  void getJumpMapDerivativeTime(state_vector_t& dg) final { dg = jumpJacobian_.template leftCols<1>(); }

  void getJumpMapDerivativeState(state_matrix_t& G) final { G = jumpJacobian_.template rightCols<STATE_DIM>(); }

  void getGuardSurfacesDerivativeTime(dynamic_vector_t& D_t_gamma) final { D_t_gamma = guardJacobian_.template leftCols<1>(); }

  void getGuardSurfacesDerivativeState(dynamic_state_matrix_t& D_x_gamma) final {
    D_x_gamma = guardJacobian_.template rightCols<STATE_DIM>();
  }

 protected:
  /**
   * Interface method to the state flow map of the hybrid system. This method should be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector.
   * @param [out] stateDerivative: state vector time derivative.
   */
  virtual void systemFlowMap(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                             ad_dynamic_vector_t& stateDerivative) const = 0;

  /**
   * Interface method to the state jump map of the hybrid system. This method can be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [out] jumpedState: jumped state.
   */
  virtual void systemJumpMap(ad_scalar_t time, const ad_dynamic_vector_t& state, ad_dynamic_vector_t& jumpedState) const {
    jumpedState = state;
  }

  /**
   * Interface method to the guard surfaces. This method can be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state.
   * @param [in] input: input vector
   * @param [out] guardSurfacesValue: A vector of guard surfaces values
   */
  virtual void systemGuardSurfaces(ad_scalar_t time, const ad_dynamic_vector_t& state, ad_dynamic_vector_t& guardSurfacesValue) const {
    guardSurfacesValue = -ad_dynamic_vector_t::Ones(1);
  }

 private:
  /**
   * Sets all the required CppAdCodeGenInterfaces
   */
  void setADInterfaces(const std::string& modelName, const std::string& modelFolder);

  /**
   * Create the forward model and derivatives.
   *
   * @param [in] verbose: display information.
   */
  void createModels(bool verbose);

  /**
   * Loads the forward model and derivatives if available. Constructs them otherwise.
   *
   * @param [in] verbose: display information
   */
  void loadModelsIfAvailable(bool verbose);

  std::unique_ptr<ad_interface_t> flowMapADInterfacePtr_;
  std::unique_ptr<ad_interface_t> jumpMapADInterfacePtr_;
  std::unique_ptr<ad_interface_t> guardSurfacesADInterfacePtr_;

  state_timeStateInput_matrix_t flowJacobian_;
  state_timeState_matrix_t jumpJacobian_;
  mode_timeState_matrix_t guardJacobian_;
};

}  // namespace ocs2

#include "implementation/SystemDynamicsBaseAD.h"

#endif /* SYSTEMDYNAMICSBASEAD_OCS2_H_ */
