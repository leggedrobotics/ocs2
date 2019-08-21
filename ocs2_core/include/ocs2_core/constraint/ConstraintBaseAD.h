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

#ifndef CONSTRAINTBASEAD_OCS2_H_
#define CONSTRAINTBASEAD_OCS2_H_

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include "ocs2_core/constraint/ConstraintBase.h"

namespace ocs2 {

/**
 * The system constraint Base with Algorithmic Differentiation (i.e. Auto Differentiation).
 * The linearized state-input constraint is defined as: \n
 * \f$ 0 = C(t) \delta x + D(t) \delta u + e(t) \f$ \n
 * The linearized state-only constraint is defined as: \n
 * \f$ 0 = F(t) \delta x + h(t) \f$ \n
 * The linearized final state-only constraint is defined as: \n
 * \f$ 0 = F_f \delta x + h_f \f$ \n
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class ConstraintBaseAD : public ConstraintBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t MAX_CONSTRAINT_DIM_ = INPUT_DIM;

  using BASE = ConstraintBase<STATE_DIM, INPUT_DIM>;

  using typename BASE::constraint1_input_matrix_t;
  using typename BASE::constraint1_state_matrix_t;
  using typename BASE::constraint1_vector_t;
  using typename BASE::constraint2_state_matrix_t;
  using typename BASE::constraint2_vector_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  using ad_interface_t = CppAdInterface<scalar_t>;
  using ad_scalar_t = typename ad_interface_t::ad_scalar_t;
  using ad_dynamic_vector_t = typename ad_interface_t::ad_dynamic_vector_t;
  using dynamic_vector_t = typename ad_interface_t::dynamic_vector_t;

  using constraint_timeStateInput_matrix_t = Eigen::Matrix<scalar_t, -1, 1 + STATE_DIM + INPUT_DIM>;
  using constraint_timeState_matrix_t = Eigen::Matrix<scalar_t, -1, 1 + STATE_DIM>;

  /**
   * Default constructor.
   *
   */
  ConstraintBaseAD();

  /**
   * Copy constructor
   */
  ConstraintBaseAD(const ConstraintBaseAD& rhs);

  /**
   * Default destructor
   *
  ~ConstraintBaseAD() override = default;

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

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) final;

  void getConstraint1(constraint1_vector_t& e) final { e = stateInputValues_; }

  void getConstraint2(constraint2_vector_t& h) final { h = stateOnlyValues_; }

  void getFinalConstraint2(constraint2_vector_t& h_f) final { h_f = stateOnlyFinalValues_; }

  void getConstraint1DerivativesState(constraint1_state_matrix_t& C) final { C = stateInputJacobian_.template middleCols<STATE_DIM>(1); }

  void getConstraint1DerivativesControl(constraint1_input_matrix_t& D) final { D = stateInputJacobian_.template rightCols<INPUT_DIM>(); }

  void getConstraint2DerivativesState(constraint2_state_matrix_t& F) final { F = stateOnlyJacobian_.template rightCols<STATE_DIM>(); }

  void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_f) final {
    F_f = stateOnlyFinalJacobian_.template rightCols<STATE_DIM>();
  }

 protected:
  /**
   * Interface method to the state-input equality constraints. This method should be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector
   * @param [out] constraintVector: constraints vector.
   */
  virtual void stateInputConstraint(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                    ad_dynamic_vector_t& constraintVector) const {
    constraintVector = ad_dynamic_vector_t(0);
  }

  /**
   * Interface method to the state-only equality constraints. This method should be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [out] constraintVector: constraint vector.
   */
  virtual void stateOnlyConstraint(ad_scalar_t time, const ad_dynamic_vector_t& state, ad_dynamic_vector_t& constraintVector) const {
    constraintVector = ad_dynamic_vector_t(0);
  }

  /**
   * Interface method to the state-only final equality constraints. This method should be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [out] constraintVector: constraint vector.
   */
  virtual void stateOnlyFinalConstraint(ad_scalar_t time, const ad_dynamic_vector_t& state, ad_dynamic_vector_t& constraintVector) const {
    constraintVector = ad_dynamic_vector_t(0);
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

  std::unique_ptr<ad_interface_t> stateInputADInterfacePtr_;
  std::unique_ptr<ad_interface_t> stateOnlyADInterfacePtr_;
  std::unique_ptr<ad_interface_t> stateOnlyFinalADInterfacePtr_;

  constraint_timeStateInput_matrix_t stateInputJacobian_;
  constraint_timeState_matrix_t stateOnlyJacobian_;
  constraint_timeState_matrix_t stateOnlyFinalJacobian_;
  constraint1_vector_t stateInputValues_;
  constraint2_vector_t stateOnlyValues_;
  constraint2_vector_t stateOnlyFinalValues_;
};

}  // namespace ocs2

#include "implementation/ConstraintBaseAD.h"

#endif /* CONSTRAINTBASEAD_OCS2_H_ */
