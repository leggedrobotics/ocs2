/******************************************************************************
Copyright (c) 2020, Johannes Pankert. All rights reserved.

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

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include "ocs2_core/cost/CostFunctionBase.h"

namespace ocs2 {

/**
 *
 * Quadratic Cost Function Base with Algorithmic Differentiation (i.e. Auto Differentiation).
 *
 * The intermediate cost term and the final cost term have the following form:
 * - \f$ L = 0.5(f(x,u,t))' (f(x,u,t)) \f$
 * - \f$ \Phi = 0.5(g(x,t))' (g(x,t)) \f$.
 *
 * The user provides implementations for f and g. The Jacobians of f and g are computed by auto differentiation.
 * This costfunction approximates the Hessians of L and \Phi by applying the chain rule and neglecting the terms
 * with hessians of f and g. Hess_{L} and H_{\Phi} are therefore guaranteed to be at least positive semidefinite.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam INTERMEDIATE_COST_DIM dim(image(f))
 * @tparam TERMINAL_COST_DIM dim(image(g))
 */
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
class QuadraticGaussNewtonCostBaseAD : public CostFunctionBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = CostFunctionBase<STATE_DIM, INPUT_DIM>;
  using typename BASE::dynamic_vector_array_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  using ad_interface_t = CppAdInterface<scalar_t>;
  using ad_scalar_t = typename ad_interface_t::ad_scalar_t;
  using ad_dynamic_vector_t = typename ad_interface_t::ad_dynamic_vector_t;
  using ad_intermediate_cost_vector_t = Eigen::Matrix<ad_scalar_t, INTERMEDIATE_COST_DIM, 1>;
  using ad_terminal_cost_vector_t = Eigen::Matrix<ad_scalar_t, TERMINAL_COST_DIM, 1>;

  using timeStateInput_vector_t = Eigen::Matrix<scalar_t, 1 + STATE_DIM + INPUT_DIM, 1>;
  using intermediate_cost_timeStateInput_matrix_t = Eigen::Matrix<scalar_t, INTERMEDIATE_COST_DIM, 1 + STATE_DIM + INPUT_DIM>;
  using terminal_cost_timeState_matrix_t = Eigen::Matrix<scalar_t, TERMINAL_COST_DIM, 1 + STATE_DIM>;
  using timeStateInput_matrix_t = Eigen::Matrix<scalar_t, 1 + STATE_DIM + INPUT_DIM, 1 + STATE_DIM + INPUT_DIM>;

  using timeState_vector_t = Eigen::Matrix<scalar_t, 1 + STATE_DIM, 1>;
  using timeState_rowVector_t = Eigen::Matrix<scalar_t, 1, 1 + STATE_DIM>;
  using timeState_matrix_t = Eigen::Matrix<scalar_t, 1 + STATE_DIM, 1 + STATE_DIM>;

  using intermediate_cost_vector_t = Eigen::Matrix<scalar_t, INTERMEDIATE_COST_DIM, 1>;
  using terminal_cost_vector_t = Eigen::Matrix<scalar_t, TERMINAL_COST_DIM, 1>;

  /**
   * Default constructor
   *
   */
  explicit QuadraticGaussNewtonCostBaseAD();

  /**
   * Copy constructor
   */
  QuadraticGaussNewtonCostBaseAD(const QuadraticGaussNewtonCostBaseAD& rhs);

  /**
   * Default destructor
   */
  virtual ~QuadraticGaussNewtonCostBaseAD() = default;

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

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u);

  void getIntermediateCost(scalar_t& L);

  void getIntermediateCostDerivativeTime(scalar_t& dLdt);

  void getIntermediateCostDerivativeState(state_vector_t& dLdx);

  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override;

  void getIntermediateCostDerivativeInput(input_vector_t& dLdu);

  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override;

  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) override;

  void getTerminalCost(scalar_t& Phi);

  void getTerminalCostDerivativeTime(scalar_t& dPhidt);

  void getTerminalCostDerivativeState(state_vector_t& dPhidx);

  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override;

 protected:
  /**
   * Gets a user-defined cost parameters, applied to the intermediate costs
   *
   * @param [in] time: Current time.
   * @return The cost function parameters at a certain time
   */
  virtual dynamic_vector_t getIntermediateParameters(scalar_t time) const { return dynamic_vector_t(0); }

  /**
   * Number of parameters for the intermediate cost function.
   * This number must be remain constant after the model libraries are created
   *
   * @return number of parameters
   */
  virtual size_t getNumIntermediateParameters() const { return 0; }

  /**
   * Gets a user-defined cost parameters, applied to the terminal costs
   *
   * @param [in] time: Current time.
   * @return The cost function parameters at a certain time
   */
  virtual dynamic_vector_t getTerminalParameters(scalar_t time) const { return dynamic_vector_t(0); }

  /**
   * Number of parameters for the terminal cost function.
   * This number must be remain constant after the model libraries are created
   *
   * @return number of parameters
   */
  virtual size_t getNumTerminalParameters() const { return 0; }

  /**
   * Interface method to the cost term f such that the intermediate cost is
   * L = 0.5(f(x,u,t))' (f(x,u,t)).
   * This method must be implemented by the derived class.
   *
   * @tparam scalar type. All the floating point operations should be with this type.
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector.
   * @param [in] parameters: parameter vector.
   * @param [out] costValue: costValues = f(x,u,t).
   */
  virtual void intermediateCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                        const ad_dynamic_vector_t& parameters, ad_intermediate_cost_vector_t& costValues) const = 0;

  /**
   * Interface method to the cost term g such that the intermediate cost is
   * \Phi = 0.5(g(x,t))' (g(x,t))
   * This method may be implemented by the derived class.
   *
   * @tparam scalar type. All the floating point operations should be with this type.
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] parameters: parameter vector.
   * @param [out] costValue: costValues = g(x,t).
   */
  virtual void terminalCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& parameters,
                                    ad_terminal_cost_vector_t& costValues) const {
    costValues = ad_terminal_cost_vector_t::Zero();
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

  std::unique_ptr<ad_interface_t> terminalADInterfacePtr_;
  std::unique_ptr<ad_interface_t> intermediateADInterfacePtr_;

  // Intermediate cost
  bool intermediateCostValuesComputed_;
  intermediate_cost_vector_t intermediateCostValues_;
  bool intermediateDerivativesComputed_;
  dynamic_vector_t intermediateParameters_;
  timeStateInput_vector_t tapedTimeStateInput_;
  intermediate_cost_timeStateInput_matrix_t intermediateJacobian_;

  // Final cost
  bool terminalCostValuesComputed_;
  terminal_cost_vector_t terminalCostValues_;
  bool terminalDerivativesComputed_;
  dynamic_vector_t terminalParameters_;
  timeState_vector_t tapedTimeState_;
  terminal_cost_timeState_matrix_t terminalJacobian_;
};

}  // namespace ocs2

#include "implementation/QuadraticGaussNewtonCostBaseAD.h"
