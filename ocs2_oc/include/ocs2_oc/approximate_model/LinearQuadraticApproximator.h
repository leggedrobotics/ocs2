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
#include <Eigen/StdVector>
#include <memory>
#include <vector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/SystemEventHandler.h>
#include <ocs2_core/misc/LinearAlgebra.h>

namespace ocs2 {

/**
 * This class is an interface class for constructing LQ approximation of the continous time optimal control problem.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class LinearQuadraticApproximator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using derivatives_base_t = DerivativesBase<STATE_DIM, INPUT_DIM>;
  using constraint_base_t = ConstraintBase<STATE_DIM, INPUT_DIM>;
  using cost_function_base_t = CostFunctionBase<STATE_DIM, INPUT_DIM>;

  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using eigen_scalar_t = typename DIMENSIONS::eigen_scalar_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using constraint1_vector_t = typename DIMENSIONS::constraint1_vector_t;
  using constraint1_state_matrix_t = typename DIMENSIONS::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename DIMENSIONS::constraint1_input_matrix_t;
  using input_constraint1_matrix_t = typename DIMENSIONS::input_constraint1_matrix_t;
  using constraint2_vector_t = typename DIMENSIONS::constraint2_vector_t;
  using constraint2_state_matrix_t = typename DIMENSIONS::constraint2_state_matrix_t;

  /**
   * Constructor
   *
   * @param [in] systemDerivatives: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraints: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunction: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] checkNumericalCharacteristics: check for the expected numerical characteristics of the model (default true)
   * @param [in] makePsdWillBePerformedLater: Whether or not the model will be rectified later outside of this class.
   */
  LinearQuadraticApproximator(const derivatives_base_t& systemDerivatives, const constraint_base_t& systemConstraints,
                              const cost_function_base_t& costFunction, const char algorithmName[] = nullptr,
                              bool checkNumericalCharacteristics = true, bool makePsdWillBePerformedLater = false)
      : systemDerivativesPtr_(systemDerivatives.clone()),
        systemConstraintsPtr_(systemConstraints.clone()),
        costFunctionPtr_(costFunction.clone()),
        algorithmName_(algorithmName),
        checkNumericalCharacteristics_(checkNumericalCharacteristics),
        makePsdWillBePerformedLater_(makePsdWillBePerformedLater) {}

  /**
   * Default destructor.
   */
  ~LinearQuadraticApproximator() = default;

  /**
   * Whether or not to check the numerical characteristics of the model.
   *
   * @param [in] checkNumericalCharacteristics: True if the numerical characteristics of the model should be checked.
   */
  void checkNumericalCharacteristics(bool checkNumericalCharacteristics) { checkNumericalCharacteristics_ = checkNumericalCharacteristics; }

  /**
   * Returns the system derivatives
   */
  derivatives_base_t& systemDerivatives() { return *systemDerivativesPtr_; }

  /**
   * Returns the constraints.
   */
  constraint_base_t& systemConstraints() { return *systemConstraintsPtr_; }

  /**
   * Returns the intermediate cost.
   */
  cost_function_base_t& costFunction() { return *costFunctionPtr_; }

  /**
   * Calculates an LQ approximate of the unconstrained optimal control problem at a given time, state, and input.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] input: The current input .
   */
  void approximateUnconstrainedLQProblem(const scalar_t& time, const state_vector_t& state, const input_vector_t& input, state_matrix_t& Am,
                                         state_input_matrix_t& Bm, size_t& ncEqStateInput, constraint1_vector_t& Ev,
                                         constraint1_state_matrix_t& Cm, constraint1_input_matrix_t& Dm, size_t& ncEqStateOnly,
                                         constraint2_vector_t& Hv, constraint2_state_matrix_t& Fm, size_t& ncIneq, scalar_array_t& h,
                                         state_vector_array_t& dhdx, input_vector_array_t& dhdu, state_matrix_array_t& ddhdxdx,
                                         input_matrix_array_t& ddhdudu, input_state_matrix_array_t& ddhdudx, eigen_scalar_t& q,
                                         state_vector_t& Qv, state_matrix_t& Qm, input_vector_t& Rv, input_matrix_t& Rm,
                                         input_state_matrix_t& Pm) {
    approximateDynamics(time, state, input, Am, Bm);
    approximateConstraints(time, state, input, ncEqStateInput, Ev, Cm, Dm, ncEqStateOnly, Hv, Fm, ncIneq, h, dhdx, dhdu, ddhdxdx, ddhdudu,
                           ddhdudx);
    approximateIntermediateCost(time, state, input, q, Qv, Qm, Rv, Rm, Pm);
  }

  /**
   * Calculates an LQ approximate of the event times process.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] input: The current input .
   */
  void approximateUnconstrainedLQProblemAtEventTime(const scalar_t& time, const state_vector_t& state, const input_vector_t& input) {
    systemConstraintsPtr_->setCurrentStateAndControl(time, state, input);
    systemConstraintsPtr_->setCurrentStateAndControl(time, state, input);

    // Final state-only equality constraint
    ncFinalEqStateOnly_ = systemConstraintsPtr_->numStateOnlyFinalConstraint(time);

    if (ncFinalEqStateOnly_ > INPUT_DIM) {
      throw std::runtime_error(
          "Number of active final type-2 constraints should be "
          "less-equal to the number of input dimension.");
    }

    // if final constraint type 2 is active
    if (ncFinalEqStateOnly_ > 0) {
      systemConstraintsPtr_->getFinalConstraint2(HvFinal_);
      systemConstraintsPtr_->getFinalConstraint2DerivativesState(FmFinal_);
    }

    // Final cost
    costFunctionPtr_->getTerminalCost(qFinal_(0));
    costFunctionPtr_->getTerminalCostDerivativeState(QvFinal_);
    costFunctionPtr_->getTerminalCostSecondDerivativeState(QmFinal_);
  }

  /**
   * Calculates linearize system dynamics.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] input: The current input .
   */
  void approximateDynamics(const scalar_t& time, const state_vector_t& state, const input_vector_t& input, state_matrix_t& Am,
                           state_input_matrix_t& Bm) {
    // set data
    systemDerivativesPtr_->setCurrentStateAndControl(time, state, input);

    // get results
    systemDerivativesPtr_->getFlowMapDerivativeState(Am);
    systemDerivativesPtr_->getFlowMapDerivativeInput(Bm);

    // checking the numerical stability
    if (checkNumericalCharacteristics_) {
      try {
        if (!Am.allFinite()) {
          throw std::runtime_error("Flow map state derivativeState is not finite.");
        }
        if (!Bm.allFinite()) {
          throw std::runtime_error("Flow map input derivativeState is not finite.");
        }

      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << time << " [sec]." << std::endl;
        std::cerr << "Am: \n" << Am << std::endl;
        std::cerr << "Bm: \n" << Bm << std::endl;
        throw;
      }
    }
  }

  /**
   * Calculates the constraints and its linearized approximation.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] input: The current input .
   */
  void approximateConstraints(const scalar_t& time, const state_vector_t& state, const input_vector_t& input, size_t& ncEqStateInput,
                              constraint1_vector_t& Ev, constraint1_state_matrix_t& Cm, constraint1_input_matrix_t& Dm,
                              size_t& ncEqStateOnly, constraint2_vector_t& Hv, constraint2_state_matrix_t& Fm, size_t& ncIneq,
                              scalar_array_t& h, state_vector_array_t& dhdx, input_vector_array_t& dhdu, state_matrix_array_t& ddhdxdx,
                              input_matrix_array_t& ddhdudu, input_state_matrix_array_t& ddhdudx) {
    // set data
    systemConstraintsPtr_->setCurrentStateAndControl(time, state, input);

    // constraint type 1
    ncEqStateInput = systemConstraintsPtr_->numStateInputConstraint(time);
    if (ncEqStateInput > INPUT_DIM) {
      throw std::runtime_error("Number of active type-1 constraints should be less-equal to the number of input dimension.");
    }
    // if constraint type 1 is active
    if (ncEqStateInput > 0) {
      systemConstraintsPtr_->getConstraint1(Ev);
      systemConstraintsPtr_->getConstraint1DerivativesState(Cm);
      systemConstraintsPtr_->getConstraint1DerivativesControl(Dm);
    }

    // constraint type 2
    ncEqStateOnly = systemConstraintsPtr_->numStateOnlyConstraint(time);
    if (ncEqStateOnly > INPUT_DIM) {
      throw std::runtime_error("Number of active type-2 constraints should be less-equal to the number of input dimension.");
    }
    // if constraint type 2 is active
    if (ncEqStateOnly > 0) {
      systemConstraintsPtr_->getConstraint2(Hv);
      systemConstraintsPtr_->getConstraint2DerivativesState(Fm);
    }

    // Inequality constraint
    ncIneq = systemConstraintsPtr_->numInequalityConstraint(time);
    if (ncIneq > 0) {
      systemConstraintsPtr_->getInequalityConstraint(h);
      systemConstraintsPtr_->getInequalityConstraintDerivativesState(dhdx);
      systemConstraintsPtr_->getInequalityConstraintDerivativesInput(dhdu);
      systemConstraintsPtr_->getInequalityConstraintSecondDerivativesState(ddhdxdx);
      systemConstraintsPtr_->getInequalityConstraintSecondDerivativesInput(ddhdudu);
      systemConstraintsPtr_->getInequalityConstraintDerivativesInputState(ddhdudx);
    }

    if (checkNumericalCharacteristics_) {
      try {
        if (ncEqStateInput > 0) {
          if (!Ev.head(ncEqStateInput).allFinite()) {
            throw std::runtime_error("Input-state constraint is not finite.");
          }
          if (!Cm.topRows(ncEqStateInput).allFinite()) {
            throw std::runtime_error("Input-state constraint derivative w.r.t. state is not finite.");
          }
          if (!Dm.topRows(ncEqStateInput).allFinite()) {
            throw std::runtime_error("Input-state constraint derivative w.r.t. input is not finite.");
          }
          size_t DmRank = LinearAlgebra::rank(Dm.topRows(ncEqStateInput));
          if (DmRank != ncEqStateInput) {
            throw std::runtime_error(
                "Input-state constraint derivative w.r.t. input is not full-row rank. It's rank "
                "is " +
                std::to_string(DmRank) + " while the expected rank is " + std::to_string(ncEqStateInput) + ".");
          }
        }
        if (ncEqStateOnly > 0) {
          if (!Hv.head(ncEqStateOnly).allFinite()) {
            throw std::runtime_error("State-only constraint is not finite.");
          }
          if (!Fm.topRows(ncEqStateOnly).allFinite()) {
            throw std::runtime_error("State-only constraint derivative w.r.t. state is not finite.");
          }
        }

      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << time << " [sec]." << std::endl;
        std::cerr << "Ev: " << Ev.head(ncEqStateInput).transpose() << std::endl;
        std::cerr << "Cm: \n" << Cm.topRows(ncEqStateInput) << std::endl;
        std::cerr << "Dm: \n" << Dm.topRows(ncEqStateInput) << std::endl;
        std::cerr << "Hv: " << Hv.head(ncEqStateOnly).transpose() << std::endl;
        std::cerr << "Fm: \n" << Fm.topRows(ncEqStateOnly) << std::endl;
        throw;
      }
    }
  }

  /**
   * Calculates the intermediate cost function and its quadratic approximation.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] input: The current input .
   */
  void approximateIntermediateCost(const scalar_t& time, const state_vector_t& state, const input_vector_t& input, eigen_scalar_t& q,
                                   state_vector_t& Qv, state_matrix_t& Qm, input_vector_t& Rv, input_matrix_t& Rm,
                                   input_state_matrix_t& Pm) {
    // set data
    costFunctionPtr_->setCurrentStateAndControl(time, state, input);

    // get results
    costFunctionPtr_->getIntermediateCost(q(0));
    costFunctionPtr_->getIntermediateCostDerivativeState(Qv);
    costFunctionPtr_->getIntermediateCostSecondDerivativeState(Qm);
    costFunctionPtr_->getIntermediateCostDerivativeInput(Rv);
    costFunctionPtr_->getIntermediateCostSecondDerivativeInput(Rm);
    costFunctionPtr_->getIntermediateCostDerivativeInputState(Pm);

    // checking the numerical stability
    if (checkNumericalCharacteristics_) {
      try {
        if (!q.allFinite()) {
          throw std::runtime_error("Intermediate cost is is not finite.");
        }
        if (!Qv.allFinite()) {
          throw std::runtime_error("Intermediate cost first derivative w.r.t. state is is not finite.");
        }
        if (!Qm.allFinite()) {
          throw std::runtime_error("Intermediate cost second derivative w.r.t. state is is not finite.");
        }
        if (!makePsdWillBePerformedLater_ && !Qm.isApprox(Qm.transpose())) {
          throw std::runtime_error("Intermediate cost second derivative w.r.t. state is is not self-adjoint.");
        }
        if (!makePsdWillBePerformedLater_ && LinearAlgebra::eigenvalues(Qm).real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error("Q matrix is not positive semi-definite. It's smallest eigenvalue is " +
                                   std::to_string(LinearAlgebra::eigenvalues(Qm).real().minCoeff()) + ".");
        }
        if (!Rv.allFinite()) {
          throw std::runtime_error("Intermediate cost first derivative w.r.t. input is is not finite.");
        }
        if (!Rm.allFinite()) {
          throw std::runtime_error("Intermediate cost second derivative w.r.t. input is is not finite.");
        }
        if (!Pm.allFinite()) {
          throw std::runtime_error("Intermediate cost second derivative w.r.t. input-state is is not finite.");
        }
        if (!makePsdWillBePerformedLater_ && !Rm.isApprox(Rm.transpose())) {
          throw std::runtime_error("Intermediate cost second derivative w.r.t. input is is not self-adjoint.");
        }
        if (!makePsdWillBePerformedLater_ && Rm.ldlt().rcond() < Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error("R matrix is not invertible. It's reciprocal condition number is " + std::to_string(Rm.ldlt().rcond()) +
                                   ".");
        }
        if (!makePsdWillBePerformedLater_ && LinearAlgebra::eigenvalues(Rm).real().minCoeff() < Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error("R matrix is not positive definite. It's smallest eigenvalue is " +
                                   std::to_string(LinearAlgebra::eigenvalues(Rm).real().minCoeff()) + ".");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << time << " [sec]." << std::endl;
        std::cerr << "x: " << state.transpose() << std::endl;
        std::cerr << "u: " << input.transpose() << std::endl;
        std::cerr << "q: " << q << std::endl;
        std::cerr << "Qv: " << Qv.transpose() << std::endl;
        std::cerr << "Qm: \n" << Qm << std::endl;
        std::cerr << "Qm eigenvalues : " << LinearAlgebra::eigenvalues(Qm).transpose() << std::endl;
        std::cerr << "Rv: " << Rv.transpose() << std::endl;
        std::cerr << "Rm: \n" << Rm << std::endl;
        std::cerr << "Rm eigenvalues : " << LinearAlgebra::eigenvalues(Rm).transpose() << std::endl;
        std::cerr << "Pm: \n" << Pm << std::endl;
        throw;
      }
    }
  }

 public:
  size_t ncFinalEqStateOnly_;
  constraint2_vector_t HvFinal_;
  constraint2_state_matrix_t FmFinal_;

  eigen_scalar_t qFinal_;
  state_vector_t QvFinal_;
  state_matrix_t QmFinal_;

 private:
  std::unique_ptr<derivatives_base_t> systemDerivativesPtr_;
  std::unique_ptr<constraint_base_t> systemConstraintsPtr_;
  std::unique_ptr<cost_function_base_t> costFunctionPtr_;

  const char* algorithmName_;
  bool checkNumericalCharacteristics_;
  bool makePsdWillBePerformedLater_;
};

}  // namespace ocs2
