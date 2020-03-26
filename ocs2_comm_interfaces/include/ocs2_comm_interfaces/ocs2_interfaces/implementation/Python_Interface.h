#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>
#include <Eigen/SVD>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
PythonInterface<STATE_DIM, INPUT_DIM>::PythonInterface(bool async)
    : run_mpc_async_(async), run_mpc_done_(false), run_mpc_requested_(false), shutdown_requested_(false) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
PythonInterface<STATE_DIM, INPUT_DIM>::~PythonInterface() {
  if (run_mpc_async_) {
    {
      std::lock_guard<std::mutex> lock(run_mpc_mutex_);
      shutdown_requested_ = true;
    }
    run_mpc_cv_.notify_one();
    run_mpc_worker_.join();
    std::cerr << "Successfully joined MPC worker thread" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::reset(CostDesiredTrajectories targetTrajectories) {
  targetTrajectories_ = std::move(targetTrajectories);
  mpcMrtInterface_->resetMpcNode(targetTrajectories_);
  cost_->setCostDesiredTrajectoriesPtr(&targetTrajectories_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::init(const RobotInterface<STATE_DIM, INPUT_DIM>& robotInterface,
                                                 std::unique_ptr<MPC_BASE<STATE_DIM, INPUT_DIM>> mpcPtr) {
  if (!mpcPtr) {
    throw std::runtime_error("[PythonInterface] Mpc pointer must be initialized before passing to the Python interface.");
  }
  mpcPtr_ = std::move(mpcPtr);

  mpcMrtInterface_.reset(new MPC_MRT_Interface<STATE_DIM, INPUT_DIM>(*mpcPtr_));

  dynamics_.reset(robotInterface.getDynamics().clone());
  dynamicsDerivatives_.reset(robotInterface.getDynamicsDerivatives().clone());
  cost_.reset(robotInterface.getCost().clone());
  if (robotInterface.getConstraintPtr()) {
    constraints_.reset(robotInterface.getConstraintPtr()->clone());
  }

  if (run_mpc_async_) {
    run_mpc_worker_ = std::thread{&PythonInterface<STATE_DIM, INPUT_DIM>::runMpcAsync, this};
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::setObservation(double t, Eigen::Ref<const state_vector_t> x) {
  SystemObservation<STATE_DIM, INPUT_DIM> observation;
  observation.time() = t;
  observation.state() = x;
  mpcMrtInterface_->setCurrentObservation(observation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::setTargetTrajectories(CostDesiredTrajectories targetTrajectories) {
  targetTrajectories_ = std::move(targetTrajectories);
  cost_->setCostDesiredTrajectoriesPtr(&targetTrajectories_);
  mpcMrtInterface_->setTargetTrajectories(targetTrajectories_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::advanceMpc() {
  if (run_mpc_async_) {
    run_mpc_requested_ = true;
    run_mpc_cv_.notify_one();
  } else {
    mpcMrtInterface_->advanceMpc();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::runMpcAsync() {
  std::cerr << "runMpcAsync started in thread " << std::this_thread::get_id() << std::endl;
  while (true) {
    std::unique_lock<std::mutex> lk(run_mpc_mutex_);
    run_mpc_cv_.wait(lk, [this] { return this->run_mpc_requested_ or this->shutdown_requested_; });
    if (shutdown_requested_) {
      return;
    }
    run_mpc_requested_ = false;
    mpcMrtInterface_->advanceMpc();
    run_mpc_done_ = true;
    lk.unlock();
    run_mpc_cv_.notify_one();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::getMpcSolution(scalar_array_t& t, state_vector_array_t& x, input_vector_array_t& u) {
  if (run_mpc_async_) {
    // make sure MPC is done before we continue
    std::unique_lock<std::mutex> lk(run_mpc_mutex_);
    run_mpc_cv_.wait(lk, [this] { return this->run_mpc_done_; });
    run_mpc_done_ = false;
    lk.unlock();
  }

  mpcMrtInterface_->updatePolicy();

  t = mpcMrtInterface_->getPolicy().timeTrajectory_;
  x = mpcMrtInterface_->getPolicy().stateTrajectory_;
  u = mpcMrtInterface_->getPolicy().inputTrajectory_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::input_state_matrix_t PythonInterface<STATE_DIM, INPUT_DIM>::getLinearFeedbackGain(
    scalar_t time) {
  input_state_matrix_t K;
  mpcMrtInterface_->getLinearFeedbackGain(time, K);
  return K;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::state_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::computeFlowMap(
    double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u) {
  state_vector_t dxdt;
  dynamics_->computeFlowMap(t, x, u, dxdt);
  return dxdt;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::setFlowMapDerivativeStateAndControl(double t, Eigen::Ref<const state_vector_t> x,
                                                                                Eigen::Ref<const input_vector_t> u) {
  dynamicsDerivatives_->setCurrentStateAndControl(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::state_matrix_t PythonInterface<STATE_DIM, INPUT_DIM>::computeFlowMapDerivativeState() {
  state_matrix_t A;
  dynamicsDerivatives_->getFlowMapDerivativeState(A);
  return A;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::state_input_matrix_t
PythonInterface<STATE_DIM, INPUT_DIM>::computeFlowMapDerivativeInput() {
  state_input_matrix_t B;
  dynamicsDerivatives_->getFlowMapDerivativeInput(B);
  return B;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
double PythonInterface<STATE_DIM, INPUT_DIM>::getIntermediateCost(double t, Eigen::Ref<const state_vector_t> x,
                                                                  Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  double L;
  cost_->getIntermediateCost(L);

  double L_penalty = 0.0;
  if (constraints_) {
    constraints_->setCurrentStateAndControl(t, x, u);
    if (constraints_->numInequalityConstraint(t) > 0) {
      scalar_array_t h;
      constraints_->getInequalityConstraint(h);
      penalty_->getPenaltyCost(h, L_penalty);
    }
  }

  return L + L_penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::state_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeState(
    double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  state_vector_t dLdx;
  cost_->getIntermediateCostDerivativeState(dLdx);

  state_vector_t dLdx_penalty;
  dLdx_penalty.setZero();

  if (constraints_) {
    constraints_->setCurrentStateAndControl(t, x, u);
    if (constraints_->numInequalityConstraint(t) > 0) {
      scalar_array_t h;
      constraints_->getInequalityConstraint(h);
      state_vector_array_t dhdx;
      constraints_->getInequalityConstraintDerivativesState(dhdx);
      penalty_->getPenaltyCostDerivativeState(h, dhdx, dLdx_penalty);
    }
  }

  return dLdx + dLdx_penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::input_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeInput(
    double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  input_vector_t dLdu;
  cost_->getIntermediateCostDerivativeInput(dLdu);

  input_vector_t dLdu_penalty;
  dLdu_penalty.setZero();

  if (constraints_) {
    constraints_->setCurrentStateAndControl(t, x, u);
    if (constraints_->numInequalityConstraint(t) > 0) {
      scalar_array_t h;
      constraints_->getInequalityConstraint(h);
      input_vector_array_t dhdu;
      constraints_->getInequalityConstraintDerivativesInput(dhdu);
      penalty_->getPenaltyCostDerivativeInput(h, dhdu, dLdu_penalty);
    }
  }

  return dLdu + dLdu_penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::input_matrix_t
PythonInterface<STATE_DIM, INPUT_DIM>::getIntermediateCostSecondDerivativeInput(double t, Eigen::Ref<const state_vector_t> x,
                                                                                Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  input_matrix_t ddLduu;
  cost_->getIntermediateCostSecondDerivativeInput(ddLduu);

  input_matrix_t ddLduu_penalty;
  ddLduu_penalty.setZero();

  if (constraints_) {
    constraints_->setCurrentStateAndControl(t, x, u);
    if (constraints_->numInequalityConstraint(t) > 0) {
      scalar_array_t h;
      constraints_->getInequalityConstraint(h);
      input_vector_array_t dhdu;
      constraints_->getInequalityConstraintDerivativesInput(dhdu);
      input_matrix_array_t ddhduu;
      constraints_->getInequalityConstraintSecondDerivativesInput(ddhduu);
      penalty_->getPenaltyCostSecondDerivativeInput(h, dhdu, ddhduu, ddLduu_penalty);
    }
  }

  return ddLduu + ddLduu_penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
double PythonInterface<STATE_DIM, INPUT_DIM>::getValueFunction(double t, Eigen::Ref<const state_vector_t> x) {
  return mpcMrtInterface_->getValueFunction(t, x);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::state_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::getValueFunctionStateDerivative(
    double t, Eigen::Ref<const state_vector_t> x) {
  state_vector_t dVdx;
  mpcMrtInterface_->getValueFunctionStateDerivative(t, x, dVdx);
  return dVdx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::dynamic_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::getStateInputConstraint(
    double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u) {
  if (!constraints_) {
    throw std::runtime_error("Cannot getStateInputConstraint if system has no constraints.");
  }
  constraints_->setCurrentStateAndControl(t, x, u);
  input_vector_t e;
  constraints_->getConstraint1(e);
  return e.head(constraints_->numStateInputConstraint(t));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::dynamic_matrix_t
PythonInterface<STATE_DIM, INPUT_DIM>::getStateInputConstraintDerivativeControl(double t, Eigen::Ref<const state_vector_t> x,
                                                                                Eigen::Ref<const input_vector_t> u) {
  if (!constraints_) {
    throw std::runtime_error("Cannot getStateInputConstraint if system has no constraints.");
  }
  constraints_->setCurrentStateAndControl(t, x, u);
  input_matrix_t D;
  constraints_->getConstraint1DerivativesControl(D);
  return D.topRows(constraints_->numStateInputConstraint(t));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::dynamic_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::getStateInputConstraintLagrangian(
    double t, Eigen::Ref<const state_vector_t> x) {
  dynamic_vector_t nu;
  //  mpcMrtInterface_->getStateInputConstraintLagrangian(t, x, nu);
  //  return nu;

  input_vector_t zero_u;
  zero_u.setZero();
  dynamic_matrix_t Dm = getStateInputConstraintDerivativeControl(t, x, zero_u);

  dynamic_vector_t c = getStateInputConstraint(t, x, zero_u);

  input_matrix_t R = getIntermediateCostSecondDerivativeInput(t, x, zero_u);
  input_vector_t r = getIntermediateCostDerivativeInput(t, x, zero_u);
  input_matrix_t RinvChol;
  LinearAlgebra::computeLinvTLinv(R, RinvChol);
  dynamic_matrix_t DmDager, DdaggerT_R_Ddagger_Chol, RmInvConstrainedChol;
  ocs2::LinearAlgebra::computeConstraintProjection(Dm, RinvChol, DmDager, DdaggerT_R_Ddagger_Chol, RmInvConstrainedChol);

  dynamicsDerivatives_->setCurrentStateAndControl(t, x, zero_u);
  state_input_matrix_t B;
  dynamicsDerivatives_->getFlowMapDerivativeInput(B);

  state_vector_t costate = getValueFunctionStateDerivative(t, x);

  nu = DmDager.transpose() * (R * DmDager * c - r - B.transpose() * costate);

  return nu;
}

}  // namespace ocs2
