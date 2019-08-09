#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>
#include <Eigen/SVD>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
PythonInterface<STATE_DIM, INPUT_DIM>::PythonInterface(bool async)
    : run_mpc_async_(async), run_mpc_done_(false), run_mpc_requested_(false), shutdown_requested_(false) {}

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

template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::reset(cost_desired_trajectories_t targetTrajectories) {
  targetTrajectories_ = std::move(targetTrajectories);
  mpcMrtInterface_->resetMpcNode(targetTrajectories_);
  cost_->setCostDesiredTrajectories(targetTrajectories_);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::init(const std::string& taskFileFolder) {
  initRobotInterface(taskFileFolder);

  mpcMrtInterface_.reset(new MPC_MRT_Interface<STATE_DIM, INPUT_DIM>(robotInterface_->getMpc(), robotInterface_->getLogicRulesPtr()));

  dynamics_.reset(robotInterface_->getDynamics().clone());
  dynamicsDerivatives_.reset(robotInterface_->getDynamicsDerivatives().clone());
  cost_.reset(robotInterface_->getCost().clone());
  if (robotInterface_->getConstraintPtr()) {
    constraints_.reset(robotInterface_->getConstraintPtr()->clone());
  }

  if (run_mpc_async_) {
    run_mpc_worker_ = std::thread{&PythonInterface<STATE_DIM, INPUT_DIM>::runMpcAsync, this};
  }
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::setObservation(double t, Eigen::Ref<const state_vector_t> x) {
  SystemObservation<STATE_DIM, INPUT_DIM> observation;
  observation.time() = t;
  observation.state() = x;
  mpcMrtInterface_->setCurrentObservation(observation);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::setTargetTrajectories(cost_desired_trajectories_t targetTrajectories) {
  targetTrajectories_ = std::move(targetTrajectories);
  cost_->setCostDesiredTrajectories(targetTrajectories_);
  mpcMrtInterface_->setTargetTrajectories(targetTrajectories_);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::advanceMpc() {
  if (run_mpc_async_) {
    run_mpc_requested_ = true;
    run_mpc_cv_.notify_one();
  } else {
    mpcMrtInterface_->advanceMpc();
  }
}

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

  t = mpcMrtInterface_->getMpcTimeTrajectory();
  x = mpcMrtInterface_->getMpcStateTrajectory();
  u = mpcMrtInterface_->getMpcInputTrajectory();
}

template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::state_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::computeFlowMap(
    double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u) {
  state_vector_t dxdt;
  dynamics_->computeFlowMap(t, x, u, dxdt);
  return dxdt;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void PythonInterface<STATE_DIM, INPUT_DIM>::setFlowMapDerivativeStateAndControl(double t, Eigen::Ref<const state_vector_t> x,
                                                                                Eigen::Ref<const input_vector_t> u) {
  dynamicsDerivatives_->setCurrentStateAndControl(t, x, u);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::state_matrix_t PythonInterface<STATE_DIM, INPUT_DIM>::computeFlowMapDerivativeState() {
  state_matrix_t A;
  dynamicsDerivatives_->getFlowMapDerivativeState(A);
  return A;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::state_input_matrix_t
PythonInterface<STATE_DIM, INPUT_DIM>::computeFlowMapDerivativeInput() {
  state_input_matrix_t B;
  dynamicsDerivatives_->getFlowMapDerivativeInput(B);
  return B;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
double PythonInterface<STATE_DIM, INPUT_DIM>::getRunningCost(double t, Eigen::Ref<const state_vector_t> x,
                                                             Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  double L;
  cost_->getIntermediateCost(L);
  return L;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::state_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::getRunningCostDerivativeState(
    double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  state_vector_t dLdx;
  cost_->getIntermediateCostDerivativeState(dLdx);
  return dLdx;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::input_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::getRunningCostDerivativeInput(
    double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  input_vector_t dLdu;
  cost_->getIntermediateCostDerivativeInput(dLdu);
  return dLdu;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::state_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::getValueFunctionStateDerivative(
    double t, Eigen::Ref<const state_vector_t> x) {
  state_vector_t dVdx;
  mpcMrtInterface_->getValueFunctionStateDerivative(t, x, dVdx);
  return dVdx;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::dynamic_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::getStateInputConstraint(
    double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u) {
  constraints_->setCurrentStateAndControl(t, x, u);
  input_vector_t e;
  constraints_->getConstraint1(e);
  return e.head(constraints_->numStateInputConstraint(t));
}

template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::dynamic_matrix_t
PythonInterface<STATE_DIM, INPUT_DIM>::getStateInputConstraintDerivativeControl(double t, Eigen::Ref<const state_vector_t> x,
                                                                                Eigen::Ref<const input_vector_t> u) {
  constraints_->setCurrentStateAndControl(t, x, u);
  input_matrix_t D;
  constraints_->getConstraint1DerivativesControl(D);
  return D.topRows(constraints_->numStateInputConstraint(t));
}

template <size_t STATE_DIM, size_t INPUT_DIM>
typename PythonInterface<STATE_DIM, INPUT_DIM>::dynamic_vector_t PythonInterface<STATE_DIM, INPUT_DIM>::getStateInputConstraintLagrangian(
    double t, Eigen::Ref<const state_vector_t> x) {
  dynamic_vector_t nu;
  mpcMrtInterface_->calculateStateInputConstraintLagrangian(t, x, nu);
  return nu;
}

}  // namespace ocs2
