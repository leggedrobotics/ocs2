#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::PythonInterface(bool async)
    : run_mpc_async_(async), run_mpc_done_(false), run_mpc_requested_(false), shutdown_requested_(false) {}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::~PythonInterface() {
  if (run_mpc_async_) {
    std::unique_lock<std::mutex> lk(run_mpc_mutex_);
    shutdown_requested_ = true;
    lk.unlock();
    run_mpc_cv_.notify_one();
    run_mpc_worker_.join();
    std::cerr << "Successfully joined MPC worker thread" << std::endl;
  }
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::reset(const cost_desired_trajectories_t& targetTrajectories){
  targetTrajectories_ = targetTrajectories;
  mpcInterface_->reset(targetTrajectories);
  cost_->setCostDesiredTrajectories(targetTrajectories_);
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::init(const std::string& taskFileFolder){
  initRobotInterface(taskFileFolder);

  //TODO(jcarius) this static cast may be dangerous. Any way to avoid it?
  auto mpcPtr = static_cast<MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>*>(robotInterface_->getMPCPtr());
  auto logicPtr = dynamic_cast<LOGIC_RULES_T*>(robotInterface_->getLogicRulesPtr());
  if(logicPtr){
    mpcInterface_.reset(new mpc_t(mpcPtr, *logicPtr, true));
  } else {
    mpcInterface_.reset(new mpc_t(mpcPtr, LOGIC_RULES_T(), true));
  }

  dynamics_.reset(robotInterface_->getDynamicsPtr()->clone());
  dynamicsDerivatives_.reset(robotInterface_->getDynamicsDerivativesPtr()->clone());

  //TODO(jcarius) this static cast may be dangerous. Any way to avoid it?
  cost_ = static_cast<cost_t*>(robotInterface_->getCostPtr());

  if (run_mpc_async_) {
    run_mpc_worker_ = std::thread{&PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runMpcAsync, this};
  }
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setObservation(double t, Eigen::Ref<const state_vector_t> x) {
  typename mpc_t::system_observation_t observation;
  observation.time() = t;
  observation.state() = x;
  mpcInterface_->setCurrentObservation(observation);
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setTargetTrajectories(const cost_desired_trajectories_t& targetTrajectories) {
  targetTrajectories_ = targetTrajectories;
  cost_->setCostDesiredTrajectories(targetTrajectories_);
  mpcInterface_->setTargetTrajectories(targetTrajectories);
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::advanceMpc() {
  if (run_mpc_async_) {
    run_mpc_requested_ = true;
    run_mpc_cv_.notify_one();
  } else {
    mpcInterface_->advanceMpc();
  }

  // set the correct logic rules once and hope they never change
//  static auto dummy = [this](){
        dynamics_->initializeModel(mpcInterface_->getLogicMachine(), 0);
        dynamicsDerivatives_->initializeModel(mpcInterface_->getLogicMachine(), 0);
        cost_->initializeModel(mpcInterface_->getLogicMachine(), 0);
//    };
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runMpcAsync() {
  std::cerr << "runMpcAsync started in thread " << std::this_thread::get_id() << std::endl;
  while (true) {
    std::unique_lock<std::mutex> lk(run_mpc_mutex_);
    run_mpc_cv_.wait(lk, [this] { return this->run_mpc_requested_ or this->shutdown_requested_; });
    if (shutdown_requested_) {
      return;
    }
    run_mpc_requested_ = false;
    mpcInterface_->advanceMpc();
    run_mpc_done_ = true;
    lk.unlock();
    run_mpc_cv_.notify_one();
  }
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getMpcSolution(scalar_array_t& t, state_vector_array_t& x,
                                                                          input_vector_array_t& u, state_matrix_array_t& sigmaX) {
  if (run_mpc_async_) {
    std::unique_lock<std::mutex> lk(run_mpc_mutex_);
    run_mpc_cv_.wait(lk, [this] { return this->run_mpc_done_; });
    run_mpc_done_ = false;
    lk.unlock();
  }

  // TODO(jcarius): should we interpolate here to expose constant time steps?!
  t = mpcInterface_->getMpcTimeTrajectory();
  x = mpcInterface_->getMpcStateTrajectory();
  u = mpcInterface_->getMpcInputTrajectory();

  sigmaX.clear();
  sigmaX.reserve(t.size());
  for (const auto& ti : t) {
    double alpha_squared = 0.1;  // damping of pseudoinverse in case ki is very small
    input_state_matrix_t ki;
    mpcInterface_->getLinearFeedbackGain(ti, ki);
    state_input_matrix_t kDagger =
        ki.transpose() * (ki * ki.transpose() + alpha_squared * input_matrix_t::Identity()).ldlt().solve(input_matrix_t::Identity());
//    std::cerr << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << std::endl;
//    std::cerr << "pyInterfaceCpp: k =\n" << ki << "\nkDagger\n" << kDagger << std::endl;
//    double beta = 0.01;  // fraction of u_max that corresponds to one std.dev.
//    input_vector_t uMaxSquared = input_vector_t::Constant(pow(100.0, 2));
    sigmaX.emplace_back(kDagger * kDagger.transpose());
//    std::cerr << "sigmaX\n" << sigmaX.back() << std::endl;
  }
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::state_vector_t
PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::computeFlowMap(double t, Eigen::Ref<const state_vector_t> x,
                                                                     Eigen::Ref<const input_vector_t> u) {
  state_vector_t dxdt;
  dynamics_->computeFlowMap(t, x, u, dxdt);
  return dxdt;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setFlowMapDerivativeStateAndControl(double t, Eigen::Ref<const state_vector_t> x,
                                                                                               Eigen::Ref<const input_vector_t> u) {
  dynamicsDerivatives_->setCurrentStateAndControl(t, x, u);
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::state_matrix_t
PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::computeFlowMapDerivativeState() {
  state_matrix_t A;
  dynamicsDerivatives_->getFlowMapDerivativeState(A);
  return A;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::state_input_matrix_t
PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::computeFlowMapDerivativeInput() {
  state_input_matrix_t B;
  dynamicsDerivatives_->getFlowMapDerivativeInput(B);
  return B;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
double PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getRunningCost(double t, Eigen::Ref<const state_vector_t> x,
                                                                            Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  double L;
  cost_->getIntermediateCost(L);
  return L;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::state_vector_t
PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getRunningCostDerivativeState(double t, Eigen::Ref<const state_vector_t> x,
                                                                                    Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  state_vector_t dLdx;
  cost_->getIntermediateCostDerivativeState(dLdx);
  return dLdx;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::input_vector_t
PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getRunningCostDerivativeInput(double t, Eigen::Ref<const state_vector_t> x,
                                                                                    Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  input_vector_t dLdu;
  cost_->getIntermediateCostDerivativeInput(dLdu);
  return dLdu;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::state_vector_t
PythonInterface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getValueFunctionStateDerivative(double t, Eigen::Ref<const state_vector_t> x) {
  state_vector_t dVdx;
  mpcInterface_->getValueFunctionStateDerivative(t, x, dVdx);
  return dVdx;
}

}  // namespace ocs2
