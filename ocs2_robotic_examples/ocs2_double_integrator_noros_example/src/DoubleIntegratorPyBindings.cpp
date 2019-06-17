

#include <ocs2_double_integrator_noros_example/DoubleIntegratorPyBindings.hpp>

namespace ocs2 {
namespace double_integrator {

DoubleIntegratorPyBindings::DoubleIntegratorPyBindings(const std::string& taskFileFolder, bool async)
    : run_mpc_async_(async), run_mpc_done_(false), run_mpc_requested_(false), shutdown_requested_(false) {
  std::cerr << "c++ DoubleIntegratorPyBindings instantiated with pid " << getpid() << " at address " << this << std::endl;

  doubleIntegratorInterface_.reset(new ocs2::double_integrator::DoubleIntegratorInterface(taskFileFolder));
  mpcInterface_.reset(new mpc_t(*doubleIntegratorInterface_->getMPCPtr(), nullLogicRules_, true));

  dynamics_.reset(doubleIntegratorInterface_->getDynamicsPtr()->clone());
  dynamicsDerivatives_.reset(doubleIntegratorInterface_->getDynamicsDerivativesPtr()->clone());

  cost_.reset(doubleIntegratorInterface_->getCostPtr()->clone());
  // TODO(jcarius) should we set the costDesiredTrajectories here?

  // initialize reference
  constexpr double time = 0.0;
  mpc_t::cost_desired_trajectories_t costDesiredTrajectories;
  costDesiredTrajectories.desiredTimeTrajectory().push_back(time);
  auto goalState = doubleIntegratorInterface_->getXFinal();
  costDesiredTrajectories.desiredStateTrajectory().push_back(goalState);
  auto desiredInput = mpc_t::input_vector_t::Zero();
  costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
  mpcInterface_->setTargetTrajectories(costDesiredTrajectories);

  if (run_mpc_async_) {
    run_mpc_worker_ = std::thread{&DoubleIntegratorPyBindings::runMpcAsync, this};
  }
}

DoubleIntegratorPyBindings::~DoubleIntegratorPyBindings() {
  if (run_mpc_async_) {
    std::unique_lock<std::mutex> lk(run_mpc_mutex_);
    shutdown_requested_ = true;
    lk.unlock();
    run_mpc_worker_.join();
    std::cerr << "Successfully joined MPC worker thread" << std::endl;
  }
}

void DoubleIntegratorPyBindings::setObservation(double t, Eigen::Ref<const state_vector_t> x) {
  mpc_t::system_observation_t observation;
  observation.time() = t;
  observation.state() = x;
  mpcInterface_->setCurrentObservation(observation);
}

void DoubleIntegratorPyBindings::advanceMpc() {
  if (run_mpc_async_) {
    run_mpc_requested_ = true;
    run_mpc_cv_.notify_one();
  } else {
    mpcInterface_->advanceMpc();
  }
}

void DoubleIntegratorPyBindings::runMpcAsync() {
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

void DoubleIntegratorPyBindings::getMpcSolution(scalar_array_t& t, state_vector_array_t& x, input_vector_array_t& u, state_matrix_array_t& sigmaX) {
  if (run_mpc_async_) {
    std::unique_lock<std::mutex> lk(run_mpc_mutex_);
    run_mpc_cv_.wait(lk, [this] { return this->run_mpc_done_; });
    run_mpc_done_ = false;
    lk.unlock();
  }

  // TODO(jcarius): should we interpolate here to expose constant time steps?!
  input_state_matrix_array_t k;
  mpcInterface_->getMpcSolution(t, x, u, k);

  sigmaX.clear();
  sigmaX.reserve(k.size());
  for(const auto& ki : k){
      double alpha = 10.0; // damping of pseudoinverse in case ki is very small
      state_input_matrix_t kDagger = ki.transpose() * (ki * ki.transpose() + alpha * input_vector_t::Constant(alpha)).ldlt().solve(input_vector_t::Identity());
      double beta = 0.01; // fraction of u_max that corresponds to one std.dev.
      input_vector_t uMaxSquared  = input_vector_t::Constant(pow(100.0, 2));
      sigmaX.emplace_back(kDagger * beta * uMaxSquared.asDiagonal() * kDagger.transpose());
    }
}

DoubleIntegratorPyBindings::state_vector_t DoubleIntegratorPyBindings::getValueFunctionStateDerivative(double t,
                                                                                                       Eigen::Ref<const state_vector_t> x) {
  state_vector_t dVdx;
  mpcInterface_->getValueFunctionStateDerivative(t, x, dVdx);
  return dVdx;
}

DoubleIntegratorPyBindings::state_vector_t DoubleIntegratorPyBindings::computeFlowMap(double t, Eigen::Ref<const state_vector_t> x,
                                                                                      Eigen::Ref<const input_vector_t> u) {
  state_vector_t dxdt;
  dynamics_->computeFlowMap(t, x, u, dxdt);
  return dxdt;
}

void DoubleIntegratorPyBindings::setFlowMapDerivativeStateAndControl(double t, Eigen::Ref<const state_vector_t> x,
                                                                     Eigen::Ref<const input_vector_t> u) {
  dynamicsDerivatives_->setCurrentStateAndControl(t, x, u);
}

DoubleIntegratorPyBindings::state_matrix_t DoubleIntegratorPyBindings::computeFlowMapDerivativeState() {
  state_matrix_t A;
  dynamicsDerivatives_->getFlowMapDerivativeState(A);
  return A;
}

DoubleIntegratorPyBindings::state_input_matrix_t DoubleIntegratorPyBindings::computeFlowMapDerivativeInput() {
  state_input_matrix_t B;
  dynamicsDerivatives_->getFlowMapDerivativeInput(B);
  return B;
}

double DoubleIntegratorPyBindings::getRunningCost(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  double L;
  cost_->getIntermediateCost(L);
  return L;
}

DoubleIntegratorPyBindings::state_vector_t DoubleIntegratorPyBindings::getRunningCostDerivativeState(double t,
                                                                                                     Eigen::Ref<const state_vector_t> x,
                                                                                                     Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  state_vector_t dLdx;
  cost_->getIntermediateCostDerivativeState(dLdx);
  return dLdx;
}

DoubleIntegratorPyBindings::input_vector_t DoubleIntegratorPyBindings::getRunningCostDerivativeInput(double t,
                                                                                                     Eigen::Ref<const state_vector_t> x,
                                                                                                     Eigen::Ref<const input_vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  input_vector_t dLdu;
  cost_->getIntermediateCostDerivativeInput(dLdu);
  return dLdu;
}

}  // namespace double_integrator
}  // namespace ocs2
