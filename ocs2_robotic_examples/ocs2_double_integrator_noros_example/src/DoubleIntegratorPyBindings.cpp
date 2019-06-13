

#include <ocs2_double_integrator_noros_example/DoubleIntegratorPyBindings.hpp>

namespace ocs2 {
namespace double_integrator {

DoubleIntegratorPyBindings::DoubleIntegratorPyBindings(const std::string& taskFileFolder) {
  doubleIntegratorInterface_.reset(new ocs2::double_integrator::DoubleIntegratorInterface(taskFileFolder));
  mpcInterface_.reset(new mpc_t(*doubleIntegratorInterface_->getMPCPtr(), nullLogicRules_, true));

  dynamics_.reset(doubleIntegratorInterface_->getDynamicsPtr()->clone());
  dynamicsDerivatives_.reset(doubleIntegratorInterface_->getDynamicsDerivativesPtr()->clone());

  cost_.reset(doubleIntegratorInterface_->getCostPtr()->clone());
  //TODO(jcarius) should we set the costDesiredTrajectories here?

  // initialize reference
  constexpr double time = 0.0;
  mpc_t::cost_desired_trajectories_t costDesiredTrajectories;
  costDesiredTrajectories.desiredTimeTrajectory().push_back(time);
  auto goalState = doubleIntegratorInterface_->getXFinal();
  costDesiredTrajectories.desiredStateTrajectory().push_back(goalState);
  auto desiredInput = mpc_t::input_vector_t::Zero();
  costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
  mpcInterface_->setTargetTrajectories(costDesiredTrajectories);
}

void DoubleIntegratorPyBindings::setObservation(double t, Eigen::Ref<const state_vector_t> x) {
  mpc_t::system_observation_t observation;
  observation.time() = t;
  observation.state() = x;
  mpcInterface_->setCurrentObservation(observation);
}

void DoubleIntegratorPyBindings::advanceMpc() { mpcInterface_->advanceMpc(); }

void DoubleIntegratorPyBindings::getMpcSolution(scalar_array_t& t, state_vector_array_t& x, input_vector_array_t& u,
                                                state_vector_array_t& Vx) {
  // TODO(jcarius): should we interpolate here to expose constant time steps?!
  mpcInterface_->getMpcSolution(t, x, u);
  Vx.clear();
  Vx.resize(t.size());
  for (size_t i = 0; i < t.size(); i++) {
    mpcInterface_->getValueFunctionStateDerivative(t[i], Vx[i]);
  }
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

double DoubleIntegratorPyBindings::getRunningCost(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u){
  cost_->setCurrentStateAndControl(t, x, u);
  double L;
  cost_->getIntermediateCost(L);
  return L;
}

DoubleIntegratorPyBindings::state_vector_t DoubleIntegratorPyBindings::getRunningCostDerivativeState(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u){
  cost_->setCurrentStateAndControl(t, x, u);
  state_vector_t dLdx;
  cost_->getIntermediateCostDerivativeState(dLdx);
  return dLdx;
}

DoubleIntegratorPyBindings::input_vector_t DoubleIntegratorPyBindings::getRunningCostDerivativeInput(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u){
  cost_->setCurrentStateAndControl(t, x, u);
  input_vector_t dLdu;
  cost_->getIntermediateCostDerivativeInput(dLdu);
  return dLdu;
}

}  // namespace double_integrator
}  // namespace ocs2
