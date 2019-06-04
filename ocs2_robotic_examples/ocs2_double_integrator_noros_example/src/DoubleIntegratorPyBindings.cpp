

#include <ocs2_double_integrator_noros_example/DoubleIntegratorPyBindings.hpp>

namespace ocs2 {
namespace double_integrator {

DoubleIntegratorPyBindings::DoubleIntegratorPyBindings(const std::string& taskFileFolder) {
  doubleIntegratorInterface_.reset(new ocs2::double_integrator::DoubleIntegratorInterface(taskFileFolder));
  mpcInterface_.reset(new mpc_t(*doubleIntegratorInterface_->getMPCPtr(), nullLogicRules_, true));

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

void DoubleIntegratorPyBindings::getMpcSolution(scalar_array_t& t, state_vector_array_t& x, input_vector_array_t& u) {
  // TODO(jcarius): should we interpolate here to expose constant time steps?!
  mpcInterface_->getMpcSolution(t, x, u);
  // TODO(jcarius) also get cost-to-go
}

}  // namespace double_integrator
}  // namespace ocs2
