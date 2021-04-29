
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsOutputPattern.h>

namespace ocs2 {

vector_t LoopshapingDynamicsOutputPattern::filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  if (loopshapingDefinition_->isDiagonal()) {
    return r_filter.getAdiag() * x_filter + r_filter.getBdiag() * u_system;
  } else {
    return r_filter.getA() * x_filter + r_filter.getB() * u_system;
  }
}

VectorFunctionLinearApproximation LoopshapingDynamicsOutputPattern::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const vector_t u_system = loopshapingDefinition_->getSystemInput(x, u);
  const vector_t x_filter = loopshapingDefinition_->getFilterState(x);
  const vector_t u_filter = loopshapingDefinition_->getFilteredInput(x, u);
  const size_t FILTER_INPUT_DIM = r_filter.getNumInputs();
  const size_t FILTER_STATE_DIM = r_filter.getNumStates();
  const auto dynamics_system = systemDynamics_->linearApproximation(t, x_system, u_system);

  VectorFunctionLinearApproximation dynamics;
  dynamics.f = loopshapingDefinition_->concatenateSystemAndFilterState(dynamics_system.f, filterFlowmap(x_filter, u_filter, u_system));

  dynamics.dfdx.resize(x.rows(), x.rows());
  dynamics.dfdx.topLeftCorner(x_system.rows(), x_system.rows()) = dynamics_system.dfdx;
  dynamics.dfdx.topRightCorner(x_system.rows(), FILTER_STATE_DIM).setZero();
  dynamics.dfdx.bottomLeftCorner(FILTER_STATE_DIM, x_system.rows()).setZero();
  dynamics.dfdx.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM) = r_filter.getA();

  dynamics.dfdu.resize(x.rows(), u.rows());
  dynamics.dfdu.topRows(x_system.rows()) = dynamics_system.dfdu;
  dynamics.dfdu.bottomRows(FILTER_STATE_DIM) = r_filter.getB();

  return dynamics;
}

}  // namespace ocs2
