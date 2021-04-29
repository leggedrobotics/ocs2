
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsEliminatePattern.h>

namespace ocs2 {

vector_t LoopshapingDynamicsEliminatePattern::filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  if (loopshapingDefinition_->isDiagonal()) {
    return s_filter.getAdiag() * x_filter + s_filter.getBdiag() * u_filter;
  } else {
    vector_t dynamics_filter;
    dynamics_filter.noalias() = s_filter.getA() * x_filter;
    dynamics_filter.noalias() += s_filter.getB() * u_filter;
    return dynamics_filter;
  }
}

VectorFunctionLinearApproximation LoopshapingDynamicsEliminatePattern::linearApproximation(scalar_t t, const vector_t& x,
                                                                                           const vector_t& u) {
  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const vector_t u_system = loopshapingDefinition_->getSystemInput(x, u);
  const vector_t x_filter = loopshapingDefinition_->getFilterState(x);
  const vector_t u_filter = loopshapingDefinition_->getFilteredInput(x, u);
  const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  const auto dynamics_system = systemDynamics_->linearApproximation(t, x_system, u_system);

  VectorFunctionLinearApproximation dynamics;
  dynamics.f = loopshapingDefinition_->concatenateSystemAndFilterState(dynamics_system.f, filterFlowmap(x_filter, u_filter, u_system));

  dynamics.dfdx.resize(x.rows(), x.rows());
  dynamics.dfdx.topLeftCorner(x_system.rows(), x_system.rows()) = dynamics_system.dfdx;
  if (isDiagonal) {
    dynamics.dfdx.topRightCorner(x_system.rows(), FILTER_STATE_DIM).noalias() = dynamics_system.dfdu * s_filter.getCdiag();
  } else {
    dynamics.dfdx.topRightCorner(x_system.rows(), FILTER_STATE_DIM).noalias() = dynamics_system.dfdu * s_filter.getC();
  }
  dynamics.dfdx.bottomLeftCorner(FILTER_STATE_DIM, x_system.rows()).setZero();
  dynamics.dfdx.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM) = s_filter.getA();

  dynamics.dfdu.resize(x.rows(), u.rows());
  if (isDiagonal) {
    dynamics.dfdu.topRows(x_system.rows()).noalias() = dynamics_system.dfdu * s_filter.getDdiag();
  } else {
    dynamics.dfdu.topRows(x_system.rows()).noalias() = dynamics_system.dfdu * s_filter.getD();
  }
  dynamics.dfdu.bottomRows(FILTER_STATE_DIM) = s_filter.getB();

  return dynamics;
}

}  // namespace ocs2
