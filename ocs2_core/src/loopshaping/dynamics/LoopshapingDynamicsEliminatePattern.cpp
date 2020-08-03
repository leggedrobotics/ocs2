
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsEliminatePattern.h>

namespace ocs2 {

vector_t LoopshapingDynamicsEliminatePattern::filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  vector_t dynamics_filter;
  dynamics_filter.noalias() = s_filter.getA() * x_filter;
  dynamics_filter.noalias() += s_filter.getB() * u_filter;

  return dynamics_filter;
}

VectorFunctionLinearApproximation LoopshapingDynamicsEliminatePattern::linearApproximation(scalar_t t, const vector_t& x,
                                                                                           const vector_t& u) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const vector_t u_system = loopshapingDefinition_->getSystemInput(x, u);
  const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  const auto dynamics_system = systemDynamics_->linearApproximation(t, x_system, u_system);

  VectorFunctionLinearApproximation dynamics;
  dynamics.f = this->computeFlowMap(t, x, u);

  dynamics.dfdx.resize(x.rows(), x.rows());
  dynamics.dfdx.topLeftCorner(x_system.rows(), x_system.rows()) = dynamics_system.dfdx;
  dynamics.dfdx.topRightCorner(x_system.rows(), FILTER_STATE_DIM).noalias() = dynamics_system.dfdu * s_filter.getC();
  dynamics.dfdx.bottomLeftCorner(FILTER_STATE_DIM, x_system.rows()).setZero();
  dynamics.dfdx.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM) = s_filter.getA();

  dynamics.dfdu.resize(x.rows(), u.rows());
  dynamics.dfdu.topRows(x_system.rows()).noalias() = dynamics_system.dfdu * s_filter.getD();
  dynamics.dfdu.bottomRows(FILTER_STATE_DIM) = s_filter.getB();

  return dynamics;
}

VectorFunctionLinearApproximation LoopshapingDynamicsEliminatePattern::jumpMapLinearApproximation(scalar_t t, const vector_t& x,
                                                                                                  const vector_t& u) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const vector_t u_system = loopshapingDefinition_->getSystemInput(x, u);
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
  const auto jumpMap_system = systemDynamics_->jumpMapLinearApproximation(t, x_system, u_system);

  VectorFunctionLinearApproximation jumpMap;
  jumpMap.f = this->computeJumpMap(t, x);

  jumpMap.dfdx.resize(x.rows(), x.rows());
  jumpMap.dfdx.topLeftCorner(x_system.rows(), x_system.rows()) = jumpMap_system.dfdx;
  jumpMap.dfdx.topRightCorner(x_system.rows(), FILTER_STATE_DIM).noalias() = jumpMap_system.dfdu * s_filter.getC();
  jumpMap.dfdx.bottomLeftCorner(FILTER_STATE_DIM, x_system.rows()).setZero();
  jumpMap.dfdx.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM).setIdentity();

  jumpMap.dfdu.resize(x.rows(), u.rows());
  jumpMap.dfdu.topRows(x_system.rows()).noalias() = jumpMap_system.dfdu * s_filter.getD();
  jumpMap.dfdu.bottomRows(FILTER_STATE_DIM).setZero();

  return jumpMap;
}

}  // namespace ocs2
