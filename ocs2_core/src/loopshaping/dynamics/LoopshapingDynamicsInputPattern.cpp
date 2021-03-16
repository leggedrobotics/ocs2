//
// Created by ruben on 14.09.18.
//

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsInputPattern.h>

namespace ocs2 {

vector_t LoopshapingDynamicsInputPattern::filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  if (loopshapingDefinition_->isDiagonal()) {
    return s_filter.getAdiag().cwiseProduct(x_filter) + s_filter.getBdiag().cwiseProduct(u_filter);
  } else {
    vector_t filterStateDerivative;
    filterStateDerivative.noalias() = s_filter.getA() * x_filter;
    filterStateDerivative.noalias() += s_filter.getB() * u_filter;
    return filterStateDerivative;
  }
}

VectorFunctionLinearApproximation LoopshapingDynamicsInputPattern::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
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
  dynamics.dfdx.topRightCorner(x_system.rows(), FILTER_STATE_DIM).setZero();
  dynamics.dfdx.bottomLeftCorner(FILTER_STATE_DIM, x_system.rows()).setZero();
  dynamics.dfdx.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM) = s_filter.getA();

  dynamics.dfdu.resize(x.rows(), u.rows());
  dynamics.dfdu.topLeftCorner(x_system.rows(), u_system.rows()) = dynamics_system.dfdu;
  dynamics.dfdu.topRightCorner(x_system.rows(), FILTER_INPUT_DIM).setZero();
  dynamics.dfdu.bottomLeftCorner(FILTER_STATE_DIM, u_system.rows()).setZero();
  dynamics.dfdu.bottomRightCorner(FILTER_STATE_DIM, FILTER_INPUT_DIM) = s_filter.getB();

  return dynamics;
}

VectorFunctionLinearApproximation LoopshapingDynamicsInputPattern::jumpMapLinearApproximation(scalar_t t, const vector_t& x,
                                                                                              const vector_t& u) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const vector_t u_system = loopshapingDefinition_->getSystemInput(x, u);
  const auto jumpMap_system = systemDynamics_->jumpMapLinearApproximation(t, x_system, u_system);

  // Filter doesn't Jump
  const vector_t jumMap_filter = loopshapingDefinition_->getFilterState(x);

  VectorFunctionLinearApproximation jumpMap;
  jumpMap.f = loopshapingDefinition_->concatenateSystemAndFilterState(jumpMap_system.f, jumMap_filter);

  jumpMap.dfdx.setZero(x.rows(), x.rows());
  jumpMap.dfdx.topLeftCorner(x_system.rows(), x_system.rows()) = jumpMap_system.dfdx;

  jumpMap.dfdu.setZero(x.rows(), u.rows());
  jumpMap.dfdu.topLeftCorner(x_system.rows(), u_system.rows()) = jumpMap_system.dfdu;

  return jumpMap;
}

}  // namespace ocs2
