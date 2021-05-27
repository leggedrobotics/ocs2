//
// Created by ruben on 14.09.18.
//

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsInputPattern.h>

namespace ocs2 {

vector_t LoopshapingDynamicsInputPattern::filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  if (loopshapingDefinition_->isDiagonal()) {
    return s_filter.getAdiag() * x_filter + s_filter.getBdiag() * u_filter;
  } else {
    vector_t filterStateDerivative;
    filterStateDerivative.noalias() = s_filter.getA() * x_filter;
    filterStateDerivative.noalias() += s_filter.getB() * u_filter;
    return filterStateDerivative;
  }
}

VectorFunctionLinearApproximation LoopshapingDynamicsInputPattern::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                       const PreComputation& preComp) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& u_filter = preCompLS.getFilteredInput();
  const auto dynamics_system = systemDynamics_->linearApproximation(t, x_system, u_system, preCompLS.getSystemPreComputation());

  VectorFunctionLinearApproximation dynamics;
  dynamics.f = loopshapingDefinition_->concatenateSystemAndFilterState(dynamics_system.f, filterFlowmap(x_filter, u_filter, u_system));

  dynamics.dfdx.resize(x.rows(), x.rows());
  dynamics.dfdx.topLeftCorner(x_system.rows(), x_system.rows()) = dynamics_system.dfdx;
  dynamics.dfdx.topRightCorner(x_system.rows(), x_filter.rows()).setZero();
  dynamics.dfdx.bottomLeftCorner(x_filter.rows(), x_system.rows()).setZero();
  dynamics.dfdx.bottomRightCorner(x_filter.rows(), x_filter.rows()) = s_filter.getA();

  dynamics.dfdu.resize(x.rows(), u.rows());
  dynamics.dfdu.topLeftCorner(x_system.rows(), u_system.rows()) = dynamics_system.dfdu;
  dynamics.dfdu.topRightCorner(x_system.rows(), u_filter.rows()).setZero();
  dynamics.dfdu.bottomLeftCorner(x_filter.rows(), u_system.rows()).setZero();
  dynamics.dfdu.bottomRightCorner(x_filter.rows(), u_filter.rows()) = s_filter.getB();

  return dynamics;
}

}  // namespace ocs2
