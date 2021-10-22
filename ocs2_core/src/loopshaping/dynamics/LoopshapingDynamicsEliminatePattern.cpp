
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsEliminatePattern.h>

namespace ocs2 {

vector_t LoopshapingDynamicsEliminatePattern::filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  if (loopshapingDefinition_->isDiagonal()) {
    return s_filter.getAdiag().diagonal().cwiseProduct(x_filter) + s_filter.getBdiag().diagonal().cwiseProduct(u_filter);
  } else {
    vector_t dynamics_filter;
    dynamics_filter.noalias() = s_filter.getA() * x_filter;
    dynamics_filter.noalias() += s_filter.getB() * u_filter;
    return dynamics_filter;
  }
}

VectorFunctionLinearApproximation LoopshapingDynamicsEliminatePattern::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                           const PreComputation& preComp) {
  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
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
  if (isDiagonal) {
    dynamics.dfdx.topRightCorner(x_system.rows(), x_filter.rows()).noalias() = dynamics_system.dfdu * s_filter.getCdiag();
  } else {
    dynamics.dfdx.topRightCorner(x_system.rows(), x_filter.rows()).noalias() = dynamics_system.dfdu * s_filter.getC();
  }
  dynamics.dfdx.bottomLeftCorner(x_filter.rows(), x_system.rows()).setZero();
  dynamics.dfdx.bottomRightCorner(x_filter.rows(), x_filter.rows()) = s_filter.getA();

  dynamics.dfdu.resize(x.rows(), u.rows());
  if (isDiagonal) {
    dynamics.dfdu.topRows(x_system.rows()).noalias() = dynamics_system.dfdu * s_filter.getDdiag();
  } else {
    dynamics.dfdu.topRows(x_system.rows()).noalias() = dynamics_system.dfdu * s_filter.getD();
  }
  dynamics.dfdu.bottomRows(x_filter.rows()) = s_filter.getB();

  return dynamics;
}

}  // namespace ocs2
