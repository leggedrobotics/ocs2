
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsOutputPattern.h>

namespace ocs2 {

vector_t LoopshapingDynamicsOutputPattern::filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  if (loopshapingDefinition_->isDiagonal()) {
    return r_filter.getAdiag().diagonal().cwiseProduct(x_filter) + r_filter.getBdiag().diagonal().cwiseProduct(u_system);
  } else {
    return r_filter.getA() * x_filter + r_filter.getB() * u_system;
  }
}

VectorFunctionLinearApproximation LoopshapingDynamicsOutputPattern::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                        const PreComputation& preComp) {
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& u_filter = preCompLS.getFilteredInput();
  const auto dynamics_system = systemDynamics_->linearApproximation(t, x_system, u_system);

  VectorFunctionLinearApproximation dynamics;
  dynamics.f = loopshapingDefinition_->concatenateSystemAndFilterState(dynamics_system.f, filterFlowmap(x_filter, u_filter, u_system));

  dynamics.dfdx.resize(x.rows(), x.rows());
  dynamics.dfdx.topLeftCorner(x_system.rows(), x_system.rows()) = dynamics_system.dfdx;
  dynamics.dfdx.topRightCorner(x_system.rows(), x_filter.rows()).setZero();
  dynamics.dfdx.bottomLeftCorner(x_filter.rows(), x_system.rows()).setZero();
  dynamics.dfdx.bottomRightCorner(x_filter.rows(), x_filter.rows()) = r_filter.getA();

  dynamics.dfdu.resize(x.rows(), u.rows());
  dynamics.dfdu.topRows(x_system.rows()) = dynamics_system.dfdu;
  dynamics.dfdu.bottomRows(x_filter.rows()) = r_filter.getB();

  return dynamics;
}

}  // namespace ocs2
