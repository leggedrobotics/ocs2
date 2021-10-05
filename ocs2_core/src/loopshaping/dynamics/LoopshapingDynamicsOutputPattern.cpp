
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsOutputPattern.h>

namespace ocs2 {

vector_t LoopshapingDynamicsOutputPattern::filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  if (loopshapingDefinition_->isDiagonal()) {
    return r_filter.getAdiag().diagonal().cwiseProduct(x_filter) + r_filter.getBdiag().diagonal().cwiseProduct(u_system);
  } else {
    vector_t dynamics_filter = r_filter.getA() * x_filter;
    dynamics_filter.noalias() += r_filter.getB() * u_system;
    return dynamics_filter;
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

  const auto stateDim = x.rows();
  const auto inputDim = u.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x_filter.rows();

  VectorFunctionLinearApproximation dynamics;
  dynamics.f = loopshapingDefinition_->concatenateSystemAndFilterState(dynamics_system.f, filterFlowmap(x_filter, u_filter, u_system));

  dynamics.dfdx.resize(stateDim, stateDim);
  dynamics.dfdx.topLeftCorner(sysStateDim, sysStateDim) = dynamics_system.dfdx;
  dynamics.dfdx.bottomLeftCorner(filtStateDim, sysStateDim).setZero();
  dynamics.dfdx.topRightCorner(sysStateDim, filtStateDim).setZero();
  dynamics.dfdx.bottomRightCorner(filtStateDim, filtStateDim) = r_filter.getA();

  dynamics.dfdu.resize(stateDim, inputDim);
  dynamics.dfdu.topRows(sysStateDim) = dynamics_system.dfdu;
  dynamics.dfdu.bottomRows(filtStateDim) = r_filter.getB();

  return dynamics;
}

}  // namespace ocs2
