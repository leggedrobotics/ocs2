//
// Created by rgrandia on 26.02.20.
//

#include "ocs2_qp_solver/wrappers/SystemWrapper.h"

namespace ocs2 {
namespace qp_solver {

Eigen::VectorXd SystemWrapper::getFlowMap(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  return p_->flowMap(t, x, u);
}

VectorFunctionLinearApproximation SystemWrapper::getLinearApproximation(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  VectorFunctionLinearApproximation linearDynamics;
  p_->setCurrentStateAndControl(t, x, u);
  linearDynamics.dfdx = p_->flowMapDerivativeState();
  linearDynamics.dfdu = p_->flowMapDerivativeInput();
  linearDynamics.f = p_->flowMap(t, x, u);
  return linearDynamics;
}

}  // namespace qp_solver
}  // namespace ocs2
