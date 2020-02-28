//
// Created by rgrandia on 26.02.20.
//

#include "ocs2_qp_solver/wrappers/SystemWrapper.h"

namespace ocs2_qp_solver {

Eigen::VectorXd SystemWrapper::getFlowMap(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  return p_->flowMap(t, x, u);
}

LinearDynamics SystemWrapper::getLinearApproximation(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  LinearDynamics linearDynamics;
  p_->setCurrentStateAndControl(t, x, u);
  linearDynamics.b = p_->flowMap(t, x, u);
  linearDynamics.A = p_->flowMapDerivativeState();
  linearDynamics.B = p_->flowMapDerivativeInput();
  return linearDynamics;
}

}