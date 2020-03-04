//
// Created by rgrandia on 26.02.20.
//

#include "ocs2_qp_solver/wrappers/CostWrapper.h"

namespace ocs2 {
namespace qp_solver {

double CostWrapper::getCost(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  p_->setCurrentStateAndControl(t, x, u);
  return p_->getCost();
}

ScalarFunctionQuadraticApproximation CostWrapper::getQuadraticApproximation(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  ScalarFunctionQuadraticApproximation quadraticCost;
  p_->setCurrentStateAndControl(t, x, u);
  quadraticCost.dfdxx = p_->getCostSecondDerivativeState();
  quadraticCost.dfdux = p_->getCostDerivativeInputState();
  quadraticCost.dfduu = p_->getCostSecondDerivativeInput();
  quadraticCost.dfdx = p_->getCostDerivativeState();
  quadraticCost.dfdu = p_->getCostDerivativeInput();
  quadraticCost.f = p_->getCost();
  return quadraticCost;
}

double CostWrapper::getTerminalCost(double t, const Eigen::VectorXd& x) {
  p_->setCurrentStateAndControl(t, x);
  return p_->getTerminalCost();
}

ScalarFunctionQuadraticApproximation CostWrapper::getTerminalQuadraticApproximation(double t, const Eigen::VectorXd& x) {
  ScalarFunctionQuadraticApproximation quadraticCost;
  p_->setCurrentStateAndControl(t, x);
  quadraticCost.dfdxx = p_->getTerminalCostSecondDerivativeState();
  quadraticCost.dfdx = p_->getTerminalCostDerivativeState();
  quadraticCost.f = p_->getTerminalCost();
  return quadraticCost;
}

}  // namespace qp_solver
}  // namespace ocs2
