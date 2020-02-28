//
// Created by rgrandia on 26.02.20.
//

#include "ocs2_qp_solver/wrappers/CostWrapper.h"

namespace ocs2_qp_solver {

double CostWrapper::getCost(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  p_->setCurrentStateAndControl(t, x, u);
  return p_->getCost();
}

QuadraticCost CostWrapper::getQuadraticApproximation(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  QuadraticCost quadraticCost;
  p_->setCurrentStateAndControl(t, x, u);
  quadraticCost.c = p_->getCost();
  quadraticCost.q = p_->getCostDerivativeState();
  quadraticCost.r = p_->getCostDerivativeInput();
  quadraticCost.Q = p_->getCostSecondDerivativeState();
  quadraticCost.R = p_->getCostSecondDerivativeInput();
  quadraticCost.P = p_->getCostDerivativeInputState();
  return quadraticCost;
}

double CostWrapper::getTerminalCost(double t, const Eigen::VectorXd& x) {
  p_->setCurrentStateAndControl(t, x);
  return p_->getTerminalCost();
}

QuadraticCost CostWrapper::getTerminalQuadraticApproximation(double t, const Eigen::VectorXd& x) {
  QuadraticCost quadraticCost;
  p_->setCurrentStateAndControl(t, x);
  quadraticCost.c = p_->getTerminalCost();
  quadraticCost.q = p_->getTerminalCostDerivativeState();
  quadraticCost.Q = p_->getTerminalCostSecondDerivativeState();
  return quadraticCost;
}

}  // namespace ocs2_qp_solver
