#include "ReferenceModel.h"

ReferenceModel::ReferenceModel(Reference* ref) : ref_(ref) {}

void ReferenceModel::operator()(const state_type& x, state_type& dxdt, const double t) {
  Eigen::Matrix<double, 1, 1> uref;
  ref_->getInput(t, uref);

  Eigen::Matrix<double, 3, 3> A;
  A << 0, 1, 0, 0, 0, 0, 0, 0, 0;
  Eigen::Matrix<double, 3, 1> B;
  B << 0, 1, 0;

  dxdt = A * x + B * uref;
}
