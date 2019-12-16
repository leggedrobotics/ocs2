#include "ocs2_ddp/test/bouncingmass/ReferenceModel.h"

ReferenceModel::ReferenceModel(Reference* ref) : ref_(ref) {}

void ReferenceModel::operator()(const state_vector_t& x, state_vector_t& dxdt, const double t) {
  input_vector_t uref;
  ref_->getInput(t, uref);

  state_matrix_t A;
  A << 0, 1, 0, 0, 0, 0, 0, 0, 0;
  state_input_matrix_t B;
  B << 0, 1, 0;

  dxdt = A * x + B * uref;
}
