#pragma once
#include <ocs2_core/Dimensions.h>
#include <iostream>

#include "Reference.h"

/*
 * 	Model describing the system dynamics used for integrating the reference input
 * 	to extend the reference past event times
 */
class Reference;

class ReferenceModel {
  using DIMENSIONS = ocs2::Dimensions<3, 1>;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;

 public:
  ReferenceModel(Reference* ref);

  void operator()(const state_vector_t& x, state_vector_t& dxdt, const double t);

 private:
  Reference* ref_;
};
