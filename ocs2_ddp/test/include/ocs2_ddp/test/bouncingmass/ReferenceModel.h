#pragma once
#include <iostream>
#include "Reference.h"

/*
 * 	Model describing the system dynamics used for integrating the reference input
 * 	to extend the reference past event times
 */
class Reference;

class ReferenceModel {
  typedef Eigen::Vector3d state_type;

 public:
  ReferenceModel(Reference* ref);

  void operator()(const state_type& x, state_type& dxdt, const double t);

 private:
  Reference* ref_;
};
