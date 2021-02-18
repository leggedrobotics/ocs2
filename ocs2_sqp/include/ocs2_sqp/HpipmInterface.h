//
// Created by rgrandia on 18.02.21.
//

#pragma once

#include <ocs2_core/Types.h>
#include <memory>
#include <vector>

namespace ocs2 {
class HpipmInterface {
 public:
  struct OcpSize {
    int N;                  // Number of stages
    std::vector<int> nu;    // Number of inputs
    std::vector<int> nx;    // Number of states
    std::vector<int> nbu;   // Number of input box inequality constraints
    std::vector<int> nbx;   // Number of state box inequality constraints
    std::vector<int> ng;    // Number of general inequality constraints
    std::vector<int> nsbu;  // Number of slack variables for input box inequalities
    std::vector<int> nsbx;  // Number of slack variables for state box inequalities
    std::vector<int> nsg;   // Number of slack variables for general inequalities
  };

  struct Settings {
    int iter_max = 30;
    scalar_t alpha_min = 1e-8;
    scalar_t mu0 = 1e4;
    scalar_t tol_stat = 1e-5;
    scalar_t tol_eq = 1e-5;
    scalar_t tol_ineq = 1e-5;
    scalar_t tol_comp = 1e-5;
    scalar_t reg_prim = 1e-12;
    int warm_start = 0;
    int pred_corr = 1;
    int ric_alg = 0;
    Settings() {}
  };

  HpipmInterface(OcpSize ocpSize, const Settings& settings = Settings());
  ~HpipmInterface();

  void solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
             std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<vector_t>& stateTrajectory,
             std::vector<vector_t>& inputTrajectory, bool verbose = false);

 private:
  class Impl;
  std::unique_ptr<Impl> pImpl_;
};

}  // namespace ocs2