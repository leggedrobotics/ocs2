/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/StateInputCostCppAd.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically modeled particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsSystem final : public SystemDynamicsBase {
 public:
  CircularKinematicsSystem() = default;
  ~CircularKinematicsSystem() override = default;

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override { return u; }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) {
    VectorFunctionLinearApproximation dynamics;
    dynamics.f = computeFlowMap(t, x, u, preComp);
    dynamics.dfdx.setZero(2, 2);
    dynamics.dfdu.setIdentity(2, 2);
    return dynamics;
  }

  CircularKinematicsSystem* clone() const override { return new CircularKinematicsSystem(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically modeled particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsCost : public StateInputCostCppAd {
 public:
  CircularKinematicsCost(const std::string& libraryPath = "/tmp/ocs2") {
    initialize(2, 2, 0, "circular_kinematics_cost", libraryPath, true, false);
  }

  CircularKinematicsCost* clone() const override { return new CircularKinematicsCost(*this); }

  ad_scalar_t costFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                           const ad_vector_t& parameters) const override {
    return 0.5 * pow(state(0) * input(1) - state(1) * input(0) - 1.0, 2) + 0.005 * input.dot(input);
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically modeled particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsConstraints final : public StateInputConstraint {
 public:
  CircularKinematicsConstraints() : StateInputConstraint(ConstraintOrder::Linear) {}
  ~CircularKinematicsConstraints() override = default;

  CircularKinematicsConstraints* clone() const override { return new CircularKinematicsConstraints(*this); }

  size_t getNumConstraints(scalar_t time) const override { return 1; }

  vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const override {
    vector_t e(1);
    e << x.dot(u);
    return e;
  }

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation e;
    e.f = getValue(t, x, u, preComp);
    e.dfdx = u.transpose();
    e.dfdu = x.transpose();
    return e;
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline OptimalControlProblem createCircularKinematicsProblem(const std::string& libraryFolder) {
  // optimal control problem
  OptimalControlProblem problem;
  problem.dynamicsPtr.reset(new CircularKinematicsSystem());

  // cost function
  problem.costPtr->add("cost", std::make_unique<CircularKinematicsCost>(libraryFolder));

  // constraint
  problem.equalityConstraintPtr->add("constraint", std::make_unique<CircularKinematicsConstraints>());

  return problem;
}

}  // namespace ocs2
