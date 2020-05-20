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

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBaseAD.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/initialization/OperatingPoints.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically model particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsSystem final : public SystemDynamicsBase {
 public:
  CircularKinematicsSystem() : SystemDynamicsBase(2, 2){};
  ~CircularKinematicsSystem() override = default;

  void computeFlowMap(const scalar_t& t, const vector_t& x, const vector_t& u, vector_t& dxdt) override { dxdt = u; }

  void getFlowMapDerivativeState(matrix_t& A) override { A.setZero(2, 2); }

  void getFlowMapDerivativeInput(matrix_t& B) override { B.setIdentity(2, 2); }

  CircularKinematicsSystem* clone() const override { return new CircularKinematicsSystem(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically model particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsCost final : public CostFunctionBaseAD {
 public:
  CircularKinematicsCost() : CostFunctionBaseAD(2, 2) {}
  ~CircularKinematicsCost() override = default;

  CircularKinematicsCost* clone() const override { return new CircularKinematicsCost(*this); }

 protected:
  void intermediateCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input, const ad_vector_t& parameters,
                                ad_scalar_t& costValue) const override {
    costValue = 0.5 * pow(state(0) * input(1) - state(1) * input(0) - 1.0, 2) + 0.005 * input.dot(input);
  }

  void terminalCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters,
                            ad_scalar_t& costValue) const override {
    costValue = 0.0;
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically model particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsConstraints final : public ConstraintBase {
 public:
  CircularKinematicsConstraints() : ConstraintBase(2, 2) {}
  ~CircularKinematicsConstraints() override = default;

  CircularKinematicsConstraints* clone() const override { return new CircularKinematicsConstraints(*this); }

  size_t numStateInputConstraint(const scalar_t& time) override { return 1; }

  void getConstraint1(vector_t& e) override {
    e.resize(2);
    e(0) = x_.dot(u_);
  }

  void getConstraint1DerivativesState(matrix_t& C) override {
    C.resize(2, 2);
    C.topRows(1) = u_.transpose();
  }

  void getConstraint1DerivativesControl(matrix_t& D) override {
    D.resize(2, 2);
    D.topRows(1) = x_.transpose();
  }
};

}  // namespace ocs2
