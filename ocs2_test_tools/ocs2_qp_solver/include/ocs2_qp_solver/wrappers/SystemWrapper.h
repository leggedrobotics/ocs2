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

//
// Created by rgrandia on 25.02.20.
//

#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <ocs2_qp_solver/QpSolverTypes.h>

namespace ocs2 {
namespace qp_solver {

/**
 * Wrapper class that wraps a SystemDynamicsBase of any size and provides a dynamic size interface.
 * The wrapper clones the system dynamics upon construction, and owns the clone.
 * This class is not thread safe, because the underlying system is not thread safe.
 */
class SystemWrapper {
 public:
  /** Templated constructor to accept system dynamics of any size */
  SystemWrapper(const ocs2::SystemDynamicsBase& systemDynamics)  // NOLINT(google-explicit-constructor)
      : p_(systemDynamics.clone()) {}

  /** Copy constructor clones the underlying system dynamics */
  SystemWrapper(const SystemWrapper& other) : p_(other.p_->clone()) {}

  /** Copy assignment clones the underlying system dynamics */
  SystemWrapper& operator=(const SystemWrapper& other) {
    *this = SystemWrapper(other);
    return *this;
  }

  /** Move constructor moves the system */
  SystemWrapper(SystemWrapper&&) noexcept = default;

  /** Move assignment moves the system */
  SystemWrapper& operator=(SystemWrapper&&) noexcept = default;

  /** Evaluates the flowmap */
  vector_t getFlowMap(scalar_t t, const vector_t& x, const vector_t& u);

  /** Gets linear approximation */
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u);

 private:
  /** System dynamics function */
  std::unique_ptr<ocs2::SystemDynamicsBase> p_;
};

}  // namespace qp_solver
}  // namespace ocs2
