/******************************************************************************
Copyright (c) 2019, Oliver Harley. All rights reserved.

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

#ifndef TEST_DERIVATIVECHECKER_H
#define TEST_DERIVATIVECHECKER_H

#include <ocs2_core/dynamics/SystemDynamicsLinearizer.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>

namespace ocs2{

class EXP0_System : public ControlledSystemBase<2,1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    typedef ControlledSystemBase<2,1> Base;
    using scalar_t = typename Base::scalar_t;

    EXP0_System() = default;
    ~EXP0_System() = default;

    void computeFlowMap( const scalar_t& t,
            const Eigen::Vector2d& x,
            const Eigen::Matrix<scalar_t,1,1>& u,
            Eigen::Vector2d& dxdt)
        {

        Eigen::Matrix2d A;
        A << 0.6, 1.2, -0.8, 3.4;
        Eigen::Vector2d B;
        B << 1, 1;

        dxdt = A*x + B*u;
    }

    EXP0_System* clone() const final {
        return new EXP0_System(*this);
    }

};

class EXP0_SystemDerivative : public DerivativesBase<2,1>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef DerivativesBase<2,1> Base;
  using scalar_t = typename Base::scalar_t;

  EXP0_SystemDerivative() = default;

  ~EXP0_SystemDerivative() = default;

  void getFlowMapDerivativeState(state_matrix_t& A) final { A << 0.6, 1.2, -0.8, 3.4; }
  void getFlowMapDerivativeInput(state_input_matrix_t& B) final { B << 1, 1; }

  EXP0_SystemDerivative* clone() const final {
    return new EXP0_SystemDerivative(*this);
  }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) final {
    Base::setCurrentStateAndControl(t, x, u);
  }
}; //EXP0_SystemDerivative

/**
 * Has incorrect linearised dynamics (A=0, B=0) but the dynamics are non-zero.
 */
class EXP1_SystemDerivative : public DerivativesBase<2,1>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef DerivativesBase<2,1> Base;
  using scalar_t = typename Base::scalar_t;


  EXP1_SystemDerivative() = default;
  ~EXP1_SystemDerivative() = default;

  void getFlowMapDerivativeState(state_matrix_t& A) final { A << 0, 0, 0, 0; }
  void getFlowMapDerivativeInput(state_input_matrix_t& B) final { B << 0, 0; }

  EXP1_SystemDerivative* clone() const final {
    return new EXP1_SystemDerivative(*this);
  }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) final {
    Base::setCurrentStateAndControl(t, x, u);
  }
}; //EXP1_SystemDerivative

/**
 * Pendulum system, \fn$ \theta = 0 \fn$ is upright
 */
class EXP2_System : public ControlledSystemBase<2,1>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef ControlledSystemBase<2,1> Base;
  using scalar_t = typename Base::scalar_t;

  EXP2_System() = default;
  ~EXP2_System() = default;

  void computeFlowMap( const scalar_t& t,
                       const Eigen::Vector2d& x,
                       const Eigen::Matrix<scalar_t,1,1>& u,
                       Eigen::Vector2d& dxdt)
  final {

    Eigen::Vector2d Ax;
    Ax << x(1), sin(x(0));
    Eigen::Vector2d B;
    B << 0, 0.1; // just random values

    dxdt = Ax + B*u;
  }

  EXP2_System* clone() const final {
    return new EXP2_System(*this);
  }

};

/**
 * Pendulum system, linearised at \fn$ \theta = 0  \fn$, up and unstable
 */
class EXP2_SystemDerivative : public DerivativesBase<2,1>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef DerivativesBase<2,1> Base;
  using scalar_t = typename Base::scalar_t;

  EXP2_SystemDerivative() = default;

  ~EXP2_SystemDerivative() = default;

  //! Linearised at \fn$ \theta=0 \fn$
  void getFlowMapDerivativeState(state_matrix_t& A) final { A << 0, 1, 1, 0; }
  void getFlowMapDerivativeInput(state_input_matrix_t& B) final { B << 0, 0.1; }

  EXP2_SystemDerivative* clone() const final {
    return new EXP2_SystemDerivative(*this);
  }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) final {
    Base::setCurrentStateAndControl(t, x, u);
  }
}; //EXP2_SystemDerivative

/**
 * Pendulum system, linearised at \fn$ \theta = \pi  \fn$ - down / stable
 */
class EXP3_SystemDerivative : public DerivativesBase<2,1>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef DerivativesBase<2,1> Base;
  using scalar_t = typename Base::scalar_t;

  EXP3_SystemDerivative() = default;

  ~EXP3_SystemDerivative() = default;

  //! Linearised at \fn$ \theta=\pi \fn$
  void getFlowMapDerivativeState(state_matrix_t& A) final { A << 0, 1, -1, 0; }
  void getFlowMapDerivativeInput(state_input_matrix_t& B) final { B << 0, 0.1; }

  EXP3_SystemDerivative* clone() const final {
    return new EXP3_SystemDerivative(*this);
  }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) final {
    Base::setCurrentStateAndControl(t, x, u);
  }
}; //EXP3_SystemDerivative

} // namespace ocs2

#endif /* TEST_DERIVATIVECHECKER_H */

