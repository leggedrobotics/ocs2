/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseCentroidalMomentumMatrixInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Ab) {
  const SCALAR_T mass = Ab(0, 0);
  Eigen::Matrix<SCALAR_T, 3, 3> Ab_22_inv = Ab.template block<3, 3>(3, 3).inverse();
  Eigen::Matrix<SCALAR_T, 6, 6> Ab_inv = Eigen::Matrix<SCALAR_T, 6, 6>::Zero();
  Ab_inv << 1.0 / mass * Eigen::Matrix<SCALAR_T, 3, 3>::Identity(), -1.0 / mass * Ab.template block<3, 3>(0, 3) * Ab_22_inv,
      Eigen::Matrix<SCALAR_T, 3, 3>::Zero(), Ab_22_inv;
  return Ab_inv;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getMappingZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> dTdz, dTdy, dTdx;
  dTdz << SCALAR_T(0),     -cos(z),        -cos(y)*sin(z),
          SCALAR_T(0),      -sin(z),        cos(y)*cos(z),
          SCALAR_T(0),     SCALAR_T(0),      SCALAR_T(0);

  dTdy << SCALAR_T(0),      SCALAR_T(0),        -sin(y)*cos(z),
          SCALAR_T(0),      SCALAR_T(0),        -sin(y)*sin(z),
          SCALAR_T(0),      SCALAR_T(0),          -cos(y);

  dTdx.setZero();
  // clang-format on

  return {dTdz, dTdy, dTdx};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getRotationMatrixZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T dc1 = -s1;
  const SCALAR_T dc2 = -s2;
  const SCALAR_T dc3 = -s3;
  const SCALAR_T ds1 = c1;
  const SCALAR_T ds2 = c2;
  const SCALAR_T ds3 = c3;

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> dRdz, dRdy, dRdx;
  dRdz << dc1 * c2,      dc1 * s2 * s3 - ds1 * c3,       dc1 * s2 * c3 + ds1 * s3,
          ds1 * c2,      ds1 * s2 * s3 + dc1 * c3,       ds1 * s2 * c3 - dc1 * s3,
          SCALAR_T(0),       SCALAR_T(0),                      SCALAR_T(0);

  dRdy << c1 * dc2,      c1 * ds2 * s3,       c1 * ds2 * c3,
          s1 * dc2,      s1 * ds2 * s3,       s1 * ds2 * c3,
           -ds2,            dc2 * s3,            dc2 * c3;

  dRdx << SCALAR_T(0),      c1 * s2 * ds3 - s1 * dc3,       c1 * s2 * dc3 + s1 * ds3,
          SCALAR_T(0),      s1 * s2 * ds3 + c1 * dc3,       s1 * s2 * dc3 - c1 * ds3,
          SCALAR_T(0),                c2 * ds3,                      c2 * dc3;
  // clang-format on

  return {dRdz, dRdy, dRdx};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 3> getCentroidalMomentumZyxGradient(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                               const CentroidalModelInfoTpl<SCALAR_T>& info,
                                                               const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
                                                               const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v) {
  using matrix_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, Eigen::Dynamic>;
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3_t = Eigen::Matrix<SCALAR_T, 3, 3>;

  const auto& data = interface.getData();
  const auto m = info.robotMass;
  const vector3_t eulerAngles = q.template segment<3>(3);
  const vector3_t eulerAnglesDerivatives = v.template segment<3>(3);
  const auto T = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAngles);
  const auto R = getRotationMatrixFromZyxEulerAngles(eulerAngles);
  matrix3_t Ibaseframe, Iworldframe;
  vector3_t rbaseframe, rworldframe;

  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      Iworldframe = data.Ig.inertia().matrix();
      Ibaseframe.noalias() = R.transpose() * (Iworldframe * R);
      rworldframe = q.template head<3>() - data.com[0];
      rbaseframe.noalias() = R.transpose() * rworldframe;
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      Ibaseframe = info.centroidalInertiaNominal;
      Iworldframe.noalias() = R * (Ibaseframe * R.transpose());
      rbaseframe = info.comToBasePositionNominal;
      rworldframe.noalias() = R * rbaseframe;
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
    }
  }

  const auto S = skewSymmetricMatrix(rworldframe);
  const auto dT = getMappingZyxGradient(eulerAngles);
  const auto dR = getRotationMatrixZyxGradient(eulerAngles);

  std::array<matrix3_t, 3> dS;
  for (size_t i = 0; i < 3; i++) {
    const vector3_t dr = dR[i] * rbaseframe;
    dS[i] = skewSymmetricMatrix(dr);
  }

  matrix_t dhdq = matrix_t::Zero(6, 3);
  for (size_t i = 0; i < 3; i++) {
    const vector3_t T_eulerAnglesDev = T * eulerAnglesDerivatives;
    const vector3_t dT_eulerAnglesDev = dT[i] * eulerAnglesDerivatives;
    const matrix3_t dR_I_Rtrans = dR[i] * Ibaseframe * R.transpose();

    dhdq.template block<3, 1>(0, i).noalias() = m * (S * dT_eulerAnglesDev);
    dhdq.template block<3, 1>(0, i).noalias() += m * (dS[i] * T_eulerAnglesDev);

    dhdq.template block<3, 1>(3, i).noalias() = (dR_I_Rtrans + dR_I_Rtrans.transpose()) * T_eulerAnglesDev;
    dhdq.template block<3, 1>(3, i).noalias() += Iworldframe * dT_eulerAnglesDev;
  }

  if (info.centroidalModelType == CentroidalModelType::FullCentroidalDynamics) {
    const auto jointVelocities = v.tail(info.actuatedDofNum);
    const vector3_t comLinearVelocityInWorldFrame = (1.0 / m) * (data.Ag.topRightCorner(3, info.actuatedDofNum) * jointVelocities);
    const vector3_t comAngularVelocityInWorldFrame =
        Iworldframe.inverse() * (data.Ag.bottomRightCorner(3, info.actuatedDofNum) * jointVelocities);
    const vector3_t linearMomentumInBaseFrame = m * (R.transpose() * comLinearVelocityInWorldFrame);
    const vector3_t angularMomentumInBaseFrame = Ibaseframe * (R.transpose() * comAngularVelocityInWorldFrame);
    for (size_t i = 0; i < 3; i++) {
      dhdq.template block<3, 1>(0, i).noalias() += dR[i] * linearMomentumInBaseFrame;
      dhdq.template block<3, 1>(3, i).noalias() += dR[i] * angularMomentumInBaseFrame;
    }
  }

  return dhdq;
}

}  // namespace ocs2
