//
// Created by rgrandia on 19.07.22.
//

#pragma once

namespace switched_model {

template <typename SCALAR_T>
struct CostElements {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector3_s_t<SCALAR_T> eulerXYZ{vector3_s_t<SCALAR_T>::Zero()};
  vector3_s_t<SCALAR_T> comPosition{vector3_s_t<SCALAR_T>::Zero()};
  vector3_s_t<SCALAR_T> comAngularVelocity{vector3_s_t<SCALAR_T>::Zero()};
  vector3_s_t<SCALAR_T> comLinearVelocity{vector3_s_t<SCALAR_T>::Zero()};
  feet_array_t<vector3_s_t<SCALAR_T>> jointPosition{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> footPosition{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> jointVelocity{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> footVelocity{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};
  feet_array_t<vector3_s_t<SCALAR_T>> contactForce{constantFeetArray<vector3_s_t<SCALAR_T>>(vector3_s_t<SCALAR_T>::Zero())};

  static size_t Size() {
    constexpr size_t baseTargets = 12;
    constexpr size_t legTargets = 15;
    return baseTargets + NUM_CONTACT_POINTS * legTargets;
  }

  Eigen::Matrix<SCALAR_T, -1, 1> asVector() const {
    constexpr size_t baseTargets = 12;
    constexpr size_t legTargets = 15;
    constexpr size_t costVectorLength = baseTargets + NUM_CONTACT_POINTS * legTargets;

    Eigen::Matrix<SCALAR_T, -1, 1> v(costVectorLength);

    // Base
    v.head(baseTargets) << eulerXYZ, comPosition, comAngularVelocity, comLinearVelocity;

    // Legs
    for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
      v.segment(baseTargets + leg * legTargets, legTargets) << jointPosition[leg], footPosition[leg], jointVelocity[leg], footVelocity[leg],
          contactForce[leg];
    }
    return v;
  }
};

}  // namespace switched_model
