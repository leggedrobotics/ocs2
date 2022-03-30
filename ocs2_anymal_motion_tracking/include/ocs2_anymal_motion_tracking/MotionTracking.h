//
// Created by rgrandia on 25.03.22.
//

#pragma once

#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

template <typename SCALAR_T>
struct Pose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector3_s_t<SCALAR_T> position = vector3_s_t<SCALAR_T>::Zero();
  Eigen::Quaternion<SCALAR_T> orientation = Eigen::Quaternion<SCALAR_T>::Identity();
};

template <typename SCALAR_T>
struct Twist {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector3_s_t<SCALAR_T> linear = vector3_s_t<SCALAR_T>::Zero();
  vector3_s_t<SCALAR_T> angular = vector3_s_t<SCALAR_T>::Zero();
};

template <typename SCALAR_T>
struct MotionTarget {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Pose<SCALAR_T> poseInWorld;
  Twist<SCALAR_T> twistInWorld;
};

/**
 * Converts flat vector of size 13 to a motion target
 * @param parameters : (position XYZ, quaternion XYZW, linear velocity XYZ, angular velocity XYZ)
 * @param target : Motion target to fill
 */
template <typename SCALAR_T, typename EIGEN_DERIVED>
void updateMotionTarget(const EIGEN_DERIVED& parameters, MotionTarget<SCALAR_T>& target) {
  target.poseInWorld.position = parameters.template head<3>();
  target.poseInWorld.orientation = parameters.template segment<4>(3);
  target.twistInWorld.linear = parameters.template segment<3>(7);
  target.twistInWorld.angular = parameters.template segment<3>(10);
}

template <typename SCALAR_T>
struct TrackingOffset {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int parentFrameId = 0;
  Pose<SCALAR_T> poseInParent;
};

template <typename SCALAR_T>
void applyTrackingOffset(const MotionTarget<SCALAR_T>& parent, const TrackingOffset<SCALAR_T>& offset, MotionTarget<SCALAR_T>& source) {
  // W_R_P * P_dp_C
  vector3_s_t<SCALAR_T> parentToSourceInWorld = parent.poseInWorld.orientation * offset.poseInParent.position;
  // W_p_C = W_p_P + W_R_P * P_dp_C
  source.poseInWorld.position = parent.poseInWorld.position + parentToSourceInWorld;
  // W_R_C = W_R_P * P_R_C
  source.poseInWorld.orientation = parent.poseInWorld.orientation * offset.poseInParent.orientation;
  // W_v_C = W_v_P + W_w_P x (W_R_P * P_dp_C)
  source.twistInWorld.linear = parent.twistInWorld.linear + parent.twistInWorld.angular.cross(parentToSourceInWorld);
  // W_a_C = W_a_P
  source.twistInWorld.angular = parent.twistInWorld.angular;
}

template <typename SCALAR_T>
void updateSourceFrames(const std::vector<MotionTarget<SCALAR_T>>& parentFrames, const std::vector<TrackingOffset<SCALAR_T>>& offsets,
                        std::vector<MotionTarget<SCALAR_T>>& sourceFrames) {
  assert(offsets.size() == sourceFrames.size());
  auto sourceFrameIt = sourceFrames.begin();
  for (auto offsetIt = offsets.cbegin(); offsetIt != offsets.end(); ++offsetIt, ++sourceFrameIt) {
    const auto& parent = parentFrames[offsetIt->parentFrameId];
    applyTrackingOffset(parent, *offsetIt, *sourceFrameIt);
  }
}

template <typename SCALAR_T>
struct AbsoluteTrackingTask {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum class Type { POSITION, ORIENTATION };
  Type type;
  const MotionTarget<SCALAR_T>* target;
  const MotionTarget<SCALAR_T>* source;
  vector3_s_t<SCALAR_T> poseWeights = vector3_s_t<SCALAR_T>::Zero();
  vector3_s_t<SCALAR_T> twistweights = vector3_s_t<SCALAR_T>::Zero();
};

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> getCostVector(const AbsoluteTrackingTask<SCALAR_T>& task) {
  switch (task.type) {
    case AbsoluteTrackingTask<SCALAR_T>::Type::Position: {
      const auto positionError = task.source->poseInWorld.position - task.target->poseInWorld.position;
      const auto velocityError = task.source->twistInWorld.linear - task.target->twistInWorld.linear;
      return task.poseWeights.template cwiseProduct(positionError) + task.twistweights.template cwiseProduct(velocityError);
    }
    case AbsoluteTrackingTask<SCALAR_T>::Type::ORIENTATION: {
      const auto orientationError = rotationError(task.source->poseInWorld.orientation.inverse().toRotationMatrix(),
                                                  task.target->poseInWorld.orientation.inverse().toRotationMatrix());
      const auto velocityError = task.source->twistInWorld.angular - task.target->twistInWorld.angular;
      return task.poseWeights.template cwiseProduct(orientationError) + task.twistweights.template cwiseProduct(velocityError);
    }
  }
}

}  // namespace switched_model