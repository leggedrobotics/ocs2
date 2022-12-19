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
  using RotType = Eigen::Matrix<SCALAR_T, 3, 3>;

  vector3_s_t<SCALAR_T> position = vector3_s_t<SCALAR_T>::Zero();
  RotType orientation = RotType::Identity();

  static Pose Random() { return {vector3_s_t<SCALAR_T>::Random(), Eigen::Quaternion<SCALAR_T>::UnitRandom().toRotationMatrix()}; }

  static Pose Zero() { return {vector3_s_t<SCALAR_T>::Zero(), RotType::Identity()}; }

  template <typename NewType>
  Pose<NewType> cast() const {
    return {position.template cast<NewType>(), orientation.template cast<NewType>()};
  }
};

template <typename SCALAR_T>
struct Twist {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector3_s_t<SCALAR_T> linear = vector3_s_t<SCALAR_T>::Zero();
  vector3_s_t<SCALAR_T> angular = vector3_s_t<SCALAR_T>::Zero();

  static Twist Random() { return {vector3_s_t<SCALAR_T>::Random(), vector3_s_t<SCALAR_T>::Random()}; }

  static Twist Zero() { return {vector3_s_t<SCALAR_T>::Zero(), vector3_s_t<SCALAR_T>::Zero()}; }
};

template <typename SCALAR_T>
struct MotionTarget {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Pose<SCALAR_T> poseInWorld;
  Twist<SCALAR_T> twistInWorld;

  static MotionTarget Random() { return {Pose<SCALAR_T>::Random(), Twist<SCALAR_T>::Random()}; }

  static MotionTarget Zero() { return {Pose<SCALAR_T>::Zero(), Twist<SCALAR_T>::Zero()}; }
};

/**
 * Converts flat vector of size 13 to a motion target
 * @param parameters : (position XYZ, quaternion XYZW, linear velocity XYZ, angular velocity XYZ)
 * @param target : Motion target to fill
 */
template <typename SCALAR_T, typename EIGEN_DERIVED>
void updateMotionTarget(const EIGEN_DERIVED& parameters, MotionTarget<SCALAR_T>& target) {
  target.poseInWorld.position = parameters.template head<3>();
  target.poseInWorld.orientation = Eigen::Quaternion<SCALAR_T>(parameters.template segment<4>(3)).toRotationMatrix();
  target.twistInWorld.linear = parameters.template segment<3>(7);
  target.twistInWorld.angular = parameters.template segment<3>(10);
}

template <typename SCALAR_T>
MotionTarget<SCALAR_T> fromVector(const Eigen::Matrix<SCALAR_T, -1, 1>& vector, int idx = 0) {
  MotionTarget<SCALAR_T> target;
  target.poseInWorld.position = vector.template segment<3>(idx);
  idx += 3;
  target.poseInWorld.orientation.col(0) = vector.template segment<3>(idx);
  idx += 3;
  target.poseInWorld.orientation.col(1) = vector.template segment<3>(idx);
  idx += 3;
  target.poseInWorld.orientation.col(2) = vector.template segment<3>(idx);
  idx += 3;
  target.twistInWorld.linear = vector.template segment<3>(idx);
  idx += 3;
  target.twistInWorld.angular = vector.template segment<3>(idx);
  return target;
}

template <typename SCALAR_T>
void addToVector(const MotionTarget<SCALAR_T>& target, Eigen::Matrix<SCALAR_T, -1, 1>& vector, int idx = 0) {
  vector.template segment<3>(idx) = target.poseInWorld.position;
  idx += 3;
  vector.template segment<3>(idx) = target.poseInWorld.orientation.col(0);
  idx += 3;
  vector.template segment<3>(idx) = target.poseInWorld.orientation.col(1);
  idx += 3;
  vector.template segment<3>(idx) = target.poseInWorld.orientation.col(2);
  idx += 3;
  vector.template segment<3>(idx) = target.twistInWorld.linear;
  idx += 3;
  vector.template segment<3>(idx) = target.twistInWorld.angular;
}

template <typename SCALAR_T>
struct TrackingOffset {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int parentFrameId = 0;
  Pose<SCALAR_T> poseInParent;

  template <typename NewType>
  TrackingOffset<NewType> cast() const {
    return {parentFrameId, poseInParent.template cast<NewType>()};
  }
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

enum TrackingTaskType : int { POSITION, ORIENTATION };

template <typename SCALAR_T>
struct AbsoluteTrackingTask {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TrackingTaskType type = TrackingTaskType::POSITION;
  size_t targetId = 0;
  size_t sourceId = 0;
  vector3_s_t<SCALAR_T> poseWeights = vector3_s_t<SCALAR_T>::Zero();
  vector3_s_t<SCALAR_T> twistweights = vector3_s_t<SCALAR_T>::Zero();

  template <typename NewType>
  AbsoluteTrackingTask<NewType> cast() const {
    AbsoluteTrackingTask<NewType> newTask;
    newTask.type = type;
    newTask.targetId = targetId;
    newTask.sourceId = sourceId;
    newTask.poseWeights = poseWeights.template cast<NewType>();
    newTask.twistweights = twistweights.template cast<NewType>();
    return newTask;
  }
};

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> getCostVector(const AbsoluteTrackingTask<SCALAR_T>& task, const std::vector<MotionTarget<SCALAR_T>>& sourceFrames,
                                    const std::vector<MotionTarget<SCALAR_T>>& targetFrames) {
  const auto& source = sourceFrames[task.sourceId];
  const auto& target = targetFrames[task.targetId];

  if (task.type == TrackingTaskType::POSITION) {
    const auto positionError = source.poseInWorld.position - target.poseInWorld.position;
    const auto velocityError = source.twistInWorld.linear - target.twistInWorld.linear;
    return task.poseWeights.template cwiseProduct(positionError) + task.twistweights.template cwiseProduct(velocityError);
  } else {
    const auto orientationError = ocs2::rotationErrorInWorld<SCALAR_T>(source.poseInWorld.orientation, target.poseInWorld.orientation);
    const auto velocityError = source.twistInWorld.angular - target.twistInWorld.angular;
    return task.poseWeights.template cwiseProduct(orientationError) + task.twistweights.template cwiseProduct(velocityError);
  }
}

}  // namespace switched_model

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
