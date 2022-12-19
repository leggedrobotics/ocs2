//
// Created by Timon Kaufmann in June 2021
//

#pragma once

#include <string>
#include <unordered_map>

#include <ros/ros.h>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_switched_model_interface/logic/Gait.h>

namespace switched_model {

class MotionCommandInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit MotionCommandInterface(const std::string& configFile);
  virtual ~MotionCommandInterface() = default;

  virtual void publishMotion(const std::pair<ocs2::TargetTrajectories, Gait>& motion) = 0;

  void getKeyboardCommand();

 private:
  void printAnimationList() const;

  std::unordered_map<std::string, std::pair<ocs2::TargetTrajectories, Gait>> motionData_;
};

}  // namespace switched_model