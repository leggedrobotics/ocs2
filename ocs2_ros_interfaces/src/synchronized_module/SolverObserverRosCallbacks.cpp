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

#include "ocs2_ros_interfaces/synchronized_module/SolverObserverRosCallbacks.h"

#include "ocs2_core/misc/LinearInterpolation.h"
#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

namespace ocs2 {
namespace ros {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SolverObserver::constraint_callback_t createConstraintCallback(::ros::NodeHandle& nodeHandle, const scalar_array_t& observingTimePoints,
                                                               const std::vector<std::string>& topicNames,
                                                               CallbackInterpolationStrategy interpolationStrategy) {
  using vector_ref_array_t = std::vector<std::reference_wrapper<const vector_t>>;

  if (observingTimePoints.size() != topicNames.size()) {
    throw std::runtime_error("[createConstraintCallback] For each observing time points, you should provide a unique topic name!");
  }

  std::vector<::ros::Publisher> constraintPublishers;
  for (const auto& name : topicNames) {
    constraintPublishers.push_back(nodeHandle.advertise<ocs2_msgs::constraint>(name, 1, true));
  }

  // note that we need to copy the publishers as the local ones will go out of scope. Good news is that ROS publisher
  // behaves like std::sharted_ptr ("Once all copies of a specific Publisher go out of scope, any subscriber status callbacks
  // associated with that handle will stop being called.")
  return [=](const scalar_array_t& timeTrajectory, const vector_ref_array_t& termConstraintArray) {
    if (!timeTrajectory.empty()) {
      for (size_t i = 0; i < observingTimePoints.size(); i++) {
        const auto t = timeTrajectory.front() + observingTimePoints[i];
        const auto indexAlpha = LinearInterpolation::timeSegment(t, timeTrajectory);
        const auto constraint = [&]() -> vector_t {
          switch (interpolationStrategy) {
            case CallbackInterpolationStrategy::nearest_time:
              return (indexAlpha.second > 0.5) ? termConstraintArray[indexAlpha.first].get()
                                               : termConstraintArray[indexAlpha.first + 1].get();
            case CallbackInterpolationStrategy::linear_interpolation:
              return LinearInterpolation::interpolate(
                  indexAlpha, termConstraintArray,
                  [](const vector_ref_array_t& array, size_t t) -> const vector_t& { return array[t].get(); });
            default:
              throw std::runtime_error("[createConstraintCallback] This CallbackInterpolationStrategy is not implemented!");
          }
        }();
        constraintPublishers[i].publish(ros_msg_conversions::createConstraintMsg(t, constraint));
      }
    }
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SolverObserver::lagrangian_callback_t createLagrangianCallback(::ros::NodeHandle& nodeHandle, const scalar_array_t& observingTimePoints,
                                                               const std::vector<std::string>& topicNames,
                                                               CallbackInterpolationStrategy interpolationStrategy) {
  if (observingTimePoints.size() != topicNames.size()) {
    throw std::runtime_error("[createLagrangianCallback] For each observing time points, you should provide a unique topic name!");
  }

  std::vector<::ros::Publisher> metricsPublishers;
  for (const auto& name : topicNames) {
    metricsPublishers.push_back(nodeHandle.advertise<ocs2_msgs::lagrangian_metrics>(name, 1, true));
  }

  // note that we need to copy the publishers as the local ones will go out of scope. Good news is that ROS publisher
  // behaves like std::sharted_ptr ("Once all copies of a specific Publisher go out of scope, any subscriber status callbacks
  // associated with that handle will stop being called.")
  return [=](const scalar_array_t& timeTrajectory, const std::vector<LagrangianMetricsConstRef>& termMetricsArray) {
    if (!timeTrajectory.empty()) {
      for (size_t i = 0; i < observingTimePoints.size(); i++) {
        const auto t = timeTrajectory.front() + observingTimePoints[i];
        const auto indexAlpha = LinearInterpolation::timeSegment(t, timeTrajectory);
        const auto metrics = [&]() -> LagrangianMetrics {
          switch (interpolationStrategy) {
            case CallbackInterpolationStrategy::nearest_time:
              return (indexAlpha.second > 0.5) ? static_cast<LagrangianMetrics>(termMetricsArray[indexAlpha.first])
                                               : static_cast<LagrangianMetrics>(termMetricsArray[indexAlpha.first + 1]);
            case CallbackInterpolationStrategy::linear_interpolation:
              return LinearInterpolation::interpolate(indexAlpha, termMetricsArray);
            default:
              throw std::runtime_error("[createLagrangianCallback] This CallbackInterpolationStrategy is not implemented!");
          }
        }();
        metricsPublishers[i].publish(ros_msg_conversions::createLagrangianMetricsMsg(t, metrics));
      }
    }
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SolverObserver::multiplier_callback_t createMultiplierCallback(::ros::NodeHandle& nodeHandle, const scalar_array_t& observingTimePoints,
                                                               const std::vector<std::string>& topicNames,
                                                               CallbackInterpolationStrategy interpolationStrategy) {
  if (observingTimePoints.size() != topicNames.size()) {
    throw std::runtime_error("[createMultiplierCallback] For each observing time points, you should provide a unique topic name!");
  }

  std::vector<::ros::Publisher> multiplierPublishers;
  for (const auto& name : topicNames) {
    multiplierPublishers.push_back(nodeHandle.advertise<ocs2_msgs::multiplier>(name, 1, true));
  }

  // note that we need to copy the publishers as the local ones will go out of scope. Good news is that ROS publisher
  // behaves like std::sharted_ptr ("Once all copies of a specific Publisher go out of scope, any subscriber status callbacks
  // associated with that handle will stop being called.")
  return [=](const scalar_array_t& timeTrajectory, const std::vector<MultiplierConstRef>& termMultiplierArray) {
    if (!timeTrajectory.empty()) {
      for (size_t i = 0; i < observingTimePoints.size(); i++) {
        const auto t = timeTrajectory.front() + observingTimePoints[i];
        const auto indexAlpha = LinearInterpolation::timeSegment(t, timeTrajectory);
        const auto multiplier = [&]() -> Multiplier {
          switch (interpolationStrategy) {
            case CallbackInterpolationStrategy::nearest_time:
              return (indexAlpha.second > 0.5) ? static_cast<Multiplier>(termMultiplierArray[indexAlpha.first])
                                               : static_cast<Multiplier>(termMultiplierArray[indexAlpha.first + 1]);
            case CallbackInterpolationStrategy::linear_interpolation:
              return LinearInterpolation::interpolate(indexAlpha, termMultiplierArray);
            default:
              throw std::runtime_error("[createMultiplierCallback] This CallbackInterpolationStrategy is not implemented!");
          }
        }();
        multiplierPublishers[i].publish(ros_msg_conversions::createMultiplierMsg(t, multiplier));
      }
    }
  };
}

}  // namespace ros
}  // namespace ocs2
