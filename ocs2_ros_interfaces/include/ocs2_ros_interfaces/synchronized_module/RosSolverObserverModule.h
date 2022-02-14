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

#pragma once

#include <algorithm>

#include <ros/ros.h>

#include <ocs2_oc/synchronized_module/SolverObserverModule.h>

#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

namespace ocs2 {

/**
 * A ROS wrapper for SolverObserverModule that publishes metrics and multiplier corresponding to the requested term at
 * requested lookahead time points. The class will publish in the following topics respectively for metrics and multipliers
 * + metrics/TERMS_NAME/xxxMsLookAhead
 * + multipliers/TERMS_NAME/xxxMsLookAhead
 *
 * where TERMS_NAME is the name of the term and xxx is the lookahead time in millisecond.
 */
class RosSolverObserverModule final : public SolverObserverModule {
 public:
  RosSolverObserverModule(std::string termsName, scalar_array_t timePoints)
      : SolverObserverModule(std::move(termsName)), timePoints_(std::move(timePoints)) {}

  /**
   * Launches the publishers. Refer to the class description to see the topic names that the module publishes on them.
   */
  void subscribe(ros::NodeHandle& nh) {
    for (const auto& t : timePoints_) {
      const int timeMs = static_cast<int>(t * 1000.0);
      const std::string topicName = termsName_ + "/" + std::to_string(timeMs) + "MsLookAhead";
      metricsPublishers_.push_back(nh.advertise<ocs2_msgs::metrics>("metrics/" + topicName, 1, true));
      multiplierPublishers_.push_back(nh.advertise<ocs2_msgs::multiplier>("multipliers/" + topicName, 1, true));
    }

    setMetricsCallback([this](const scalar_array_t& timeTrajectory, const std::vector<MetricsConstRef>& termsMetrics) {
      this->observeTermMetrics(timeTrajectory, termsMetrics);
    });

    setMultiplierCallback([this](const scalar_array_t& timeTrajectory, const std::vector<MultiplierConstRef>& termMultiplierArray) {
      this->observeTermMultiplier(timeTrajectory, termMultiplierArray);
    });
  }

  ~RosSolverObserverModule() override = default;

 private:
  void observeTermMetrics(const scalar_array_t& timeTrajectory, const std::vector<MetricsConstRef>& termsMetrics) {
    for (size_t i = 0; i < timePoints_.size(); i++) {
      const auto t = timeTrajectory.front() + timePoints_[i];
      const auto ind = findNearestIndex(timeTrajectory, t);
      if (ind >= 0) {
        metricsPublishers_[i].publish(ros_msg_conversions::createMetricsMsg(t, termsMetrics[ind]));
      }
    }
  }

  void observeTermMultiplier(const scalar_array_t& timeTrajectory, const std::vector<MultiplierConstRef>& termMultiplierArray) {
    for (size_t i = 0; i < timePoints_.size(); i++) {
      const auto t = timeTrajectory.front() + timePoints_[i];
      const auto ind = findNearestIndex(timeTrajectory, t);
      if (ind >= 0) {
        multiplierPublishers_[i].publish(ros_msg_conversions::createMultiplierMsg(t, termMultiplierArray[ind]));
      }
    }
  }

  // Find the closest entry in the timeTrajectory to the given time. The return value is in the range [-1, timeTrajectory.size()).
  // Minus one is the case where the input time array is empty.
  int findNearestIndex(const scalar_array_t& timeTrajectory, scalar_t time) const {
    const auto itr = std::lower_bound(timeTrajectory.cbegin(), timeTrajectory.cend(), time);
    if (itr != timeTrajectory.cend()) {
      return std::distance(timeTrajectory.cbegin(), itr);
    } else {
      return timeTrajectory.size() - 1;
    }
  }

  const scalar_array_t timePoints_;
  std::vector<ros::Publisher> metricsPublishers_;
  std::vector<ros::Publisher> multiplierPublishers_;
};

}  // namespace ocs2
