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

#include "ocs2_ros_interfaces/synchronized_module/RosAugmentedLagrangianCallbacks.h"

#include "ocs2_core/misc/LinearInterpolation.h"
#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

namespace ocs2 {
namespace ros {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AugmentedLagrangianObserver::metrics_callback_t createMetricsCallback(const scalar_array_t& observingTimePoints,
                                                                      std::vector<::ros::Publisher>& metricsPublishers) {
  if (observingTimePoints.size() != metricsPublishers.size()) {
    throw std::runtime_error("[createMetricsCallback] For each observing time points, you should provide a publisher!");
  }

  return [=](const scalar_array_t& timeTrajectory, const std::vector<LagrangianMetricsConstRef>& termMetricsArray) {
    if (!timeTrajectory.empty()) {
      for (size_t i = 0; i < observingTimePoints.size(); i++) {
        const auto t = timeTrajectory.front() + observingTimePoints[i];
        const auto indexAlpha = LinearInterpolation::timeSegment(t, timeTrajectory);
        const auto metrics = LinearInterpolation::interpolate(indexAlpha, termMetricsArray);
        metricsPublishers[i].publish(ros_msg_conversions::createMetricsMsg(t, metrics));
      }
    }
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AugmentedLagrangianObserver::multiplier_callback_t createMultiplierCallback(const scalar_array_t& observingTimePoints,
                                                                            std::vector<::ros::Publisher>& multiplierPublishers) {
  if (observingTimePoints.size() != multiplierPublishers.size()) {
    throw std::runtime_error("[createMetricsCallback] For each observing time points, you should provide a publisher!");
  }

  return [=](const scalar_array_t& timeTrajectory, const std::vector<MultiplierConstRef>& termMultiplierArray) {
    if (!timeTrajectory.empty()) {
      for (size_t i = 0; i < observingTimePoints.size(); i++) {
        const auto t = timeTrajectory.front() + observingTimePoints[i];
        const auto indexAlpha = LinearInterpolation::timeSegment(t, timeTrajectory);
        const auto multiplier = LinearInterpolation::interpolate(indexAlpha, termMultiplierArray);
        multiplierPublishers[i].publish(ros_msg_conversions::createMultiplierMsg(t, multiplier));
      }
    }
  };
}

}  // namespace ros
}  // namespace ocs2
