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

#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

namespace ocs2 {
namespace ros_msg_helpers {

geometry_msgs::msg::Point getPointMsg(const Eigen::Vector3d& point) {
  geometry_msgs::msg::Point pointMsg;
  pointMsg.x = point.x();
  pointMsg.y = point.y();
  pointMsg.z = point.z();
  return pointMsg;
}

geometry_msgs::msg::Vector3 getVectorMsg(const Eigen::Vector3d& vec) {
  geometry_msgs::msg::Vector3 vecMsg;
  vecMsg.x = vec.x();
  vecMsg.y = vec.y();
  vecMsg.z = vec.z();
  return vecMsg;
}

geometry_msgs::msg::Quaternion getOrientationMsg(const Eigen::Quaterniond& orientation) {
  geometry_msgs::msg::Quaternion orientationMsg;
  orientationMsg.x = orientation.x();
  orientationMsg.y = orientation.y();
  orientationMsg.z = orientation.z();
  orientationMsg.w = orientation.w();
  return orientationMsg;
}

std_msgs::msg::Header getHeaderMsg(const std::string& frame_id, const rclcpp::Time& timeStamp) {
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = timeStamp;
  return header;
}

visualization_msgs::msg::Marker getLineMsg(std::vector<geometry_msgs::msg::Point>&& points, std::array<double, 3> color, double lineWidth) {
  visualization_msgs::msg::Marker line;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.scale.x = lineWidth;
  line.color = getColor(color);
  line.points = std::move(points);
  line.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  return line;
}

std_msgs::msg::ColorRGBA getColor(std::array<double, 3> rgb, double alpha /* = 1.0*/) {
  std_msgs::msg::ColorRGBA colorMsg;
  colorMsg.r = rgb[0];
  colorMsg.g = rgb[1];
  colorMsg.b = rgb[2];
  colorMsg.a = alpha;
  return colorMsg;
}

}  // namespace ros_msg_helpers
}  // namespace ocs2
