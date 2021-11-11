/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <grid_map_msgs/GridMap.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <raisim/object/terrain/HeightMap.hpp>

namespace ocs2 {

/**
 * @brief RaisimHeightmapRosConverter is a helper class for transmitting the Raisim height map via ROS for visualization and distribution
 * across processes
 */
class RaisimHeightmapRosConverter {
 public:
  /**
   * @brief Default constructor
   * @note This will instantiate a ros::NodeHandle. ros::init(...) should be called by the user before instantiation of this class.
   */
  RaisimHeightmapRosConverter() = default;

  ~RaisimHeightmapRosConverter() = default;

  /**
   * @brief Converts the Raisim height map to a GridMap ROS message
   * @param[in] heightMap: The heightMap object from Raisim
   * @param[in] frameId: The frameId for the GridMap ROS message
   * @return The converted GridMap ROS message
   */
  static grid_map_msgs::GridMapPtr convertHeightmapToGridmap(const raisim::HeightMap& heightMap, const std::string& frameId = "world");

  /**
   * @brief Converts a GridMap ROS message to a Raisim height map
   * @param[in] gridMap: The GridMap ROS message
   * @return The converted raisim heightmap
   */
  static std::unique_ptr<raisim::HeightMap> convertGridmapToHeightmap(const grid_map_msgs::GridMapConstPtr& gridMap);

  /**
   * @brief Publishes the Raisim height map through a ROS publisher. It internally calls convertHeightmapToGridmap
   * @note The publisher is latched, hence the timing of publishing is not dependent on the subscriber's state
   * @param[in] frameId: The frameId for the GridMap ROS message
   * @param[in] heightMap: The heightMap object from Raisim
   */
  void publishGridmap(const raisim::HeightMap& heightMap, const std::string& frameId = "world");

  /**
   * @brief Obtains the raisim heightmap from ros, e.g., if the terrain is created and published by another node
   * @note Using this method without an instantiation of this class assumes that the ros node has already been fully initialized
   * @param[in] timeout Maximum waiting time if no message is received
   * @return The received raisim heightmap and GridMap (or both nullptr if no message was received before the timeout)
   */
  static std::pair<std::unique_ptr<raisim::HeightMap>, grid_map_msgs::GridMapConstPtr> getHeightmapFromRos(double timeout = 5.0);

 private:
  ros::NodeHandle nodeHandle_;
  std::unique_ptr<ros::Publisher> gridmapPublisher_;
};

}  // namespace ocs2
