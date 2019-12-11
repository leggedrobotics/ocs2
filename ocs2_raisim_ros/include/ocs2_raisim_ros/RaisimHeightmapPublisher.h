#include <grid_map_msgs/GridMap.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <raisim/object/terrain/HeightMap.hpp>

namespace ocs2 {

/**
 * @brief RaisimHeightmapPublisher is a helper class for visualizing the Raisim height map in RViz
 */
class RaisimHeightmapPublisher {
 public:
  RaisimHeightmapPublisher();
  ~RaisimHeightmapPublisher() = default;

  /**
   * @brief Converts the Raisim height map to a GridMap ROS message
   * @param heightMap: The heightMap object from Raisim
   * @return The converted GridMap ROS message
   */
  static grid_map_msgs::GridMapPtr convertHeightmapToGridmap(const raisim::HeightMap& heightMap);

  /**
   * @brief Publishes the Raisim height map through a ROS publisher. It internally calls convertHeightmapToGridmap
   * @note The publisher is latched, hence the timing of publishing is not dependent on the subscriber's state
   * @param heightMap: The heightMap object from Raisim
   */
  void publishGridmap(const raisim::HeightMap& heightMap) const;

 private:
  ros::NodeHandle nodeHandle_;
  ros::Publisher gridmapPublisher_;
};

}  // namespace ocs2
