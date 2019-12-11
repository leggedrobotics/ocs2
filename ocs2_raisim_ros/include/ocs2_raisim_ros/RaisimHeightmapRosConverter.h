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
   * @return The converted GridMap ROS message
   */
  static grid_map_msgs::GridMapPtr convertHeightmapToGridmap(const raisim::HeightMap& heightMap);

  /**
   * @brief Converts a GridMap ROS message to a Raisim height map
   * @param[in] gridMap: The GridMap ROS message
   * @return The converted raisim heightmap
   */
  static std::unique_ptr<raisim::HeightMap> convertGridmapToHeightmap(const grid_map_msgs::GridMapConstPtr& gridMap);

  /**
   * @brief Publishes the Raisim height map through a ROS publisher. It internally calls convertHeightmapToGridmap
   * @note The publisher is latched, hence the timing of publishing is not dependent on the subscriber's state
   * @param[in] heightMap: The heightMap object from Raisim
   */
  void publishGridmap(const raisim::HeightMap& heightMap);

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
