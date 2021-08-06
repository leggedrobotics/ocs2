#include "../../../ocs2_raisim/ocs2_raisim_ros/include/ocs2_raisim_ros/RaisimHeightmapRosConverter.h"

#include <ros/topic.h>

namespace ocs2 {

grid_map_msgs::GridMapPtr RaisimHeightmapRosConverter::convertHeightmapToGridmap(const raisim::HeightMap& heightMap) {
  grid_map_msgs::GridMapPtr gridMapMsg(new grid_map_msgs::GridMap());

  gridMapMsg->info.header.frame_id = "world";
  gridMapMsg->info.header.stamp = ros::Time::now();

  const auto xResolution = heightMap.getXSize() / static_cast<double>(heightMap.getXSamples());
  const auto yResolution = heightMap.getYSize() / static_cast<double>(heightMap.getYSamples());
  if (std::abs(xResolution - yResolution) > 1e-9) {
    throw std::runtime_error("RaisimHeightmapRosConverter::convertHeightmapToGridmap - Resolution in x and y must be identical");
  }
  gridMapMsg->info.resolution = xResolution;

  gridMapMsg->info.length_x = heightMap.getXSize();
  gridMapMsg->info.length_y = heightMap.getYSize();

  gridMapMsg->info.pose.position.x = heightMap.getCenterX();
  gridMapMsg->info.pose.position.y = heightMap.getCenterY();
  gridMapMsg->info.pose.orientation.w = 1.0;

  gridMapMsg->layers.emplace_back("elevation");
  std_msgs::Float32MultiArray dataArray;
  dataArray.layout.dim.resize(2);
  dataArray.layout.dim[0].label = "column_index";
  dataArray.layout.dim[0].stride = heightMap.getHeightVector().size();
  dataArray.layout.dim[0].size = heightMap.getXSamples();
  dataArray.layout.dim[1].label = "row_index";
  dataArray.layout.dim[1].stride = heightMap.getYSamples();
  dataArray.layout.dim[1].size = heightMap.getYSamples();
  dataArray.data.insert(dataArray.data.begin(), heightMap.getHeightVector().rbegin(), heightMap.getHeightVector().rend());
  gridMapMsg->data.push_back(dataArray);

  return gridMapMsg;
}

std::unique_ptr<raisim::HeightMap> RaisimHeightmapRosConverter::convertGridmapToHeightmap(const grid_map_msgs::GridMapConstPtr& gridMap) {
  if (gridMap->data[0].layout.dim[0].label != "column_index" or gridMap->data[0].layout.dim[1].label != "row_index") {
    throw std::runtime_error("RaisimHeightmapRosConverter::convertGridmapToHeightmap - Layout of gridMap currently not supported");
  }

  const int xSamples = gridMap->data[0].layout.dim[0].size;
  const int ySamples = gridMap->data[0].layout.dim[1].size;
  const double xSize = gridMap->info.length_x;
  const double ySize = gridMap->info.length_y;
  const double centerX = gridMap->info.pose.position.x;
  const double centerY = gridMap->info.pose.position.y;

  std::vector<double> height(gridMap->data[0].data.rbegin(), gridMap->data[0].data.rend());

  std::unique_ptr<raisim::HeightMap> heightMap(new raisim::HeightMap(xSamples, ySamples, xSize, ySize, centerX, centerY, height));
  return heightMap;
}

void RaisimHeightmapRosConverter::publishGridmap(const raisim::HeightMap& heightMap) {
  if (!gridmapPublisher_) {
    gridmapPublisher_.reset(new ros::Publisher(nodeHandle_.advertise<grid_map_msgs::GridMap>("/raisim_heightmap", 1, true)));
  }
  auto gridMapMsg = convertHeightmapToGridmap(heightMap);
  gridmapPublisher_->publish(gridMapMsg);
}

std::pair<std::unique_ptr<raisim::HeightMap>, grid_map_msgs::GridMapConstPtr> RaisimHeightmapRosConverter::getHeightmapFromRos(
    double timeout) {
  auto gridMapMsg = ros::topic::waitForMessage<grid_map_msgs::GridMap>("/raisim_heightmap", ros::Duration(timeout));
  return {gridMapMsg ? convertGridmapToHeightmap(gridMapMsg) : nullptr, gridMapMsg};
}

}  // namespace ocs2
