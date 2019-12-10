#include <ocs2_raisim_ros/RaisimHeightmapPublisher.h>

namespace ocs2 {

RaisimHeightmapPublisher::RaisimHeightmapPublisher() {
  gridmapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/raisim_heightmap", 1, true);
}

grid_map_msgs::GridMapPtr RaisimHeightmapPublisher::convertHeightmapToGridmap(const raisim::HeightMap& heightMap) {
  grid_map_msgs::GridMapPtr gridMapMsg(new grid_map_msgs::GridMap());

  gridMapMsg->info.header.frame_id = "world";
  gridMapMsg->info.header.stamp = ros::Time::now();

  const auto xResolution = heightMap.getXSize() / static_cast<double>(heightMap.getXSamples());
  const auto yResolution = heightMap.getYSize() / static_cast<double>(heightMap.getYSamples());
  if (std::abs(xResolution - yResolution) > 1e-9) {
    throw std::runtime_error("RaisimHeightmapPublisher::convertHeightmapToGridmap - Resolution in x and y must be identical");
  }
  gridMapMsg->info.resolution = xResolution;

  gridMapMsg->info.length_x = heightMap.getXSize();
  gridMapMsg->info.length_y = heightMap.getYSize();

  gridMapMsg->info.pose.position.x = heightMap.getCenterX();
  gridMapMsg->info.pose.position.y = heightMap.getCenterY();

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

void RaisimHeightmapPublisher::publishGridmap(const raisim::HeightMap& heightMap) const {
  auto gridMapMsg = convertHeightmapToGridmap(heightMap);
  gridmapPublisher_.publish(gridMapMsg);
}
}  // namespace ocs2
