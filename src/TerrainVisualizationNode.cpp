//
// Created by rgrandia on 08.07.19.
//

#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"

#include <ocs2_switched_model_interface/ground/TerrainModel.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "TerrainVisualizationNode");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  std::vector<ros::Publisher> polygonPublishers;

  switched_model::TerrainModel terrainModel;
  terrainModel.loadTestTerrain();
  auto& polytopes = terrainModel.getPolytopes();

  for (int i=0; i<polytopes.size(); i++){
    std::string name = "polygon" + std::to_string(i);
    polygonPublishers.emplace_back(n.advertise<geometry_msgs::PolygonStamped>(name, 1));
  }

  int count = 0;
  while (ros::ok())
  {
    for (int i=0; i<polytopes.size(); i++){
      polygonPublishers[i].publish(switched_model::toRos(polytopes[i], "world"));
    }
    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }

  return 0;
}